#include "RemoteControl.h"

#include "Logger.h"

#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if REMOTE_INPUT_BACKEND == REMOTE_BACKEND_USB_HOST
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#endif

static constexpr uint8_t RAW_REPORT_LOG_BYTES = 64;
static constexpr uint32_t REMOTE_DEBUG_MIN_INTERVAL_MS = 100;
static constexpr uint32_t REMOTE_DEBUG_HEARTBEAT_MS = 1000;
static constexpr int16_t REMOTE_DEBUG_AXIS_DELTA = 20;
static constexpr uint16_t REMOTE_DEBUG_CHANNEL_DELTA_US = 20;
static constexpr uint8_t BUTTON_A_BIT = 0;
static constexpr uint8_t BUTTON_B_BIT = 1;
static constexpr uint8_t BUTTON_X_BIT = 2;
static constexpr uint8_t BUTTON_Y_BIT = 3;
static constexpr uint8_t BUTTON_LB_BIT = 4;
static constexpr uint8_t BUTTON_RB_BIT = 5;
static constexpr uint8_t BUTTON_SELECT_BIT = 6;
static constexpr uint8_t BUTTON_START_BIT = 7;
static constexpr int16_t TRIGGER_ACTION_THRESHOLD = 250;
static constexpr uint8_t HID_MAX_FIELDS = 64;
static constexpr uint8_t HID_MAX_LOCAL_USAGES = 32;
static constexpr uint8_t HID_MAX_REPORT_IDS = 8;

enum HidFieldKind : uint8_t {
  HID_FIELD_LX,
  HID_FIELD_LY,
  HID_FIELD_RX,
  HID_FIELD_RY,
  HID_FIELD_LT,
  HID_FIELD_RT,
  HID_FIELD_HAT,
  HID_FIELD_BUTTON,
  HID_FIELD_BUTTON_ARRAY,
};

struct HidInputField {
  HidFieldKind kind;
  uint8_t reportId;
  uint16_t bitOffset;
  uint8_t bitSize;
  int32_t logicalMin;
  int32_t logicalMax;
  uint16_t usagePage;
  uint16_t usage;
  uint8_t buttonIndex;
};

struct HidReportLayout {
  HidInputField fields[HID_MAX_FIELDS];
  uint8_t fieldCount;
  bool valid;
  bool usesReportIds;
};

struct GamepadState {
  int16_t lx;
  int16_t ly;
  int16_t rx;
  int16_t ry;
  int16_t lt;
  int16_t rt;
  uint32_t buttons;
  uint8_t hat;
  uint8_t reportId;
  uint8_t raw[RAW_REPORT_LOG_BYTES];
  uint8_t rawLength;
  uint8_t axisCenter[4];
  bool axisCalibrated;
  bool hatIsButtonMask;
  bool connected;
  bool reportSeen;
  uint32_t lastReportUs;
};

static portMUX_TYPE gamepadMux = portMUX_INITIALIZER_UNLOCKED;
static GamepadState gamepadState = {};
static HidReportLayout hidReportLayout = {};
static bool remoteZoneLatch[REMOTE_CHANNEL_COUNT][3] = {};

static bool channelLooksValid(uint16_t value) {
  return value >= REMOTE_MIN_PULSE_US && value <= REMOTE_MAX_PULSE_US;
}

static void setGamepadDisconnected() {
  portENTER_CRITICAL(&gamepadMux);
  gamepadState.connected = false;
  gamepadState.reportSeen = false;
  gamepadState.axisCalibrated = false;
  gamepadState.lx = 0;
  gamepadState.ly = 0;
  gamepadState.rx = 0;
  gamepadState.ry = 0;
  gamepadState.lt = 0;
  gamepadState.rt = 0;
  gamepadState.buttons = 0;
  gamepadState.hat = 0;
  gamepadState.rawLength = 0;
  gamepadState.hatIsButtonMask = false;
  gamepadState.lastReportUs = 0;
  portEXIT_CRITICAL(&gamepadMux);
}

static int16_t decodeAxis(uint8_t value, uint8_t center) {
  int delta = static_cast<int>(value) - static_cast<int>(center);
  if (delta > 127) {
    delta -= 256;
  } else if (delta < -128) {
    delta += 256;
  }
  return constrain(delta * 4, -500, 500);
}

static uint16_t axisToPulse(int16_t axis) {
  axis = constrain(axis, -500, 500);
  return static_cast<uint16_t>(REMOTE_CENTER_US + axis);
}

static uint16_t buttonSwitchPulse(uint32_t buttons, uint8_t lowBit,
                                  uint8_t highBit) {
  if (buttons & (1UL << lowBit)) {
    return SWITCH_LOW_MIN_US;
  }
  if (buttons & (1UL << highBit)) {
    return SWITCH_HIGH_MAX_US;
  }
  return REMOTE_CENTER_US;
}

static uint16_t triggerSwitchPulse(int16_t leftTrigger, int16_t rightTrigger) {
  if (leftTrigger >= TRIGGER_ACTION_THRESHOLD) {
    return SWITCH_LOW_MIN_US;
  }
  if (rightTrigger >= TRIGGER_ACTION_THRESHOLD) {
    return SWITCH_HIGH_MAX_US;
  }
  return REMOTE_CENTER_US;
}

static void hatToMotionAxes(uint8_t hat, bool buttonMask, int16_t *walk,
                            int16_t *strafe) {
  if (walk == nullptr || strafe == nullptr) {
    return;
  }

  *walk = 0;
  *strafe = 0;
  if (buttonMask) {
    if (hat & 0x01) {
      *walk = 500;
    } else if (hat & 0x02) {
      *walk = -500;
    }
    if (hat & 0x08) {
      *strafe = 500;
    } else if (hat & 0x04) {
      *strafe = -500;
    }
    return;
  }

  switch (hat) {
    case 0:
      *walk = 500;
      break;
    case 1:
      *walk = 500;
      *strafe = 500;
      break;
    case 2:
      *strafe = 500;
      break;
    case 3:
      *walk = -500;
      *strafe = 500;
      break;
    case 4:
      *walk = -500;
      break;
    case 5:
      *walk = -500;
      *strafe = -500;
      break;
    case 6:
      *strafe = -500;
      break;
    case 7:
      *walk = 500;
      *strafe = -500;
      break;
    default:
      break;
  }
}

static uint8_t reportAxisOffset(uint8_t reportId, const uint8_t *data,
                                size_t length) {
  if (length >= 7 && reportId != 0 && data[0] == reportId) {
    return 1;
  }
  if (length >= 7 && reportId == 0 && data[0] > 0 && data[0] <= 0x0f) {
    return 1;
  }
  return 0;
}

static int32_t readHidItemValue(const uint8_t *data, uint8_t size,
                                bool signedValue) {
  int32_t value = 0;
  for (uint8_t i = 0; i < size; ++i) {
    value |= static_cast<int32_t>(data[i]) << (8 * i);
  }
  if (signedValue && size > 0 && size < 4) {
    const int32_t signBit = 1 << (size * 8 - 1);
    if (value & signBit) {
      value |= ~((1 << (size * 8)) - 1);
    }
  }
  return value;
}

static uint32_t readHidBits(const uint8_t *data, size_t length,
                            uint16_t bitOffset, uint8_t bitSize,
                            bool *ok) {
  if (bitSize == 0 || bitSize > 32 ||
      static_cast<uint32_t>(bitOffset) + bitSize > length * 8) {
    *ok = false;
    return 0;
  }

  uint32_t value = 0;
  for (uint8_t i = 0; i < bitSize; ++i) {
    const uint16_t sourceBit = bitOffset + i;
    if (data[sourceBit / 8] & (1u << (sourceBit % 8))) {
      value |= 1u << i;
    }
  }
  *ok = true;
  return value;
}

static int32_t signExtendHidValue(uint32_t value, uint8_t bitSize) {
  if (bitSize == 0 || bitSize >= 32 || ((value >> (bitSize - 1)) & 1u) == 0) {
    return static_cast<int32_t>(value);
  }
  return static_cast<int32_t>(value | (~0u << bitSize));
}

static int16_t normalizeHidAxis(int32_t value, int32_t logicalMin,
                                int32_t logicalMax) {
  if (logicalMax <= logicalMin) {
    return 0;
  }
  const int64_t numerator =
      (static_cast<int64_t>(value) * 2 - logicalMin - logicalMax) * 500;
  const int64_t denominator = logicalMax - logicalMin;
  return static_cast<int16_t>(
      constrain(static_cast<int>(numerator / denominator), -500, 500));
}

static const char *hidFieldName(HidFieldKind kind) {
  switch (kind) {
  case HID_FIELD_LX:
    return "lx";
  case HID_FIELD_LY:
    return "ly";
  case HID_FIELD_RX:
    return "rx";
  case HID_FIELD_RY:
    return "ry";
  case HID_FIELD_LT:
    return "lt";
  case HID_FIELD_RT:
    return "rt";
  case HID_FIELD_HAT:
    return "hat";
  case HID_FIELD_BUTTON:
    return "button";
  case HID_FIELD_BUTTON_ARRAY:
    return "buttonArray";
  }
  return "?";
}

static void addHidField(HidReportLayout *layout, const HidInputField &field) {
  if (layout->fieldCount >= HID_MAX_FIELDS) {
    return;
  }
  layout->fields[layout->fieldCount++] = field;
  layout->valid = true;
  layout->usesReportIds = layout->usesReportIds || field.reportId != 0;
}

static bool parseHidReportMap(const uint8_t *data, size_t length,
                              HidReportLayout *layout) {
  if (data == nullptr || layout == nullptr || length == 0) {
    return false;
  }

  *layout = {};
  uint16_t reportBitOffsets[HID_MAX_REPORT_IDS] = {};
  uint8_t reportIds[HID_MAX_REPORT_IDS] = {};
  uint8_t reportIdCount = 1;
  reportIds[0] = 0;

  uint16_t usagePage = 0;
  int32_t logicalMin = 0;
  int32_t logicalMax = 255;
  uint8_t reportSize = 0;
  uint8_t reportCount = 0;
  uint8_t reportId = 0;
  uint32_t localUsages[HID_MAX_LOCAL_USAGES] = {};
  uint8_t localUsageCount = 0;
  uint32_t usageMin = 0;
  uint32_t usageMax = 0;
  bool hasUsageRange = false;
  bool rxAssigned = false;
  bool ryAssigned = false;
  bool ltAssigned = false;
  bool rtAssigned = false;

  auto getReportIndex = [&](uint8_t id) -> uint8_t {
    for (uint8_t i = 0; i < reportIdCount; ++i) {
      if (reportIds[i] == id) {
        return i;
      }
    }
    if (reportIdCount < HID_MAX_REPORT_IDS) {
      reportIds[reportIdCount] = id;
      reportBitOffsets[reportIdCount] = 0;
      return reportIdCount++;
    }
    return 0;
  };

  auto clearLocal = [&]() {
    localUsageCount = 0;
    usageMin = 0;
    usageMax = 0;
    hasUsageRange = false;
  };

  size_t pos = 0;
  while (pos < length) {
    const uint8_t prefix = data[pos++];
    if (prefix == 0xfe) {
      if (pos + 1 >= length) {
        break;
      }
      const uint8_t longSize = data[pos];
      pos += 2 + longSize;
      continue;
    }

    const uint8_t sizeCode = prefix & 0x03;
    const uint8_t itemSize = sizeCode == 3 ? 4 : sizeCode;
    const uint8_t itemType = (prefix >> 2) & 0x03;
    const uint8_t itemTag = (prefix >> 4) & 0x0f;
    if (pos + itemSize > length) {
      break;
    }

    const bool signedItem =
        itemType == 1 && (itemTag == 1 || (itemTag == 2 && logicalMin < 0));
    const int32_t value = readHidItemValue(data + pos, itemSize, signedItem);
    const uint32_t unsignedValue =
        static_cast<uint32_t>(readHidItemValue(data + pos, itemSize, false));
    pos += itemSize;

    if (itemType == 1) {
      switch (itemTag) {
      case 0:
        usagePage = static_cast<uint16_t>(unsignedValue);
        break;
      case 1:
        logicalMin = value;
        break;
      case 2:
        logicalMax = value;
        break;
      case 7:
        reportSize = static_cast<uint8_t>(unsignedValue);
        break;
      case 8:
        reportId = static_cast<uint8_t>(unsignedValue);
        getReportIndex(reportId);
        break;
      case 9:
        reportCount = static_cast<uint8_t>(unsignedValue);
        break;
      default:
        break;
      }
    } else if (itemType == 2) {
      switch (itemTag) {
      case 0:
        if (localUsageCount < HID_MAX_LOCAL_USAGES) {
          localUsages[localUsageCount++] = unsignedValue;
        }
        break;
      case 1:
        usageMin = unsignedValue;
        hasUsageRange = true;
        break;
      case 2:
        usageMax = unsignedValue;
        hasUsageRange = true;
        break;
      default:
        break;
      }
    } else if (itemType == 0 && itemTag == 8) {
      const bool isConstant = (unsignedValue & 0x01) != 0;
      const bool isVariable = (unsignedValue & 0x02) != 0;
      const uint8_t reportIndex = getReportIndex(reportId);
      uint16_t bitOffset = reportBitOffsets[reportIndex];

      if (!isConstant) {
        for (uint8_t i = 0; i < reportCount; ++i) {
          uint32_t fullUsage = 0;
          if (i < localUsageCount) {
            fullUsage = localUsages[i];
          } else if (hasUsageRange && usageMin + i <= usageMax) {
            fullUsage = usageMin + i;
          }

          uint16_t fieldUsagePage = usagePage;
          uint16_t usage = static_cast<uint16_t>(fullUsage);
          if (fullUsage > 0xffff) {
            fieldUsagePage = static_cast<uint16_t>(fullUsage >> 16);
            usage = static_cast<uint16_t>(fullUsage & 0xffff);
          }

          HidInputField field = {};
          field.reportId = reportId;
          field.bitOffset = bitOffset + i * reportSize;
          field.bitSize = reportSize;
          field.logicalMin = logicalMin;
          field.logicalMax = logicalMax;
          field.usagePage = fieldUsagePage;
          field.usage = usage;
          field.buttonIndex = 0xff;

          if (fieldUsagePage == 0x01 && isVariable) {
            if (usage == 0x30) {
              field.kind = HID_FIELD_LX;
              addHidField(layout, field);
            } else if (usage == 0x31) {
              field.kind = HID_FIELD_LY;
              addHidField(layout, field);
            } else if (usage == 0x33) {
              field.kind = HID_FIELD_RX;
              rxAssigned = true;
              addHidField(layout, field);
            } else if (usage == 0x34) {
              field.kind = HID_FIELD_RY;
              ryAssigned = true;
              addHidField(layout, field);
            } else if ((usage == 0x32 || usage == 0x36) && !ltAssigned) {
              field.kind = HID_FIELD_LT;
              ltAssigned = true;
              addHidField(layout, field);
            } else if ((usage == 0x35 || usage == 0x37) && !rtAssigned) {
              field.kind = HID_FIELD_RT;
              rtAssigned = true;
              addHidField(layout, field);
            } else if (usage == 0x39) {
              field.kind = HID_FIELD_HAT;
              addHidField(layout, field);
            }
          } else if (fieldUsagePage == 0x02 && isVariable) {
            if ((usage == 0xc5 || usage == 0xc4) && !ltAssigned) {
              field.kind = HID_FIELD_LT;
              ltAssigned = true;
              addHidField(layout, field);
            } else if ((usage == 0xc4 || usage == 0xc5) && !rtAssigned) {
              field.kind = HID_FIELD_RT;
              rtAssigned = true;
              addHidField(layout, field);
            }
          } else if (fieldUsagePage == 0x09 && isVariable && usage > 0 &&
                     usage <= 32) {
            field.kind = HID_FIELD_BUTTON;
            field.buttonIndex = static_cast<uint8_t>(usage - 1);
            addHidField(layout, field);
          } else if (fieldUsagePage == 0x09 && !isVariable && hasUsageRange &&
                     usageMin > 0 && usageMax <= 32) {
            field.kind = HID_FIELD_BUTTON_ARRAY;
            field.usage = static_cast<uint16_t>(usageMin);
            field.buttonIndex = static_cast<uint8_t>(usageMin - 1);
            addHidField(layout, field);
          }
        }
      }

      reportBitOffsets[reportIndex] += reportSize * reportCount;
      clearLocal();
    }
  }

  return layout->valid;
}

static void logHidReportLayout(const HidReportLayout &layout) {
  if (!layout.valid) {
    logPrintln("HID report parser: no gamepad fields found.");
    return;
  }

  logPrintf("HID report parser: fields=%u reportIds=%u\r\n",
            layout.fieldCount, layout.usesReportIds ? 1 : 0);
  for (uint8_t i = 0; i < layout.fieldCount; ++i) {
    const HidInputField &field = layout.fields[i];
    if (field.kind == HID_FIELD_BUTTON) {
      logPrintf("  field %u: id=%u %s%u bit=%u size=%u logical=%ld..%ld\r\n",
                i, field.reportId, hidFieldName(field.kind),
                field.buttonIndex + 1, field.bitOffset, field.bitSize,
                static_cast<long>(field.logicalMin),
                static_cast<long>(field.logicalMax));
    } else {
      logPrintf("  field %u: id=%u %s bit=%u size=%u logical=%ld..%ld\r\n",
                i, field.reportId, hidFieldName(field.kind),
                field.bitOffset, field.bitSize,
                static_cast<long>(field.logicalMin),
                static_cast<long>(field.logicalMax));
    }
  }
}

static bool hidLayoutHasReportId(uint8_t reportId) {
  for (uint8_t i = 0; i < hidReportLayout.fieldCount; ++i) {
    if (hidReportLayout.fields[i].reportId == reportId) {
      return true;
    }
  }
  return false;
}

static bool hidLayoutSingleReportId(uint8_t *reportId) {
  bool found = false;
  uint8_t current = 0;
  for (uint8_t i = 0; i < hidReportLayout.fieldCount; ++i) {
    const uint8_t id = hidReportLayout.fields[i].reportId;
    if (!found) {
      current = id;
      found = true;
    } else if (id != current) {
      return false;
    }
  }
  if (found && reportId != nullptr) {
    *reportId = current;
  }
  return found;
}

static bool decodeGamepadFromParsedHidReport(uint8_t reportId,
                                             const uint8_t *data,
                                             size_t length) {
  if (!hidReportLayout.valid || data == nullptr || length == 0) {
    return false;
  }

  uint8_t effectiveReportId = reportId;
  uint8_t payloadOffset = 0;
  if (hidReportLayout.usesReportIds) {
    if (hidLayoutHasReportId(data[0])) {
      effectiveReportId = data[0];
      payloadOffset = 1;
    } else if (reportId != 0 && hidLayoutHasReportId(reportId)) {
      effectiveReportId = reportId;
    } else if (!hidLayoutSingleReportId(&effectiveReportId)) {
      return false;
    }
  }
  if (payloadOffset >= length) {
    return false;
  }

  bool sawInput = false;
  int16_t lx = 0;
  int16_t ly = 0;
  int16_t rx = 0;
  int16_t ry = 0;
  int16_t lt = 0;
  int16_t rt = 0;
  uint32_t buttons = 0;
  uint8_t hat = 0x0f;

  for (uint8_t i = 0; i < hidReportLayout.fieldCount; ++i) {
    const HidInputField &field = hidReportLayout.fields[i];
    if (field.reportId != effectiveReportId) {
      continue;
    }

    bool ok = false;
    uint32_t rawValue =
        readHidBits(data + payloadOffset, length - payloadOffset,
                    field.bitOffset, field.bitSize, &ok);
    if (!ok) {
      continue;
    }

    int32_t value = field.logicalMin < 0
                        ? signExtendHidValue(rawValue, field.bitSize)
                        : static_cast<int32_t>(rawValue);

    switch (field.kind) {
    case HID_FIELD_LX:
      lx = normalizeHidAxis(value, field.logicalMin, field.logicalMax);
      sawInput = true;
      break;
    case HID_FIELD_LY:
      ly = normalizeHidAxis(value, field.logicalMin, field.logicalMax);
      sawInput = true;
      break;
    case HID_FIELD_RX:
      rx = normalizeHidAxis(value, field.logicalMin, field.logicalMax);
      sawInput = true;
      break;
    case HID_FIELD_RY:
      ry = normalizeHidAxis(value, field.logicalMin, field.logicalMax);
      sawInput = true;
      break;
    case HID_FIELD_LT:
      lt = normalizeHidAxis(value, field.logicalMin, field.logicalMax);
      sawInput = true;
      break;
    case HID_FIELD_RT:
      rt = normalizeHidAxis(value, field.logicalMin, field.logicalMax);
      sawInput = true;
      break;
    case HID_FIELD_HAT:
      hat = static_cast<uint8_t>(rawValue);
      sawInput = true;
      break;
    case HID_FIELD_BUTTON:
      if (rawValue != 0 && field.buttonIndex < 32) {
        buttons |= 1UL << field.buttonIndex;
      }
      sawInput = true;
      break;
    case HID_FIELD_BUTTON_ARRAY:
      if (rawValue >= field.usage && rawValue <= field.logicalMax &&
          rawValue > 0 && rawValue <= 32) {
        buttons |= 1UL << (rawValue - 1);
      }
      sawInput = true;
      break;
    }
  }

  if (!sawInput) {
    return false;
  }

  portENTER_CRITICAL(&gamepadMux);
  gamepadState.lx = lx;
  gamepadState.ly = ly;
  gamepadState.rx = rx;
  gamepadState.ry = ry;
  gamepadState.lt = lt;
  gamepadState.rt = rt;
  gamepadState.buttons = buttons;
  gamepadState.hat = hat;
  gamepadState.reportId = effectiveReportId;
  gamepadState.rawLength = min(length, static_cast<size_t>(RAW_REPORT_LOG_BYTES));
  memcpy(gamepadState.raw, data, gamepadState.rawLength);
  gamepadState.hatIsButtonMask = false;
  gamepadState.connected = true;
  gamepadState.reportSeen = true;
  gamepadState.lastReportUs = micros();
  portEXIT_CRITICAL(&gamepadMux);
  return true;
}

static void formatRawBytes(const uint8_t *data, size_t length, char *out,
                           size_t outSize) {
  if (outSize == 0) {
    return;
  }
  out[0] = '\0';
  size_t pos = 0;
  for (size_t i = 0; i < length && pos + 3 < outSize; ++i) {
    pos += snprintf(out + pos, outSize - pos, "%02X ", data[i]);
  }
}

static void updateGamepadFromHidReport(uint8_t reportId, const uint8_t *data,
                                       size_t length) {
  if (data == nullptr || length == 0) {
    return;
  }

  if (decodeGamepadFromParsedHidReport(reportId, data, length)) {
    return;
  }

  const uint8_t axisOffset = reportAxisOffset(reportId, data, length);
  const uint8_t effectiveReportId =
      axisOffset == 1 && reportId == 0 ? data[0] : reportId;
  if (length < axisOffset + 4) {
    portENTER_CRITICAL(&gamepadMux);
    gamepadState.connected = true;
    gamepadState.reportSeen = false;
    gamepadState.reportId = effectiveReportId;
    gamepadState.rawLength = min(length, static_cast<size_t>(RAW_REPORT_LOG_BYTES));
    memcpy(gamepadState.raw, data, gamepadState.rawLength);
    portEXIT_CRITICAL(&gamepadMux);
    return;
  }

  uint8_t center[4] = {};
  bool calibrated = false;

  portENTER_CRITICAL(&gamepadMux);
  calibrated = gamepadState.axisCalibrated;
  memcpy(center, gamepadState.axisCenter, sizeof(center));
  portEXIT_CRITICAL(&gamepadMux);

  if (!calibrated) {
    for (uint8_t i = 0; i < 4; ++i) {
      center[i] = data[axisOffset + i];
    }
    calibrated = true;
    logPrintf("Remote HID axis center: %u %u %u %u\r\n", center[0],
              center[1], center[2], center[3]);
  }

  uint32_t buttons = 0;
  uint8_t hat = 0x0f;
  const uint8_t buttonOffset = axisOffset + 4;
  if (length > buttonOffset) {
    buttons = data[buttonOffset];
    hat = data[buttonOffset] & 0x0f;
  }
  if (length > buttonOffset + 1) {
    buttons |= static_cast<uint32_t>(data[buttonOffset + 1]) << 8;
  }
  if (length > buttonOffset + 2) {
    buttons |= static_cast<uint32_t>(data[buttonOffset + 2]) << 16;
  }
  if (length > buttonOffset + 3) {
    buttons |= static_cast<uint32_t>(data[buttonOffset + 3]) << 24;
  }

  portENTER_CRITICAL(&gamepadMux);
  memcpy(gamepadState.axisCenter, center, sizeof(center));
  gamepadState.axisCalibrated = calibrated;
  gamepadState.lx = decodeAxis(data[axisOffset + 0], center[0]);
  gamepadState.ly = decodeAxis(data[axisOffset + 1], center[1]);
  gamepadState.rx = decodeAxis(data[axisOffset + 2], center[2]);
  gamepadState.ry = decodeAxis(data[axisOffset + 3], center[3]);
  gamepadState.lt = 0;
  gamepadState.rt = 0;
  gamepadState.buttons = buttons;
  gamepadState.hat = hat;
  gamepadState.reportId = effectiveReportId;
  gamepadState.rawLength = min(length, static_cast<size_t>(RAW_REPORT_LOG_BYTES));
  memcpy(gamepadState.raw, data, gamepadState.rawLength);
  gamepadState.hatIsButtonMask = false;
  gamepadState.connected = true;
  gamepadState.reportSeen = true;
  gamepadState.lastReportUs = micros();
  portEXIT_CRITICAL(&gamepadMux);
}


#ifndef REMOTE_LOG_INPUT_SUMMARY
#define REMOTE_LOG_INPUT_SUMMARY 0
#endif

#ifndef USB_LOG_RAW_REPORTS
#define USB_LOG_RAW_REPORTS 0
#endif

#if REMOTE_INPUT_BACKEND == REMOTE_BACKEND_USB_HOST
static usb_host_client_handle_t usbClient = nullptr;
static usb_device_handle_t usbDevice = nullptr;
static usb_transfer_t *usbInTransfer = nullptr;
static uint8_t usbInterfaceNumber = 0;
static bool usbInterfaceClaimed = false;
static bool usbReady = false;

enum UsbInputProtocol : uint8_t {
  USB_INPUT_PROTOCOL_HID,
  USB_INPUT_PROTOCOL_XINPUT,
};

static UsbInputProtocol usbInputProtocol = USB_INPUT_PROTOCOL_HID;

struct UsbInputEndpoint {
  UsbInputProtocol protocol;
  uint8_t interfaceNumber;
  uint8_t alternateSetting;
  uint8_t endpointAddress;
  uint16_t mps;
  uint8_t score;
};

static int16_t normalizeXinputAxis(int16_t value) {
  return static_cast<int16_t>(
      constrain(static_cast<int>(value / 66), -500, 500));
}

static bool updateGamepadFromXinputReport(const uint8_t *data, size_t length) {
  if (data == nullptr || length < 14) {
    return false;
  }

  const uint8_t *report = data;
  if (data[0] == 0x00 && data[1] == 0x14 && length >= 14) {
    report = data;
  } else if (length >= 15 && data[1] == 0x00 && data[2] == 0x14) {
    report = data + 1;
  }

  if (report[0] != 0x00 || report[1] < 0x14) {
    return false;
  }

  uint32_t buttons = 0;
  if (report[3] & 0x10) {
    buttons |= 1UL << BUTTON_A_BIT;
  }
  if (report[3] & 0x20) {
    buttons |= 1UL << BUTTON_B_BIT;
  }
  if (report[3] & 0x40) {
    buttons |= 1UL << BUTTON_X_BIT;
  }
  if (report[3] & 0x80) {
    buttons |= 1UL << BUTTON_Y_BIT;
  }
  if (report[3] & 0x01) {
    buttons |= 1UL << BUTTON_LB_BIT;
  }
  if (report[3] & 0x02) {
    buttons |= 1UL << BUTTON_RB_BIT;
  }
  if (report[2] & 0x20) {
    buttons |= 1UL << BUTTON_SELECT_BIT;
  }
  if (report[2] & 0x10) {
    buttons |= 1UL << BUTTON_START_BIT;
  }

  const int16_t lx = static_cast<int16_t>(report[6] | (report[7] << 8));
  const int16_t ly = static_cast<int16_t>(report[8] | (report[9] << 8));
  const int16_t rx = static_cast<int16_t>(report[10] | (report[11] << 8));
  const int16_t ry = static_cast<int16_t>(report[12] | (report[13] << 8));

  portENTER_CRITICAL(&gamepadMux);
  gamepadState.lx = normalizeXinputAxis(lx);
  gamepadState.ly = -normalizeXinputAxis(ly);
  gamepadState.rx = normalizeXinputAxis(rx);
  gamepadState.ry = -normalizeXinputAxis(ry);
  gamepadState.lt = map(report[4], 0, 255, 0, 500);
  gamepadState.rt = map(report[5], 0, 255, 0, 500);
  gamepadState.buttons = buttons;
  gamepadState.hat = report[2] & 0x0f;
  gamepadState.reportId = 0;
  gamepadState.rawLength = min(length, static_cast<size_t>(RAW_REPORT_LOG_BYTES));
  memcpy(gamepadState.raw, data, gamepadState.rawLength);
  gamepadState.hatIsButtonMask = true;
  gamepadState.connected = true;
  gamepadState.reportSeen = true;
  gamepadState.lastReportUs = micros();
  portEXIT_CRITICAL(&gamepadMux);
  return true;
}

static void logUsbReport(const uint8_t *data, size_t length) {
#if USB_LOG_RAW_REPORTS
  static uint8_t lastRaw[RAW_REPORT_LOG_BYTES] = {};
  static uint8_t lastRawLength = 0;
  const uint8_t rawLength = min(length, static_cast<size_t>(RAW_REPORT_LOG_BYTES));
  if (rawLength == lastRawLength && memcmp(lastRaw, data, rawLength) == 0) {
    return;
  }

  char rawText[RAW_REPORT_LOG_BYTES * 3 + 1] = {};
  formatRawBytes(data, rawLength, rawText, sizeof(rawText));
  logPrintf("USB %s report: len=%u raw=%s\r\n",
            usbInputProtocol == USB_INPUT_PROTOCOL_XINPUT ? "XInput" : "HID",
            static_cast<unsigned>(length), rawText);
  memcpy(lastRaw, data, rawLength);
  lastRawLength = rawLength;
#else
  (void)data;
  (void)length;
#endif
}

static void usbTransferCallback(usb_transfer_t *transfer) {
  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED &&
      transfer->actual_num_bytes > 0) {
    logUsbReport(transfer->data_buffer, transfer->actual_num_bytes);
    if (usbInputProtocol == USB_INPUT_PROTOCOL_XINPUT) {
      updateGamepadFromXinputReport(transfer->data_buffer,
                                    transfer->actual_num_bytes);
    } else {
      updateGamepadFromHidReport(0, transfer->data_buffer,
                                 transfer->actual_num_bytes);
    }
  } else if (transfer->status != USB_TRANSFER_STATUS_COMPLETED &&
             transfer->status != USB_TRANSFER_STATUS_NO_DEVICE) {
    logPrintf("USB input transfer status=%d actual=%d\r\n",
              static_cast<int>(transfer->status),
              static_cast<int>(transfer->actual_num_bytes));
  }

  if (usbReady && transfer->status != USB_TRANSFER_STATUS_NO_DEVICE) {
    esp_err_t err = usb_host_transfer_submit(transfer);
    if (err != ESP_OK) {
      logPrintf("USB input resubmit failed: 0x%x\r\n", err);
      if (err == ESP_ERR_INVALID_STATE) {
        usbReady = false;
      }
    }
  }
}

static bool usbInterfaceLooksXinput(const usb_intf_desc_t *intf,
                                    uint16_t vid, uint16_t pid) {
  if (intf == nullptr) {
    return false;
  }

  const bool flydigiApex5 = vid == 0x37d7 && pid == 0x2501;
  const bool xbox360VendorClass =
      intf->bInterfaceClass == USB_CLASS_VENDOR_SPEC &&
      (intf->bInterfaceSubClass == 0x5d || intf->bInterfaceProtocol == 0x01);
  return xbox360VendorClass ||
         (flydigiApex5 && intf->bInterfaceClass == USB_CLASS_VENDOR_SPEC);
}

static bool findUsbInputEndpoint(const usb_config_desc_t *config,
                                 uint16_t vid, uint16_t pid,
                                 UsbInputEndpoint *selected) {
  if (config == nullptr || selected == nullptr) {
    return false;
  }

  *selected = {};
  const usb_standard_desc_t *desc =
      reinterpret_cast<const usb_standard_desc_t *>(config);
  const usb_intf_desc_t *currentInterface = nullptr;
  int offset = 0;

  while (desc != nullptr) {
    if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      currentInterface = reinterpret_cast<const usb_intf_desc_t *>(desc);
      logPrintf("USB interface: if=%u alt=%u class=0x%02x sub=0x%02x "
                "proto=0x%02x eps=%u\r\n",
                currentInterface->bInterfaceNumber,
                currentInterface->bAlternateSetting,
                currentInterface->bInterfaceClass,
                currentInterface->bInterfaceSubClass,
                currentInterface->bInterfaceProtocol,
                currentInterface->bNumEndpoints);
    } else if (desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT &&
               currentInterface != nullptr) {
      const usb_ep_desc_t *ep = reinterpret_cast<const usb_ep_desc_t *>(desc);
      const bool isIn = USB_EP_DESC_GET_EP_DIR(ep) != 0;
      const bool isInterrupt =
          USB_EP_DESC_GET_XFERTYPE(ep) == USB_TRANSFER_TYPE_INTR;
      const uint16_t endpointMps = USB_EP_DESC_GET_MPS(ep);
      logPrintf("  USB endpoint: ep=0x%02x attr=0x%02x mps=%u interval=%u\r\n",
                ep->bEndpointAddress, ep->bmAttributes, endpointMps,
                ep->bInterval);

      if (isIn && isInterrupt) {
        UsbInputEndpoint candidate = {};
        candidate.interfaceNumber = currentInterface->bInterfaceNumber;
        candidate.alternateSetting = currentInterface->bAlternateSetting;
        candidate.endpointAddress = ep->bEndpointAddress;
        candidate.mps = endpointMps;

        if (usbInterfaceLooksXinput(currentInterface, vid, pid)) {
          candidate.protocol = USB_INPUT_PROTOCOL_XINPUT;
          candidate.score = 100;
        } else if (currentInterface->bInterfaceClass == USB_CLASS_HID) {
          candidate.protocol = USB_INPUT_PROTOCOL_HID;
          candidate.score = 10;
        } else {
          candidate.score = 0;
        }

        if (candidate.score > selected->score) {
          *selected = candidate;
        }
      }
    }
    desc = usb_parse_next_descriptor(desc, config->wTotalLength, &offset);
  }
  return selected->score > 0;
}

static void closeUsbDevice() {
  usbReady = false;
  setGamepadDisconnected();
  if (usbInTransfer != nullptr) {
    usb_host_transfer_free(usbInTransfer);
    usbInTransfer = nullptr;
  }
  if (usbDevice != nullptr && usbInterfaceClaimed) {
    usb_host_interface_release(usbClient, usbDevice, usbInterfaceNumber);
    usbInterfaceClaimed = false;
  }
  if (usbDevice != nullptr) {
    usb_host_device_close(usbClient, usbDevice);
    usbDevice = nullptr;
  }
}

static void openUsbDevice(uint8_t address) {
  if (usbDevice != nullptr) {
    logPrintln("USB device ignored: one input device is already open.");
    return;
  }

  esp_err_t err = usb_host_device_open(usbClient, address, &usbDevice);
  if (err != ESP_OK) {
    logPrintf("USB device open failed: 0x%x\r\n", err);
    return;
  }

  const usb_device_desc_t *deviceDesc = nullptr;
  const usb_config_desc_t *configDesc = nullptr;
  usb_host_get_device_descriptor(usbDevice, &deviceDesc);
  usb_host_get_active_config_descriptor(usbDevice, &configDesc);
  if (deviceDesc != nullptr) {
    logPrintf("USB device: vid=0x%04x pid=0x%04x class=0x%02x\r\n",
              deviceDesc->idVendor, deviceDesc->idProduct,
              deviceDesc->bDeviceClass);
  }

  UsbInputEndpoint inputEndpoint = {};
  if (configDesc == nullptr ||
      !findUsbInputEndpoint(configDesc,
                            deviceDesc != nullptr ? deviceDesc->idVendor : 0,
                            deviceDesc != nullptr ? deviceDesc->idProduct : 0,
                            &inputEndpoint)) {
    logPrintln("USB input interrupt IN endpoint not found.");
    closeUsbDevice();
    return;
  }

  usbInterfaceNumber = inputEndpoint.interfaceNumber;
  usbInputProtocol = inputEndpoint.protocol;
  err = usb_host_interface_claim(usbClient, usbDevice, usbInterfaceNumber,
                                 inputEndpoint.alternateSetting);
  if (err != ESP_OK) {
    logPrintf("USB interface claim failed: 0x%x\r\n", err);
    closeUsbDevice();
    return;
  }
  usbInterfaceClaimed = true;

  const int bufferSize = usb_round_up_to_mps(64, inputEndpoint.mps);
  err = usb_host_transfer_alloc(bufferSize, 0, &usbInTransfer);
  if (err != ESP_OK) {
    logPrintf("USB transfer alloc failed: 0x%x\r\n", err);
    closeUsbDevice();
    return;
  }

  usbInTransfer->device_handle = usbDevice;
  usbInTransfer->bEndpointAddress = inputEndpoint.endpointAddress;
  usbInTransfer->callback = usbTransferCallback;
  usbInTransfer->context = nullptr;
  usbInTransfer->num_bytes = bufferSize;
  usbReady = true;

  logPrintf("USB %s connected: if=%u alt=%u ep=0x%02x mps=%u\r\n",
            usbInputProtocol == USB_INPUT_PROTOCOL_XINPUT ? "XInput" : "HID",
            usbInterfaceNumber, inputEndpoint.alternateSetting,
            inputEndpoint.endpointAddress, inputEndpoint.mps);
  err = usb_host_transfer_submit(usbInTransfer);
  if (err != ESP_OK) {
    logPrintf("USB transfer submit failed: 0x%x\r\n", err);
    closeUsbDevice();
  }
}

static void usbClientEvent(const usb_host_client_event_msg_t *event, void *) {
  if (event->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    logPrintf("USB device attached: address=%u\r\n", event->new_dev.address);
    openUsbDevice(event->new_dev.address);
  } else if (event->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    logPrintln("USB device removed.");
    closeUsbDevice();
  }
}

static void usbHostDaemonTask(void *) {
  while (true) {
    uint32_t eventFlags = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &eventFlags);
  }
}

static void usbHostClientTask(void *) {
  while (true) {
    if (usbClient != nullptr) {
      usb_host_client_handle_events(usbClient, pdMS_TO_TICKS(20));
    } else {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

static void initUsbRemoteReceiver() {
  usb_host_config_t hostConfig = {};
  hostConfig.skip_phy_setup = false;
  hostConfig.intr_flags = ESP_INTR_FLAG_LEVEL1;
  esp_err_t err = usb_host_install(&hostConfig);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    logPrintf("USB host install failed: 0x%x\r\n", err);
    return;
  }

  xTaskCreatePinnedToCore(usbHostDaemonTask, "usb_host_daemon", 4096, nullptr,
                          2, nullptr, 0);

  usb_host_client_config_t clientConfig = {};
  clientConfig.is_synchronous = false;
  clientConfig.max_num_event_msg = 5;
  clientConfig.async.client_event_callback = usbClientEvent;
  clientConfig.async.callback_arg = nullptr;
  err = usb_host_client_register(&clientConfig, &usbClient);
  if (err != ESP_OK) {
    logPrintf("USB host client register failed: 0x%x\r\n", err);
    return;
  }

  xTaskCreatePinnedToCore(usbHostClientTask, "usb_hid_client", 4096, nullptr, 2,
                          nullptr, 0);
  logPrintln("Remote backend: USB Host HID");
}
#endif

static int centeredRemoteValue(uint16_t value) {
  if (!channelLooksValid(value)) {
    return 0;
  }

  int centered = static_cast<int>(value) - static_cast<int>(REMOTE_CENTER_US);
  if (abs(centered) < REMOTE_DEADZONE_US) {
    centered = 0;
  }
  return constrain(centered, -500, 500);
}

static bool axisMovedEnough(int16_t current, int16_t previous) {
  return abs(static_cast<int>(current) - static_cast<int>(previous)) >=
         REMOTE_DEBUG_AXIS_DELTA;
}

static bool channelMovedEnough(uint16_t current, uint16_t previous) {
  return abs(static_cast<int>(current) - static_cast<int>(previous)) >=
         REMOTE_DEBUG_CHANNEL_DELTA_US;
}

static MotionCommand decodeMotionCommandFromValues(int walk, int strafe,
                                                   int turn) {
  if (turn != 0) {
    return turn > 0 ? MOTION_TURN_RIGHT : MOTION_TURN_LEFT;
  }

  if (walk == 0 && strafe == 0) {
    return MOTION_IDLE;
  }

  if (abs(walk) >= abs(strafe)) {
    return walk > 0 ? MOTION_WALK_FORWARD : MOTION_WALK_BACKWARD;
  }

  return strafe > 0 ? MOTION_WALK_RIGHT : MOTION_WALK_LEFT;
}

static const char *motionCommandName(MotionCommand command) {
  switch (command) {
  case MOTION_IDLE:
    return "idle";
  case MOTION_WALK_FORWARD:
    return "walk_forward";
  case MOTION_WALK_BACKWARD:
    return "walk_backward";
  case MOTION_WALK_LEFT:
    return "walk_left";
  case MOTION_WALK_RIGHT:
    return "walk_right";
  case MOTION_TURN_LEFT:
    return "turn_left";
  case MOTION_TURN_RIGHT:
    return "turn_right";
  }
  return "unknown";
}

static bool channelInZone(uint16_t value, uint16_t minUs, uint16_t maxUs) {
  return channelLooksValid(value) && value >= minUs && value <= maxUs;
}

static void appendControlName(char *out, size_t outSize, const char *name) {
  if (outSize == 0 || name == nullptr || name[0] == '\0') {
    return;
  }

  const size_t len = strlen(out);
  if (len >= outSize - 1) {
    return;
  }
  snprintf(out + len, outSize - len, "%s%s", len == 0 ? "" : ",", name);
}

static void formatUsefulControls(const RemoteSnapshot &snapshot, char *out,
                                 size_t outSize) {
  if (outSize == 0) {
    return;
  }
  out[0] = '\0';

  const int walk = centeredRemoteValue(snapshot.channels[REMOTE_WALK_CHANNEL]);
  const int strafe =
      centeredRemoteValue(snapshot.channels[REMOTE_STRAFE_CHANNEL]);
  const int turn = centeredRemoteValue(snapshot.channels[REMOTE_TURN_CHANNEL]);

  if (walk > 0) {
    appendControlName(out, outSize, "forward");
  } else if (walk < 0) {
    appendControlName(out, outSize, "backward");
  }
  if (strafe < 0) {
    appendControlName(out, outSize, "left");
  } else if (strafe > 0) {
    appendControlName(out, outSize, "right");
  }
  if (turn < 0) {
    appendControlName(out, outSize, "turn_left");
  } else if (turn > 0) {
    appendControlName(out, outSize, "turn_right");
  }

  if (channelInZone(snapshot.channels[REMOTE_SYSTEM_CHANNEL],
                    SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US)) {
    appendControlName(out, outSize, "Start_stand");
  }
  if (channelInZone(snapshot.channels[REMOTE_POSE_CHANNEL],
                    SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US)) {
    appendControlName(out, outSize, "Y_Stand");
  }
  if (channelInZone(snapshot.channels[REMOTE_SYSTEM_CHANNEL],
                    SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US)) {
    appendControlName(out, outSize, "Select_unload");
  }
  if (channelInZone(snapshot.channels[REMOTE_POSE_CHANNEL],
                    SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US)) {
    appendControlName(out, outSize, "A_Squad");
  }
  if (channelInZone(snapshot.channels[REMOTE_GETUP_CHANNEL],
                    SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US)) {
    appendControlName(out, outSize, "X");
  }
  if (channelInZone(snapshot.channels[REMOTE_GETUP_CHANNEL],
                    SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US)) {
    appendControlName(out, outSize, "B");
  }
  if (channelInZone(snapshot.channels[REMOTE_PUNCH_CHANNEL],
                    SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US)) {
    appendControlName(out, outSize, "LB_left_punch");
  }
  if (channelInZone(snapshot.channels[REMOTE_PUNCH_CHANNEL],
                    SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US)) {
    appendControlName(out, outSize, "RB");
  }
  if (channelInZone(snapshot.channels[REMOTE_HOOK_CHANNEL],
                    SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US)) {
    appendControlName(out, outSize, "LT");
  }
  if (channelInZone(snapshot.channels[REMOTE_HOOK_CHANNEL],
                    SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US)) {
    appendControlName(out, outSize, "RT");
  }

  if (out[0] == '\0') {
    strncpy(out, "-", outSize - 1);
    out[outSize - 1] = '\0';
  }
}

static void appendMappedHatButtons(const GamepadState &state, char *out,
                                   size_t outSize) {
  if (state.hatIsButtonMask) {
    if (state.hat & 0x01) {
      appendControlName(out, outSize, "DpadUp");
    }
    if (state.hat & 0x02) {
      appendControlName(out, outSize, "DpadDown");
    }
    if (state.hat & 0x04) {
      appendControlName(out, outSize, "DpadLeft");
    }
    if (state.hat & 0x08) {
      appendControlName(out, outSize, "DpadRight");
    }
    return;
  }

  switch (state.hat) {
    case 0:
      appendControlName(out, outSize, "DpadUp");
      break;
    case 1:
      appendControlName(out, outSize, "DpadUp");
      appendControlName(out, outSize, "DpadRight");
      break;
    case 2:
      appendControlName(out, outSize, "DpadRight");
      break;
    case 3:
      appendControlName(out, outSize, "DpadDown");
      appendControlName(out, outSize, "DpadRight");
      break;
    case 4:
      appendControlName(out, outSize, "DpadDown");
      break;
    case 5:
      appendControlName(out, outSize, "DpadDown");
      appendControlName(out, outSize, "DpadLeft");
      break;
    case 6:
      appendControlName(out, outSize, "DpadLeft");
      break;
    case 7:
      appendControlName(out, outSize, "DpadUp");
      appendControlName(out, outSize, "DpadLeft");
      break;
    default:
      break;
  }
}

static void formatMappedButtons(const GamepadState &state, char *out,
                                size_t outSize) {
  if (outSize == 0) {
    return;
  }
  out[0] = '\0';

  if (state.buttons & (1UL << BUTTON_A_BIT)) {
    appendControlName(out, outSize, "A");
  }
  if (state.buttons & (1UL << BUTTON_B_BIT)) {
    appendControlName(out, outSize, "B");
  }
  if (state.buttons & (1UL << BUTTON_X_BIT)) {
    appendControlName(out, outSize, "X");
  }
  if (state.buttons & (1UL << BUTTON_Y_BIT)) {
    appendControlName(out, outSize, "Y");
  }
  if (state.buttons & (1UL << BUTTON_LB_BIT)) {
    appendControlName(out, outSize, "LB");
  }
  if (state.buttons & (1UL << BUTTON_RB_BIT)) {
    appendControlName(out, outSize, "RB");
  }
  if (state.lt >= TRIGGER_ACTION_THRESHOLD) {
    appendControlName(out, outSize, "LT");
  }
  if (state.rt >= TRIGGER_ACTION_THRESHOLD) {
    appendControlName(out, outSize, "RT");
  }
  if (state.buttons & (1UL << BUTTON_SELECT_BIT)) {
    appendControlName(out, outSize, "Select");
  }
  if (state.buttons & (1UL << BUTTON_START_BIT)) {
    appendControlName(out, outSize, "Start");
  }
  appendMappedHatButtons(state, out, outSize);

  if (out[0] == '\0') {
    strncpy(out, "-", outSize - 1);
    out[outSize - 1] = '\0';
  }
}

bool isRemoteControlEnabled() {
  return ENABLE_REMOTE_CONTROL;
}

const char *remoteBackendName() {
#if REMOTE_INPUT_BACKEND == REMOTE_BACKEND_USB_HOST
  return "USB Host HID";
#else
  return "unknown";
#endif
}

void initRemoteReceiver() {
  setGamepadDisconnected();
#if REMOTE_INPUT_BACKEND == REMOTE_BACKEND_USB_HOST
  initUsbRemoteReceiver();
#else
  logPrintln("Remote backend disabled: unsupported REMOTE_INPUT_BACKEND.");
#endif
}

RemoteSnapshot readRemoteSnapshot() {
  GamepadState state = {};

  portENTER_CRITICAL(&gamepadMux);
  state = gamepadState;
  portEXIT_CRITICAL(&gamepadMux);

  RemoteSnapshot snapshot = {};
  for (uint8_t i = 0; i < REMOTE_CHANNEL_COUNT; ++i) {
    snapshot.channels[i] = REMOTE_CENTER_US;
  }

  int16_t dpadWalk = 0;
  int16_t dpadStrafe = 0;
  hatToMotionAxes(state.hat, state.hatIsButtonMask, &dpadWalk, &dpadStrafe);

  snapshot.channels[REMOTE_WALK_CHANNEL] =
      axisToPulse(dpadWalk != 0 ? dpadWalk : -state.ly);
  snapshot.channels[REMOTE_STRAFE_CHANNEL] =
      axisToPulse(dpadStrafe != 0 ? dpadStrafe : state.lx);
  snapshot.channels[REMOTE_TURN_CHANNEL] = axisToPulse(state.rx);
  snapshot.channels[REMOTE_PUNCH_CHANNEL] =
      buttonSwitchPulse(state.buttons, BUTTON_LB_BIT, BUTTON_RB_BIT);
  snapshot.channels[REMOTE_SYSTEM_CHANNEL] =
      buttonSwitchPulse(state.buttons, BUTTON_SELECT_BIT, BUTTON_START_BIT);
  snapshot.channels[REMOTE_HOOK_CHANNEL] =
      triggerSwitchPulse(state.lt, state.rt);
  snapshot.channels[REMOTE_POSE_CHANNEL] =
      buttonSwitchPulse(state.buttons, BUTTON_A_BIT, BUTTON_Y_BIT);
  snapshot.channels[REMOTE_GETUP_CHANNEL] =
      buttonSwitchPulse(state.buttons, BUTTON_X_BIT, BUTTON_B_BIT);

  snapshot.ageUs =
      state.lastReportUs == 0 ? UINT32_MAX : micros() - state.lastReportUs;
  snapshot.active = state.connected && state.reportSeen &&
                    snapshot.ageUs <= REMOTE_FAILSAFE_US &&
                    channelLooksValid(snapshot.channels[REMOTE_WALK_CHANNEL]) &&
                    channelLooksValid(snapshot.channels[REMOTE_STRAFE_CHANNEL]) &&
                    channelLooksValid(snapshot.channels[REMOTE_TURN_CHANNEL]);
  return snapshot;
}

void reportRemoteSnapshot(const RemoteSnapshot &snapshot) {
#if REMOTE_LOG_INPUT_SUMMARY
  static uint32_t lastReportMs = 0;
  static bool hasLastReport = false;
  static GamepadState lastState = {};
  static RemoteSnapshot lastSnapshot = {};

  const uint32_t now = millis();

  GamepadState state = {};
  portENTER_CRITICAL(&gamepadMux);
  state = gamepadState;
  portEXIT_CRITICAL(&gamepadMux);

  bool changed =
      !hasLastReport || snapshot.active != lastSnapshot.active ||
      state.connected != lastState.connected ||
      state.reportSeen != lastState.reportSeen ||
      state.buttons != lastState.buttons || state.hat != lastState.hat ||
      state.hatIsButtonMask != lastState.hatIsButtonMask ||
      state.reportId != lastState.reportId ||
      axisMovedEnough(state.lx, lastState.lx) ||
      axisMovedEnough(state.ly, lastState.ly) ||
      axisMovedEnough(state.rx, lastState.rx) ||
      axisMovedEnough(state.ry, lastState.ry) ||
      axisMovedEnough(state.lt, lastState.lt) ||
      axisMovedEnough(state.rt, lastState.rt);

  for (uint8_t i = 0; i < REMOTE_CHANNEL_COUNT; ++i) {
    changed = changed ||
              channelMovedEnough(snapshot.channels[i],
                                 lastSnapshot.channels[i]);
  }

  const uint32_t interval =
      changed ? REMOTE_DEBUG_MIN_INTERVAL_MS : REMOTE_DEBUG_HEARTBEAT_MS;
  if (hasLastReport && now - lastReportMs < interval) {
    return;
  }
  lastReportMs = now;
  hasLastReport = true;
  lastState = state;
  lastSnapshot = snapshot;

  char controlsText[160] = {};
  formatUsefulControls(snapshot, controlsText, sizeof(controlsText));

  if (!snapshot.active) {
    return;
  }

  if (strcmp(controlsText, "-") == 0) {
    return;
  }

  logPrintf("%s action: %s\r\n", remoteBackendName(), controlsText);
#else
  (void)snapshot;
#endif
}

bool consumeSwitchZone(uint8_t channel, uint16_t value, uint16_t minUs,
                       uint16_t maxUs, uint8_t zone) {
  if (channel >= REMOTE_CHANNEL_COUNT || zone >= 3) {
    return false;
  }

  const bool now = channelLooksValid(value) && value >= minUs && value <= maxUs;
  const bool triggered = now && !remoteZoneLatch[channel][zone];
  remoteZoneLatch[channel][zone] = now;
  return triggered;
}

MotionCommand decodeMotionCommand(const RemoteSnapshot &snapshot) {
  const int walk = centeredRemoteValue(snapshot.channels[REMOTE_WALK_CHANNEL]);
  const int strafe =
      centeredRemoteValue(snapshot.channels[REMOTE_STRAFE_CHANNEL]);
  const int turn = centeredRemoteValue(snapshot.channels[REMOTE_TURN_CHANNEL]);
  return decodeMotionCommandFromValues(walk, strafe, turn);
}
