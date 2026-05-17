#include "ServoSystem.h"

#include "FashionStar_UartServo.h"
#include "FashionStar_UartServoProtocol.h"
#include "Logger.h"

// Your wiring:
// Servo driver TX -> ESP32-S3 RX.
// Servo driver RX -> ESP32-S3 TX.
// GND must be common.
static constexpr int SERVO_A_TX_PIN = 17;
static constexpr int SERVO_A_RX_PIN = 18;
// Servo B is temporarily disabled.
// static constexpr int SERVO_B_TX_PIN = 35;
// static constexpr int SERVO_B_RX_PIN = 38;
static constexpr uint32_t SERVO_BAUDRATE = 115200;

// Set to false to skip servo scanning/initialization and command directly.
static constexpr bool ENABLE_SERVO_DETECTION = false;

static constexpr uint8_t CONFIGURED_SERVO_START_ID = 0;
static constexpr uint8_t CONFIGURED_SERVO_END_ID = 16;
static constexpr uint8_t CONFIGURED_SERVO_COUNT =
    CONFIGURED_SERVO_END_ID - CONFIGURED_SERVO_START_ID + 1;

// ID setting mode is intentionally disabled by default.
// To change one servo from ID 0 to ID 1:
// 1. Disconnect all other servos from the bus.
// 2. Set SET_ID_MODE to true.
// 3. Set OLD_SERVO_ID/NEW_SERVO_ID.
// 4. Upload once, then set SET_ID_MODE back to false and upload again.
static constexpr bool SET_ID_MODE = false;
static constexpr uint8_t OLD_SERVO_ID = 0;
static constexpr uint8_t NEW_SERVO_ID = 16;
static constexpr uint8_t SCAN_START_ID = 0;
static constexpr uint8_t SCAN_END_ID = 254;

static constexpr bool LOG_SERVO_COMMANDS = false;
// Mode 1 sends angle + interval + power only. Mode 2 also sends t_acc/t_dec,
// which changes the timing profile relative to the JSON motion data.
static constexpr uint8_t SYNC_MODE_RAW_ANGLE_BY_INTERVAL = 1;
static constexpr uint16_t MIN_RAMP_TIME_MS = 20;

struct ServoBus {
  const char *name;
  HardwareSerial *serial;
  FSUS_Protocol *protocol;
  FSUS_Servo *servos;
  bool *online;
  int rxPin;
  int txPin;
};

HardwareSerial ServoSerialA(1);
FSUS_Protocol protocolA;
FSUS_Servo servosA[CONFIGURED_SERVO_COUNT];

bool servoOnlineA[CONFIGURED_SERVO_COUNT] = {};

ServoBus servoBuses[] = {
    {"A", &ServoSerialA, &protocolA, servosA, servoOnlineA, SERVO_A_RX_PIN,
     SERVO_A_TX_PIN},
    // {"B", &ServoSerialB, &protocolB, servosB, servoOnlineB,
    //  SERVO_B_RX_PIN, SERVO_B_TX_PIN},
};
static constexpr uint8_t SERVO_BUS_COUNT =
    sizeof(servoBuses) / sizeof(servoBuses[0]);

static const char *statusName(FSUS_STATUS status) {
  switch (status) {
    case FSUS_STATUS_SUCCESS:
      return "SUCCESS";
    case FSUS_STATUS_FAIL:
      return "FAIL";
    case FSUS_STATUS_WRONG_RESPONSE_HEADER:
      return "WRONG_RESPONSE_HEADER";
    case FSUS_STATUS_UNKOWN_CMD_ID:
      return "UNKNOWN_CMD_ID";
    case FSUS_STATUS_SIZE_TOO_BIG:
      return "SIZE_TOO_BIG";
    case FSUS_STATUS_CHECKSUM_ERROR:
      return "CHECKSUM_ERROR";
    case FSUS_STATUS_ID_NOT_MATCH:
      return "ID_NOT_MATCH";
    case FSUS_STATUS_TIMEOUT:
      return "TIMEOUT";
    default:
      return "UNKNOWN_STATUS";
  }
}

static bool pingServo(ServoBus &bus, uint8_t id, bool printOffline = true) {
  FSUS_Servo probe(id, bus.protocol);
  const bool online = probe.ping();
  if (online || printOffline) {
    logPrintf("bus %s servo #%u is %s, status=%s\r\n", bus.name, id,
              online ? "online" : "offline",
              statusName(bus.protocol->responsePack.recv_status));
  }
  return online;
}

static bool setServoId(ServoBus &bus, uint8_t oldId, uint8_t newId) {
  if (newId > 254) {
    logPrintln("Invalid new ID. Valid range is 0..254.");
    return false;
  }

  uint8_t content[] = {newId};
  bus.protocol->emptyCache();
  bus.protocol->sendWriteData(oldId, FSUS_PARAM_SERVO_ID, sizeof(content),
                              content);

  FSUS_SERVO_ID_T replyServoId = 0;
  uint8_t replyAddress = 0;
  bool result = false;
  const FSUS_STATUS status =
      bus.protocol->recvWriteData(&replyServoId, &replyAddress, &result);

  logPrintf(
      "bus %s set ID %u -> %u: status=%s, replyServoId=%u, address=%u, "
      "result=%s\r\n",
      bus.name, oldId, newId, statusName(status), replyServoId, replyAddress,
      result ? "true" : "false");

  return status == FSUS_STATUS_SUCCESS && replyAddress == FSUS_PARAM_SERVO_ID &&
         result;
}

static void runIdSettingMode(ServoBus &bus) {
  logPrintln("ID setting mode");
  logPrintln("Only one servo should be connected to the bus.");
  logPrintf("Bus %s changing servo ID from %u to %u\r\n", bus.name,
            OLD_SERVO_ID, NEW_SERVO_ID);

  if (!pingServo(bus, OLD_SERVO_ID)) {
    logPrintln("Old ID did not respond. Check wiring or OLD_SERVO_ID.");
    return;
  }

  if (!setServoId(bus, OLD_SERVO_ID, NEW_SERVO_ID)) {
    logPrintln("ID write failed.");
    return;
  }

  delay(500);
  logPrintln("Verifying new ID...");
  pingServo(bus, NEW_SERVO_ID);
}

static void scanServos(ServoBus &bus) {
  logPrintf("Bus %s scanning servo IDs %u..%u\r\n", bus.name, SCAN_START_ID,
            SCAN_END_ID);
  bool found = false;
  for (uint16_t id = SCAN_START_ID; id <= SCAN_END_ID; ++id) {
    if (pingServo(bus, id, false)) {
      found = true;
    }
    delay(50);
  }

  if (!found) {
    logPrintf("No servo found on bus %s in %u..%u.\r\n", bus.name,
              SCAN_START_ID, SCAN_END_ID);
  }
}

static bool initServo(ServoBus &bus, FSUS_Servo &servo) {
  logPrintf("Testing bus %s configured servo id=%u\r\n", bus.name,
            servo.servoId);
  if (!servo.ping()) {
    logPrintf("bus %s servo #%u is offline, status=%s\r\n", bus.name,
              servo.servoId,
              statusName(bus.protocol->responsePack.recv_status));
    return false;
  }

  logPrintf("bus %s servo #%u is online.\r\n", bus.name, servo.servoId);
  servo.init();
  return true;
}

static void initConfiguredServoObjects(ServoBus &bus) {
  for (uint8_t i = 0; i < CONFIGURED_SERVO_COUNT; ++i) {
    bus.servos[i].servoId = CONFIGURED_SERVO_START_ID + i;
    bus.servos[i].protocol = bus.protocol;
    bus.servos[i].isOnline = false;
    bus.servos[i].isMTurn = false;
    bus.servos[i].angleMin = FSUS_SERVO_ANGLE_MIN;
    bus.servos[i].angleMax = FSUS_SERVO_ANGLE_MAX;
    bus.servos[i].speed = FSUS_SERVO_SPEED;
    bus.servos[i].kAngleReal2Raw = FSUS_K_ANGLE_REAL2RAW;
    bus.servos[i].bAngleReal2Raw = FSUS_B_ANGLE_REAL2RAW;
    bus.online[i] = false;
  }
}

static void refreshConfiguredServos(ServoBus &bus) {
  for (uint8_t i = 0; i < CONFIGURED_SERVO_COUNT; ++i) {
    bus.online[i] = initServo(bus, bus.servos[i]);
  }
}

static bool anyConfiguredServoOnline() {
  for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
    for (uint8_t i = 0; i < CONFIGURED_SERVO_COUNT; ++i) {
      if (servoBuses[busIndex].online[i]) {
        return true;
      }
    }
  }
  return false;
}

static bool shouldCommandServo(bool online) {
  return !ENABLE_SERVO_DETECTION || online;
}

static uint16_t rampTimeForInterval(uint16_t intervalMs) {
  (void)intervalMs;
  return MIN_RAMP_TIME_MS;
}

static void setServoAngle(ServoBus &bus, FSUS_Servo &servo, bool online,
                          float angle, uint16_t intervalMs = 1000) {
  if (!shouldCommandServo(online)) {
    return;
  }

  if (LOG_SERVO_COMMANDS) {
    logPrintf("Set bus %s servo #%u raw angle = %.1f deg\r\n", bus.name,
              servo.servoId, angle);
  }

  const uint16_t rampMs = rampTimeForInterval(intervalMs);
  servo.setRawAngleByInterval(angle, intervalMs, rampMs, rampMs, 0);
}

static bool addSyncTarget(ServoBus &bus, uint8_t servoIndex, float angle,
                          uint16_t intervalMs,
                          FSUS_sync_servo *syncTargets,
                          uint8_t &syncCount) {
  if (!shouldCommandServo(bus.online[servoIndex])) {
    return false;
  }

  if (syncCount >= CONFIGURED_SERVO_COUNT) {
    logPrintf("Too many sync targets on bus %s\r\n", bus.name);
    return false;
  }

  const uint16_t rampMs = rampTimeForInterval(intervalMs);
  FSUS_sync_servo &syncTarget = syncTargets[syncCount++];
  syncTarget.servoId = bus.servos[servoIndex].servoId;
  syncTarget.angle = angle;
  syncTarget.velocity = 0;
  syncTarget.interval = intervalMs;
  syncTarget.interval_multiturn = 0;
  syncTarget.t_acc = rampMs;
  syncTarget.t_dec = rampMs;
  syncTarget.power = 0;
  return true;
}

static void sendSyncTargets(ServoBus &bus, FSUS_sync_servo *syncTargets,
                            uint8_t syncCount, uint16_t intervalMs) {
  if (syncCount == 0) {
    return;
  }

  if (LOG_SERVO_COMMANDS) {
    logPrintf("Sync bus %s targets=%u interval=%u ms\r\n", bus.name,
              syncCount, intervalMs);
  }

  bus.protocol->sendSyncCommand(syncCount, SYNC_MODE_RAW_ANGLE_BY_INTERVAL,
                                syncTargets);
}

static void reportServoAngle(ServoBus &bus, FSUS_Servo &servo, bool online) {
  if (!ENABLE_SERVO_DETECTION || !online) {
    return;
  }

  const float currentAngle = servo.queryRawAngle();
  logPrintf("bus %s servo #%u current raw angle = %.1f deg, status=%s\r\n",
            bus.name, servo.servoId, currentAngle,
            statusName(bus.protocol->responsePack.recv_status));
}

static bool configuredServoIndex(uint8_t servoId, uint8_t &index) {
  if (servoId < CONFIGURED_SERVO_START_ID ||
      servoId > CONFIGURED_SERVO_END_ID) {
    return false;
  }

  index = servoId - CONFIGURED_SERVO_START_ID;
  return true;
}

bool isServoDetectionEnabled() {
  return ENABLE_SERVO_DETECTION;
}

void setupServoSystem() {
  logPrintln("FashionStar UART humanoid remote control on ESP32-S3");
  logPrintf("Servo UART A: baud=%lu RX=%d TX=%d\r\n", SERVO_BAUDRATE,
            SERVO_A_RX_PIN, SERVO_A_TX_PIN);
  logPrintln("Servo UART B: disabled");

  for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
    ServoBus &bus = servoBuses[busIndex];
    bus.serial->begin(SERVO_BAUDRATE, SERIAL_8N1, bus.rxPin, bus.txPin);
    bus.protocol->baudrate = SERVO_BAUDRATE;
    bus.protocol->serial = bus.serial;
  }
  delay(100);

  for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
    initConfiguredServoObjects(servoBuses[busIndex]);
  }

  if (SET_ID_MODE) {
    runIdSettingMode(servoBuses[0]);
    while (true) {
      delay(1000);
    }
  }

  if (ENABLE_SERVO_DETECTION) {
    for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
      scanServos(servoBuses[busIndex]);
      refreshConfiguredServos(servoBuses[busIndex]);
    }
  } else {
    logPrintln("Servo detection disabled. Commands will be sent directly.");
  }
}

bool serviceServoDetection() {
  static uint32_t lastRetryMs = 0;

  if (!ENABLE_SERVO_DETECTION || anyConfiguredServoOnline()) {
    return true;
  }

  if (millis() - lastRetryMs >= 3000) {
    lastRetryMs = millis();
    logPrintln();
    logPrintln("No configured servos online. Rescanning...");
    for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
      scanServos(servoBuses[busIndex]);
      refreshConfiguredServos(servoBuses[busIndex]);
    }
  }
  delay(100);
  return false;
}

void moveAllAndReport(float angle) {
  for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
    ServoBus &bus = servoBuses[busIndex];
    for (uint8_t i = 0; i < CONFIGURED_SERVO_COUNT; ++i) {
      setServoAngle(bus, bus.servos[i], bus.online[i], angle);
    }
  }
  delay(1200);

  for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
    ServoBus &bus = servoBuses[busIndex];
    for (uint8_t i = 0; i < CONFIGURED_SERVO_COUNT; ++i) {
      reportServoAngle(bus, bus.servos[i], bus.online[i]);
    }
  }
}

void unloadAllServos() {
  logPrintln("Servo unload: all configured servos");
  for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
    ServoBus &bus = servoBuses[busIndex];
    for (uint8_t i = 0; i < CONFIGURED_SERVO_COUNT; ++i) {
      if (!shouldCommandServo(bus.online[i])) {
        continue;
      }
      bus.servos[i].StopOnControlUnloading();
    }
  }
}

void applyTargets(const JointTarget *targets, uint8_t count,
                  uint16_t intervalMs) {
  FSUS_sync_servo syncTargets[SERVO_BUS_COUNT][CONFIGURED_SERVO_COUNT] = {};
  uint8_t syncCounts[SERVO_BUS_COUNT] = {};

  for (uint8_t i = 0; i < count; ++i) {
    const JointTarget &target = targets[i];
    if (target.busIndex != SERVO_BUS_ALL && target.busIndex >= SERVO_BUS_COUNT) {
      logPrintf("Invalid target bus index %u\r\n", target.busIndex);
      continue;
    }

    uint8_t servoIndex = 0;
    if (!configuredServoIndex(target.servoId, servoIndex)) {
      logPrintf("Invalid target servo id %u\r\n", target.servoId);
      continue;
    }

    if (target.busIndex == SERVO_BUS_ALL) {
      for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
        ServoBus &bus = servoBuses[busIndex];
        addSyncTarget(bus, servoIndex, target.rawAngle, intervalMs,
                      syncTargets[busIndex], syncCounts[busIndex]);
      }
    } else {
      ServoBus &bus = servoBuses[target.busIndex];
      addSyncTarget(bus, servoIndex, target.rawAngle, intervalMs,
                    syncTargets[target.busIndex], syncCounts[target.busIndex]);
    }
  }

  for (uint8_t busIndex = 0; busIndex < SERVO_BUS_COUNT; ++busIndex) {
    sendSyncTargets(servoBuses[busIndex], syncTargets[busIndex],
                    syncCounts[busIndex], intervalMs);
  }

  delay(intervalMs);
}
