#include <Arduino.h>

#include "FashionStar_UartServo.h"
#include "FashionStar_UartServoProtocol.h"

// Your wiring:
// Controller TX -> ESP32-S3 GPIO17, so GPIO17 is ESP RX.
// Controller RX -> ESP32-S3 GPIO16, so GPIO16 is ESP TX.
// GND must be common.
static constexpr int SERVO_RX_PIN = 17;
static constexpr int SERVO_TX_PIN = 16;
static constexpr uint32_t SERVO_BAUDRATE = 115200;

// Change these if your two servos use different IDs.
static constexpr uint8_t SERVO0_ID = 0;
static constexpr uint8_t SERVO1_ID = 1;
static constexpr uint8_t SERVO2_ID = 2;

// ID setting mode is intentionally disabled by default.
// To change one servo from ID 0 to ID 1:
// 1. Disconnect all other servos from the bus.
// 2. Set SET_ID_MODE to true.
// 3. Set OLD_SERVO_ID/NEW_SERVO_ID.
// 4. Upload once, then set SET_ID_MODE back to false and upload again.
static constexpr bool SET_ID_MODE = false;
static constexpr uint8_t OLD_SERVO_ID = 0;
static constexpr uint8_t NEW_SERVO_ID = 2;

HardwareSerial ServoSerial(1);
FSUS_Protocol protocol;
FSUS_Servo servo0(SERVO0_ID, &protocol);
FSUS_Servo servo1(SERVO1_ID, &protocol);
FSUS_Servo servo2(SERVO2_ID, &protocol);

bool servo0Online = false;
bool servo1Online = false;
bool servo2Online = false;

static void waitForSerialMonitor(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!Serial && millis() - start < timeoutMs) {
    delay(10);
  }
}

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

static bool pingServo(uint8_t id) {
  FSUS_Servo probe(id, &protocol);
  const bool online = probe.ping();
  Serial.printf("servo #%u is %s, status=%s\r\n", id,
                online ? "online" : "offline",
                statusName(protocol.responsePack.recv_status));
  return online;
}

static bool setServoId(uint8_t oldId, uint8_t newId) {
  if (newId > 254) {
    Serial.println("Invalid new ID. Valid range is 0..254.");
    return false;
  }

  uint8_t content[] = {newId};
  protocol.emptyCache();
  protocol.sendWriteData(oldId, FSUS_PARAM_SERVO_ID, sizeof(content), content);

  FSUS_SERVO_ID_T replyServoId = 0;
  uint8_t replyAddress = 0;
  bool result = false;
  const FSUS_STATUS status =
      protocol.recvWriteData(&replyServoId, &replyAddress, &result);

  Serial.printf(
      "Set ID %u -> %u: status=%s, replyServoId=%u, address=%u, result=%s\r\n",
      oldId, newId, statusName(status), replyServoId, replyAddress,
      result ? "true" : "false");

  return status == FSUS_STATUS_SUCCESS && replyAddress == FSUS_PARAM_SERVO_ID &&
         result;
}

static void runIdSettingMode() {
  Serial.println("ID setting mode");
  Serial.println("Only one servo should be connected to the bus.");
  Serial.printf("Changing servo ID from %u to %u\r\n", OLD_SERVO_ID,
                NEW_SERVO_ID);

  if (!pingServo(OLD_SERVO_ID)) {
    Serial.println("Old ID did not respond. Check wiring or OLD_SERVO_ID.");
    return;
  }

  if (!setServoId(OLD_SERVO_ID, NEW_SERVO_ID)) {
    Serial.println("ID write failed.");
    return;
  }

  delay(500);
  Serial.println("Verifying new ID...");
  pingServo(NEW_SERVO_ID);
}

static void scanServos() {
  Serial.println("Scanning servo IDs 0..9");
  bool found = false;
  for (uint8_t id = 0; id <= 9; ++id) {
    if (pingServo(id)) {
      found = true;
    }
    delay(50);
  }

  if (!found) {
    Serial.println("No servo found in 0..9.");
  }
}

static bool initServo(FSUS_Servo &servo) {
  Serial.printf("Testing configured servo id=%u\r\n", servo.servoId);
  if (!servo.ping()) {
    Serial.printf("servo #%u is offline, status=%s\r\n", servo.servoId,
                  statusName(protocol.responsePack.recv_status));
    return false;
  }

  Serial.printf("servo #%u is online.\r\n", servo.servoId);
  servo.init();
  return true;
}

static void refreshConfiguredServos() {
  servo0Online = initServo(servo0);
  servo1Online = initServo(servo1);
  servo2Online = initServo(servo2);
}

static bool anyConfiguredServoOnline() {
  return servo0Online || servo1Online || servo2Online;
}

static void setServoAngle(FSUS_Servo &servo, bool online, float angle) {
  if (!online) {
    return;
  }

  Serial.printf("Set servo #%u raw angle = %.1f deg\r\n", servo.servoId, angle);
  servo.setRawAngleByInterval(angle, 1000, 100, 100, 0);
}

static void reportServoAngle(FSUS_Servo &servo, bool online) {
  if (!online) {
    return;
  }

  const float currentAngle = servo.queryRawAngle();
  Serial.printf("servo #%u current raw angle = %.1f deg, status=%s\r\n",
                servo.servoId, currentAngle,
                statusName(protocol.responsePack.recv_status));
}

static void moveAllAndReport(float servo0Angle, float servo1Angle,
                             float servo2Angle) {
  setServoAngle(servo0, servo0Online, servo0Angle);
  setServoAngle(servo1, servo1Online, servo1Angle);
  setServoAngle(servo2, servo2Online, servo2Angle);
  delay(1200);

  reportServoAngle(servo0, servo0Online);
  reportServoAngle(servo1, servo1Online);
  reportServoAngle(servo2, servo2Online);
}

void setup() {
  Serial.begin(115200);
  waitForSerialMonitor(3000);
  delay(200);

  Serial.println();
  Serial.println("FashionStar UART servo test on ESP32-S3");
  Serial.printf("Servo UART: baud=%lu RX=%d TX=%d\r\n", SERVO_BAUDRATE,
                SERVO_RX_PIN, SERVO_TX_PIN);

  ServoSerial.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  protocol.baudrate = SERVO_BAUDRATE;
  protocol.serial = &ServoSerial;
  delay(100);

  if (SET_ID_MODE) {
    runIdSettingMode();
    while (true) {
      delay(1000);
    }
  }

  scanServos();
  refreshConfiguredServos();
}

void loop() {
  static uint32_t lastRetryMs = 0;

  if (!anyConfiguredServoOnline()) {
    if (millis() - lastRetryMs >= 3000) {
      lastRetryMs = millis();
      Serial.println();
      Serial.println("No configured servos online. Rescanning...");
      scanServos();
      refreshConfiguredServos();
    }
    delay(100);
    return;
  }

  moveAllAndReport(45.0f, 90.0f, 180.0f);
  delay(1000);

  moveAllAndReport(-45.0f, -90.0f, -180.0f);
  delay(1000);
}
