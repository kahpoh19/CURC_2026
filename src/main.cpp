#include <Arduino.h>

#include "Logger.h"
#include "Motions.h"
#include "RemoteControl.h"
#include "ServoSystem.h"

static bool remoteHadSignal = false;

static void runServoSweepTest() {
  moveAllAndReport(-90.0f);
  delay(1000);

  moveAllAndReport(90.0f);
  delay(1000);
}

static void handleRemoteLoss() {
  if (remoteHadSignal) {
    logPrintln("Remote signal lost. Waiting for Start.");
    remoteHadSignal = false;
    resetMotionState();
  }
  delay(REMOTE_CONTROL_PERIOD_MS);
}

static void handleRemoteAcquire() {
  if (remoteHadSignal) {
    return;
  }

  logPrintln("Remote signal acquired.");
  remoteHadSignal = true;
  resetMotionState();
}

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
  waitForSerialMonitor(3000);
  delay(200);

  logPrintln();
  logPrintf("Remote backend selected: %s\r\n", remoteBackendName());

  if (isRemoteControlEnabled()) {
    initRemoteReceiver();
  }

  setupServoSystem();

  // ── boot-time servo self-test: move all to 0° ─────────────────
  logPrintln("=== Boot servo test: moving all servos to 0 degrees ===");
  moveAllAndReport(0.0f);
  logPrintln("=== Boot servo test done. Waiting for remote... ===");

  if (isRemoteControlEnabled()) {
    logPrintln(
        "Remote map: Y/Start Stand, Select unload, A Squad, "
        "LB/RB/LT/RT/X/B JSON motions, D-pad/left stick "
        "Forward/Backward/Moveleft/MoveRight, right stick RotateLeft/"
        "RotateRight.");
  }
}

void loop() {
  if (!serviceServoDetection()) {
    return;
  }

  if (!isRemoteControlEnabled()) {
    runServoSweepTest();
    return;
  }

  const RemoteSnapshot snapshot = readRemoteSnapshot();
  reportRemoteSnapshot(snapshot);

  // ── debug: print channel values once per second ──────────────
  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs >= 2000) {
    logPrintf("[SNAP] active=%d ch=[%u %u %u %u %u %u %u %u %u %u]\r\n",
              snapshot.active,
              snapshot.channels[0], snapshot.channels[1], snapshot.channels[2],
              snapshot.channels[3], snapshot.channels[4], snapshot.channels[5],
              snapshot.channels[6], snapshot.channels[7], snapshot.channels[8],
              snapshot.channels[9]);
    lastDebugMs = millis();
  }

  if (!snapshot.active) {
    handleRemoteLoss();
    return;
  }

  handleRemoteAcquire();

  if (handleRemoteActions(snapshot)) {
    delay(REMOTE_CONTROL_PERIOD_MS);
    return;
  }

  handleMotionCommand(decodeMotionCommand(snapshot));
  delay(REMOTE_CONTROL_PERIOD_MS);
}
