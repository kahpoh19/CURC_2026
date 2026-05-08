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
    logPrintln("PPM remote signal lost. Returning to stand pose.");
    applyStandPose();
    remoteHadSignal = false;
    resetMotionState();
  }
  delay(REMOTE_CONTROL_PERIOD_MS);
}

static void handleRemoteAcquire() {
  if (remoteHadSignal) {
    return;
  }

  logPrintln("PPM remote signal acquired.");
  remoteHadSignal = true;
  resetMotionState();
  applyStandPose();
}

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
  waitForSerialMonitor(3000);
  delay(200);

  logPrintln();
  setupServoSystem();

  if (isRemoteControlEnabled()) {
    initRemoteReceiver();
    applyStandPose();
    logPrintln(
        "Remote map: ch2 walk, ch4 turn, ch5 low/high left/right punch, "
        "ch6 low/center/high stand/stand/guard.");
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
