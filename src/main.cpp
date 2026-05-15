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

  if (isRemoteControlEnabled()) {
    logPrintln(
        "Remote map: Start stand, Select unload, D-pad/left stick four-way "
        "walk, right stick horizontal turn, LB/RB punch, LT/RT hook.");
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
