#include "Motions.h"

#include "Logger.h"
#include "ServoSystem.h"

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

static constexpr uint16_t STAND_INTERVAL_MS = 350;
static constexpr uint16_t WALK_INTERVAL_MS = 360;
static constexpr uint16_t PUNCH_INTERVAL_MS = 220;
static constexpr uint32_t WALK_STEP_INTERVAL_MS = 620;

static constexpr uint8_t BUS_ANY = SERVO_BUS_ALL;

// Real humanoid servo ID map.
static constexpr uint8_t SERVO_LEFT_ARM_BOTTOM = 0;
static constexpr uint8_t SERVO_LEFT_ARM_MIDDLE = 1;
static constexpr uint8_t SERVO_LEFT_ARM_TOP = 2;
static constexpr uint8_t SERVO_LEFT_SHOULDER = 3;
static constexpr uint8_t SERVO_RIGHT_SHOULDER = 4;
static constexpr uint8_t SERVO_RIGHT_ARM_BOTTOM = 5;
static constexpr uint8_t SERVO_RIGHT_ARM_MIDDLE = 6;
static constexpr uint8_t SERVO_RIGHT_ARM_TOP = 7;
static constexpr uint8_t SERVO_LEFT_LEG_CONNECTOR = 8;
static constexpr uint8_t SERVO_RIGHT_LEG_CONNECTOR = 9;
static constexpr uint8_t SERVO_LEFT_LEG_TOP = 10;
static constexpr uint8_t SERVO_LEFT_LEG_MIDDLE = 11;
static constexpr uint8_t SERVO_LEFT_LEG_BOTTOM = 12;
static constexpr uint8_t SERVO_RIGHT_LEG_TOP = 13;
static constexpr uint8_t SERVO_RIGHT_LEG_MIDDLE = 14;
static constexpr uint8_t SERVO_RIGHT_LEG_BOTTOM = 15;
static constexpr uint8_t SERVO_WAIST_CENTER = 16;

static bool standPoseApplied = false;
static uint32_t lastWalkStepMs = 0;
static uint8_t walkPhase = 0;

static constexpr JointTarget STAND_POSE[] = {
    {BUS_ANY, SERVO_LEFT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_LEFT_ARM_MIDDLE, 12.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, 0.0f},
    {BUS_ANY, SERVO_LEFT_SHOULDER, 8.0f},
    {BUS_ANY, SERVO_RIGHT_SHOULDER, -8.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_MIDDLE, -12.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, 0.0f},
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, 0.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, 0.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, 0.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, 0.0f},
    {BUS_ANY, SERVO_LEFT_LEG_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, 0.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, 0.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_WAIST_CENTER, 0.0f},
};

static constexpr JointTarget GUARD_POSE[] = {
    {BUS_ANY, SERVO_LEFT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_LEFT_ARM_MIDDLE, 42.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, 28.0f},
    {BUS_ANY, SERVO_LEFT_SHOULDER, 10.0f},
    {BUS_ANY, SERVO_RIGHT_SHOULDER, -10.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_MIDDLE, -42.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, 28.0f},
};

static constexpr JointTarget LEFT_PUNCH_EXTEND[] = {
    {BUS_ANY, SERVO_LEFT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_LEFT_ARM_MIDDLE, 0.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, 64.0f},
    {BUS_ANY, SERVO_LEFT_SHOULDER, 4.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_MIDDLE, -48.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, 18.0f},
};

static constexpr JointTarget RIGHT_PUNCH_EXTEND[] = {
    {BUS_ANY, SERVO_RIGHT_SHOULDER, -4.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_MIDDLE, 0.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, 64.0f},
    {BUS_ANY, SERVO_LEFT_ARM_MIDDLE, 48.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, 18.0f},
};

static constexpr JointTarget WALK_FORWARD_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, 5.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, 5.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, 14.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, -18.0f},
    {BUS_ANY, SERVO_LEFT_LEG_BOTTOM, 8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, -12.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, 8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_BOTTOM, -6.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, -16.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, 16.0f},
};

static constexpr JointTarget WALK_FORWARD_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, -5.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, -5.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, -12.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, 8.0f},
    {BUS_ANY, SERVO_LEFT_LEG_BOTTOM, -6.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, 14.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, -18.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_BOTTOM, 8.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, 16.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, -16.0f},
};

static constexpr JointTarget WALK_BACKWARD_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, 5.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, 5.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, -14.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, 10.0f},
    {BUS_ANY, SERVO_LEFT_LEG_BOTTOM, -8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, 12.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, -14.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_BOTTOM, 6.0f},
};

static constexpr JointTarget WALK_BACKWARD_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, -5.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, -5.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, 12.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, -14.0f},
    {BUS_ANY, SERVO_LEFT_LEG_BOTTOM, 6.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, -14.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, 10.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_BOTTOM, -8.0f},
};

static constexpr JointTarget TURN_LEFT_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, -9.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, 9.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, -8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, 8.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, 8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, -12.0f},
};

static constexpr JointTarget TURN_LEFT_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, 9.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, -9.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, 8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, -8.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, -12.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, 8.0f},
};

static constexpr JointTarget TURN_RIGHT_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, 9.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, -9.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, 8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, -8.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, -12.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, 8.0f},
};

static constexpr JointTarget TURN_RIGHT_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_LEG_CONNECTOR, -9.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_CONNECTOR, 9.0f},
    {BUS_ANY, SERVO_LEFT_LEG_TOP, -8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_TOP, 8.0f},
    {BUS_ANY, SERVO_LEFT_LEG_MIDDLE, 8.0f},
    {BUS_ANY, SERVO_RIGHT_LEG_MIDDLE, -12.0f},
};

static void playLeftPunch() {
  logPrintln("Remote action: left punch");
  applyGuardPose();
  applyTargets(LEFT_PUNCH_EXTEND, ARRAY_COUNT(LEFT_PUNCH_EXTEND),
               PUNCH_INTERVAL_MS);
  delay(80);
  applyGuardPose();
  applyStandPose();
}

static void playRightPunch() {
  logPrintln("Remote action: right punch");
  applyGuardPose();
  applyTargets(RIGHT_PUNCH_EXTEND, ARRAY_COUNT(RIGHT_PUNCH_EXTEND),
               PUNCH_INTERVAL_MS);
  delay(80);
  applyGuardPose();
  applyStandPose();
}

static void applyWalkPhase(const JointTarget *phaseA, uint8_t phaseACount,
                           const JointTarget *phaseB, uint8_t phaseBCount) {
  if (walkPhase == 0) {
    applyTargets(phaseA, phaseACount, WALK_INTERVAL_MS);
  } else {
    applyTargets(phaseB, phaseBCount, WALK_INTERVAL_MS);
  }
  walkPhase ^= 1;
  standPoseApplied = false;
}

void applyStandPose(uint16_t intervalMs) {
  applyTargets(STAND_POSE, ARRAY_COUNT(STAND_POSE), intervalMs);
  standPoseApplied = true;
}

void applyGuardPose() {
  applyTargets(GUARD_POSE, ARRAY_COUNT(GUARD_POSE), STAND_INTERVAL_MS);
  standPoseApplied = false;
}

void resetMotionState() {
  walkPhase = 0;
  lastWalkStepMs = 0;
  standPoseApplied = false;
}

bool handleRemoteActions(const RemoteSnapshot &snapshot) {
  bool actionRan = false;

  if (consumeSwitchZone(REMOTE_PUNCH_CHANNEL,
                        snapshot.channels[REMOTE_PUNCH_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playLeftPunch();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_PUNCH_CHANNEL,
                        snapshot.channels[REMOTE_PUNCH_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    playRightPunch();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_MODE_CHANNEL,
                        snapshot.channels[REMOTE_MODE_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    logPrintln("Remote action: stand");
    applyStandPose();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_MODE_CHANNEL,
                        snapshot.channels[REMOTE_MODE_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    logPrintln("Remote action: guard");
    applyGuardPose();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_MODE_CHANNEL,
                        snapshot.channels[REMOTE_MODE_CHANNEL],
                        SWITCH_CENTER_MIN_US, SWITCH_CENTER_MAX_US, 2)) {
    logPrintln("Remote action: relax to stand");
    applyStandPose();
    actionRan = true;
  }

  if (actionRan) {
    lastWalkStepMs = millis();
  }
  return actionRan;
}

void handleMotionCommand(MotionCommand command) {
  if (command == MOTION_IDLE) {
    if (!standPoseApplied) {
      applyStandPose();
    }
    return;
  }

  const uint32_t now = millis();
  if (now - lastWalkStepMs < WALK_STEP_INTERVAL_MS) {
    return;
  }
  lastWalkStepMs = now;

  switch (command) {
    case MOTION_WALK_FORWARD:
      applyWalkPhase(WALK_FORWARD_PHASE_A, ARRAY_COUNT(WALK_FORWARD_PHASE_A),
                     WALK_FORWARD_PHASE_B, ARRAY_COUNT(WALK_FORWARD_PHASE_B));
      break;
    case MOTION_WALK_BACKWARD:
      applyWalkPhase(WALK_BACKWARD_PHASE_A, ARRAY_COUNT(WALK_BACKWARD_PHASE_A),
                     WALK_BACKWARD_PHASE_B, ARRAY_COUNT(WALK_BACKWARD_PHASE_B));
      break;
    case MOTION_TURN_LEFT:
      applyWalkPhase(TURN_LEFT_PHASE_A, ARRAY_COUNT(TURN_LEFT_PHASE_A),
                     TURN_LEFT_PHASE_B, ARRAY_COUNT(TURN_LEFT_PHASE_B));
      break;
    case MOTION_TURN_RIGHT:
      applyWalkPhase(TURN_RIGHT_PHASE_A, ARRAY_COUNT(TURN_RIGHT_PHASE_A),
                     TURN_RIGHT_PHASE_B, ARRAY_COUNT(TURN_RIGHT_PHASE_B));
      break;
    case MOTION_IDLE:
    default:
      break;
  }
}
