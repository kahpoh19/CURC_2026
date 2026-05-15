#include "Motions.h"

#include "Logger.h"
#include "ServoSystem.h"

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

static constexpr uint16_t STAND_INTERVAL_MS = 350;
static constexpr uint16_t WALK_INTERVAL_MS = 320;
static constexpr uint16_t PUNCH_INTERVAL_MS = 220;
static constexpr uint32_t WALK_STEP_INTERVAL_MS = WALK_INTERVAL_MS;

// If the robot loads the wrong foot before swinging, flip this between 1.0 and
// -1.0 before changing the actual gait angles below.
static constexpr float WALK_BALANCE_SIGN = -1.0f;

static constexpr float HIP_OPEN_NEUTRAL_DEG = 0.0f;
static constexpr float WALK_SWING_KNEE_LIFT_DEG = -34.0f;
static constexpr float WALK_SWING_ANKLE_FORWARD_DEG = 30.0f;
static constexpr float WALK_SUPPORT_KNEE_BEND_DEG = 0.0f;
static constexpr float WALK_SUPPORT_ANKLE_PUSH_DEG = 0.0f;
static constexpr float WALK_FOOT_ROLL_SHIFT_DEG = 15.0f;
static constexpr float WALK_WAIST_SWING_DEG = 4.0f;
static constexpr float WALK_ARM_SWING_DEG = 12.0f;
static constexpr float SIDE_STEP_HIP_OPEN_DEG = 10.0f;
static constexpr float SIDE_STEP_FOOT_ROLL_DEG = 12.0f;
static constexpr float SIDE_STEP_WAIST_DEG = 5.0f;

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
static constexpr uint8_t SERVO_LEFT_HIP_OPEN = 8;
static constexpr uint8_t SERVO_RIGHT_HIP_OPEN = 9;
static constexpr uint8_t SERVO_LEFT_KNEE = 10;
static constexpr uint8_t SERVO_LEFT_ANKLE_PITCH = 11;
static constexpr uint8_t SERVO_LEFT_FOOT_ROLL = 12;
static constexpr uint8_t SERVO_RIGHT_KNEE = 13;
static constexpr uint8_t SERVO_RIGHT_ANKLE_PITCH = 14;
static constexpr uint8_t SERVO_RIGHT_FOOT_ROLL = 15;
static constexpr uint8_t SERVO_WAIST_CENTER = 16;

static bool standPoseApplied = false;
static bool motionArmed = false;
static uint32_t lastWalkStepMs = 0;
static uint8_t walkPhase = 0;
static MotionCommand lastMotionCommand = MOTION_IDLE;

static constexpr JointTarget STAND_POSE[] = {
    {BUS_ANY, SERVO_LEFT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_LEFT_ARM_MIDDLE, 12.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, 0.0f},
    {BUS_ANY, SERVO_LEFT_SHOULDER, 8.0f},
    {BUS_ANY, SERVO_RIGHT_SHOULDER, -8.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_MIDDLE, -12.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, 0.0f},
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, 0.0f},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, 0.0f},
    {BUS_ANY, SERVO_LEFT_KNEE, 0.0f},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, 0.0f},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL, 0.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, 0.0f},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, 0.0f},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL, 0.0f},
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

static constexpr JointTarget LEFT_HOOK_EXTEND[] = {
    {BUS_ANY, SERVO_WAIST_CENTER, 22.0f},
    {BUS_ANY, SERVO_LEFT_SHOULDER, 26.0f},
    {BUS_ANY, SERVO_LEFT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_LEFT_ARM_MIDDLE, 18.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, 56.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_MIDDLE, -42.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, 24.0f},
};

static constexpr JointTarget RIGHT_HOOK_EXTEND[] = {
    {BUS_ANY, SERVO_WAIST_CENTER, -22.0f},
    {BUS_ANY, SERVO_RIGHT_SHOULDER, -26.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_BOTTOM, 0.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_MIDDLE, -18.0f},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, 56.0f},
    {BUS_ANY, SERVO_LEFT_ARM_MIDDLE, 42.0f},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, 24.0f},
};

static constexpr JointTarget WALK_FORWARD_SHIFT_RIGHT[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, 0.0f},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, 0.0f},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL,
     -WALK_BALANCE_SIGN * WALK_FOOT_ROLL_SHIFT_DEG},
    {BUS_ANY, SERVO_RIGHT_KNEE, WALK_SUPPORT_KNEE_BEND_DEG},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, WALK_SUPPORT_ANKLE_PUSH_DEG},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL,
     WALK_BALANCE_SIGN * WALK_FOOT_ROLL_SHIFT_DEG},
    {BUS_ANY, SERVO_WAIST_CENTER, -WALK_WAIST_SWING_DEG},
};

static constexpr JointTarget WALK_FORWARD_LEFT_SWING[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, WALK_SWING_KNEE_LIFT_DEG},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, WALK_SWING_ANKLE_FORWARD_DEG},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL,
     -WALK_BALANCE_SIGN * WALK_FOOT_ROLL_SHIFT_DEG},
    {BUS_ANY, SERVO_RIGHT_KNEE, WALK_SUPPORT_KNEE_BEND_DEG},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, WALK_SUPPORT_ANKLE_PUSH_DEG},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL,
     WALK_BALANCE_SIGN * WALK_FOOT_ROLL_SHIFT_DEG},
    {BUS_ANY, SERVO_WAIST_CENTER, -WALK_WAIST_SWING_DEG},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, -WALK_ARM_SWING_DEG},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, WALK_ARM_SWING_DEG},
};

static constexpr JointTarget WALK_FORWARD_SHIFT_LEFT[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, WALK_SUPPORT_KNEE_BEND_DEG},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, WALK_SUPPORT_ANKLE_PUSH_DEG},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL,
     -WALK_BALANCE_SIGN * WALK_FOOT_ROLL_SHIFT_DEG},
    {BUS_ANY, SERVO_RIGHT_KNEE, 0.0f},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, 0.0f},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL,
     WALK_BALANCE_SIGN * WALK_FOOT_ROLL_SHIFT_DEG},
    {BUS_ANY, SERVO_WAIST_CENTER, WALK_WAIST_SWING_DEG},
};

static constexpr JointTarget WALK_FORWARD_RIGHT_SWING[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, WALK_SUPPORT_KNEE_BEND_DEG},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, WALK_SUPPORT_ANKLE_PUSH_DEG},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL,
     -WALK_BALANCE_SIGN * WALK_FOOT_ROLL_SHIFT_DEG},
    {BUS_ANY, SERVO_RIGHT_KNEE, WALK_SWING_KNEE_LIFT_DEG},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, WALK_SWING_ANKLE_FORWARD_DEG},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL,
     WALK_BALANCE_SIGN * WALK_FOOT_ROLL_SHIFT_DEG},
    {BUS_ANY, SERVO_WAIST_CENTER, WALK_WAIST_SWING_DEG},
    {BUS_ANY, SERVO_LEFT_ARM_TOP, WALK_ARM_SWING_DEG},
    {BUS_ANY, SERVO_RIGHT_ARM_TOP, -WALK_ARM_SWING_DEG},
};

static constexpr JointTarget WALK_BACKWARD_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, -20.0f},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, 16.0f},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL, -10.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, 18.0f},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, -21.0f},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL, 8.0f},
};

static constexpr JointTarget WALK_BACKWARD_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, 18.0f},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, -21.0f},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL, 8.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, -20.0f},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, 16.0f},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL, -10.0f},
};

static constexpr JointTarget TURN_LEFT_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, -8.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, 8.0f},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, 8.0f},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, -12.0f},
};

static constexpr JointTarget TURN_LEFT_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, 8.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, -8.0f},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, -12.0f},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, 8.0f},
};

static constexpr JointTarget TURN_RIGHT_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, 8.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, -8.0f},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, -12.0f},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, 8.0f},
};

static constexpr JointTarget TURN_RIGHT_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, HIP_OPEN_NEUTRAL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, -8.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, 8.0f},
    {BUS_ANY, SERVO_LEFT_ANKLE_PITCH, 8.0f},
    {BUS_ANY, SERVO_RIGHT_ANKLE_PITCH, -12.0f},
};

static constexpr JointTarget WALK_LEFT_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, -SIDE_STEP_HIP_OPEN_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, -SIDE_STEP_HIP_OPEN_DEG * 0.5f},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL, -SIDE_STEP_FOOT_ROLL_DEG},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL, SIDE_STEP_FOOT_ROLL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, -10.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, 8.0f},
    {BUS_ANY, SERVO_WAIST_CENTER, -SIDE_STEP_WAIST_DEG},
};

static constexpr JointTarget WALK_LEFT_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, -SIDE_STEP_HIP_OPEN_DEG * 0.5f},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, -SIDE_STEP_HIP_OPEN_DEG},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL, SIDE_STEP_FOOT_ROLL_DEG},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL, -SIDE_STEP_FOOT_ROLL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, 8.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, -10.0f},
    {BUS_ANY, SERVO_WAIST_CENTER, SIDE_STEP_WAIST_DEG},
};

static constexpr JointTarget WALK_RIGHT_PHASE_A[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, SIDE_STEP_HIP_OPEN_DEG * 0.5f},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, SIDE_STEP_HIP_OPEN_DEG},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL, -SIDE_STEP_FOOT_ROLL_DEG},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL, SIDE_STEP_FOOT_ROLL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, -10.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, 8.0f},
    {BUS_ANY, SERVO_WAIST_CENTER, SIDE_STEP_WAIST_DEG},
};

static constexpr JointTarget WALK_RIGHT_PHASE_B[] = {
    {BUS_ANY, SERVO_LEFT_HIP_OPEN, SIDE_STEP_HIP_OPEN_DEG},
    {BUS_ANY, SERVO_RIGHT_HIP_OPEN, SIDE_STEP_HIP_OPEN_DEG * 0.5f},
    {BUS_ANY, SERVO_LEFT_FOOT_ROLL, SIDE_STEP_FOOT_ROLL_DEG},
    {BUS_ANY, SERVO_RIGHT_FOOT_ROLL, -SIDE_STEP_FOOT_ROLL_DEG},
    {BUS_ANY, SERVO_LEFT_KNEE, 8.0f},
    {BUS_ANY, SERVO_RIGHT_KNEE, -10.0f},
    {BUS_ANY, SERVO_WAIST_CENTER, -SIDE_STEP_WAIST_DEG},
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

static void playLeftHookPunch() {
  logPrintln("Remote action: left hook punch");
  applyGuardPose();
  applyTargets(LEFT_HOOK_EXTEND, ARRAY_COUNT(LEFT_HOOK_EXTEND),
               PUNCH_INTERVAL_MS);
  delay(100);
  applyGuardPose();
  applyStandPose();
}

static void playRightHookPunch() {
  logPrintln("Remote action: right hook punch");
  applyGuardPose();
  applyTargets(RIGHT_HOOK_EXTEND, ARRAY_COUNT(RIGHT_HOOK_EXTEND),
               PUNCH_INTERVAL_MS);
  delay(100);
  applyGuardPose();
  applyStandPose();
}

static void applyWalkPhase(const JointTarget *phaseA, uint8_t phaseACount,
                           const JointTarget *phaseB, uint8_t phaseBCount) {
  if ((walkPhase & 1) == 0) {
    applyTargets(phaseA, phaseACount, WALK_INTERVAL_MS);
  } else {
    applyTargets(phaseB, phaseBCount, WALK_INTERVAL_MS);
  }
  ++walkPhase;
  standPoseApplied = false;
}

static void applyForwardWalkPhase() {
  switch (walkPhase % 4) {
    case 0:
      applyTargets(WALK_FORWARD_SHIFT_RIGHT,
                   ARRAY_COUNT(WALK_FORWARD_SHIFT_RIGHT), WALK_INTERVAL_MS);
      break;
    case 1:
      applyTargets(WALK_FORWARD_LEFT_SWING,
                   ARRAY_COUNT(WALK_FORWARD_LEFT_SWING), WALK_INTERVAL_MS);
      break;
    case 2:
      applyTargets(WALK_FORWARD_SHIFT_LEFT,
                   ARRAY_COUNT(WALK_FORWARD_SHIFT_LEFT), WALK_INTERVAL_MS);
      break;
    case 3:
    default:
      applyTargets(WALK_FORWARD_RIGHT_SWING,
                   ARRAY_COUNT(WALK_FORWARD_RIGHT_SWING), WALK_INTERVAL_MS);
      break;
  }
  ++walkPhase;
  standPoseApplied = false;
}

static const char *motionCommandActionName(MotionCommand command) {
  switch (command) {
    case MOTION_WALK_FORWARD:
      return "walk forward";
    case MOTION_WALK_BACKWARD:
      return "walk backward";
    case MOTION_WALK_LEFT:
      return "walk left";
    case MOTION_WALK_RIGHT:
      return "walk right";
    case MOTION_TURN_LEFT:
      return "turn left";
    case MOTION_TURN_RIGHT:
      return "turn right";
    case MOTION_IDLE:
    default:
      return "stand";
  }
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
  lastMotionCommand = MOTION_IDLE;
  standPoseApplied = false;
  motionArmed = false;
}

bool handleRemoteActions(const RemoteSnapshot &snapshot) {
  bool actionRan = false;

  if (consumeSwitchZone(REMOTE_SYSTEM_CHANNEL,
                        snapshot.channels[REMOTE_SYSTEM_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    logPrintln("Remote action: start stand");
    motionArmed = true;
    applyStandPose();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_SYSTEM_CHANNEL,
                        snapshot.channels[REMOTE_SYSTEM_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    logPrintln("Remote action: select unload servos");
    motionArmed = false;
    standPoseApplied = false;
    unloadAllServos();
    actionRan = true;
  }

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

  if (consumeSwitchZone(REMOTE_HOOK_CHANNEL,
                        snapshot.channels[REMOTE_HOOK_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playLeftHookPunch();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_HOOK_CHANNEL,
                        snapshot.channels[REMOTE_HOOK_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    playRightHookPunch();
    actionRan = true;
  }

  if (actionRan) {
    lastWalkStepMs = millis();
  }
  return actionRan;
}

void handleMotionCommand(MotionCommand command) {
  if (command != lastMotionCommand) {
    walkPhase = 0;
    lastMotionCommand = command;
  }

  if (command == MOTION_IDLE) {
    if (motionArmed && !standPoseApplied) {
      logPrintln("Remote action: stand");
      applyStandPose();
    }
    return;
  }

  const uint32_t now = millis();
  if (now - lastWalkStepMs < WALK_STEP_INTERVAL_MS) {
    return;
  }
  lastWalkStepMs = now;
  logPrintf("Remote action: %s\r\n", motionCommandActionName(command));

  switch (command) {
    case MOTION_WALK_FORWARD:
      applyForwardWalkPhase();
      break;
    case MOTION_WALK_BACKWARD:
      applyWalkPhase(WALK_BACKWARD_PHASE_A, ARRAY_COUNT(WALK_BACKWARD_PHASE_A),
                     WALK_BACKWARD_PHASE_B, ARRAY_COUNT(WALK_BACKWARD_PHASE_B));
      break;
    case MOTION_WALK_LEFT:
      applyWalkPhase(WALK_LEFT_PHASE_A, ARRAY_COUNT(WALK_LEFT_PHASE_A),
                     WALK_LEFT_PHASE_B, ARRAY_COUNT(WALK_LEFT_PHASE_B));
      break;
    case MOTION_WALK_RIGHT:
      applyWalkPhase(WALK_RIGHT_PHASE_A, ARRAY_COUNT(WALK_RIGHT_PHASE_A),
                     WALK_RIGHT_PHASE_B, ARRAY_COUNT(WALK_RIGHT_PHASE_B));
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
