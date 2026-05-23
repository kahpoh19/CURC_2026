#include "Motions.h"

#include "Logger.h"
#include "ServoSystem.h"

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))

static constexpr uint8_t BUS_ANY = SERVO_BUS_ALL;
static constexpr uint8_t MAX_FRAME_TARGETS = 17;
static constexpr uint16_t TURN_REPEAT_PAUSE_MS = 200;

struct ServoAngle {
  uint8_t servoId;
  float rawAngle;
};

struct MotionFrame {
  const ServoAngle *targets;
  uint8_t targetCount;
  uint16_t moveMs;
  uint16_t delayMs;
};

static bool idlePoseApplied = false;
static bool motionArmed = false;
static bool returnToStandPending = false;
static bool danceActive = false;

// Tune motions by editing {servo id, angle}. moveMs is JSON time; delayMs is JSON delay.

// ═══ AUTO-GENERATED POSES BEGIN (tools/json_to_motion.py --apply) ═══

// 默认
static constexpr ServoAngle DEFAULT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, 0.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 0.0f}, {9, 0.0f},
    {10, 0.0f}, {11, 0.0f}, {12, 0.0f}, {13, 0.0f}, {14, 0.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr MotionFrame DEFAULT_MOTION[] = {
    {DEFAULT_FRAME_0, ARRAY_COUNT(DEFAULT_FRAME_0), 300, 500},
};


// stand 2.0
static constexpr ServoAngle STAND_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame STAND_MOTION[] = {
    {STAND_FRAME_0, ARRAY_COUNT(STAND_FRAME_0), 700, 500},
};


// squad 2.0
static constexpr ServoAngle SQUAD_FRAME_0[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame SQUAD_MOTION[] = {
    {SQUAD_FRAME_0, ARRAY_COUNT(SQUAD_FRAME_0), 700, 500},
};


// Defence
static constexpr ServoAngle DEFENCE_FRAME_0[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, -90.0f},
};

static constexpr MotionFrame DEFENCE_MOTION[] = {
    {DEFENCE_FRAME_0, ARRAY_COUNT(DEFENCE_FRAME_0), 300, 0},
};


// Final Kick
static constexpr ServoAngle FINAL_KICK_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_1[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, -60.0f}, {4, 60.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 50.0f}, {9, 5.0f},
    {10, 40.0f}, {11, -40.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, 30.0f}, {16, -100.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_2[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, -80.0f}, {4, 80.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 50.0f}, {9, -30.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, 30.0f}, {16, -100.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_3[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, -80.0f}, {4, 80.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 50.0f}, {9, -40.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, 30.0f}, {16, -100.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_4[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 50.0f}, {4, -50.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 0.0f}, {9, -50.0f},
    {10, 90.0f}, {11, -80.0f}, {12, -25.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -15.0f}, {16, -80.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_5[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 60.0f}, {4, -60.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 30.0f}, {9, -50.0f},
    {10, 90.0f}, {11, -80.0f}, {12, -25.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -15.0f}, {16, -80.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_6[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 60.0f}, {4, -60.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 20.0f}, {9, -50.0f},
    {10, 90.0f}, {11, -80.0f}, {12, -25.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -15.0f}, {16, -80.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_7[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 40.0f}, {4, -40.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, -80.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_8[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 40.0f}, {4, -40.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_9[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame FINAL_KICK_MOTION[] = {
    {FINAL_KICK_FRAME_0, ARRAY_COUNT(FINAL_KICK_FRAME_0), 800, 0},
    {FINAL_KICK_FRAME_1, ARRAY_COUNT(FINAL_KICK_FRAME_1), 500, 100},
    {FINAL_KICK_FRAME_2, ARRAY_COUNT(FINAL_KICK_FRAME_2), 200, 300},
    {FINAL_KICK_FRAME_3, ARRAY_COUNT(FINAL_KICK_FRAME_3), 50, 500},
    {FINAL_KICK_FRAME_4, ARRAY_COUNT(FINAL_KICK_FRAME_4), 500, 100},
    {FINAL_KICK_FRAME_5, ARRAY_COUNT(FINAL_KICK_FRAME_5), 200, 300},
    {FINAL_KICK_FRAME_6, ARRAY_COUNT(FINAL_KICK_FRAME_6), 50, 500},
    {FINAL_KICK_FRAME_7, ARRAY_COUNT(FINAL_KICK_FRAME_7), 300, 100},
    {FINAL_KICK_FRAME_8, ARRAY_COUNT(FINAL_KICK_FRAME_8), 300, 100},
    {FINAL_KICK_FRAME_9, ARRAY_COUNT(FINAL_KICK_FRAME_9), 700, 500},
};


// Punch Left
static constexpr ServoAngle PUNCH_LEFT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 80.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -35.0f}, {12, 10.0f}, {13, -35.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 90.0f},
};

static constexpr ServoAngle PUNCH_LEFT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -35.0f}, {12, 10.0f}, {13, -35.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 90.0f},
};

static constexpr ServoAngle PUNCH_LEFT_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 90.0f}, {6, 0.0f}, {7, 30.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -35.0f}, {12, 10.0f}, {13, -35.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 90.0f},
};

static constexpr ServoAngle PUNCH_LEFT_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame PUNCH_LEFT_MOTION[] = {
    {PUNCH_LEFT_FRAME_0, ARRAY_COUNT(PUNCH_LEFT_FRAME_0), 500, 200},
    {PUNCH_LEFT_FRAME_1, ARRAY_COUNT(PUNCH_LEFT_FRAME_1), 50, 300},
    {PUNCH_LEFT_FRAME_2, ARRAY_COUNT(PUNCH_LEFT_FRAME_2), 100, 0},
    {PUNCH_LEFT_FRAME_3, ARRAY_COUNT(PUNCH_LEFT_FRAME_3), 700, 100},
};


// Dance
static constexpr ServoAngle DANCE_FRAME_0[] = {
    {0, -90.0f}, {1, -30.0f}, {2, -70.0f}, {3, -60.0f}, {4, 30.0f},
    {5, 70.0f}, {6, 120.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 20.0f}, {11, -20.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle DANCE_FRAME_1[] = {
    {0, -70.0f}, {1, -120.0f}, {2, -70.0f}, {3, -30.0f}, {4, 60.0f},
    {5, 90.0f}, {6, 30.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -20.0f}, {14, 20.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame DANCE_MOTION[] = {
    {DANCE_FRAME_0, ARRAY_COUNT(DANCE_FRAME_0), 276, 200},
    {DANCE_FRAME_1, ARRAY_COUNT(DANCE_FRAME_1), 276, 200},
};


// standup back
static constexpr ServoAngle STANDUP_BACK_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, 0.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_BACK_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, 0.0f}, {3, 180.0f}, {4, -180.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_BACK_FRAME_2[] = {
    {0, -30.0f}, {1, -90.0f}, {2, -80.0f}, {3, 180.0f}, {4, -180.0f},
    {5, 30.0f}, {6, 90.0f}, {7, 80.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_BACK_FRAME_3[] = {
    {0, -100.0f}, {1, -90.0f}, {2, -80.0f}, {3, 180.0f}, {4, -180.0f},
    {5, 100.0f}, {6, 90.0f}, {7, 80.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_BACK_FRAME_4[] = {
    {0, -30.0f}, {1, -90.0f}, {2, -80.0f}, {3, 55.0f}, {4, -55.0f},
    {5, 30.0f}, {6, 90.0f}, {7, 80.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_BACK_FRAME_5[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame STANDUP_BACK_MOTION[] = {
    {STANDUP_BACK_FRAME_0, ARRAY_COUNT(STANDUP_BACK_FRAME_0), 500, 200},
    {STANDUP_BACK_FRAME_1, ARRAY_COUNT(STANDUP_BACK_FRAME_1), 500, 200},
    {STANDUP_BACK_FRAME_2, ARRAY_COUNT(STANDUP_BACK_FRAME_2), 500, 500},
    {STANDUP_BACK_FRAME_3, ARRAY_COUNT(STANDUP_BACK_FRAME_3), 700, 100},
    {STANDUP_BACK_FRAME_4, ARRAY_COUNT(STANDUP_BACK_FRAME_4), 1200, 2000},
    {STANDUP_BACK_FRAME_5, ARRAY_COUNT(STANDUP_BACK_FRAME_5), 1000, 500},
};


// standup front
static constexpr ServoAngle STANDUP_FRONT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, 0.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_FRONT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, 0.0f}, {3, -180.0f}, {4, 180.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_FRONT_FRAME_2[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -70.0f}, {3, -180.0f}, {4, 180.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -90.0f}, {12, 10.0f}, {13, -90.0f}, {14, 90.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_FRONT_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, -20.0f}, {4, 20.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -90.0f}, {12, 10.0f}, {13, -90.0f}, {14, 90.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_FRONT_FRAME_4[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame STANDUP_FRONT_MOTION[] = {
    {STANDUP_FRONT_FRAME_0, ARRAY_COUNT(STANDUP_FRONT_FRAME_0), 500, 200},
    {STANDUP_FRONT_FRAME_1, ARRAY_COUNT(STANDUP_FRONT_FRAME_1), 500, 200},
    {STANDUP_FRONT_FRAME_2, ARRAY_COUNT(STANDUP_FRONT_FRAME_2), 500, 200},
    {STANDUP_FRONT_FRAME_3, ARRAY_COUNT(STANDUP_FRONT_FRAME_3), 1000, 1000},
    {STANDUP_FRONT_FRAME_4, ARRAY_COUNT(STANDUP_FRONT_FRAME_4), 1000, 100},
};


// Elbow Left
static constexpr ServoAngle ELBOW_LEFT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 70.0f},
    {5, 60.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle ELBOW_LEFT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 70.0f},
    {5, 110.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle ELBOW_LEFT_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 70.0f},
    {5, 90.0f}, {6, 0.0f}, {7, 0.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 120.0f},
};

static constexpr ServoAngle ELBOW_LEFT_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 130.0f},
    {5, 100.0f}, {6, 0.0f}, {7, 90.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 120.0f},
};

static constexpr ServoAngle ELBOW_LEFT_FRAME_4[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 130.0f},
    {5, 100.0f}, {6, 0.0f}, {7, 90.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle ELBOW_LEFT_FRAME_5[] = {
    {0, 10.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, -10.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame ELBOW_LEFT_MOTION[] = {
    {ELBOW_LEFT_FRAME_0, ARRAY_COUNT(ELBOW_LEFT_FRAME_0), 300, 0},
    {ELBOW_LEFT_FRAME_1, ARRAY_COUNT(ELBOW_LEFT_FRAME_1), 100, 0},
    {ELBOW_LEFT_FRAME_2, ARRAY_COUNT(ELBOW_LEFT_FRAME_2), 200, 500},
    {ELBOW_LEFT_FRAME_3, ARRAY_COUNT(ELBOW_LEFT_FRAME_3), 600, 100},
    {ELBOW_LEFT_FRAME_4, ARRAY_COUNT(ELBOW_LEFT_FRAME_4), 500, 100},
    {ELBOW_LEFT_FRAME_5, ARRAY_COUNT(ELBOW_LEFT_FRAME_5), 500, 0},
};


// Punch Right
static constexpr ServoAngle PUNCH_RIGHT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -80.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 35.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 35.0f},
    {15, -10.0f}, {16, -90.0f},
};

static constexpr ServoAngle PUNCH_RIGHT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, 0.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 35.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 35.0f},
    {15, -10.0f}, {16, -90.0f},
};

static constexpr ServoAngle PUNCH_RIGHT_FRAME_2[] = {
    {0, -90.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 35.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 35.0f},
    {15, -10.0f}, {16, -90.0f},
};

static constexpr ServoAngle PUNCH_RIGHT_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame PUNCH_RIGHT_MOTION[] = {
    {PUNCH_RIGHT_FRAME_0, ARRAY_COUNT(PUNCH_RIGHT_FRAME_0), 500, 200},
    {PUNCH_RIGHT_FRAME_1, ARRAY_COUNT(PUNCH_RIGHT_FRAME_1), 50, 300},
    {PUNCH_RIGHT_FRAME_2, ARRAY_COUNT(PUNCH_RIGHT_FRAME_2), 100, 0},
    {PUNCH_RIGHT_FRAME_3, ARRAY_COUNT(PUNCH_RIGHT_FRAME_3), 700, 100},
};


// Elbow Right
static constexpr ServoAngle ELBOW_RIGHT_FRAME_0[] = {
    {0, -60.0f}, {1, 0.0f}, {2, -70.0f}, {3, -70.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle ELBOW_RIGHT_FRAME_1[] = {
    {0, -110.0f}, {1, 0.0f}, {2, -70.0f}, {3, -70.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle ELBOW_RIGHT_FRAME_2[] = {
    {0, -90.0f}, {1, 0.0f}, {2, 0.0f}, {3, -70.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, -120.0f},
};

static constexpr ServoAngle ELBOW_RIGHT_FRAME_3[] = {
    {0, -100.0f}, {1, 0.0f}, {2, -90.0f}, {3, -130.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, -120.0f},
};

static constexpr ServoAngle ELBOW_RIGHT_FRAME_4[] = {
    {0, -100.0f}, {1, 0.0f}, {2, -90.0f}, {3, -130.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle ELBOW_RIGHT_FRAME_5[] = {
    {0, 10.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, -10.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame ELBOW_RIGHT_MOTION[] = {
    {ELBOW_RIGHT_FRAME_0, ARRAY_COUNT(ELBOW_RIGHT_FRAME_0), 300, 0},
    {ELBOW_RIGHT_FRAME_1, ARRAY_COUNT(ELBOW_RIGHT_FRAME_1), 100, 0},
    {ELBOW_RIGHT_FRAME_2, ARRAY_COUNT(ELBOW_RIGHT_FRAME_2), 200, 500},
    {ELBOW_RIGHT_FRAME_3, ARRAY_COUNT(ELBOW_RIGHT_FRAME_3), 600, 100},
    {ELBOW_RIGHT_FRAME_4, ARRAY_COUNT(ELBOW_RIGHT_FRAME_4), 500, 100},
    {ELBOW_RIGHT_FRAME_5, ARRAY_COUNT(ELBOW_RIGHT_FRAME_5), 500, 0},
};

// ═══ AUTO-GENERATED POSES END ═══

// ═══ AUTO-GENERATED GAIT MOTIONS BEGIN (tools/json_to_motion.py --apply) ═══

// Forward
static constexpr ServoAngle FORWARD_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, -40.0f}, {4, -20.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 25.0f}, {11, -41.0f}, {12, 8.0f}, {13, -50.0f}, {14, 25.0f},
    {15, -8.0f}, {16, -10.0f},
};

static constexpr ServoAngle FORWARD_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, -20.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 37.0f}, {11, -27.0f}, {12, 8.0f}, {13, -37.0f}, {14, 27.0f},
    {15, -8.0f}, {16, 0.0f},
};

static constexpr ServoAngle FORWARD_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 20.0f}, {4, 40.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 50.0f}, {11, -25.0f}, {12, 8.0f}, {13, -25.0f}, {14, 41.0f},
    {15, -8.0f}, {16, 10.0f},
};

static constexpr ServoAngle FORWARD_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 20.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 37.0f}, {11, -27.0f}, {12, 8.0f}, {13, -37.0f}, {14, 27.0f},
    {15, -8.0f}, {16, 0.0f},
};

static constexpr MotionFrame FORWARD_MOTION[] = {
    {FORWARD_FRAME_0, ARRAY_COUNT(FORWARD_FRAME_0), 200, 0},
    {FORWARD_FRAME_1, ARRAY_COUNT(FORWARD_FRAME_1), 200, 0},
    {FORWARD_FRAME_2, ARRAY_COUNT(FORWARD_FRAME_2), 200, 0},
    {FORWARD_FRAME_3, ARRAY_COUNT(FORWARD_FRAME_3), 200, 0},
};


// backward 2.0
static constexpr ServoAngle BACKWARD_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle BACKWARD_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, -20.0f}, {4, -20.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -50.0f}, {12, 10.0f}, {13, -50.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle BACKWARD_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle BACKWARD_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 20.0f}, {4, 20.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 50.0f}, {11, -10.0f}, {12, 10.0f}, {13, -30.0f}, {14, 50.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame BACKWARD_MOTION[] = {
    {BACKWARD_FRAME_0, ARRAY_COUNT(BACKWARD_FRAME_0), 300, 0},
    {BACKWARD_FRAME_1, ARRAY_COUNT(BACKWARD_FRAME_1), 300, 0},
    {BACKWARD_FRAME_2, ARRAY_COUNT(BACKWARD_FRAME_2), 300, 0},
    {BACKWARD_FRAME_3, ARRAY_COUNT(BACKWARD_FRAME_3), 300, 0},
};


// Rotate Left 2.0
static constexpr ServoAngle ROTATELEFT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 20.0f}, {11, -20.0f}, {12, 10.0f}, {13, -10.0f}, {14, 50.0f},
    {15, -10.0f}, {16, 60.0f},
};

static constexpr ServoAngle ROTATELEFT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame ROTATELEFT_MOTION[] = {
    {ROTATELEFT_FRAME_0, ARRAY_COUNT(ROTATELEFT_FRAME_0), 300, 200},
    {ROTATELEFT_FRAME_1, ARRAY_COUNT(ROTATELEFT_FRAME_1), 800, 0},
};


//  Rotate Right 2.0
static constexpr ServoAngle ROTATERIGHT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -50.0f}, {12, 10.0f}, {13, -20.0f}, {14, 20.0f},
    {15, -10.0f}, {16, -60.0f},
};

static constexpr ServoAngle ROTATERIGHT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame ROTATERIGHT_MOTION[] = {
    {ROTATERIGHT_FRAME_0, ARRAY_COUNT(ROTATERIGHT_FRAME_0), 300, 200},
    {ROTATERIGHT_FRAME_1, ARRAY_COUNT(ROTATERIGHT_FRAME_1), 800, 0},
};


// Moveleft 3.0
static constexpr ServoAngle MOVELEFT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 0.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -50.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -30.0f},
    {10, 30.0f}, {11, -30.0f}, {12, 0.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 50.0f}, {8, 10.0f}, {9, -20.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -20.0f}, {14, 20.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame MOVELEFT_MOTION[] = {
    {MOVELEFT_FRAME_0, ARRAY_COUNT(MOVELEFT_FRAME_0), 300, 0},
    {MOVELEFT_FRAME_1, ARRAY_COUNT(MOVELEFT_FRAME_1), 300, 0},
    {MOVELEFT_FRAME_2, ARRAY_COUNT(MOVELEFT_FRAME_2), 300, 0},
    {MOVELEFT_FRAME_3, ARRAY_COUNT(MOVELEFT_FRAME_3), 300, 0},
};


// Moveright 3.0
static constexpr ServoAngle MOVERIGHT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVERIGHT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 50.0f}, {8, 30.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -30.0f}, {14, 30.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVERIGHT_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -50.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 20.0f}, {9, -10.0f},
    {10, 20.0f}, {11, -20.0f}, {12, 0.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVERIGHT_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame MOVERIGHT_MOTION[] = {
    {MOVERIGHT_FRAME_0, ARRAY_COUNT(MOVERIGHT_FRAME_0), 300, 0},
    {MOVERIGHT_FRAME_1, ARRAY_COUNT(MOVERIGHT_FRAME_1), 300, 0},
    {MOVERIGHT_FRAME_2, ARRAY_COUNT(MOVERIGHT_FRAME_2), 300, 0},
    {MOVERIGHT_FRAME_3, ARRAY_COUNT(MOVERIGHT_FRAME_3), 300, 0},
};

// ═══ AUTO-GENERATED GAIT MOTIONS END ═══


static void applyFrame(const MotionFrame &frame) {
  const uint8_t count = frame.targetCount < MAX_FRAME_TARGETS
                            ? frame.targetCount
                            : MAX_FRAME_TARGETS;
  JointTarget targets[MAX_FRAME_TARGETS];
  for (uint8_t i = 0; i < count; ++i) {
    targets[i].busIndex = BUS_ANY;
    targets[i].servoId = frame.targets[i].servoId;
    targets[i].rawAngle = frame.targets[i].rawAngle;
  }

  applyTargets(targets, count, frame.moveMs);
  if (frame.delayMs > 0) {
    delay(frame.delayMs);
  }
}

static void playMotion(const char *name, const MotionFrame *frames,
                       uint8_t frameCount, bool leaveIdlePoseApplied) {
  logPrintf("Remote action: %s\r\n", name);
  for (uint8_t i = 0; i < frameCount; ++i) {
    applyFrame(frames[i]);
  }
  idlePoseApplied = leaveIdlePoseApplied;
}

static void playStandMotion(uint16_t intervalMs) {
  MotionFrame frame = STAND_MOTION[0];
  frame.moveMs = intervalMs;
  playMotion("Stand", &frame, 1, true);
}

static void playSquadMotion() {
  returnToStandPending = false;
  playMotion("Squad", SQUAD_MOTION, ARRAY_COUNT(SQUAD_MOTION), true);
}

static void playDefaultMotion() {
  returnToStandPending = false;
  playMotion("Default", DEFAULT_MOTION, ARRAY_COUNT(DEFAULT_MOTION), true);
}

static void playLeftPunch() {
  returnToStandPending = false;
  playMotion("PunchLeft", PUNCH_LEFT_MOTION, ARRAY_COUNT(PUNCH_LEFT_MOTION), true);
}

static void playRightPunch() {
  returnToStandPending = false;
  playMotion("PunchRight", PUNCH_RIGHT_MOTION, ARRAY_COUNT(PUNCH_RIGHT_MOTION), true);
}

static void playLeftHookPunch() {
  returnToStandPending = false;
  playMotion("ElbowLeft", ELBOW_LEFT_MOTION, ARRAY_COUNT(ELBOW_LEFT_MOTION), true);
}

static void playRightHookPunch() {
  returnToStandPending = false;
  playMotion("ElbowRight", ELBOW_RIGHT_MOTION, ARRAY_COUNT(ELBOW_RIGHT_MOTION), true);
}

static void playForwardMotion() {
  playMotion("Forward", FORWARD_MOTION, ARRAY_COUNT(FORWARD_MOTION), false);
}

static void playBackwardMotion() {
  playMotion("Backward", BACKWARD_MOTION, ARRAY_COUNT(BACKWARD_MOTION), false);
}

static void playMoveleftMotion() {
  playMotion("Moveleft", MOVELEFT_MOTION, ARRAY_COUNT(MOVELEFT_MOTION), false);
}

static void playMoveRightMotion() {
  playMotion("MoveRight", MOVERIGHT_MOTION, ARRAY_COUNT(MOVERIGHT_MOTION),
             false);
}

static void playRotateLeftMotion() {
  playMotion("RotateLeft", ROTATELEFT_MOTION, ARRAY_COUNT(ROTATELEFT_MOTION),
             false);
}

static void playRotateRightMotion() {
  playMotion("RotateRight", ROTATERIGHT_MOTION,
             ARRAY_COUNT(ROTATERIGHT_MOTION), false);
}

static void playDefenceMotion() {
  returnToStandPending = false;
  playMotion("Defence", DEFENCE_MOTION, ARRAY_COUNT(DEFENCE_MOTION), true);
}

static void playFinalKickMotion() {
  returnToStandPending = false;
  playMotion("FinalKick", FINAL_KICK_MOTION, ARRAY_COUNT(FINAL_KICK_MOTION), true);
}

static void playPunchMotion() {
  returnToStandPending = false;
  playMotion("PunchLeft", PUNCH_LEFT_MOTION, ARRAY_COUNT(PUNCH_LEFT_MOTION), true);
}

static void playDanceMotion() {
  returnToStandPending = false;
  playMotion("Dance", DANCE_MOTION, ARRAY_COUNT(DANCE_MOTION), true);
}

static void playStandupBackMotion() {
  returnToStandPending = false;
  playMotion("StandupBack", STANDUP_BACK_MOTION, ARRAY_COUNT(STANDUP_BACK_MOTION), true);
}

static void playStandupFrontMotion() {
  returnToStandPending = false;
  playMotion("StandupFront", STANDUP_FRONT_MOTION, ARRAY_COUNT(STANDUP_FRONT_MOTION), true);
}

void applyStandPose(uint16_t intervalMs) {
  playStandMotion(intervalMs);
  returnToStandPending = false;
}

void resetMotionState() {
  idlePoseApplied = false;
  motionArmed = false;
  returnToStandPending = false;
  danceActive = false;
}

void toggleDanceMotion() {
  danceActive = !danceActive;
  if (danceActive) {
    returnToStandPending = false;
  } else {
    applyStandPose();
  }
}

void cancelAction() {
  danceActive = false;
  returnToStandPending = false;
  applyStandPose();
}

bool isDanceActive() {
  return danceActive;
}

// ═══ AUTO-GENERATED BUTTON MAP BEGIN (tools/apply_button_map.py --apply) ═══

bool handleRemoteActions(const RemoteSnapshot &snapshot) {
  bool actionRan = false;

  if (consumeSwitchZone(REMOTE_SYSTEM_CHANNEL,
                        snapshot.channels[REMOTE_SYSTEM_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    logPrintln("Remote action: unload servos");
    motionArmed = false;
    idlePoseApplied = false;
    returnToStandPending = false;
    unloadAllServos();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_SYSTEM_CHANNEL,
                        snapshot.channels[REMOTE_SYSTEM_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    cancelAction();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_POSE_CHANNEL,
                        snapshot.channels[REMOTE_POSE_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    motionArmed = true;
    applyStandPose();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_POSE_CHANNEL,
                        snapshot.channels[REMOTE_POSE_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playSquadMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_GETUP_CHANNEL,
                        snapshot.channels[REMOTE_GETUP_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    playDefenceMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_GETUP_CHANNEL,
                        snapshot.channels[REMOTE_GETUP_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    toggleDanceMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_PUNCH_CHANNEL,
                        snapshot.channels[REMOTE_PUNCH_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playLeftHookPunch();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_PUNCH_CHANNEL,
                        snapshot.channels[REMOTE_PUNCH_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    playRightHookPunch();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_HOOK_CHANNEL,
                        snapshot.channels[REMOTE_HOOK_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playPunchMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_HOOK_CHANNEL,
                        snapshot.channels[REMOTE_HOOK_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    playRightPunch();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_DPAD_VERTICAL_CHANNEL,
                        snapshot.channels[REMOTE_DPAD_VERTICAL_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    playStandupBackMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_DPAD_VERTICAL_CHANNEL,
                        snapshot.channels[REMOTE_DPAD_VERTICAL_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playStandupFrontMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_DPAD_HORIZONTAL_CHANNEL,
                        snapshot.channels[REMOTE_DPAD_HORIZONTAL_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    cancelAction();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_DPAD_HORIZONTAL_CHANNEL,
                        snapshot.channels[REMOTE_DPAD_HORIZONTAL_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    cancelAction();
    actionRan = true;
  }

  return actionRan;
}
// ═══ AUTO-GENERATED BUTTON MAP END ═══

void handleMotionCommand(MotionCommand command) {
  if (danceActive) {
    playDanceMotion();
    return;
  }

  if (command == MOTION_IDLE) {
    if ((motionArmed || returnToStandPending) && !idlePoseApplied) {
      applyStandPose();
    }
    return;
  }

  switch (command) {
    case MOTION_WALK_FORWARD:
      playForwardMotion();
      break;
    case MOTION_WALK_BACKWARD:
      playBackwardMotion();
      break;
    case MOTION_WALK_LEFT:
      playMoveleftMotion();
      break;
    case MOTION_WALK_RIGHT:
      playMoveRightMotion();
      break;
    case MOTION_TURN_LEFT:
      playRotateLeftMotion();
      break;
    case MOTION_TURN_RIGHT:
      playRotateRightMotion();
      break;
    case MOTION_IDLE:
    default:
      return;
  }

  returnToStandPending = true;
}
