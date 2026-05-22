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
    {STAND_FRAME_0, ARRAY_COUNT(STAND_FRAME_0), 300, 500},
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
    {DEFENCE_FRAME_0, ARRAY_COUNT(DEFENCE_FRAME_0), 800, 500},
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
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 50.0f}, {9, -30.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, 10.0f}, {16, -100.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_4[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 40.0f}, {4, -40.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, -30.0f}, {9, -50.0f},
    {10, 90.0f}, {11, -80.0f}, {12, -25.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -15.0f}, {16, -80.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_5[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 40.0f}, {4, -40.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 50.0f}, {8, 30.0f}, {9, -50.0f},
    {10, 90.0f}, {11, -80.0f}, {12, -25.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -15.0f}, {16, -80.0f},
};

static constexpr ServoAngle FINAL_KICK_FRAME_6[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -50.0f}, {3, 40.0f}, {4, -40.0f},
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


// Punch
static constexpr ServoAngle PUNCH_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 80.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -35.0f}, {12, 10.0f}, {13, -35.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 90.0f},
};

static constexpr ServoAngle PUNCH_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 30.0f}, {11, -35.0f}, {12, 10.0f}, {13, -35.0f}, {14, 30.0f},
    {15, -10.0f}, {16, 90.0f},
};

static constexpr ServoAngle PUNCH_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame PUNCH_MOTION[] = {
    {PUNCH_FRAME_0, ARRAY_COUNT(PUNCH_FRAME_0), 500, 0},
    {PUNCH_FRAME_1, ARRAY_COUNT(PUNCH_FRAME_1), 100, 500},
    {PUNCH_FRAME_2, ARRAY_COUNT(PUNCH_FRAME_2), 700, 100},
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
    {0, -30.0f}, {1, 0.0f}, {2, -80.0f}, {3, 180.0f}, {4, -180.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 80.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_BACK_FRAME_3[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -80.0f}, {3, 35.0f}, {4, -35.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 80.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_BACK_FRAME_4[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame STANDUP_BACK_MOTION[] = {
    {STANDUP_BACK_FRAME_0, ARRAY_COUNT(STANDUP_BACK_FRAME_0), 500, 200},
    {STANDUP_BACK_FRAME_1, ARRAY_COUNT(STANDUP_BACK_FRAME_1), 500, 200},
    {STANDUP_BACK_FRAME_2, ARRAY_COUNT(STANDUP_BACK_FRAME_2), 500, 500},
    {STANDUP_BACK_FRAME_3, ARRAY_COUNT(STANDUP_BACK_FRAME_3), 1500, 1200},
    {STANDUP_BACK_FRAME_4, ARRAY_COUNT(STANDUP_BACK_FRAME_4), 700, 500},
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
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle STANDUP_FRONT_FRAME_3[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -70.0f}, {3, -20.0f}, {4, 20.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 90.0f}, {11, -80.0f}, {12, 10.0f}, {13, -90.0f}, {14, 80.0f},
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
    {STANDUP_FRONT_FRAME_3, ARRAY_COUNT(STANDUP_FRONT_FRAME_3), 1500, 1000},
    {STANDUP_FRONT_FRAME_4, ARRAY_COUNT(STANDUP_FRONT_FRAME_4), 700, 100},
};

// ═══ AUTO-GENERATED POSES END ═══

static constexpr ServoAngle LB_FRAME_0[] = {
    {5, 110.0f}, {6, 0.0f}, {7, 50.0f}, {4, 100.0f},
    {3, 10.0f}, {0, -30.0f}, {1, 0.0f}, {2, -30.0f},
    {9, -10.0f}, {8, 10.0f}, {13, -85.0f}, {14, 65.0f},
    {15, -10.0f}, {10, 85.0f}, {11, -65.0f}, {12, 10.0f},
    {16, -40.0f},
};
static constexpr ServoAngle LB_FRAME_1[] = {
    {5, 110.0f}, {6, 0.0f}, {7, 50.0f}, {4, 100.0f},
    {3, -10.0f}, {0, -10.0f}, {1, 0.0f}, {2, -30.0f},
    {9, -15.0f}, {8, 15.0f}, {13, -85.0f}, {14, 65.0f},
    {15, -15.0f}, {10, 85.0f}, {11, -65.0f}, {12, 15.0f},
    {16, 100.0f},
};
static constexpr ServoAngle LB_FRAME_2[] = {
    {5, 40.0f}, {6, 0.0f}, {7, 0.0f}, {4, 100.0f},
    {3, -10.0f}, {0, -10.0f}, {1, 0.0f}, {2, -50.0f},
    {9, -15.0f}, {8, 15.0f}, {13, -85.0f}, {14, 65.0f},
    {15, -15.0f}, {10, 85.0f}, {11, -65.0f}, {12, 15.0f},
    {16, 100.0f},
};
static constexpr ServoAngle LB_FRAME_3[] = {
    {5, 30.0f}, {6, 0.0f}, {7, 20.0f}, {4, -10.0f},
    {3, 10.0f}, {0, -30.0f}, {1, 0.0f}, {2, -20.0f},
    {9, -10.0f}, {8, 10.0f}, {13, -65.0f}, {14, 50.0f},
    {15, -10.0f}, {10, 65.0f}, {11, -50.0f}, {12, 10.0f},
    {16, 0.0f},
};
static constexpr MotionFrame LB_MOTION[] = {
    {LB_FRAME_0, ARRAY_COUNT(LB_FRAME_0), 500, 0},
    {LB_FRAME_1, ARRAY_COUNT(LB_FRAME_1), 200, 300},
    {LB_FRAME_2, ARRAY_COUNT(LB_FRAME_2), 200, 300},
    {LB_FRAME_3, ARRAY_COUNT(LB_FRAME_3), 700, 0},
};

static constexpr ServoAngle LT_FRAME_0[] = {
    {0, -100.0f}, {1, 0.0f}, {2, -0.4f}, {3, 7.3f},
    {4, -25.5f}, {5, 133.0f}, {6, 90.0f}, {7, -5.0f},
    {8, 15.0f}, {9, -15.0f}, {10, 70.0f}, {11, -69.0f},
    {12, 15.0f}, {13, -31.0f}, {14, 70.0f}, {15, -25.0f},
    {16, 100.0f},
};
static constexpr ServoAngle LT_FRAME_1[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -30.0f}, {3, 10.0f},
    {4, -10.0f}, {5, 30.0f}, {6, 0.0f}, {7, 30.0f},
    {8, 10.0f}, {9, -10.0f}, {10, 55.0f}, {11, -35.0f},
    {12, 10.0f}, {13, -55.0f}, {14, 35.0f}, {15, -10.0f},
    {16, 0.0f},
};
static constexpr MotionFrame LT_MOTION[] = {
    {LT_FRAME_0, ARRAY_COUNT(LT_FRAME_0), 300, 300},
    {LT_FRAME_1, ARRAY_COUNT(LT_FRAME_1), 1000, 200},
};

static constexpr ServoAngle RB_FRAME_0[] = {
    {0, -110.0f}, {1, 0.0f}, {2, -50.0f}, {3, -100.0f},
    {4, -10.0f}, {5, 30.0f}, {6, 0.0f}, {7, 30.0f},
    {8, 10.0f}, {9, -10.0f}, {10, 85.0f}, {11, -65.0f},
    {12, 10.0f}, {13, -85.0f}, {14, 65.0f}, {15, -10.0f},
    {16, 40.0f},
};
static constexpr ServoAngle RB_FRAME_1[] = {
    {0, -110.0f}, {1, 0.0f}, {2, -50.0f}, {3, -100.0f},
    {4, 10.0f}, {5, 10.0f}, {6, 0.0f}, {7, 30.0f},
    {8, 15.0f}, {9, -15.0f}, {10, 85.0f}, {11, -65.0f},
    {12, 15.0f}, {13, -85.0f}, {14, 65.0f}, {15, -15.0f},
    {16, -100.0f},
};
static constexpr ServoAngle RB_FRAME_2[] = {
    {0, -40.0f}, {1, 0.0f}, {2, 0.0f}, {3, -100.0f},
    {4, 10.0f}, {5, 10.0f}, {6, 0.0f}, {7, 50.0f},
    {8, 15.0f}, {9, -15.0f}, {10, 85.0f}, {11, -65.0f},
    {12, 15.0f}, {13, -85.0f}, {14, 65.0f}, {15, -15.0f},
    {16, -100.0f},
};
static constexpr ServoAngle RB_FRAME_3[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -20.0f}, {3, 10.0f},
    {4, -10.0f}, {5, 30.0f}, {6, 0.0f}, {7, 20.0f},
    {8, 10.0f}, {9, -10.0f}, {10, 65.0f}, {11, -50.0f},
    {12, 10.0f}, {13, -65.0f}, {14, 50.0f}, {15, -10.0f},
    {16, 0.0f},
};
static constexpr MotionFrame RB_MOTION[] = {
    {RB_FRAME_0, ARRAY_COUNT(RB_FRAME_0), 500, 0},
    {RB_FRAME_1, ARRAY_COUNT(RB_FRAME_1), 200, 300},
    {RB_FRAME_2, ARRAY_COUNT(RB_FRAME_2), 200, 300},
    {RB_FRAME_3, ARRAY_COUNT(RB_FRAME_3), 700, 0},
};

static constexpr ServoAngle RT_FRAME_0[] = {
    {0, -120.4f}, {1, -40.0f}, {2, 11.3f}, {3, -60.9f},
    {4, -10.0f}, {5, 30.0f}, {6, 0.0f}, {7, 30.0f},
    {8, 15.0f}, {9, -15.0f}, {10, 30.0f}, {11, -60.0f},
    {12, 25.0f}, {13, -65.0f}, {14, 45.0f}, {15, -15.0f},
    {16, -100.0f},
};
static constexpr ServoAngle RT_FRAME_1[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -30.0f}, {3, 10.0f},
    {4, -10.0f}, {5, 30.0f}, {6, 0.0f}, {7, 30.0f},
    {8, 10.0f}, {9, -10.0f}, {10, 55.0f}, {11, -35.0f},
    {12, 10.0f}, {13, -55.0f}, {14, 35.0f}, {15, -10.0f},
    {16, 0.0f},
};
static constexpr MotionFrame RT_MOTION[] = {
    {RT_FRAME_0, ARRAY_COUNT(RT_FRAME_0), 300, 300},
    {RT_FRAME_1, ARRAY_COUNT(RT_FRAME_1), 700, 0},
};

static constexpr ServoAngle X_FRAME_0[] = {
    {0, -30.0f}, {1, -90.0f}, {2, 90.0f}, {3, 10.0f},
    {4, -10.0f}, {5, 30.0f}, {6, 90.0f}, {7, -90.0f},
    {8, 15.0f}, {9, -15.0f}, {10, 100.0f}, {11, -80.0f},
    {12, 15.0f}, {13, -100.0f}, {14, 80.0f}, {15, -15.0f},
    {16, 0.0f},
};
static constexpr ServoAngle X_FRAME_1[] = {
    {0, 0.0f}, {1, -120.0f}, {2, 95.0f}, {3, 160.0f},
    {4, -160.0f}, {5, 0.0f}, {6, 120.0f}, {7, -95.0f},
    {8, 15.0f}, {9, -15.0f}, {10, 100.0f}, {11, -80.0f},
    {12, 15.0f}, {13, -100.0f}, {14, 80.0f}, {15, -15.0f},
    {16, 0.0f},
};
static constexpr MotionFrame X_MOTION[] = {
    {X_FRAME_0, ARRAY_COUNT(X_FRAME_0), 800, 1000},
    {X_FRAME_1, ARRAY_COUNT(X_FRAME_1), 350, 200},
};

static constexpr ServoAngle B_FRAME_0[] = {
    {0, -30.0f}, {1, -90.0f}, {2, 90.0f}, {3, 0.0f},
    {4, 0.0f}, {5, 30.0f}, {6, 90.0f}, {7, -90.0f},
    {8, 15.0f}, {9, -15.0f}, {10, 100.0f}, {11, -80.0f},
    {12, 15.0f}, {13, -100.0f}, {14, 80.0f}, {15, -15.0f},
    {16, 0.0f},
};
static constexpr ServoAngle B_FRAME_1[] = {
    {0, -30.0f}, {1, -90.0f}, {2, 90.0f}, {3, -150.0f},
    {4, 150.0f}, {5, 30.0f}, {6, 90.0f}, {7, -90.0f},
    {8, 15.0f}, {9, -15.0f}, {10, 100.0f}, {11, -80.0f},
    {12, 15.0f}, {13, -100.0f}, {14, 80.0f}, {15, -15.0f},
    {16, 0.0f},
};
static constexpr MotionFrame B_MOTION[] = {
    {B_FRAME_0, ARRAY_COUNT(B_FRAME_0), 800, 800},
    {B_FRAME_1, ARRAY_COUNT(B_FRAME_1), 200, 0},
};

// Forward
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


// Moveleft 2.0
static constexpr ServoAngle MOVELEFT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 0.0f}, {9, 0.0f},
    {10, 40.0f}, {11, -40.0f}, {12, 0.0f}, {13, -40.0f}, {14, 40.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 50.0f}, {8, 10.0f}, {9, 0.0f},
    {10, 50.0f}, {11, -50.0f}, {12, 0.0f}, {13, -40.0f}, {14, 40.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 10.0f}, {9, -30.0f},
    {10, 40.0f}, {11, -40.0f}, {12, 0.0f}, {13, -40.0f}, {14, 40.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_4[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 10.0f}, {9, -30.0f},
    {10, 40.0f}, {11, -40.0f}, {12, -20.0f}, {13, -40.0f}, {14, 40.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_5[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, 0.0f},
    {10, 50.0f}, {11, -50.0f}, {12, 15.0f}, {13, -50.0f}, {14, 50.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr MotionFrame MOVELEFT_MOTION[] = {
    {MOVELEFT_FRAME_0, ARRAY_COUNT(MOVELEFT_FRAME_0), 800, 0},
    {MOVELEFT_FRAME_1, ARRAY_COUNT(MOVELEFT_FRAME_1), 400, 0},
    {MOVELEFT_FRAME_2, ARRAY_COUNT(MOVELEFT_FRAME_2), 200, 300},
    {MOVELEFT_FRAME_3, ARRAY_COUNT(MOVELEFT_FRAME_3), 200, 0},
    {MOVELEFT_FRAME_4, ARRAY_COUNT(MOVELEFT_FRAME_4), 500, 0},
    {MOVELEFT_FRAME_5, ARRAY_COUNT(MOVELEFT_FRAME_5), 400, 0},
};


// Moveright 2.0
static constexpr ServoAngle MOVERIGHT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 10.0f}, {11, -10.0f}, {12, 10.0f}, {13, -10.0f}, {14, 10.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVERIGHT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -70.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 0.0f}, {9, 0.0f},
    {10, 40.0f}, {11, -40.0f}, {12, 0.0f}, {13, -40.0f}, {14, 40.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVERIGHT_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -50.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 70.0f}, {8, 0.0f}, {9, -10.0f},
    {10, 40.0f}, {11, -40.0f}, {12, 0.0f}, {13, -50.0f}, {14, 50.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVERIGHT_FRAME_3[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 30.0f}, {9, -10.0f},
    {10, 40.0f}, {11, -40.0f}, {12, 0.0f}, {13, -40.0f}, {14, 40.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr MotionFrame MOVERIGHT_MOTION[] = {
    {MOVERIGHT_FRAME_0, ARRAY_COUNT(MOVERIGHT_FRAME_0), 800, 0},
    {MOVERIGHT_FRAME_1, ARRAY_COUNT(MOVERIGHT_FRAME_1), 400, 0},
    {MOVERIGHT_FRAME_2, ARRAY_COUNT(MOVERIGHT_FRAME_2), 200, 200},
    {MOVERIGHT_FRAME_3, ARRAY_COUNT(MOVERIGHT_FRAME_3), 200, 500},
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

static void playLeftPunch() {
  returnToStandPending = false;
  playMotion("LB", LB_MOTION, ARRAY_COUNT(LB_MOTION), true);
}

static void playRightPunch() {
  returnToStandPending = false;
  playMotion("RB", RB_MOTION, ARRAY_COUNT(RB_MOTION), true);
}

static void playLeftHookPunch() {
  returnToStandPending = false;
  playMotion("LT", LT_MOTION, ARRAY_COUNT(LT_MOTION), true);
}

static void playRightHookPunch() {
  returnToStandPending = false;
  playMotion("RT", RT_MOTION, ARRAY_COUNT(RT_MOTION), true);
}

static void playFrontGetUpMotion() {
  returnToStandPending = false;
  playMotion("X", X_MOTION, ARRAY_COUNT(X_MOTION), true);
}

static void playBackGetUpMotion() {
  returnToStandPending = false;
  playMotion("B", B_MOTION, ARRAY_COUNT(B_MOTION), true);
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
  playMotion("Punch", PUNCH_MOTION, ARRAY_COUNT(PUNCH_MOTION), true);
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
}

// ═══ AUTO-GENERATED BUTTON MAP BEGIN (tools/apply_button_map.py --apply) ═══

bool handleRemoteActions(const RemoteSnapshot &snapshot) {
  bool actionRan = false;

  if (consumeSwitchZone(REMOTE_SYSTEM_CHANNEL,
                        snapshot.channels[REMOTE_SYSTEM_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    motionArmed = true;
    applyStandPose();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_SYSTEM_CHANNEL,
                        snapshot.channels[REMOTE_SYSTEM_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    logPrintln("Remote action: unload servos");
    motionArmed = false;
    idlePoseApplied = false;
    returnToStandPending = false;
    unloadAllServos();
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
    playStandupBackMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_GETUP_CHANNEL,
                        snapshot.channels[REMOTE_GETUP_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playStandupFrontMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_PUNCH_CHANNEL,
                        snapshot.channels[REMOTE_PUNCH_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playPunchMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_PUNCH_CHANNEL,
                        snapshot.channels[REMOTE_PUNCH_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    playFinalKickMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_HOOK_CHANNEL,
                        snapshot.channels[REMOTE_HOOK_CHANNEL],
                        SWITCH_LOW_MIN_US, SWITCH_LOW_MAX_US, 0)) {
    playDanceMotion();
    actionRan = true;
  }

  if (consumeSwitchZone(REMOTE_HOOK_CHANNEL,
                        snapshot.channels[REMOTE_HOOK_CHANNEL],
                        SWITCH_HIGH_MIN_US, SWITCH_HIGH_MAX_US, 1)) {
    playDefenceMotion();
    actionRan = true;
  }

  return actionRan;
}
// ═══ AUTO-GENERATED BUTTON MAP END ═══

void handleMotionCommand(MotionCommand command) {
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
