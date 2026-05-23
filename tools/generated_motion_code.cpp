// 自动生成的动作帧代码
// 来源: try.json
// 工具: tools/json_to_motion.py
// 动作组数: 9
// 舵机数: 17 (ID 0-16)
//
// 用法: 复制以下代码粘贴到 src/Motions.cpp 中对应位置
//
// 帧数组命名: {GROUP}_FRAME_{N}  (ServoAngle[])
// 动作数组命名: {GROUP}_MOTION  (MotionFrame[])
// moveMs = 帧的 time 字段
// delayMs = 帧的 delay 字段 (null → 0)

// ── 默认 (DEFAULT) ──────────────────────────────────────────────
static constexpr ServoAngle DEFAULT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, 0.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 0.0f}, {8, 0.0f}, {9, 0.0f},
    {10, 0.0f}, {11, 0.0f}, {12, 0.0f}, {13, 0.0f}, {14, 0.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr MotionFrame DEFAULT_MOTION[] = {
    {DEFAULT_FRAME_0, ARRAY_COUNT(DEFAULT_FRAME_0), 300, 500},
};

// ── Stand (STAND) ─────────────────────────────────────────────
static constexpr ServoAngle STAND_FRAME_0[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -30.0f}, {3, 10.0f}, {4, -10.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 30.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 55.0f}, {11, -35.0f}, {12, 10.0f}, {13, -55.0f}, {14, 35.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame STAND_MOTION[] = {
    {STAND_FRAME_0, ARRAY_COUNT(STAND_FRAME_0), 500, 0},
};

// ── Squad (SQUAD) ─────────────────────────────────────────────
static constexpr ServoAngle SQUAD_FRAME_0[] = {
    {0, -30.0f}, {1, 12.7f}, {2, -56.8f}, {3, -1.6f}, {4, 7.8f},
    {5, 35.0f}, {6, -12.6f}, {7, 45.4f}, {8, -0.1f}, {9, -4.6f},
    {10, 97.7f}, {11, -86.7f}, {12, -3.5f}, {13, -93.6f}, {14, 83.1f},
    {15, -5.5f}, {16, -1.0f},
};

static constexpr MotionFrame SQUAD_MOTION[] = {
    {SQUAD_FRAME_0, ARRAY_COUNT(SQUAD_FRAME_0), 300, 0},
};

// ── Forward (FORWARD) ─────────────────────────────────────────
static constexpr ServoAngle FORWARD_FRAME_0[] = {
    {0, -44.0f}, {1, 0.0f}, {2, -30.0f}, {3, -30.0f}, {4, -20.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 30.0f}, {8, 12.0f}, {9, -12.0f},
    {10, 35.0f}, {11, -41.0f}, {12, 5.5f}, {13, -60.0f}, {14, 25.0f},
    {15, -9.0f}, {16, -10.0f},
};

static constexpr ServoAngle FORWARD_FRAME_1[] = {
    {0, -44.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 30.0f}, {8, 17.0f}, {9, -17.0f},
    {10, 47.0f}, {11, -27.0f}, {12, 9.5f}, {13, -47.0f}, {14, 27.0f},
    {15, -14.0f}, {16, 0.0f},
};

static constexpr ServoAngle FORWARD_FRAME_2[] = {
    {0, -44.0f}, {1, 0.0f}, {2, -30.0f}, {3, 30.0f}, {4, 20.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 30.0f}, {8, 12.0f}, {9, -12.0f},
    {10, 60.0f}, {11, -25.0f}, {12, 9.0f}, {13, -35.0f}, {14, 41.0f},
    {15, -5.5f}, {16, 10.0f},
};

static constexpr ServoAngle FORWARD_FRAME_3[] = {
    {0, -44.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 30.0f}, {8, 17.0f}, {9, -17.0f},
    {10, 47.5f}, {11, -27.7f}, {12, 10.0f}, {13, -47.7f}, {14, 27.5f},
    {15, -18.5f}, {16, 0.0f},
};

static constexpr MotionFrame FORWARD_MOTION[] = {
    {FORWARD_FRAME_0, ARRAY_COUNT(FORWARD_FRAME_0), 200, 0},
    {FORWARD_FRAME_1, ARRAY_COUNT(FORWARD_FRAME_1), 200, 0},
    {FORWARD_FRAME_2, ARRAY_COUNT(FORWARD_FRAME_2), 200, 0},
    {FORWARD_FRAME_3, ARRAY_COUNT(FORWARD_FRAME_3), 200, 0},
};

// ── Backward (BACKWARD) ───────────────────────────────────────
static constexpr ServoAngle BACKWARD_FRAME_0[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -40.0f}, {3, 10.0f}, {4, 10.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 40.0f}, {8, 11.0f}, {9, -11.0f},
    {10, 58.0f}, {11, -37.0f}, {12, 4.0f}, {13, -28.0f}, {14, 75.0f},
    {15, -8.0f}, {16, -10.0f},
};

static constexpr ServoAngle BACKWARD_FRAME_1[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -40.0f}, {3, 20.0f}, {4, 20.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 40.0f}, {8, 11.0f}, {9, -11.0f},
    {10, 76.0f}, {11, -37.0f}, {12, 6.5f}, {13, -29.0f}, {14, 75.0f},
    {15, -6.5f}, {16, -5.0f},
};

static constexpr ServoAngle BACKWARD_FRAME_2[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -40.0f}, {3, 10.0f}, {4, 10.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 40.0f}, {8, 11.0f}, {9, -11.0f},
    {10, 52.0f}, {11, -56.0f}, {12, 7.2f}, {13, -43.5f}, {14, 56.0f},
    {15, -5.2f}, {16, 0.0f},
};

static constexpr ServoAngle BACKWARD_FRAME_3[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -40.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 40.0f}, {8, 11.0f}, {9, -11.0f},
    {10, 28.0f}, {11, -75.0f}, {12, 8.0f}, {13, -58.0f}, {14, 37.0f},
    {15, -4.0f}, {16, 5.0f},
};

static constexpr ServoAngle BACKWARD_FRAME_4[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -40.0f}, {3, -10.0f}, {4, -10.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 40.0f}, {8, 11.0f}, {9, -11.0f},
    {10, 29.0f}, {11, -75.0f}, {12, 6.5f}, {13, -76.0f}, {14, 37.0f},
    {15, -6.5f}, {16, 10.0f},
};

static constexpr ServoAngle BACKWARD_FRAME_5[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -40.0f}, {3, 10.0f}, {4, -10.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 40.0f}, {8, 14.0f}, {9, -14.0f},
    {10, 60.0f}, {11, -50.0f}, {12, 10.0f}, {13, -60.0f}, {14, 50.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame BACKWARD_MOTION[] = {
    {BACKWARD_FRAME_0, ARRAY_COUNT(BACKWARD_FRAME_0), 300, 0},
    {BACKWARD_FRAME_1, ARRAY_COUNT(BACKWARD_FRAME_1), 300, 0},
    {BACKWARD_FRAME_2, ARRAY_COUNT(BACKWARD_FRAME_2), 300, 0},
    {BACKWARD_FRAME_3, ARRAY_COUNT(BACKWARD_FRAME_3), 300, 0},
    {BACKWARD_FRAME_4, ARRAY_COUNT(BACKWARD_FRAME_4), 300, 0},
    {BACKWARD_FRAME_5, ARRAY_COUNT(BACKWARD_FRAME_5), 300, 0},
};

// ── Moveleft (MOVELEFT) ───────────────────────────────────────
static constexpr ServoAngle MOVELEFT_FRAME_0[] = {
    {0, -45.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 45.0f}, {6, 0.0f}, {7, 30.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 48.0f}, {11, -37.0f}, {12, 0.0f}, {13, -53.0f}, {14, 61.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_1[] = {
    {0, -32.7f}, {1, 0.0f}, {2, -42.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 42.7f}, {6, 0.0f}, {7, 44.0f}, {8, 12.0f}, {9, 5.0f},
    {10, 58.0f}, {11, -64.0f}, {12, 7.0f}, {13, -59.0f}, {14, 64.0f},
    {15, 4.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_2[] = {
    {0, -42.7f}, {1, 0.0f}, {2, -44.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 32.7f}, {6, 0.0f}, {7, 42.0f}, {8, -5.0f}, {9, -12.0f},
    {10, 59.0f}, {11, -64.0f}, {12, -4.0f}, {13, -58.0f}, {14, 64.0f},
    {15, -7.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_3[] = {
    {0, -42.7f}, {1, 0.0f}, {2, -44.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 32.7f}, {6, 0.0f}, {7, 42.0f}, {8, 0.0f}, {9, -12.0f},
    {10, 50.0f}, {11, -55.0f}, {12, -4.0f}, {13, -50.0f}, {14, 55.0f},
    {15, -7.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVELEFT_FRAME_4[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -30.0f}, {3, 10.0f}, {4, -10.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 20.0f}, {8, 25.0f}, {9, -25.0f},
    {10, 45.0f}, {11, -50.0f}, {12, 10.0f}, {13, -45.0f}, {14, 50.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame MOVELEFT_MOTION[] = {
    {MOVELEFT_FRAME_0, ARRAY_COUNT(MOVELEFT_FRAME_0), 200, 0},
    {MOVELEFT_FRAME_1, ARRAY_COUNT(MOVELEFT_FRAME_1), 200, 0},
    {MOVELEFT_FRAME_2, ARRAY_COUNT(MOVELEFT_FRAME_2), 200, 0},
    {MOVELEFT_FRAME_3, ARRAY_COUNT(MOVELEFT_FRAME_3), 200, 0},
    {MOVELEFT_FRAME_4, ARRAY_COUNT(MOVELEFT_FRAME_4), 500, 0},
};

// ── MoveRight (MOVE_RIGHT) ────────────────────────────────────
static constexpr ServoAngle MOVE_RIGHT_FRAME_0[] = {
    {0, -45.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 45.0f}, {6, 0.0f}, {7, 30.0f}, {8, 10.0f}, {9, -10.0f},
    {10, 53.0f}, {11, -61.0f}, {12, 10.0f}, {13, -48.0f}, {14, 37.0f},
    {15, 0.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVE_RIGHT_FRAME_1[] = {
    {0, -42.7f}, {1, 0.0f}, {2, -44.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 32.7f}, {6, 0.0f}, {7, 42.0f}, {8, -5.0f}, {9, -12.0f},
    {10, 59.0f}, {11, -64.0f}, {12, -4.0f}, {13, -58.0f}, {14, 64.0f},
    {15, -7.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVE_RIGHT_FRAME_2[] = {
    {0, -32.7f}, {1, 0.0f}, {2, -42.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 42.7f}, {6, 0.0f}, {7, 44.0f}, {8, 12.0f}, {9, 5.0f},
    {10, 58.0f}, {11, -63.0f}, {12, 7.0f}, {13, -59.0f}, {14, 64.0f},
    {15, 4.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVE_RIGHT_FRAME_3[] = {
    {0, -32.7f}, {1, 0.0f}, {2, -42.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 42.7f}, {6, 0.0f}, {7, 44.0f}, {8, 12.0f}, {9, 0.0f},
    {10, 50.0f}, {11, -55.0f}, {12, 7.0f}, {13, -50.0f}, {14, 55.0f},
    {15, 4.0f}, {16, 0.0f},
};

static constexpr ServoAngle MOVE_RIGHT_FRAME_4[] = {
    {0, -30.0f}, {1, 0.0f}, {2, -30.0f}, {3, 10.0f}, {4, -10.0f},
    {5, 30.0f}, {6, 0.0f}, {7, 30.0f}, {8, 25.0f}, {9, -25.0f},
    {10, 45.0f}, {11, -50.0f}, {12, 10.0f}, {13, -45.0f}, {14, 50.0f},
    {15, -10.0f}, {16, 0.0f},
};

static constexpr MotionFrame MOVE_RIGHT_MOTION[] = {
    {MOVE_RIGHT_FRAME_0, ARRAY_COUNT(MOVE_RIGHT_FRAME_0), 200, 0},
    {MOVE_RIGHT_FRAME_1, ARRAY_COUNT(MOVE_RIGHT_FRAME_1), 200, 0},
    {MOVE_RIGHT_FRAME_2, ARRAY_COUNT(MOVE_RIGHT_FRAME_2), 200, 0},
    {MOVE_RIGHT_FRAME_3, ARRAY_COUNT(MOVE_RIGHT_FRAME_3), 200, 0},
    {MOVE_RIGHT_FRAME_4, ARRAY_COUNT(MOVE_RIGHT_FRAME_4), 500, 0},
};

// ── RotateLeft (ROTATE_LEFT) ──────────────────────────────────
static constexpr ServoAngle ROTATE_LEFT_FRAME_0[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -30.0f}, {3, 40.0f}, {4, 30.0f},
    {5, 45.0f}, {6, 0.0f}, {7, 30.0f}, {8, 15.0f}, {9, -20.0f},
    {10, 35.0f}, {11, -35.0f}, {12, 10.0f}, {13, -55.0f}, {14, 55.0f},
    {15, -12.0f}, {16, 0.0f},
};

static constexpr ServoAngle ROTATE_LEFT_FRAME_1[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -30.0f}, {3, 20.0f}, {4, 40.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 15.0f}, {9, -20.0f},
    {10, 38.0f}, {11, -36.0f}, {12, 12.0f}, {13, -54.0f}, {14, 55.0f},
    {15, -14.0f}, {16, 24.0f},
};

static constexpr ServoAngle ROTATE_LEFT_FRAME_2[] = {
    {0, 0.0f}, {1, 0.0f}, {2, -30.0f}, {3, 30.0f}, {4, 30.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 15.0f}, {9, -20.0f},
    {10, 35.0f}, {11, -33.0f}, {12, 10.0f}, {13, -57.0f}, {14, 58.0f},
    {15, -12.0f}, {16, 34.5f},
};

static constexpr ServoAngle ROTATE_LEFT_FRAME_3[] = {
    {0, -45.0f}, {1, 0.0f}, {2, -30.0f}, {3, 10.0f}, {4, 40.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 15.0f}, {9, -20.0f},
    {10, 36.0f}, {11, -35.0f}, {12, 12.0f}, {13, -57.0f}, {14, 56.0f},
    {15, -13.0f}, {16, 45.0f},
};

static constexpr ServoAngle ROTATE_LEFT_FRAME_4[] = {
    {0, -45.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 45.0f}, {6, 0.0f}, {7, 30.0f}, {8, 17.0f}, {9, -17.0f},
    {10, 45.0f}, {11, -35.0f}, {12, 20.0f}, {13, -45.0f}, {14, 38.0f},
    {15, -20.0f}, {16, 0.0f},
};

static constexpr MotionFrame ROTATE_LEFT_MOTION[] = {
    {ROTATE_LEFT_FRAME_0, ARRAY_COUNT(ROTATE_LEFT_FRAME_0), 200, 0},
    {ROTATE_LEFT_FRAME_1, ARRAY_COUNT(ROTATE_LEFT_FRAME_1), 200, 0},
    {ROTATE_LEFT_FRAME_2, ARRAY_COUNT(ROTATE_LEFT_FRAME_2), 200, 0},
    {ROTATE_LEFT_FRAME_3, ARRAY_COUNT(ROTATE_LEFT_FRAME_3), 200, 0},
    {ROTATE_LEFT_FRAME_4, ARRAY_COUNT(ROTATE_LEFT_FRAME_4), 200, 0},
};

// ── RotateRight (ROTATE_RIGHT) ────────────────────────────────
static constexpr ServoAngle ROTATE_RIGHT_FRAME_0[] = {
    {0, -25.0f}, {1, 0.0f}, {2, -30.0f}, {3, -30.0f}, {4, -30.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 20.0f}, {9, -15.0f},
    {10, 55.0f}, {11, -55.0f}, {12, 12.0f}, {13, -35.0f}, {14, 35.0f},
    {15, -5.0f}, {16, 0.0f},
};

static constexpr ServoAngle ROTATE_RIGHT_FRAME_1[] = {
    {0, -25.0f}, {1, 0.0f}, {2, -30.0f}, {3, -20.0f}, {4, -20.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 20.0f}, {9, -15.0f},
    {10, 52.0f}, {11, -52.0f}, {12, 13.0f}, {13, -38.0f}, {14, 38.0f},
    {15, -7.0f}, {16, -24.0f},
};

static constexpr ServoAngle ROTATE_RIGHT_FRAME_2[] = {
    {0, -25.0f}, {1, 0.0f}, {2, -30.0f}, {3, -30.0f}, {4, -30.0f},
    {5, 0.0f}, {6, 0.0f}, {7, 30.0f}, {8, 20.0f}, {9, -15.0f},
    {10, 55.0f}, {11, -55.0f}, {12, 12.0f}, {13, -35.0f}, {14, 35.0f},
    {15, -5.0f}, {16, -48.0f},
};

static constexpr ServoAngle ROTATE_RIGHT_FRAME_3[] = {
    {0, -40.0f}, {1, 0.0f}, {2, -30.0f}, {3, -10.0f}, {4, -10.0f},
    {5, 22.5f}, {6, 0.0f}, {7, 30.0f}, {8, 20.0f}, {9, -15.0f},
    {10, 54.0f}, {11, -53.0f}, {12, 12.8f}, {13, -36.0f}, {14, 37.0f},
    {15, -6.5f}, {16, -54.0f},
};

static constexpr ServoAngle ROTATE_RIGHT_FRAME_4[] = {
    {0, -25.0f}, {1, 0.0f}, {2, -30.0f}, {3, 0.0f}, {4, 0.0f},
    {5, 45.0f}, {6, 0.0f}, {7, 30.0f}, {8, 20.0f}, {9, -15.0f},
    {10, 40.0f}, {11, -40.0f}, {12, 20.0f}, {13, -40.0f}, {14, 35.0f},
    {15, -20.0f}, {16, 0.0f},
};

static constexpr MotionFrame ROTATE_RIGHT_MOTION[] = {
    {ROTATE_RIGHT_FRAME_0, ARRAY_COUNT(ROTATE_RIGHT_FRAME_0), 200, 0},
    {ROTATE_RIGHT_FRAME_1, ARRAY_COUNT(ROTATE_RIGHT_FRAME_1), 200, 0},
    {ROTATE_RIGHT_FRAME_2, ARRAY_COUNT(ROTATE_RIGHT_FRAME_2), 200, 0},
    {ROTATE_RIGHT_FRAME_3, ARRAY_COUNT(ROTATE_RIGHT_FRAME_3), 200, 0},
    {ROTATE_RIGHT_FRAME_4, ARRAY_COUNT(ROTATE_RIGHT_FRAME_4), 200, 0},
};

// ════════════════════════════════════════════════════════
// 生成的动作列表:
//   DEFAULT — 1 帧
//   STAND — 1 帧
//   SQUAD — 1 帧
//   FORWARD — 4 帧
//   BACKWARD — 6 帧
//   MOVELEFT — 5 帧
//   MOVE_RIGHT — 5 帧
//   ROTATE_LEFT — 5 帧
//   ROTATE_RIGHT — 5 帧
// ════════════════════════════════════════════════════════
