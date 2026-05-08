#ifndef SERVO_SYSTEM_H
#define SERVO_SYSTEM_H

#include <Arduino.h>

static constexpr uint8_t SERVO_BUS_A = 0;
static constexpr uint8_t SERVO_BUS_B = 1;
static constexpr uint8_t SERVO_BUS_ALL = 255;

struct JointTarget {
  uint8_t busIndex;
  uint8_t servoId;
  float rawAngle;
};

bool isServoDetectionEnabled();
void setupServoSystem();
bool serviceServoDetection();
void moveAllAndReport(float angle);
void applyTargets(const JointTarget *targets, uint8_t count,
                  uint16_t intervalMs);

#endif
