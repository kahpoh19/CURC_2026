#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <Arduino.h>

static constexpr bool ENABLE_REMOTE_CONTROL = true;

static constexpr int REMOTE_PPM_PIN = 2;
static constexpr uint8_t REMOTE_CHANNEL_COUNT = 10;
static constexpr uint16_t REMOTE_SYNC_GAP_US = 3000;
static constexpr uint16_t REMOTE_MIN_PULSE_US = 900;
static constexpr uint16_t REMOTE_MAX_PULSE_US = 2100;
static constexpr uint16_t REMOTE_CENTER_US =
    (REMOTE_MIN_PULSE_US + REMOTE_MAX_PULSE_US) / 2;
static constexpr uint16_t REMOTE_DEADZONE_US = 100;
static constexpr uint32_t REMOTE_FAILSAFE_US = 350000;
static constexpr uint32_t REMOTE_CONTROL_PERIOD_MS = 20;

// Channel indexes follow the PPM order used by IS_Robot2025.
static constexpr uint8_t REMOTE_WALK_CHANNEL = 1;
static constexpr uint8_t REMOTE_TURN_CHANNEL = 3;
static constexpr uint8_t REMOTE_PUNCH_CHANNEL = 4;
static constexpr uint8_t REMOTE_MODE_CHANNEL = 5;

static constexpr uint16_t SWITCH_LOW_MIN_US = 900;
static constexpr uint16_t SWITCH_LOW_MAX_US = 1150;
static constexpr uint16_t SWITCH_CENTER_MIN_US = 1400;
static constexpr uint16_t SWITCH_CENTER_MAX_US = 1600;
static constexpr uint16_t SWITCH_HIGH_MIN_US = 1850;
static constexpr uint16_t SWITCH_HIGH_MAX_US = 2100;

enum MotionCommand {
  MOTION_IDLE,
  MOTION_WALK_FORWARD,
  MOTION_WALK_BACKWARD,
  MOTION_TURN_LEFT,
  MOTION_TURN_RIGHT,
};

struct RemoteSnapshot {
  uint16_t channels[REMOTE_CHANNEL_COUNT];
  uint32_t ageUs;
  bool active;
};

bool isRemoteControlEnabled();
void initRemoteReceiver();
RemoteSnapshot readRemoteSnapshot();
void reportRemoteSnapshot(const RemoteSnapshot &snapshot);
bool consumeSwitchZone(uint8_t channel, uint16_t value, uint16_t minUs,
                       uint16_t maxUs, uint8_t zone);
MotionCommand decodeMotionCommand(const RemoteSnapshot &snapshot);

#endif
