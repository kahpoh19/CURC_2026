#include "RemoteControl.h"

#include "Logger.h"

volatile uint16_t remoteChannels[REMOTE_CHANNEL_COUNT] = {};
volatile uint32_t remoteLastEdgeUs = 0;
volatile uint32_t remoteLastFrameUs = 0;
volatile uint8_t remoteCurrentChannel = 0;

static bool remoteZoneLatch[REMOTE_CHANNEL_COUNT][3] = {};

static bool channelLooksValid(uint16_t value) {
  return value >= REMOTE_MIN_PULSE_US && value <= REMOTE_MAX_PULSE_US;
}

static void IRAM_ATTR ppmInterruptHandler() {
  const uint32_t currentTimeUs = micros();
  const uint32_t duration = currentTimeUs - remoteLastEdgeUs;
  remoteLastEdgeUs = currentTimeUs;

  if (duration > REMOTE_SYNC_GAP_US) {
    remoteCurrentChannel = 0;
    remoteLastFrameUs = currentTimeUs;
    return;
  }

  if (duration >= REMOTE_MIN_PULSE_US &&
      duration <= REMOTE_MAX_PULSE_US &&
      remoteCurrentChannel < REMOTE_CHANNEL_COUNT) {
    remoteChannels[remoteCurrentChannel] = static_cast<uint16_t>(duration);
    ++remoteCurrentChannel;
  }
}

static int centeredRemoteValue(uint16_t value) {
  if (!channelLooksValid(value)) {
    return 0;
  }

  int centered = static_cast<int>(value) - static_cast<int>(REMOTE_CENTER_US);
  if (abs(centered) < REMOTE_DEADZONE_US) {
    centered = 0;
  }
  return constrain(centered, -500, 500);
}

bool isRemoteControlEnabled() {
  return ENABLE_REMOTE_CONTROL;
}

void initRemoteReceiver() {
  remoteLastEdgeUs = micros();
  remoteLastFrameUs = 0;
  remoteCurrentChannel = 0;
  for (uint8_t i = 0; i < REMOTE_CHANNEL_COUNT; ++i) {
    remoteChannels[i] = 0;
  }

  pinMode(REMOTE_PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REMOTE_PPM_PIN), ppmInterruptHandler,
                  FALLING);

  logPrintf("PPM remote: pin=%d channels=%u pulse=%u..%uus sync>%uus\r\n",
            REMOTE_PPM_PIN, REMOTE_CHANNEL_COUNT, REMOTE_MIN_PULSE_US,
            REMOTE_MAX_PULSE_US, REMOTE_SYNC_GAP_US);
}

RemoteSnapshot readRemoteSnapshot() {
  RemoteSnapshot snapshot = {};
  uint32_t lastFrameUs = 0;

  noInterrupts();
  for (uint8_t i = 0; i < REMOTE_CHANNEL_COUNT; ++i) {
    snapshot.channels[i] = remoteChannels[i];
  }
  lastFrameUs = remoteLastFrameUs;
  interrupts();

  snapshot.ageUs = lastFrameUs == 0 ? UINT32_MAX : micros() - lastFrameUs;
  snapshot.active = lastFrameUs != 0 && snapshot.ageUs <= REMOTE_FAILSAFE_US &&
                    channelLooksValid(snapshot.channels[REMOTE_WALK_CHANNEL]) &&
                    channelLooksValid(snapshot.channels[REMOTE_TURN_CHANNEL]);
  return snapshot;
}

void reportRemoteSnapshot(const RemoteSnapshot &snapshot) {
  static uint32_t lastReportMs = 0;
  const uint32_t now = millis();
  if (now - lastReportMs < 1000) {
    return;
  }
  lastReportMs = now;

  if (!snapshot.active) {
    logPrintf("PPM remote inactive, age=%luus\r\n",
              static_cast<unsigned long>(snapshot.ageUs));
    return;
  }

  logPrintf("PPM ch1=%u ch2=%u ch4=%u ch5=%u ch6=%u age=%luus\r\n",
            snapshot.channels[0], snapshot.channels[1],
            snapshot.channels[3], snapshot.channels[4],
            snapshot.channels[5],
            static_cast<unsigned long>(snapshot.ageUs));
}

bool consumeSwitchZone(uint8_t channel, uint16_t value, uint16_t minUs,
                       uint16_t maxUs, uint8_t zone) {
  if (channel >= REMOTE_CHANNEL_COUNT || zone >= 3) {
    return false;
  }

  const bool now = channelLooksValid(value) && value >= minUs && value <= maxUs;
  const bool triggered = now && !remoteZoneLatch[channel][zone];
  remoteZoneLatch[channel][zone] = now;
  return triggered;
}

MotionCommand decodeMotionCommand(const RemoteSnapshot &snapshot) {
  const int walk = centeredRemoteValue(snapshot.channels[REMOTE_WALK_CHANNEL]);
  const int turn = centeredRemoteValue(snapshot.channels[REMOTE_TURN_CHANNEL]);

  if (walk == 0 && turn == 0) {
    return MOTION_IDLE;
  }

  if (abs(walk) >= abs(turn)) {
    return walk > 0 ? MOTION_WALK_FORWARD : MOTION_WALK_BACKWARD;
  }

  return turn > 0 ? MOTION_TURN_RIGHT : MOTION_TURN_LEFT;
}
