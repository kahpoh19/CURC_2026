#ifndef MOTIONS_H
#define MOTIONS_H

#include <Arduino.h>

#include "RemoteControl.h"

void applyStandPose(uint16_t intervalMs = 500);
void resetMotionState();
bool handleRemoteActions(const RemoteSnapshot &snapshot);
void handleMotionCommand(MotionCommand command);

#endif
