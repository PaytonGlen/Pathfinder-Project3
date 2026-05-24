// Shared drive tuning values for the master Arduino.

#ifndef DRIVE_CONFIG_H
#define DRIVE_CONFIG_H

#include <Arduino.h>

const byte LEFT_DRIVE_SPEED = 173;
const byte RIGHT_DRIVE_SPEED = 180;
const byte TURN_SPEED = 220;

const unsigned long MS_PER_FOOT = 800;
const unsigned long MS_PER_90_DEG = 450;

const bool RIGHT_FORWARD_HIGH = true;
const bool LEFT_FORWARD_HIGH = true;

#endif
