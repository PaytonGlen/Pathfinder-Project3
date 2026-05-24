// Shared IR sensor helpers.

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

const int IR_OBSTACLE_THRESHOLD = 125;

int readIrRaw(uint8_t pin);
bool isIrBlocked(uint8_t pin);

#endif
