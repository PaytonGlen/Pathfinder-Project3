// Shared IR sensor functions.

#include "IrSensor.h"

int readIrRaw(uint8_t pin) {
  return analogRead(pin);
}

bool isIrBlocked(uint8_t pin) {
  return analogRead(pin) > IR_OBSTACLE_THRESHOLD;
}
