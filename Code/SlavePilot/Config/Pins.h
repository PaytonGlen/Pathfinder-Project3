// Shared hardware pin assignments for the master Arduino.

/*
    Some changes will need to be made here:
    - There will be 6 US Sensors instead of just 1
    - There will be 6 IR Sensors
    - I/O will need to be laid out for that
*/

#ifndef PINS_H
#define PINS_H

typedef uint8_t byte;

#include <Arduino.h>

const byte PIN_STBY = 3;
const byte PIN_PWMA = 5;
const byte PIN_PWMB = 6;
const byte PIN_AIN1 = 7;
const byte PIN_BIN1 = 8;

// Only one US_Sensor on the pilot arduino
const byte PIN_TRIG_FRONT = 13;
const byte PIN_ECHO_FRONT = 12;

const byte PIN_SERVO_PAN = 10;
const byte PIN_SERVO_TILT = 11;

const byte PIN_RANDOM_SEED = A3;

#endif
