/*
The purpose of this file is to hold all of the constants and pins of the I/O
*/

#include <Arduino.h>
typedef uint8_t byte;

// Label pins for US Sensors #1 - #6
// Echo pins are even #
// Trigger pins are odd #

// #0 is there 12 pm one. (exists on the pilot)

// #1 -- 1pm direction
const byte US_Sensor1_Echo_Pin = 32
const byte US_Sensor1_Trig_Pin = 33;

// #2 -- 3pm direction
const byte US_Sensor2_Echo_Pin = 26;
const byte US_Sensor2_Trig_Pin = 27;

// #3 -- 9pm direction
const byte US_Sensor3_Echo_Pin = 51;
const byte US_Sensor3_Trig_Pin = 53;

// #4 -- 11pm direction
const byte US_Sensor4_Echo_Pin = 50;
const byte US_Sensor4_Trig_Pin = 52;

// These no longer exist. will be the ToF light sensors.
/*
const byte IR_Sensor6_Pin = A2;
const byte IR_Sensor5_Pin = A3;
const byte IR_Sensor4_Pin = A4;
const byte IR_Sensor1_Pin = A5;
*/
