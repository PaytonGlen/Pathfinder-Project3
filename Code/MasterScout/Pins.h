/*
The purpose of this file is to hold all of the constants and pins of the I/O
*/

#include <Arduino.h>
typedef uint8_t byte;

// Label pins for US Sensors #1 - #6
// Echo pins are even #
// Trigger pins are odd #

const byte US_Sensor1_Echo_Pin = 6;
const byte US_Sensor1_Trig_Pin = 7;

const byte US_Sensor2_Echo_Pin = 4;
const byte US_Sensor2_Trig_Pin = 5;

const byte US_Sensor3_Echo_Pin = 2;
const byte US_Sensor3_Trig_Pin = 3;

const byte US_Sensor4_Echo_Pin = 12;
const byte US_Sensor4_Trig_Pin = 13;

const byte US_Sensor5_Echo_Pin = 10;
const byte US_Sensor5_Trig_Pin = 11;

const byte US_Sensor6_Echo_Pin = 8;
const byte US_Sensor6_Trig_Pin = 9;

// These no longer exist. will be the ToF light sensors.
/*
const byte IR_Sensor6_Pin = A2;
const byte IR_Sensor5_Pin = A3;
const byte IR_Sensor4_Pin = A4;
const byte IR_Sensor1_Pin = A5;
*/
