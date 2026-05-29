/*
The purpose of this file is to serve as a location for generic utility functions that can be used throughought the project.
Some functions like scale(), would be useful in more than one location so it would live in this file.

This file also hold I2C, and driving functions
*/

#include <Wire.h>

// ─── I2C Command Bytes ───────────────────────────────────────────────────────
// First byte of every I2C packet tells the Pilot what follows.
const uint8_t CMD_MOTORS = 0x01;  // Byte 1: leftSpeed, Byte 2: rightSpeed
const uint8_t CMD_ARM    = 0x02;  // Byte 1: servo angle
const uint8_t CMD_TURN   = 0x03;  // Byte 1: direction (0 = right, 1 = left)

const float default_sensitivity = 0.05;

int max(int first, int second, int third, int fourth, int fifth, int sixth)
{
    int maxValue = first;

    if (second > maxValue)
    {
        maxValue = second;
    }

    if (third > maxValue)
    {
        maxValue = third;
    }

    if (fourth > maxValue)
    {
        maxValue = fourth;
    }

    if (fifth > maxValue)
    {
        maxValue = fifth;
    }

    if (sixth > maxValue)
    {
        maxValue = sixth;
    }

    return maxValue;
}

void driveForward()
{
    Wire.beginTransmission(PILOT_I2C_ADDRESS);
    Wire.write(CMD_MOTORS);
    Wire.write(const_LEFT_DRIVE_SPEED);
    Wire.write(const_RIGHT_DRIVE_SPEED);
    Wire.endTransmission();
}

void sendArmAngle(uint8_t angle)
{
    Wire.beginTransmission(PILOT_I2C_ADDRESS);
    Wire.write(CMD_ARM);
    Wire.write(angle);
    Wire.endTransmission();
}

// Tells Pilot to execute a pivot turn into a detected gap.
// turnRight = true  → right track reverses, left track drives forward (car turns right)
// turnRight = false → left track reverses, right track drives forward (car turns left)
void sendTurn(bool turnRight)
{
    Wire.beginTransmission(PILOT_I2C_ADDRESS);
    Wire.write(CMD_TURN);
    Wire.write(turnRight ? 0 : 1);
    Wire.endTransmission();
}

int scale(int value, float sensitivity)
{
    float scaled = sensitivity * value * value;
    return (value >= 0 ? 1 : -1) * round(scaled);
}

void Deep_Search()
{
    ScanAll(Sensors, readings, 6);
}