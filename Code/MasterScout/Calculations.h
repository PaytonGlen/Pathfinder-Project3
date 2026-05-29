/*
    The purpose of this code is to hold the functions used to perform calculations necessary for adjusting motor speeds.
    It holds functions like derivative() and PD_Loops()
*/
#include <Arduino.h>

// Uncomment to enable serial debug output
// #define DEBUG

struct Motor_Speeds
{
    uint8_t LEFT_DRIVE_SPEED;
    uint8_t RIGHT_DRIVE_SPEED;
};

const uint8_t const_LEFT_DRIVE_SPEED  = 125;
const uint8_t const_RIGHT_DRIVE_SPEED = 130;

int derivative(int firstScan, int secondScan)
{
    int delta = firstScan - secondScan;
    int sign  = (delta >= 0) ? 1 : -1;

    #ifdef DEBUG
        Serial.print("Derivative delta: "); Serial.println(delta);
        Serial.print("Derivative sign: ");  Serial.println(sign);
    #endif

    return sign * (delta * delta);
}

int PD_Loop(int firstScan, int secondScan)
{
    const int targetDist  = 13;  // desired distance from wall in cm — tune on hardware
    const int DEFAULT_KP  = 3;   // proportional gain fallback
    const int DEFAULT_KD  = 1;   // derivative gain fallback

    // Use Overseer-supplied gains if they have been received (-1 = not yet sent)
    int Kp = (overseerKp >= 0) ? overseerKp : DEFAULT_KP;
    int Kd = (overseerKd >= 0) ? overseerKd : DEFAULT_KD;

    int error      = firstScan - targetDist;
    int d          = derivative(firstScan, secondScan);
    int correction = (Kp * error) + (Kd * d);

    #ifdef DEBUG
        Serial.print("PD_Loop d_out: ");     Serial.println(d);
        Serial.print("PD_Loop correction: "); Serial.println(correction);
    #endif

    return correction;  // positive = steer away from obstacle, negative = ease back toward wall
}

Motor_Speeds calculateSpeeds(int sensorIndex, int correction)
{
    Motor_Speeds Adjustments;

    if (sensorIndex <= 2)   // obstacle on the right — turn left
    {
        Adjustments.LEFT_DRIVE_SPEED  = constrain(const_LEFT_DRIVE_SPEED  - correction, 0, 255);
        Adjustments.RIGHT_DRIVE_SPEED = constrain(const_RIGHT_DRIVE_SPEED + correction, 0, 255);
    }
    else                    // obstacle on the left — turn right
    {
        Adjustments.LEFT_DRIVE_SPEED  = constrain(const_LEFT_DRIVE_SPEED  + correction, 0, 255);
        Adjustments.RIGHT_DRIVE_SPEED = constrain(const_RIGHT_DRIVE_SPEED - correction, 0, 255);
    }

    return Adjustments;
}