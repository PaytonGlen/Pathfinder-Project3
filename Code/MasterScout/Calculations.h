// The purpose of this code is to calculate motor speeds based on object detection (distances)
#include <Arduino.h>

typedef uint8_t byte;

struct Motor_Speeds
{
    int LEFT_DRIVE_SPEED;
    int RIGHT_DRIVE_SPEED;
};

const byte const_LEFT_DRIVE_SPEED = 173;
const byte const_RIGHT_DRIVE_SPEED = 180;

int derivative(int firstScan, int secondScan)
{
    int delta = firstScan - secondScan;
    int sign = (delta >= 0) ? 1 : -1;
    return sign * (delta * delta);
}


int PD_Loop(int currentDist, int firstScan, int secondScan)
{
    int targetDist = 6;  // desired distance from wall in inches
    int error = currentDist - targetDist;
    int d = derivative(firstScan, secondScan);

    const int Kp = 3;   // tune these on the actual hardware
    const int Kd = 1;

    int correction = (Kp * error) + (Kd * d);
    return correction;  // positive = steer away, negative = ease back
}

Motor_Speeds calculateSpeeds(int correction, int sensorIndex)
{
    if (sensorIndex <= 3)   // object on the right-hand side. Turn left
    {
        int left_speed = const_LEFT_DRIVE_SPEED;    // reduce this one
        int right_speed = const_RIGHT_DRIVE_SPEED;
    } else 
    {
        int left_speed = const_LEFT_DRIVE_SPEED;    
        int right_speed = const_RIGHT_DRIVE_SPEED;  // reduce this one
    }
}