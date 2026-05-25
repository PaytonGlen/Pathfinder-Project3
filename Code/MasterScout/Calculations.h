// The purpose of this code is to calculate motor speeds based on object detection (distances)
#include <Arduino.h>

typedef uint8_t byte;

struct Motor_Speeds
{
    int LEFT_DRIVE_SPEED;
    int RIGHT_DRIVE_SPEED;
};

const byte const_LEFT_DRIVE_SPEED = 125;
const byte const_RIGHT_DRIVE_SPEED = 130;

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

Motor_Speeds calculateSpeeds(int sensorIndex, int correction)
{
    if (sensorIndex <= 2)   // object on the right-hand side. Turn left
    {
        int left_speed = const_LEFT_DRIVE_SPEED - correction;    // reduce this one
        int right_speed = const_RIGHT_DRIVE_SPEED + correction;
    } else 
    {
        int left_speed = const_LEFT_DRIVE_SPEED + correction;    
        int right_speed = const_RIGHT_DRIVE_SPEED - correction;  // reduce this one
    }
}