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
    Serial.println("Derivative delta: ");
    Serial.println(delta);

    int sign = (delta >= 0) ? 1 : -1;
    Serial.println("Derivative sign: ");
    Serial.println(sign);

    return sign * (delta * delta);
}


int PD_Loop(int firstScan, int secondScan)
{
    int targetDist = 6;  // desired distance from wall in inches. May need tuning
    int error = firstScan - targetDist;
    int d = derivative(firstScan, secondScan);
     Serial.println("PD_Loop d_out: ");
    Serial.println(d);

    const int Kp = 3;   // tune these on the actual hardware
    const int Kd = 1;

    int correction = (Kp * error) + (Kd * d);
    Serial.println("PD_Loop correction: ");
    Serial.println(correction);

    return correction;  // positive = steer away, negative = ease back
}

Motor_Speeds calculateSpeeds(int sensorIndex, int correction)
{
    Motor_Speeds Adjustments;

    if (sensorIndex <= 2)   // object on the right-hand side. Turn left
    {
        Adjustments.LEFT_DRIVE_SPEED = const_LEFT_DRIVE_SPEED - correction;    // reduce this one
        Adjustments.RIGHT_DRIVE_SPEED = const_RIGHT_DRIVE_SPEED + correction;
    } else 
    {
        Adjustments.LEFT_DRIVE_SPEED = const_LEFT_DRIVE_SPEED + correction;    
        Adjustments.RIGHT_DRIVE_SPEED = const_RIGHT_DRIVE_SPEED - correction;  // reduce this one
    }

    return Adjustments;
}