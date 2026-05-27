#include <Wire.h>

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
    Wire.write(const_LEFT_DRIVE_SPEED);
    Wire.write(const_RIGHT_DRIVE_SPEED);
    Wire.endTransmission();
}

int scale(int value, float sensitivity)
{
    float scaled = sensitivity * value * value;
    return (value >= 0 ? 1 : -1) * round(scaled);
}