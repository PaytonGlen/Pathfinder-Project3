// Shared MPU-6050 helpers.

#ifndef MPU_6050_H
#define MPU_6050_H

typedef int16_t short;

#include <Arduino.h>

void initMpu();
void readMpu(short &ax, short &ay, short &az,
             short &gx, short &gy, short &gz);

#endif
