// Shared ultrasonic sensor helpers.

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

void initUltrasonic(uint8_t trigPin, uint8_t echoPin);
int readDistanceCm(uint8_t trigPin, uint8_t echoPin);

#endif
