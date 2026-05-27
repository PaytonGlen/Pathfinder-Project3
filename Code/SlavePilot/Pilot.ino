// Pilot Arduino — receives motor speeds from scout over I2C and drives the tracks.

#include <Wire.h>
#include "Config/Pins.h"
#include "Config/DriveConfig.h"
#include "Motors/MotorControl.h"

// Must match PILOT_I2C_ADDRESS defined in the scout's SensorScan.h
const uint8_t MY_I2C_ADDRESS = 0x08;

// ─── I2C Receive Handler ─────────────────────────────────────────────────────
// Called automatically when the scout sends 2 bytes.
// Byte 0 = left track speed, Byte 1 = right track speed (both 0–255).

void onReceiveMotorSpeeds(int numBytes)
{
    if (numBytes < 2 || Wire.available() < 2) return;

    uint8_t leftSpeed  = Wire.read();
    uint8_t rightSpeed = Wire.read();

    setLeftTrack(true, leftSpeed);
    setRightTrack(true, rightSpeed);
}

// ─── Setup / Loop ────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(9600);
    Serial.println(F("=== Pilot ready ==="));

    initMotors();
    enableMotors();

    Wire.begin(MY_I2C_ADDRESS);          // join I2C bus as slave
    Wire.onReceive(onReceiveMotorSpeeds); // register receive handler
}

void loop()
{
    // All driving is event-driven via the I2C receive interrupt.
    // Nothing needed here — the onReceive callback fires automatically.
}