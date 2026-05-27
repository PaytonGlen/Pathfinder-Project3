// Pilot Arduino — receives motor, arm, and turn commands from Scout over I2C.
// CMD_MOTORS (0x01): left speed, right speed
// CMD_ARM    (0x02): pan angle
// CMD_TURN   (0x03): direction (0 = right, 1 = left)

#include <Wire.h>
#include "Config/Pins.h"
#include "Config/DriveConfig.h"
#include "Motors/MotorControl.h"
#include "Arm/ScanArm.h"

// Must match PILOT_I2C_ADDRESS defined in the scout's SensorScan.h
const uint8_t MY_I2C_ADDRESS = 0x08;

// ─── I2C Command Bytes ───────────────────────────────────────────────────────
// Must match values in Scout's Functions.h
const uint8_t CMD_MOTORS = 0x01;
const uint8_t CMD_ARM    = 0x02;
const uint8_t CMD_TURN   = 0x03;

// ─── Turn Config ─────────────────────────────────────────────────────────────
// Duration and speed of the pivot turn — tune on hardware.
// At base speed a ~350ms pivot gives roughly a 90-degree turn on carpet.
const unsigned long TURN_DURATION_MS = 350;
const uint8_t       TURN_SPEED       = 150;

// ─── Pivot Turn ──────────────────────────────────────────────────────────────
// Blocks for TURN_DURATION_MS, then stops.
// Called from the I2C interrupt, so keep it short — no Serial prints.
void executeTurn(uint8_t dir)
{
    if (dir == 0)  // turn right: left forward, right reverse
    {
        setLeftTrack(true,  TURN_SPEED);
        setRightTrack(false, TURN_SPEED);
    }
    else           // turn left: right forward, left reverse
    {
        setRightTrack(true,  TURN_SPEED);
        setLeftTrack(false, TURN_SPEED);
    }

    delay(TURN_DURATION_MS);
    stopMotors();
}

// ─── I2C Receive Handler ─────────────────────────────────────────────────────
// First byte is the command type; remaining bytes depend on the command.

void onReceive(int numBytes)
{
    if (!Wire.available()) return;
    uint8_t cmd = Wire.read();

    if (cmd == CMD_MOTORS && Wire.available() >= 2)
    {
        uint8_t leftSpeed  = Wire.read();
        uint8_t rightSpeed = Wire.read();
        setLeftTrack(true, leftSpeed);
        setRightTrack(true, rightSpeed);
    }
    else if (cmd == CMD_ARM && Wire.available() >= 1)
    {
        uint8_t angle = Wire.read();
        panTo(angle);
    }
    else if (cmd == CMD_TURN && Wire.available() >= 1)
    {
        uint8_t dir = Wire.read();
        executeTurn(dir);
    }
}

// ─── Setup / Loop ────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(9600);
    Serial.println(F("=== Pilot ready ==="));

    initMotors();
    enableMotors();
    initScanArm();   // attach servo and center arm

    Wire.begin(MY_I2C_ADDRESS);  // join I2C bus as slave
    Wire.onReceive(onReceive);   // register receive handler
}

void loop()
{
    // All driving is event-driven via the I2C receive interrupt.
    // Nothing needed here — the onReceive callback fires automatically.
}