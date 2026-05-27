// Bluetooth beacon tracking using two HM-10 modules on a rotating scan arm.
// The arm sweeps left and right, sampling RSSI from both modules at each step.
// The angle with the strongest combined signal = beacon heading.

#ifndef BT_TRACKING_H
#define BT_TRACKING_H

#include <Arduino.h>
#include "HM10.h"
#include <Functions.h>   // sendArmAngle() and CMD_ARM live here

// ─── Arm Constants ───────────────────────────────────────────────────────────
// Angles sent to Pilot over I2C — Pilot drives the physical servo.

const int ARM_LEFT       = 110;  // max left angle  — matches Pilot PAN_LEFT
const int ARM_RIGHT      = 0;    // max right angle — matches Pilot PAN_RIGHT
const int ARM_CENTER     = 50;   // forward-facing  — matches Pilot PAN_CENTER (calibrated)
const int ARM_STEP_DEG   = 10;   // degrees per sweep increment

// ─── HM-10 Serial Ports ──────────────────────────────────────────────────────
// Left module on Serial1, right module on Serial2.
// TODO: confirm wiring matches these ports.

// ─── Sweep Tuning ────────────────────────────────────────────────────────────
const unsigned int DEEP_STEP_MS  = 150;  // ms between steps — slow for accuracy
const int          DEEP_SWEEPS   = 3;    // number of full sweeps for Deep_Search

const unsigned int LIGHT_STEP_MS = 120;  // ms between steps — must be >= servo settle time

// ─── State ───────────────────────────────────────────────────────────────────

int beaconHeading = ARM_CENTER;  // last known beacon direction — updated by searches

// ─── Helpers ─────────────────────────────────────────────────────────────────

// Moves arm to angle, waits for it to settle, then samples RSSI from both modules.
// Returns combined signal strength (sum of both RSSI values — less negative = stronger).
int sampleAt(int angle)
{
    sendArmAngle(angle);          // Scout sends angle to Pilot over I2C
    delay(DEEP_STEP_MS);          // wait for physical movement + RSSI settle

    int rssiLeft  = queryRSSI(Serial1);
    int rssiRight = queryRSSI(Serial2);

    // Both return 0 on timeout — treat 0 as very weak signal
    if (rssiLeft  == 0) rssiLeft  = -100;
    if (rssiRight == 0) rssiRight = -100;

    return rssiLeft + rssiRight;  // higher (less negative) = beacon is in this direction
}

// ─── Deep_Search ─────────────────────────────────────────────────────────────
// Blocking. Sweeps arm slowly left and right DEEP_SWEEPS times.
// Finds the angle with the strongest combined RSSI and stores it as beaconHeading.
// Call on startup or when beacon is lost.

void Deep_Search()
{
    int bestAngle  = ARM_CENTER;
    int bestSignal = -9999;

    for (int sweep = 0; sweep < DEEP_SWEEPS; sweep++)
    {
        // Alternate direction each sweep for even coverage
        int startAngle = (sweep % 2 == 0) ? ARM_LEFT : ARM_RIGHT;
        int endAngle   = (sweep % 2 == 0) ? ARM_RIGHT : ARM_LEFT;
        int step       = (endAngle > startAngle) ? ARM_STEP_DEG : -ARM_STEP_DEG;

        for (int angle = startAngle; angle != endAngle; angle += step)
        {
            int signal = sampleAt(angle);

            if (signal > bestSignal)
            {
                bestSignal = signal;
                bestAngle  = angle;
            }
        }
    }

    beaconHeading = bestAngle;
    sendArmAngle(beaconHeading);   // leave arm pointing at beacon

    #ifdef DEBUG
        Serial.print(F("Deep_Search heading: "));
        Serial.println(beaconHeading);
        Serial.print(F("Best signal: "));
        Serial.println(bestSignal);
    #endif
}

// ─── Light_Search ────────────────────────────────────────────────────────────
// Non-blocking. Call every loop() iteration while TRACKING.
// Advances the arm one step per LIGHT_STEP_MS using millis() — no delay().
// The millis() gap doubles as servo settle time from the previous step.
// Updates beaconHeading at the end of each sweep pass.

void Light_Search()
{
    static int           currentAngle = ARM_CENTER;
    static int           stepDir      = ARM_STEP_DEG;   // positive = moving toward ARM_LEFT
    static unsigned long lastStep     = 0;
    static int           bestSignal   = -9999;
    static int           bestAngle    = ARM_CENTER;

    unsigned long now = millis();
    if (now - lastStep < LIGHT_STEP_MS) return;  // not time yet — yield immediately
    lastStep = now;

    // Sample RSSI at current position (servo settled since last call)
    int rssiLeft  = queryRSSI(Serial1);
    int rssiRight = queryRSSI(Serial2);
    if (rssiLeft  == 0) rssiLeft  = -100;   // timeout = treat as very weak
    if (rssiRight == 0) rssiRight = -100;

    int signal = rssiLeft + rssiRight;
    if (signal > bestSignal)
    {
        bestSignal = signal;
        bestAngle  = currentAngle;
    }

    // Move arm to next position for next call to sample
    currentAngle += stepDir;
    sendArmAngle(currentAngle);

    // Reverse at limits — update heading at end of each sweep
    if (currentAngle >= ARM_LEFT)
    {
        currentAngle  = ARM_LEFT;
        stepDir       = -ARM_STEP_DEG;
        beaconHeading = bestAngle;   // commit best angle found this sweep
        bestSignal    = -9999;       // reset for next sweep

        #ifdef DEBUG
            Serial.print(F("Light_Search heading: "));
            Serial.println(beaconHeading);
        #endif
    }
    else if (currentAngle <= ARM_RIGHT)
    {
        currentAngle  = ARM_RIGHT;
        stepDir       = ARM_STEP_DEG;
        beaconHeading = bestAngle;
        bestSignal    = -9999;

        #ifdef DEBUG
            Serial.print(F("Light_Search heading: "));
            Serial.println(beaconHeading);
        #endif
    }
}

#endif
