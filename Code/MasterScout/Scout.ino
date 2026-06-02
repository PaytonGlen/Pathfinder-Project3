/*
    This is the main program. Upload this program into the arduino to get it functioning
*/

#include <Wire.h>
#include <Pins.h>
#include <Variables.h>
#include <SensorScan.h>
#include <Calculations.h>
#include "Bluetooth/BT_Tracking.h"
#include "TurnLog.h"
#include "OverseerLink.h"

const byte count      = 6;
CarState currentState = TRACKING;  // start in TRACKING mode

void setup()
{
    Serial.begin(9600);
    
    // No bluetooth yet
    // Serial1.begin(9600);  // HM-10 left module
    // Serial2.begin(115200);  // Overseer link

    Wire.begin();         // scout is I2C master

    clearTurnLog();       // fresh run — reset EEPROM stack and notify Overseer
    Deep_Search();        // get initial beacon heading before moving. 
                          // This function also gets the sensor readings

    Serial.println(F("Scout ready"));
}

void loop()
{
    // 1. Scan all sensors
    ScanAll(Sensors, readings, count);

    // 2. Update state based on sensor results
    if (anyBlocked(readings, count))
        currentState = AVOIDING;
    else
        currentState = TRACKING;

    // 3. Act based on current state
    switch (currentState)
    {
        case TRACKING:
        {
            driveForward();   // sends base speeds to pilot over I2C
            Light_Search();   // non-blocking beacon sweep — updates beaconHeading

            // Check if any sensor just detected a gap
            int gap = gapDirection(readings, count);
            if (gap != -1)
            {
                // Only turn into the gap if it roughly aligns with the beacon heading.
                // beaconHeading is a servo angle (0–110). Map sensor index to rough angle:
                // sensors 0–2 (right side) ≈ ARM_RIGHT half, sensors 3–5 (left) ≈ ARM_LEFT half
                bool gapOnRight  = (gap <= 2);
                bool beaconRight = (beaconHeading < ARM_CENTER);

                if (gapOnRight == beaconRight)
                {
                    // Gap aligns with beacon — confirm heading then commit to turn
                    Deep_Search();
                    logTurn(gap, readings[gap].usDist);  // push to EEPROM + stream to Overseer
                    sendTurn(gapOnRight);                 // CMD_TURN → Pilot executes pivot

                    #ifdef DEBUG
                        Serial.print(F("Gap detected at sensor "));
                        Serial.print(gap);
                        Serial.println(gapOnRight ? F(" — turning right") : F(" — turning left"));
                    #endif
                }
            }
            break;
        }

        case AVOIDING:
            direction(readings, count);  // PD correction → I2C → pilot
            break;
    }

    // ----------------------------------------------- No Analyst controller yet -----------------------------------------------
    // Stream telemetry to Overseer every 500ms
    // streamTelemetry(currentState, readings, count, beaconHeading);

    // Check for parameter updates from Overseer
    // checkOverseerCommands();
    // ---------------------------------------------------------------------------------------------------------------------------------------------
    #ifdef DEBUG
        Serial.print(F("State: "));
        Serial.println(currentState == TRACKING ? "TRACKING" : "AVOIDING");
    #endif
}
