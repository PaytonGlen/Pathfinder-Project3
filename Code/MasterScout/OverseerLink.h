// Scout → Overseer serial link over Serial3.
// Streams periodic telemetry packets for SD logging and parameter tuning.
// Also listens for parameter updates sent back from the Overseer.
//
// Packet types sent TO Overseer:
//   STATE,<TRACKING|AVOIDING>
//   SENSOR,<index>,<usDist>,<irRaw>,<blocked>,<gapDetected>
//   BEACON,<heading>,<beaconHeading>
//   PARAMS,<Kp>,<Kd>
//   TURN,<direction>,<usDist>      — sent by TurnLog.h
//   CLEAR                          — sent by TurnLog.h
//
// Packet types received FROM Overseer:
//   KP,<value>   — update proportional gain
//   KD,<value>   — update derivative gain

#ifndef OVERSEER_LINK_H
#define OVERSEER_LINK_H

#include <Arduino.h>
#include "Variables.h"

// How often to send a full telemetry snapshot (ms)
const unsigned long TELEMETRY_INTERVAL_MS = 500;

// Tunable PD parameters — Overseer can update these at runtime
int overseerKp = 3;
int overseerKd = 1;

// ─── Send Helpers ────────────────────────────────────────────────────────────

void sendState(CarState state)
{
    Serial3.print(F("STATE,"));
    Serial3.println(state == TRACKING ? F("TRACKING") : F("AVOIDING"));
}

void sendSensors(ScanResult readings[], int count)
{
    for (int i = 0; i < count; i++)
    {
        Serial3.print(F("SENSOR,"));
        Serial3.print(i);                        Serial3.print(',');
        Serial3.print(readings[i].usDist);       Serial3.print(',');
        Serial3.print(readings[i].irRaw);        Serial3.print(',');
        Serial3.print(readings[i].blocked);      Serial3.print(',');
        Serial3.println(readings[i].gapDetected);
    }
}

void sendBeacon(int heading)
{
    Serial3.print(F("BEACON,"));
    Serial3.println(heading);
}

void sendParams()
{
    Serial3.print(F("PARAMS,"));
    Serial3.print(overseerKp);
    Serial3.print(',');
    Serial3.println(overseerKd);
}

// ─── Main Send Function ───────────────────────────────────────────────────────
// Call every loop(). Sends a full snapshot every TELEMETRY_INTERVAL_MS.

void streamTelemetry(CarState state, ScanResult readings[], int count, int beaconHeading)
{
    static unsigned long lastSend = 0;
    unsigned long now = millis();

    if (now - lastSend < TELEMETRY_INTERVAL_MS) return;
    lastSend = now;

    sendState(state);
    sendSensors(readings, count);
    sendBeacon(beaconHeading);
    sendParams();
}

// ─── Receive Handler ─────────────────────────────────────────────────────────
// Call every loop(). Checks Serial3 for parameter update packets from Overseer.

void checkOverseerCommands()
{
    static char    rxBuf[32];
    static uint8_t rxLen = 0;

    while (Serial3.available())
    {
        char c = Serial3.read();

        if (c == '\n' || c == '\r')
        {
            if (rxLen > 0)
            {
                rxBuf[rxLen] = '\0';

                // Parse KP,<value>
                if (strncmp(rxBuf, "KP,", 3) == 0)
                {
                    overseerKp = atoi(rxBuf + 3);
                    Serial.print(F("Overseer updated Kp: "));
                    Serial.println(overseerKp);
                }
                // Parse KD,<value>
                else if (strncmp(rxBuf, "KD,", 3) == 0)
                {
                    overseerKd = atoi(rxBuf + 3);
                    Serial.print(F("Overseer updated Kd: "));
                    Serial.println(overseerKd);
                }

                rxLen = 0;
            }
        }
        else if (rxLen < sizeof(rxBuf) - 1)
        {
            rxBuf[rxLen++] = c;
        }
        else
        {
            rxLen = 0;  // buffer overflow — discard
        }
    }
}

#endif
