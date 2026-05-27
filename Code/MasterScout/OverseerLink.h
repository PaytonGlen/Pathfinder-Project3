// Scout → Overseer serial link over Serial3 @ 115200 baud.
// Streams periodic telemetry packets for SD logging and adaptive tuning.
// Listens for parameter updates sent back from the Overseer.
//
// ── Packets sent TO Overseer (Scout → Serial3 TX) ────────────────────────────
//   STATE,<TRACKING|AVOIDING>
//   SENSOR,<idx>,<usDist>,<irRaw>,<blocked>,<gapDetected>
//       idx 0–5 = clock position (0=1o'clock … 5=11o'clock)
//       blocked/gapDetected = 0 or 1
//   BEACON,<heading>
//       heading = servo angle of strongest RSSI (0–180°)
//   PARAMS,<Kp>,<Kd>
//       Scout's current PD gains — sent each telemetry cycle for Overseer reference
//   TURN,<sensorIdx>,<usDist>      — sent by TurnLog.h on each committed turn
//       sensorIdx = gap sensor that triggered the turn (0–5)
//   CLEAR                          — sent by TurnLog.h on clearTurnLog()
//
// ── Packets received FROM Overseer (Overseer Serial1 TX → Scout Serial3 RX) ──
//   KP,<value>                     — set Scout's overseerKp
//   KD,<value>                     — set Scout's overseerKd
//   BIAS,<b0>,<b1>,<b2>,<b3>,<b4>,<b5>,<conf>
//       6 bias values (1–5) + confidence (0–100).
//       conf >= 70 → hard-set; conf < 70 → blend with current value.
//   BIAS_DELTA,<idx>,<delta>
//       Single-sensor emergency update. Applied immediately, no blending.
//       delta is typically +1 or -1.

#ifndef OVERSEER_LINK_H
#define OVERSEER_LINK_H

#include <Arduino.h>
#include "Variables.h"

// How often to send a full telemetry snapshot (ms)
const unsigned long TELEMETRY_INTERVAL_MS = 500;

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
                // Parse BIAS,<b0>,<b1>,<b2>,<b3>,<b4>,<b5>,<conf>
                // conf (0–100) drives blend vs hard-set:
                //   conf >= 70 → hard-set (Overseer is confident, apply directly)
                //   conf <  70 → blend: new = (old * 2 + overseer) / 3  (conservative)
                else if (strncmp(rxBuf, "BIAS,", 5) == 0)
                {
                    char* p = rxBuf + 5;
                    int vals[6] = {0};
                    int count = 0;
                    while (p != nullptr && count < 6)
                    {
                        vals[count++] = atoi(p);
                        p = strchr(p, ',');
                        if (p) p++;
                    }
                    // Read optional confidence (7th field)
                    uint8_t conf = 100;
                    if (p != nullptr) conf = (uint8_t)atoi(p);

                    for (int i = 0; i < count && i < 6; i++)
                    {
                        int v = vals[i];
                        if (v < 1 || v > 5) continue;
                        if (conf >= 70)
                        {
                            // High confidence — apply directly
                            Sensors[i].bias = (uint8_t)v;
                        }
                        else
                        {
                            // Low confidence — blend toward Overseer value conservatively
                            int blended = ((int)Sensors[i].bias * 2 + v + 1) / 3;
                            Sensors[i].bias = (uint8_t)constrain(blended, 1, 5);
                        }
                    }
                    #ifdef DEBUG
                        Serial.print(F("Overseer biases (conf="));
                        Serial.print(conf);
                        Serial.println(')');
                    #endif
                }
                // Parse BIAS_DELTA,<idx>,<delta>
                // Single-sensor emergency update — apply immediately, no blending.
                else if (strncmp(rxBuf, "BIAS_DELTA,", 11) == 0)
                {
                    char* p = rxBuf + 11;
                    int idx = atoi(p);
                    p = strchr(p, ',');
                    if (p != nullptr)
                    {
                        p++;
                        int delta = atoi(p);
                        if (idx >= 0 && idx < 6)
                        {
                            int newBias = constrain((int)Sensors[idx].bias + delta, 1, 5);
                            Sensors[idx].bias = (uint8_t)newBias;
                            #ifdef DEBUG
                                Serial.print(F("Bias delta sensor "));
                                Serial.print(idx);
                                Serial.print(' ');
                                Serial.println(delta > 0 ? F("+1") : F("-1"));
                            #endif
                        }
                    }
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
