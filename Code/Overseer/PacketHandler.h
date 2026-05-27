#ifndef OVERSEER_PACKETHANDLER_H
#define OVERSEER_PACKETHANDLER_H

#include "BiasEngine.h"
#include "SDLogger.h"
#include "Stats.h"

// ─── Serial Buffer ───────────────────────────────────────────────────────────

static char    rxBuf[64];
static uint8_t rxLen = 0;

// ─── Handlers ────────────────────────────────────────────────────────────────

void handleTurn(char* data)
{
    // TURN,<sensorIdx>,<usDist>
    // sensorIdx is the clock-position sensor (0–5) that triggered the gap turn.
    char line[48];
    snprintf(line, sizeof(line), "%lu,%s", millis(), data);
    appendLine(TURN_LOG, line);

    uint8_t sensorIdx = (uint8_t)atoi(data);

    turnTracker.totalTurns++;
    turnTracker.pending           = true;
    turnTracker.postTurnCount     = 0;
    turnTracker.trackingCount     = 0;
    turnTracker.originSensorIdx   = (sensorIdx < SENSOR_COUNT) ? sensorIdx : 0;

    // Credit the originating sensor — outcome will be recorded in handleState()
    if (sensorIdx < SENSOR_COUNT)
        stats[sensorIdx].turnOriginCount++;
}

void handleSensor(char* data)
{
    // SENSOR,<index>,<usDist>,<irRaw>,<blocked>,<gapDetected>
    char line[64];
    snprintf(line, sizeof(line), "%lu,%s", millis(), data);
    appendLine(SENSOR_LOG, line);

    // ── Parse fields ──────────────────────────────────────────────────────────
    uint8_t idx    = (uint8_t)atoi(data);
    char*   p      = strchr(data, ','); if (!p) return; p++;
    int     usDist = atoi(p);
    p = strchr(p, ','); if (!p) return; p++;  // advance past usDist
    int     irRaw  = atoi(p);
    p = strchr(p, ','); if (!p) return; p++;  // advance past irRaw
    bool    blocked = (atoi(p) != 0);

    if (idx >= SENSOR_COUNT) return;

    // ── Accumulate window counts ──────────────────────────────────────────────
    stats[idx].totalScans++;
    if (blocked)                              stats[idx].blockedCount++;
    if (usDist > 0 && usDist < CLOSE_CALL_CM) stats[idx].closeCallCount++;

    // ── False-positive detection ──────────────────────────────────────────────
    // FP = IR says blocked (high raw value) but US reads clearly far.
    // Suggests sensor noise, reflective surface, or miscalibration.
    bool irBlocked = (irRaw > 125);  // matches IR_OBSTACLE_THRESHOLD in Scout
    bool usClear   = (usDist > FP_US_MIN_CM);
    if (irBlocked && usClear && !blocked)
        stats[idx].fpCount++;

    // ── Event-triggered emergency delta ──────────────────────────────────────
    // If a sensor accumulates EMERGENCY_CLOSE_CALLS close calls without the
    // window completing, send an immediate +1 delta to Scout without waiting.
    if (stats[idx].closeCallCount >= EMERGENCY_CLOSE_CALLS)
    {
        sendBiasDelta(idx, +1);
        stats[idx].closeCallCount = 0;  // reset to prevent repeated triggers
    }

    // ── Trigger window recalculation ─────────────────────────────────────────
    // Fire when sensor 0 completes UPDATE_EVERY_SCANS readings (one full cycle).
    if (idx == 0 && stats[0].totalScans >= UPDATE_EVERY_SCANS)
        updateBiases();
}

void handleState(char* data)
{
    // STATE,<TRACKING|AVOIDING>
    char line[32];
    snprintf(line, sizeof(line), "%lu,%s", millis(), data);
    appendLine(STATE_LOG, line);

    bool isTracking = (strncmp(data, "TRACKING", 8) == 0);

    // ── Post-turn outcome evaluation ─────────────────────────────────────────
    if (turnTracker.pending)
    {
        if (isTracking) turnTracker.trackingCount++;
        turnTracker.postTurnCount++;

        if (turnTracker.postTurnCount >= POST_TURN_EVAL)
        {
            bool turnSucceeded = (turnTracker.trackingCount > POST_TURN_EVAL / 2);

            if (turnSucceeded)
            {
                turnTracker.successfulTurns++;
                // Credit the sensor that triggered this turn
                uint8_t orig = turnTracker.originSensorIdx;
                if (orig < SENSOR_COUNT)
                    stats[orig].turnSuccessCount++;
            }
            turnTracker.pending = false;
        }
    }

    // ── Oscillation detection ─────────────────────────────────────────────────
    // Count TRACKING↔AVOIDING transitions in a rolling OSCIL_WINDOW packet window.
    // Frequent flipping = unstable thresholds or overly aggressive gains.
    if (isTracking != lastStateWasTracking)
        oscillationCount++;

    lastStateWasTracking = isTracking;
    oscilPacketsSeen++;

    if (oscilPacketsSeen >= OSCIL_WINDOW)
    {
        // Window complete — oscillationCount is now available for BiasEngine
        oscilPacketsSeen = 0;
        oscillationCount = 0;  // reset for next window
    }
}

void handleBeacon(char* data)
{
    // BEACON,<heading>
    char line[32];
    snprintf(line, sizeof(line), "%lu,%s", millis(), data);
    appendLine(BEACON_LOG, line);

    int heading = atoi(data);
    if (lastBeaconHeading >= 0)
    {
        int delta = abs(heading - lastBeaconHeading);
        // EMA alpha = 1/8 (slow smoothing — beacon heading changes gradually)
        beaconConfidenceEma += (delta - beaconConfidenceEma) >> 3;
    }
    lastBeaconHeading = heading;
}

void handleParams(char* data)
{
    // PARAMS,<Kp>,<Kd> — Scout reports its current gains each telemetry cycle.
    // Overseer uses this to track drift from what it last commanded.
    // No action needed here; gain updates are pushed proactively by updateGains().
    (void)data;
}

// ─── Router ──────────────────────────────────────────────────────────────────

void handlePacket(char* packet)
{
    if      (strncmp(packet, "TURN,",   5) == 0) handleTurn(packet + 5);
    else if (strncmp(packet, "SENSOR,", 7) == 0) handleSensor(packet + 7);
    else if (strncmp(packet, "STATE,",  6) == 0) handleState(packet + 6);
    else if (strncmp(packet, "BEACON,", 7) == 0) handleBeacon(packet + 7);
    else if (strncmp(packet, "PARAMS,", 7) == 0) handleParams(packet + 7);
    else if (strcmp(packet,  "CLEAR")    == 0)
    {
        appendLine(TURN_LOG,   "--- RUN CLEARED ---");
        appendLine(SENSOR_LOG, "--- RUN CLEARED ---");
        appendLine(STATE_LOG,  "--- RUN CLEARED ---");
        appendLine(BEACON_LOG, "--- RUN CLEARED ---");
        appendLine(BIAS_LOG,   "--- RUN CLEARED ---");
        resetStats();
    }
}

// ─── Serial Tick — call every loop() ─────────────────────────────────────────

void readSerial()
{
    while (Serial1.available())
    {
        char c = Serial1.read();

        if (c == '\n' || c == '\r')
        {
            if (rxLen > 0)
            {
                rxBuf[rxLen] = '\0';
                handlePacket(rxBuf);
                rxLen = 0;
            }
        }
        else if (rxLen < sizeof(rxBuf) - 1)
        {
            rxBuf[rxLen++] = c;
        }
        else
        {
            rxLen = 0;  // buffer overflow — discard and resync
        }
    }
}

#endif
