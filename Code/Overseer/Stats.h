#ifndef OVERSEER_STATS_H
#define OVERSEER_STATS_H

#include "Config.h"

// ─── Per-Sensor Stats ────────────────────────────────────────────────────────

struct SensorStats
{
    // Window accumulators — reset after each UPDATE_EVERY_SCANS window
    uint16_t totalScans;
    uint16_t blockedCount;
    uint16_t closeCallCount;
    uint16_t fpCount;           // IR blocked but US clearly unobstructed (false positive)

    // Persistent across windows
    uint8_t  currentBias;
    uint8_t  lastSentBias;      // bias value at last transmission — used for delta calc
    uint8_t  windowCount;       // how many full windows completed — drives confidence

    // EMA of per-window rates (0–100 scale).
    // Updated at window end: ema += (newRate - ema) >> EMA_ALPHA_SHIFT
    int16_t  blockedRateEma;
    int16_t  closeCallRateEma;
    int16_t  fpRateEma;

    // Per-sensor turn outcome — populated by handleTurn() + handleState() in PacketHandler
    uint8_t  turnOriginCount;   // turns where this sensor was the avoidance trigger
    uint8_t  turnSuccessCount;  // of those, how many succeeded (robot returned to TRACKING)
};

SensorStats stats[SENSOR_COUNT];

// ─── Turn Tracker ────────────────────────────────────────────────────────────

struct TurnTracker
{
    uint8_t  totalTurns;
    uint8_t  successfulTurns;
    bool     pending;            // true = evaluating outcome of most recent turn
    uint8_t  postTurnCount;      // state packets seen since last turn
    uint8_t  trackingCount;      // TRACKING states seen in post-turn window
    uint8_t  originSensorIdx;    // sensor index (0–5) that triggered this turn
};

TurnTracker turnTracker;

// ─── Beacon Confidence ───────────────────────────────────────────────────────
// EMA of |heading delta| between consecutive BEACON packets.
// Higher = heading jumping around = beacon signal unreliable.
// Integer EMA, alpha = 1/8: ema += (delta - ema) >> 3

int beaconConfidenceEma = 0;
int lastBeaconHeading   = -1;

// ─── Oscillation Tracker ─────────────────────────────────────────────────────
// Counts TRACKING↔AVOIDING state transitions in a short window.
// High flip rate = unstable thresholds or overly aggressive gains.

uint8_t  oscillationCount  = 0;   // transitions in current OSCIL_WINDOW
uint8_t  oscilPacketsSeen  = 0;   // state packets in current oscillation window
bool     lastStateWasTracking = true;

// ─── Adaptive Gains ──────────────────────────────────────────────────────────

float adaptiveKp = KP_BASE;
float adaptiveKd = KD_BASE;

// ─── Reset ───────────────────────────────────────────────────────────────────

void resetStats()
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        stats[i].totalScans      = 0;
        stats[i].blockedCount    = 0;
        stats[i].closeCallCount  = 0;
        stats[i].fpCount         = 0;
        stats[i].currentBias     = BASE_BIASES[i];
        stats[i].lastSentBias    = BASE_BIASES[i];
        stats[i].windowCount     = 0;
        stats[i].blockedRateEma  = 0;
        stats[i].closeCallRateEma= 0;
        stats[i].fpRateEma       = 0;
        stats[i].turnOriginCount = 0;
        stats[i].turnSuccessCount= 0;
    }
    turnTracker          = { 0, 0, false, 0, 0, 0 };
    lastBeaconHeading    = -1;
    beaconConfidenceEma  = 0;
    oscillationCount     = 0;
    oscilPacketsSeen     = 0;
    lastStateWasTracking = true;
    adaptiveKp           = KP_BASE;
    adaptiveKd           = KD_BASE;
}

void resetWindowAccumulators()
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        // Update EMAs before clearing window counts
        if (stats[i].totalScans > 0)
        {
            int16_t blockedRate   = (int16_t)(stats[i].blockedCount   * 100 / stats[i].totalScans);
            int16_t closeCallRate = (int16_t)(stats[i].closeCallCount * 100 / stats[i].totalScans);
            int16_t fpRate        = (int16_t)(stats[i].fpCount        * 100 / stats[i].totalScans);

            stats[i].blockedRateEma   += (blockedRate   - stats[i].blockedRateEma)   >> EMA_ALPHA_SHIFT;
            stats[i].closeCallRateEma += (closeCallRate - stats[i].closeCallRateEma) >> EMA_ALPHA_SHIFT;
            stats[i].fpRateEma        += (fpRate        - stats[i].fpRateEma)        >> EMA_ALPHA_SHIFT;

            stats[i].windowCount = min(stats[i].windowCount + 1, 20);  // cap at 20
        }

        stats[i].totalScans     = 0;
        stats[i].blockedCount   = 0;
        stats[i].closeCallCount = 0;
        stats[i].fpCount        = 0;
    }
    turnTracker.totalTurns      = 0;
    turnTracker.successfulTurns = 0;
}

#endif
