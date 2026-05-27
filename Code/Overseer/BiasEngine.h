#ifndef OVERSEER_BIASENGINE_H
#define OVERSEER_BIASENGINE_H

#include "Config.h"
#include "Stats.h"
#include "SDLogger.h"

// ─── Role Classification ──────────────────────────────────────────────────────
// Index: 0=1o'clock  1=3o'clock  2=5o'clock  3=7o'clock  4=9o'clock  5=11o'clock

enum SensorRole { FRONT, SIDE, REAR };

inline SensorRole getRole(uint8_t idx)
{
    if (idx == 0 || idx == 5) return FRONT;
    if (idx == 1 || idx == 4) return SIDE;
    return REAR;
}

// ─── Confidence Score ─────────────────────────────────────────────────────────
// Confidence (0–100) reflects how much data backs the current bias values.
// Rises 25 points per completed window; caps at 100.
// Also penalised when oscillation is high (unstable environment = less trust).

uint8_t computeConfidence()
{
    // Use minimum window count across all sensors (weakest data set drives confidence)
    uint8_t minWindows = 255;
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
        if (stats[i].windowCount < minWindows) minWindows = stats[i].windowCount;

    int conf = (int)minWindows * 25;

    // Penalise for high oscillation — environment is unstable, biases may not hold
    if (oscillationCount > HIGH_OSCIL_THRESH)
        conf -= (oscillationCount - HIGH_OSCIL_THRESH) * 4;

    return (uint8_t)constrain(conf, 0, 100);
}

// ─── Send / Log ───────────────────────────────────────────────────────────────

// Full BIAS update with confidence: BIAS,b0,b1,b2,b3,b4,b5,conf
void sendBiasUpdate(uint8_t conf)
{
    Serial1.print(F("BIAS,"));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        Serial1.print(stats[i].currentBias);
        Serial1.print(',');
    }
    Serial1.println(conf);
}

// Single-sensor delta update: BIAS_DELTA,idx,delta
// Sent immediately for emergency conditions without waiting for full window.
void sendBiasDelta(uint8_t idx, int8_t delta)
{
    if (idx >= SENSOR_COUNT) return;

    int newBias = constrain((int)stats[idx].currentBias + delta, 1, 5);
    stats[idx].currentBias = (uint8_t)newBias;

    Serial1.print(F("BIAS_DELTA,"));
    Serial1.print(idx);
    Serial1.print(',');
    Serial1.println(delta);
}

void logBiasUpdate(uint8_t conf)
{
    char line[64];
    snprintf(line, sizeof(line), "%lu,%d,%d,%d,%d,%d,%d,%d",
             millis(),
             stats[0].currentBias, stats[1].currentBias,
             stats[2].currentBias, stats[3].currentBias,
             stats[4].currentBias, stats[5].currentBias,
             conf);
    appendLine(BIAS_LOG, line);
}

// ─── Adaptive KP/KD ──────────────────────────────────────────────────────────
// Adjusts PD gains based on a smoothed turn success rate and oscillation count.
//
// Uses an EMA of per-window success rate (turnSuccessRateEma) rather than the
// raw window rate. This prevents a single cluttered-environment window from
// swinging gains far in the wrong direction.
//
// Gain update steps are also scaled by confidence: below GAIN_CONF_THRESH the
// step is halved, so early-run data causes only tentative adjustments.

void updateGains()
{
    if (turnTracker.totalTurns < MIN_TURNS_FOR_KP) return;

    // Update the turn success EMA with this window's rate
    int16_t windowSuccessPct = (turnTracker.totalTurns > 0)
        ? (int16_t)((uint32_t)turnTracker.successfulTurns * 100 / turnTracker.totalTurns)
        : 50;
    turnSuccessRateEma += (windowSuccessPct - turnSuccessRateEma) >> GAIN_EMA_SHIFT;

    // Confidence gate: scale step size down when data is sparse
    uint8_t conf = computeConfidence();
    float   step = (conf >= GAIN_CONF_THRESH) ? 1.0f : 0.5f;

    // ── Kp: driven by smoothed turn success rate ──────────────────────────────
    // Poor success  → raise Kp (react more aggressively to obstacles)
    // Strong success with low oscillation → gently lower Kp
    if (turnSuccessRateEma < 35)
        adaptiveKp = constrain(adaptiveKp + 0.4f * step, KP_MIN, KP_MAX);
    else if (turnSuccessRateEma > 75 && oscillationCount <= HIGH_OSCIL_THRESH)
        adaptiveKp = constrain(adaptiveKp - 0.2f * step, KP_MIN, KP_MAX);

    // ── Kd: driven by oscillation rate ───────────────────────────────────────
    // High oscillation → raise Kd to damp rapid corrections
    // Stable run → allow Kd to drift back toward baseline
    if (oscillationCount > HIGH_OSCIL_THRESH)
        adaptiveKd = constrain(adaptiveKd + 0.3f * step, KD_MIN, KD_MAX);
    else if (oscillationCount == 0 && adaptiveKd > KD_BASE)
        adaptiveKd = constrain(adaptiveKd - 0.1f * step, KD_MIN, KD_MAX);

    // Send to Scout (Scout stores Kp/Kd as ints)
    Serial1.print(F("KP,"));
    Serial1.println((int)(adaptiveKp + 0.5f));
    Serial1.print(F("KD,"));
    Serial1.println((int)(adaptiveKd + 0.5f));
}

// ─── Bias Calculation ────────────────────────────────────────────────────────
//
// Weighted proportional formula per sensor:
//
//   delta = w_blocked * blockedPct
//         + w_close   * closeCallPct
//         - W_FP_PENALTY * fpPct
//         + beaconContrib      (FRONT only, when heading is unstable)
//         - oscillPenalty      (SIDE only, when robot is flipping states rapidly)
//         - turnFailPenalty    (when this sensor's own gap turns keep failing)
//
//   bias = clamp(BASE + clamp(delta, -2, +2), 1, 5)
//
// Uses EMA-smoothed rates after the first window to resist noise from short windows.

void updateBiases()
{
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
        if (stats[i].totalScans == 0) continue;

        SensorRole role = getRole(i);

        // ── Choose rates: use EMA once enough data; raw window rate early on ──
        float blockedPct, closeCallPct, fpPct;
        if (stats[i].windowCount >= 2)
        {
            // EMA has had time to converge — use it for stability
            blockedPct   = (float)stats[i].blockedRateEma;
            closeCallPct = (float)stats[i].closeCallRateEma;
            fpPct        = (float)stats[i].fpRateEma;
        }
        else
        {
            // First couple of windows — use live window rate
            blockedPct   = (float)stats[i].blockedCount   * 100.0f / stats[i].totalScans;
            closeCallPct = (float)stats[i].closeCallCount * 100.0f / stats[i].totalScans;
            fpPct        = (float)stats[i].fpCount        * 100.0f / stats[i].totalScans;
        }

        // ── Role-specific weights ─────────────────────────────────────────────
        float wBlocked = (role == FRONT) ? W_BLOCKED_FRONT :
                         (role == SIDE)  ? W_BLOCKED_SIDE  : W_BLOCKED_REAR;
        float wClose   = (role == FRONT) ? W_CLOSE_FRONT   :
                         (role == SIDE)  ? W_CLOSE_SIDE    : W_CLOSE_REAR;

        // ── Core delta ────────────────────────────────────────────────────────
        float delta = 0.0f;
        delta += wBlocked * blockedPct;
        delta += wClose   * closeCallPct;
        delta -= W_FP_PENALTY * fpPct;  // noisy / miscalibrated sensor → pull back

        // ── Beacon uncertainty boost (FRONT only) ─────────────────────────────
        // Unstable beacon heading → be more conservative up front
        if (role == FRONT && beaconConfidenceEma > LOW_CONF_THRESHOLD)
            delta += W_BEACON_FRONT * (float)beaconConfidenceEma;

        // ── Oscillation penalty (SIDE only) ───────────────────────────────────
        // If robot is flipping TRACKING/AVOIDING rapidly, side sensors may be
        // triggering on minor perturbations — reduce their weight temporarily
        if (role == SIDE && oscillationCount > HIGH_OSCIL_THRESH)
            delta -= W_OSCIL_SIDE * (float)(oscillationCount - HIGH_OSCIL_THRESH);

        // ── Per-sensor turn outcome ───────────────────────────────────────────
        // Penalise sensors whose gap detections repeatedly lead to failed turns.
        // "Failed" = robot did not return to TRACKING within POST_TURN_EVAL states.
        if (stats[i].turnOriginCount >= 2)
        {
            uint8_t senSuccPct = (uint8_t)((uint32_t)stats[i].turnSuccessCount * 100
                                           / stats[i].turnOriginCount);
            if (senSuccPct < 40) delta -= TURN_FAIL_PENALTY;
        }

        // ── Apply delta and clamp ─────────────────────────────────────────────
        delta = constrain(delta, -BIAS_DELTA_MAX, BIAS_DELTA_MAX);
        int newBias = (int)((float)BASE_BIASES[i] + delta + 0.5f);
        stats[i].currentBias = (uint8_t)constrain(newBias, 1, 5);
    }

    uint8_t conf = computeConfidence();
    sendBiasUpdate(conf);
    logBiasUpdate(conf);
    updateGains();
    resetWindowAccumulators();
}

#endif
