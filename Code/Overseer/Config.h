#ifndef OVERSEER_CONFIG_H
#define OVERSEER_CONFIG_H

#include <Arduino.h>

// ─── Pins ────────────────────────────────────────────────────────────────────

const uint8_t SD_CS_PIN = 10;

// ─── Log Files ───────────────────────────────────────────────────────────────

const char TURN_LOG[]   = "turns.csv";
const char SENSOR_LOG[] = "sensors.csv";
const char STATE_LOG[]  = "states.csv";
const char BEACON_LOG[] = "beacon.csv";
const char BIAS_LOG[]   = "bias.csv";

// ─── Sensor Config ───────────────────────────────────────────────────────────

const uint8_t SENSOR_COUNT = 6;

// Must match initial bias values in Scout's SensorScan.h Sensors[] array.
// Index: 0=1o'clock  1=3o'clock  2=5o'clock  3=7o'clock  4=9o'clock  5=11o'clock
const uint8_t BASE_BIASES[SENSOR_COUNT] = { 4, 3, 1, 1, 3, 4 };

// ─── Tuning Thresholds (legacy — kept for reference) ─────────────────────────

const uint8_t CLOSE_CALL_CM      = 6;   // usDist below this counts as a close call
const uint8_t UPDATE_EVERY_SCANS = 50;  // recalculate after N scans on sensor 0
const uint8_t POST_TURN_EVAL     = 10;  // state packets to observe after a turn
const int     LOW_CONF_THRESHOLD = 20;  // beacon EMA delta > this = low confidence

// ─── Weighted Bias Formula ────────────────────────────────────────────────────
// bias_delta = w_blocked*blockedPct + w_close*closePct - w_fp*fpPct
//            + beaconContrib (front only) - oscillPenalty (side only) - turnFailPenalty
// Total delta is clamped to [-2, +2] before adding to BASE and clamping to [1, 5].

// Role: FRONT (idx 0, 5), SIDE (idx 1, 4), REAR (idx 2, 3)
const float W_BLOCKED_FRONT  = 0.022f;  // at 100% blocked  → +2.2 (clamped to +2)
const float W_BLOCKED_SIDE   = 0.018f;
const float W_BLOCKED_REAR   = 0.010f;

const float W_CLOSE_FRONT    = 0.012f;  // at 100% close calls → +1.2 additional
const float W_CLOSE_SIDE     = 0.008f;
const float W_CLOSE_REAR     = 0.005f;

const float W_FP_PENALTY     = 0.015f;  // per % false-positive rate → up to -1.5
                                         // FP = IR blocked but US clearly unobstructed

const float W_BEACON_FRONT   = 0.030f;  // per unit of beacon EMA → +1.5 at EMA=50
const float W_OSCIL_SIDE     = 0.020f;  // per oscillation above threshold → penalty

const float TURN_FAIL_PENALTY = 0.60f;  // subtracted when per-sensor turn success < 40%
const float BIAS_DELTA_MAX    = 2.0f;   // maximum swing from BASE in either direction

// ─── False-Positive Detection ─────────────────────────────────────────────────
const uint8_t FP_US_MIN_CM = 18;        // US must read > this for IR-block to count as FP
// Must match IR_OBSTACLE_THRESHOLD in Scout's Variables.h.
// If you change the threshold on Scout, update this value too.
const int     IR_BLOCK_RAW_THRESHOLD = 125;

// ─── Event-Triggered Updates ─────────────────────────────────────────────────
const uint8_t EMERGENCY_CLOSE_CALLS = 4;  // close calls in window before immediate delta send

// ─── Oscillation Detection ───────────────────────────────────────────────────
const uint8_t OSCIL_WINDOW       = 20;   // state packets per oscillation window
const uint8_t HIGH_OSCIL_THRESH  = 8;    // transitions above this = unstable

// ─── Adaptive KP/KD ──────────────────────────────────────────────────────────
// Sent back to Scout when turn success or recovery time indicates gains need adjustment.
const float   KP_BASE          = 3.0f;
const float   KD_BASE          = 1.0f;
const float   KP_MAX           = 5.0f;
const float   KP_MIN           = 1.5f;
const float   KD_MAX           = 3.0f;
const float   KD_MIN           = 0.5f;
const uint8_t MIN_TURNS_FOR_KP = 5;     // wait for this many turns before adjusting KP/KD
// Gain changes are gated on confidence to prevent drift from sparse/noisy windows.
// Below this threshold the gain update step is scaled down proportionally.
const uint8_t GAIN_CONF_THRESH = 50;    // confidence below 50 → half-step gain change
// EMA for turn success rate used in gain adaptation (separate from per-sensor bias EMA).
const uint8_t GAIN_EMA_SHIFT   = 3;     // alpha = 1/8 — slow smoothing, resists single bad window

// ─── Confidence ───────────────────────────────────────────────────────────────
// Confidence is 0–100. Scout blends vs hard-sets biases based on this.
const uint8_t BIAS_CONF_HARD_SET_THRESH = 70;  // >= this → Scout applies immediately
// Confidence formula: conf = clamp(windowCount * 25, 0, 100)
//   25 per window = full confidence after 4 windows (~200 scans)

// ─── EMA ─────────────────────────────────────────────────────────────────────
const uint8_t EMA_ALPHA_SHIFT = 2;  // alpha = 1/4  (faster adaptation than beacon EMA's 1/8)

#endif
