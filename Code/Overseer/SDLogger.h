#ifndef OVERSEER_SDLOGGER_H
#define OVERSEER_SDLOGGER_H

#include <SD.h>
#include <SPI.h>
#include "Config.h"

bool sdReady = false;

// ─── Helpers ─────────────────────────────────────────────────────────────────

void appendLine(const char* filename, const char* line)
{
    if (!sdReady) return;
    File f = SD.open(filename, FILE_WRITE);
    if (f) { f.println(line); f.close(); }
}

// ─── Init ────────────────────────────────────────────────────────────────────

void initLogFiles()
{
    if (!SD.exists(TURN_LOG))   appendLine(TURN_LOG,   "millis,direction,usDist");
    if (!SD.exists(SENSOR_LOG)) appendLine(SENSOR_LOG, "millis,index,usDist,irRaw,blocked,gapDetected");
    if (!SD.exists(STATE_LOG))  appendLine(STATE_LOG,  "millis,state");
    if (!SD.exists(BEACON_LOG)) appendLine(BEACON_LOG, "millis,heading");
    if (!SD.exists(BIAS_LOG))   appendLine(BIAS_LOG,   "millis,b0,b1,b2,b3,b4,b5");
}

#endif
