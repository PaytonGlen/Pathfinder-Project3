// Ring-buffer turn log stored in Scout EEPROM.
// Each entry records the sensor index and distance at time of turn.
// Entries can be popped newest-first to retrace the route in reverse.
// Also streams each entry to the Overseer via Serial3 for SD logging.
//
// Layout:
//   Bytes 0–1  count  — number of valid entries (0 to MAX_ENTRIES)
//   Bytes 2–3  head   — index of the oldest entry (0 to MAX_ENTRIES-1)
//   Bytes 4+   entries — ring slots, each ENTRY_SIZE bytes
//
// When the buffer is full, logTurn() overwrites the oldest slot (at head)
// and advances head, so the log always holds the most recent MAX_ENTRIES turns.
// popTurn() removes from the newest end (head + count - 1), preserving order.

#ifndef TURN_LOG_H
#define TURN_LOG_H

#include <Arduino.h>
#include <EEPROM.h>

// ─── Entry Layout ────────────────────────────────────────────────────────────

struct TurnEntry
{
    uint8_t direction;  // sensor index 0–5 (clock position of gap turned into)
    int16_t usDist;     // ultrasonic distance at time of turn (cm)
};

const int ENTRY_SIZE  = sizeof(TurnEntry);  // 3 bytes
const int HDR_SIZE    = 4;                  // bytes 0–1 = count, bytes 2–3 = head
const int RING_BASE   = HDR_SIZE;
const int MAX_ENTRIES = (EEPROM.length() - HDR_SIZE) / ENTRY_SIZE;

// ─── Header Accessors ────────────────────────────────────────────────────────

static int  getCount() { int v; EEPROM.get(0, v); return (v < 0 || v > MAX_ENTRIES) ? 0 : v; }
static int  getHead()  { int v; EEPROM.get(2, v); return (v < 0 || v >= MAX_ENTRIES) ? 0 : v; }
static void setCount(int v) { EEPROM.put(0, v); }
static void setHead(int v)  { EEPROM.put(2, v); }

// Returns current number of logged turns (0 to MAX_ENTRIES).
int turnCount() { return getCount(); }

// ─── Push / Pop ───────────────────────────────────────────────────────────────

// Logs a turn to EEPROM and streams it to Overseer via Serial3.
// If the buffer is full, the oldest entry is overwritten and head advances.
void logTurn(uint8_t direction, int usDist)
{
    int count = getCount();
    int head  = getHead();
    int slot;

    if (count < MAX_ENTRIES)
    {
        // Buffer not full — write to next empty slot after newest entry
        slot = (head + count) % MAX_ENTRIES;
        setCount(count + 1);
    }
    else
    {
        // Buffer full — overwrite the oldest slot (at head) and advance head
        slot = head;
        setHead((head + 1) % MAX_ENTRIES);
        // count stays at MAX_ENTRIES
    }

    TurnEntry entry = { direction, (int16_t)usDist };
    EEPROM.put(RING_BASE + slot * ENTRY_SIZE, entry);

    // Stream to Overseer for SD logging: TURN,<sensorIdx>,<usDist>
    Serial3.print(F("TURN,"));
    Serial3.print(direction);
    Serial3.print(',');
    Serial3.println(usDist);

    #ifdef DEBUG
        Serial.print(F("TurnLog push: slot="));
        Serial.print(slot);
        Serial.print(F(" dir="));
        Serial.print(direction);
        Serial.print(F(" dist="));
        Serial.println(usDist);
    #endif
}

// Retrieves and removes the most recent turn entry (newest end of ring).
// Returns false if the log is empty.
bool popTurn(TurnEntry& out)
{
    int count = getCount();
    if (count <= 0) return false;

    int head    = getHead();
    int newest  = (head + count - 1) % MAX_ENTRIES;

    EEPROM.get(RING_BASE + newest * ENTRY_SIZE, out);
    setCount(count - 1);
    // head does not move — we removed from the newest end

    #ifdef DEBUG
        Serial.print(F("TurnLog pop: slot="));
        Serial.print(newest);
        Serial.print(F(" dir="));
        Serial.print(out.direction);
        Serial.print(F(" dist="));
        Serial.println(out.usDist);
    #endif

    return true;
}

// Clears the ring buffer (resets count and head — does not erase EEPROM data).
void clearTurnLog()
{
    setCount(0);
    setHead(0);
    Serial3.println(F("CLEAR"));
}

#endif
