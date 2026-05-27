// Stack-based turn log stored in Scout EEPROM.
// Each entry records the direction turned and distance at time of turn.
// Entries can be popped to retrace the route in reverse.
// Also streams each entry to the Overseer via Serial3 for SD logging.

#ifndef TURN_LOG_H
#define TURN_LOG_H

#include <Arduino.h>
#include <EEPROM.h>

// ─── Entry Layout ────────────────────────────────────────────────────────────
// Each TurnEntry is 3 bytes: direction (1) + usDist (2)
// Stack grows upward from STACK_BASE. Stack pointer stored at address 0.

struct TurnEntry
{
    uint8_t direction;  // sensor index 0–5 (clock position of gap turned into)
    int16_t usDist;     // ultrasonic distance at time of turn (cm)
};

const int  ENTRY_SIZE  = sizeof(TurnEntry);   // 3 bytes
const int  STACK_BASE  = 2;                   // entries start at byte 2 (bytes 0–1 = stack pointer)
const int  MAX_ENTRIES = (EEPROM.length() - STACK_BASE) / ENTRY_SIZE;

// ─── Stack Pointer ────────────────────────────────────────────────────────────

// Returns current stack depth (number of logged turns)
int turnCount()
{
    int count;
    EEPROM.get(0, count);
    if (count < 0 || count > MAX_ENTRIES) return 0;  // corrupted — treat as empty
    return count;
}

static void setTurnCount(int count)
{
    EEPROM.put(0, count);
}

// ─── Push / Pop ───────────────────────────────────────────────────────────────

// Logs a turn to EEPROM and streams it to Overseer via Serial3.
// Call when the car commits to turning into a gap.
void logTurn(uint8_t direction, int usDist)
{
    int count = turnCount();
    if (count >= MAX_ENTRIES)
    {
        Serial.println(F("TurnLog: EEPROM full, oldest entry overwritten"));
        count = MAX_ENTRIES - 1;  // make room by capping (simple overflow handling)
    }

    TurnEntry entry = { direction, (int16_t)usDist };
    int addr = STACK_BASE + (count * ENTRY_SIZE);
    EEPROM.put(addr, entry);
    setTurnCount(count + 1);

    // Stream to Overseer for SD logging
    // Format: "TURN,<direction>,<usDist>\n"
    Serial3.print(F("TURN,"));
    Serial3.print(direction);
    Serial3.print(',');
    Serial3.println(usDist);

    #ifdef DEBUG
        Serial.print(F("TurnLog push: dir="));
        Serial.print(direction);
        Serial.print(F(" dist="));
        Serial.println(usDist);
    #endif
}

// Retrieves and removes the most recent turn entry.
// Returns false if the stack is empty (nothing to retrace).
bool popTurn(TurnEntry& out)
{
    int count = turnCount();
    if (count <= 0) return false;

    int addr = STACK_BASE + ((count - 1) * ENTRY_SIZE);
    EEPROM.get(addr, out);
    setTurnCount(count - 1);

    #ifdef DEBUG
        Serial.print(F("TurnLog pop: dir="));
        Serial.print(out.direction);
        Serial.print(F(" dist="));
        Serial.println(out.usDist);
    #endif

    return true;
}

// Clears the turn log (resets stack pointer — does not erase EEPROM data).
void clearTurnLog()
{
    setTurnCount(0);
    Serial3.println(F("CLEAR"));
}

#endif
