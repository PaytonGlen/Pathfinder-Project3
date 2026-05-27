// Overseer Arduino (Uno) — receives telemetry from Scout via Serial and logs to SD.
// Parses all packet types, writes CSV rows, and can send parameter updates back.
//
// Wiring:
//   Scout Serial3 TX → Overseer pin 0 (RX)
//   Scout Serial3 RX → Overseer pin 1 (TX)
//   SD card CS       → pin 10 (standard SPI)

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// ─── Pins ────────────────────────────────────────────────────────────────────

const uint8_t SD_CS_PIN = 10;

// ─── Log Files ───────────────────────────────────────────────────────────────

const char TURN_LOG[]   = "turns.csv";
const char SENSOR_LOG[] = "sensors.csv";
const char STATE_LOG[]  = "states.csv";
const char BEACON_LOG[] = "beacon.csv";

// ─── State ───────────────────────────────────────────────────────────────────

static char    rxBuf[64];
static uint8_t rxLen  = 0;
static bool    sdReady = false;

// ─── SD Helpers ──────────────────────────────────────────────────────────────

void appendLine(const char* filename, const char* line)
{
    if (!sdReady) return;
    File f = SD.open(filename, FILE_WRITE);
    if (f)
    {
        f.println(line);
        f.close();
    }
}

void initLogFiles()
{
    if (!SD.exists(TURN_LOG))
    {
        appendLine(TURN_LOG,   "millis,direction,usDist");
    }
    if (!SD.exists(SENSOR_LOG))
    {
        appendLine(SENSOR_LOG, "millis,index,usDist,irRaw,blocked,gapDetected");
    }
    if (!SD.exists(STATE_LOG))
    {
        appendLine(STATE_LOG,  "millis,state");
    }
    if (!SD.exists(BEACON_LOG))
    {
        appendLine(BEACON_LOG, "millis,heading");
    }
}

// ─── Packet Handlers ─────────────────────────────────────────────────────────

void handleTurn(char* data)
{
    // TURN,<direction>,<usDist>
    char line[48];
    snprintf(line, sizeof(line), "%lu,%s", millis(), data);
    appendLine(TURN_LOG, line);
}

void handleSensor(char* data)
{
    // SENSOR,<index>,<usDist>,<irRaw>,<blocked>,<gapDetected>
    char line[64];
    snprintf(line, sizeof(line), "%lu,%s", millis(), data);
    appendLine(SENSOR_LOG, line);
}

void handleState(char* data)
{
    // STATE,<TRACKING|AVOIDING>
    char line[32];
    snprintf(line, sizeof(line), "%lu,%s", millis(), data);
    appendLine(STATE_LOG, line);
}

void handleBeacon(char* data)
{
    // BEACON,<heading>
    char line[32];
    snprintf(line, sizeof(line), "%lu,%s", millis(), data);
    appendLine(BEACON_LOG, line);
}

void handleParams(char* data)
{
    // PARAMS,<Kp>,<Kd> — just echo to Serial for now
    // TODO: evaluate run performance and send back updated KP/KD
    Serial.print(F("Params: "));
    Serial.println(data);
}

// ─── Packet Router ───────────────────────────────────────────────────────────

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
    }
}

// ─── Setup / Loop ────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(9600);
    Serial.println(F("=== Overseer ready ==="));

    sdReady = SD.begin(SD_CS_PIN);
    if (!sdReady)
        Serial.println(F("Overseer: SD init failed"));
    else
    {
        initLogFiles();
        Serial.println(F("Overseer: SD ready"));
    }
}

void loop()
{
    while (Serial.available())
    {
        char c = Serial.read();

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
            rxLen = 0;
        }
    }
}
