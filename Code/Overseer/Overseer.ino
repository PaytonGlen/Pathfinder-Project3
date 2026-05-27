// Overseer Arduino (Uno) — receives telemetry from Scout via Serial, logs to SD,
// and adaptively tunes sensor bias values based on obstacle frequency,
// collision proximity, turn success rate, and beacon confidence.
//
// Wiring:
//   Scout Serial3 TX → Overseer pin 0 (RX)
//   Scout Serial3 RX → Overseer pin 1 (TX)
//   SD card CS       → pin 10 (standard SPI)

#include "PacketHandler.h"

void setup()
{
    Serial.begin(115200);
    Serial.println(F("=== Overseer ready ==="));

    resetStats();

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
    readSerial();
}
