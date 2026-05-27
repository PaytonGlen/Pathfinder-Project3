// Low-level HM-10 Bluetooth module helper.
// Queries RSSI over a hardware serial port using AT commands.

#ifndef HM10_H
#define HM10_H

#include <Arduino.h>

// Timeout waiting for HM-10 RSSI response (ms)
const unsigned int HM10_TIMEOUT_MS = 200;

// Sends AT+RSSI? to the module and parses the response.
// Returns RSSI as a negative integer (e.g. -65 dBm).
// Returns 0 if the module doesn't respond within HM10_TIMEOUT_MS.
int queryRSSI(HardwareSerial& port)
{
    // Flush any stale data before querying
    while (port.available()) port.read();

    port.print("AT+RSSI?");

    unsigned long start = millis();
    String response = "";

    // Read until we get a full response or timeout
    while (millis() - start < HM10_TIMEOUT_MS)
    {
        if (port.available())
            response += (char)port.read();

        // Response format: "OK+RSSI:-65\r\n"
        if (response.indexOf("OK+RSSI:") >= 0 && response.indexOf('\r') >= 0)
            break;
    }

    int idx = response.indexOf("OK+RSSI:");
    if (idx < 0) return 0;  // no valid response

    // Parse the number after "OK+RSSI:" (will be negative)
    return response.substring(idx + 8).toInt();
}

#endif
