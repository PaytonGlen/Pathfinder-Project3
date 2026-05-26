#include <Pins.h>
#include <Variables.h>
#include <SensorScan.h>
#include <Calculations.h>
#include <SoftwareSerial.h>


SoftwareSerial BT(10,11);   // RX, TX
ScanResult readings[6];

const byte count = 6;

void setup()
{
    Serial.begin(9600);
    BT.begin(9600);
    Serial.println("HM-10 ready");
}

void loop()
{
     // Phone -> Serial Monitor
    while (BT.available()) Serial.write(BT.read());

    // Serial Monitor -> Phone
    while (Serial.available()) BT.write(Serial.read());

    ScanAll(Sensors, readings, count);
    direction(readings, count);

}