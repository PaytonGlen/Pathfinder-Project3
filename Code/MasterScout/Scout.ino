#include <Pins.h>
#include <Variables.h>
#include <SensorScan.h>
#include <Calculations.h>

ScanResult readings[6];

const byte count = 6;

void loop()
{

    ScanAll(Sensors, readings, count);

    direction(readings, count);

}