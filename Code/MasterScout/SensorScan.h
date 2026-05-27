#include <Pins.h>
#include <NetworkVars.h>
#include <IrSensor.h>
#include <Calculations.h>
#include <US_Logic.h>
#include <Functions.h>
#include <Wire.h>

// I2C address the pilot Arduino is listening on
const uint8_t PILOT_I2C_ADDRESS = 0x08;

// Uncomment to enable serial debug output
// #define DEBUG

// ─── Structs ────────────────────────────────────────────────────────────────

struct SensorDirection
{
    const char* Label;  // clock position label e.g. "1 o'clock"
    int         trigPin;
    int         echoPin;
    int         irPin;
    uint8_t     bias;   // priority weight: higher = more important direction
};

struct ScanResult
{
    int     usDist;      // ultrasonic distance in cm (0 = timeout / out of range)
    int     irRaw;       // raw IR analogRead value (0–1023, higher = closer)
    bool    blocked;     // true if BOTH sensors confirm obstacle (Schmitt trigger)
    bool    gapDetected; // true if sensor was blocked last scan and distance jumped significantly
    int     prevDist;    // usDist from previous scan — used for PD derivative + gap detection
    uint8_t echoPin;
    uint8_t trigPin;
    uint8_t bias;
};

// ─── Sensor Array ────────────────────────────────────────────────────────────

SensorDirection Sensors[6] =
{
    { "1 o'clock",  US_Sensor1_Trig_Pin, US_Sensor1_Echo_Pin, IR_Sensor1_Pin, 4 },
    { "3 o'clock",  US_Sensor2_Trig_Pin, US_Sensor2_Echo_Pin, IR_Sensor2_Pin, 3 },
    { "5 o'clock",  US_Sensor3_Trig_Pin, US_Sensor3_Echo_Pin, IR_Sensor3_Pin, 1 },
    { "7 o'clock",  US_Sensor4_Trig_Pin, US_Sensor4_Echo_Pin, IR_Sensor4_Pin, 1 },
    { "9 o'clock",  US_Sensor5_Trig_Pin, US_Sensor5_Echo_Pin, IR_Sensor5_Pin, 3 },
    { "11 o'clock", US_Sensor6_Trig_Pin, US_Sensor6_Echo_Pin, IR_Sensor6_Pin, 4 }
};

// Global scan result array — declared here so prevDist persists between loop() iterations
ScanResult readings[6];

// ─── Forward Declarations ────────────────────────────────────────────────────

Motor_Speeds objectDetected(uint8_t direction, ScanResult& reading);
void         ScanAll(SensorDirection sensors[], ScanResult readings[], int count);

// ─── Functions ───────────────────────────────────────────────────────────────

// Reads one sensor pair and returns a populated ScanResult.
// prev is the result from the last scan — used to carry prevDist forward.
ScanResult SensorDetect(SensorDirection& s, ScanResult& prev)
{
    ScanResult r;

    r.prevDist = prev.usDist;
    r.usDist   = readDistanceCm(s.trigPin, s.echoPin);
    r.irRaw    = analogRead(s.irPin);
    r.echoPin  = s.echoPin;
    r.trigPin  = s.trigPin;
    r.bias     = s.bias;

    // Schmitt trigger — hysteresis prevents rapid toggling at the edge of detection range.
    // If already blocked: stay blocked until usDist rises above SCHMITT_CLEAR_CM.
    // If not blocked:     only trigger if usDist drops below SCHMITT_BLOCK_CM.
    // IR must also confirm in both cases to reduce false positives.
    bool irBlocked = (r.irRaw > IR_OBSTACLE_THRESHOLD);
    bool usValid   = (r.usDist > 0);  // 0 = sensor timeout / out of range

    if (prev.blocked)
        r.blocked = usValid && (r.usDist < SCHMITT_CLEAR_CM) && irBlocked;
    else
        r.blocked = usValid && (r.usDist < SCHMITT_BLOCK_CM) && irBlocked;

    // Gap detection — sensor was blocked last scan but distance jumped significantly.
    // Indicates the car has passed the edge of a wall and an opening is present.
    bool wasBlocked   = prev.blocked;
    bool distJumped   = usValid && (r.usDist - r.prevDist) > GAP_THRESHOLD_CM;
    r.gapDetected     = wasBlocked && !r.blocked && distJumped;

    #ifdef DEBUG
        Serial.print("IR threshold: ");  Serial.println(IR_OBSTACLE_THRESHOLD);
        Serial.print("IR raw: ");        Serial.println(r.irRaw);
        Serial.print("US dist: ");       Serial.println(r.usDist);
        Serial.print("Prev dist: ");     Serial.println(r.prevDist);
    #endif

    return r;
}

// Scans all sensors and populates the readings array.
void ScanAll(SensorDirection sensors[], ScanResult readings[], int count)
{
    for (int i = 0; i < count; i++)
    {
        readings[i] = SensorDetect(sensors[i], readings[i]);

        #ifdef DEBUG
            Serial.print("Sensor ");    Serial.print(i);
            Serial.print(" IR raw: ");  Serial.println(readings[i].irRaw);
            Serial.print(" US dist: "); Serial.println(readings[i].usDist);
            Serial.print(" prevDist: "); Serial.println(readings[i].prevDist);
        #endif
    }
}

// Calculates motor speed adjustments for the detected obstacle direction.
Motor_Speeds objectDetected(uint8_t direction, ScanResult& reading)
{
    Motor_Speeds result = calculateSpeeds(direction, PD_Loop(reading.usDist, reading.prevDist));

    #ifdef DEBUG
        Serial.print("Left Motor: ");  Serial.println(result.LEFT_DRIVE_SPEED);
        Serial.print("Right Motor: "); Serial.println(result.RIGHT_DRIVE_SPEED);
    #endif

    return result;
}

// Returns true if any sensor in the array is currently blocked.
// Used by the state machine to transition between TRACKING and AVOIDING.
bool anyBlocked(ScanResult readings[], int count)
{
    for (int i = 0; i < count; i++)
        if (readings[i].blocked) return true;
    return false;
}

// Returns the index of the first sensor reporting a gap, or -1 if none.
// Caller should cross-reference with beaconHeading before committing to a turn.
int gapDirection(ScanResult readings[], int count)
{
    for (int i = 0; i < count; i++)
        if (readings[i].gapDetected) return i;
    return -1;
}

// Selects the highest-priority blocked sensor and reacts.
// Priority: highest bias wins. Tiebreaker: closest obstacle wins.
void direction(ScanResult readings[], int count)
{
    int highestBias  = 0;
    int highestIndex = -1;

    for (int i = 0; i < count; i++)
    {
        if (readings[i].blocked)
        {
            if (readings[i].bias > highestBias)
            {
                highestBias  = readings[i].bias;
                highestIndex = i;
            }
            else if (readings[i].bias == highestBias &&
                     readings[i].usDist < readings[highestIndex].usDist)
            {
                highestIndex = i;
            }
        }
    }

    if (highestIndex != -1)
    {
        Motor_Speeds speeds = objectDetected(highestIndex, readings[highestIndex]);

        // Send left and right motor speeds to the pilot over I2C (2 bytes)
        Wire.beginTransmission(PILOT_I2C_ADDRESS);
        Wire.write(speeds.LEFT_DRIVE_SPEED);
        Wire.write(speeds.RIGHT_DRIVE_SPEED);
        Wire.endTransmission();
    }
}