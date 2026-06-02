
#include <Wire.h>

// ─── Pins ─────────────────────────────────────────────────────────────────────

const byte US_Sensor1_Echo_Pin = 32;
const byte US_Sensor1_Trig_Pin = 33;

const byte US_Sensor2_Echo_Pin = 26;
const byte US_Sensor2_Trig_Pin = 27;

const byte US_Sensor3_Echo_Pin = 51;
const byte US_Sensor3_Trig_Pin = 53;

const byte US_Sensor4_Echo_Pin = 50;
const byte US_Sensor4_Trig_Pin = 52;

const uint8_t BLOCK_CONFIRM = 2;
const uint8_t CLEAR_CONFIRM = 2;

// ─── I2C ──────────────────────────────────────────────────────────────────────

const uint8_t PILOT_ADDR = 0x08;
const uint8_t CMD_MOTORS = 0x01;

// ─── Tuning ───────────────────────────────────────────────────────────────────

const uint8_t       BASE_SPEED_L     = 100;
const uint8_t       BASE_SPEED_R     = 102;
const int           SCHMITT_BLOCK_CM = 18;   // triggers blocked when dist drops below this
const int           SCHMITT_CLEAR_CM = 21;   // clears blocked when dist rises above this
const int           PD_TARGET_CM     = 13;   // desired following distance
const int           DEFAULT_KP       = 6;
const int           DEFAULT_KD       = 2;
const unsigned long ECHO_TIMEOUT_US  = 6000; // ~5m max range

// ─── Data structures ──────────────────────────────────────────────────────────

struct SensorConfig
{
    const char* label;
    uint8_t     trig;
    uint8_t     echo;
    uint8_t     bias;
};

struct ScanResult
{
    int  usDist;
    int  prevDist;
    bool blocked;
};

enum CarState { TRACKING, AVOIDING };

// ─── Sensor table ─────────────────────────────────────────────────────────────

const int SENSOR_COUNT = 4;

SensorConfig sensors[SENSOR_COUNT] = {
    { "1 o'clock",  US_Sensor1_Trig_Pin, US_Sensor1_Echo_Pin, 4 },
    { "3 o'clock",  US_Sensor2_Trig_Pin, US_Sensor2_Echo_Pin, 3 },
    { "9 o'clock",  US_Sensor3_Trig_Pin, US_Sensor3_Echo_Pin, 3 },
    { "11 o'clock", US_Sensor4_Trig_Pin, US_Sensor4_Echo_Pin, 4 },
};

ScanResult readings[SENSOR_COUNT];
CarState   currentState = TRACKING;

// ─── Ultrasonic ───────────────────────────────────────────────────────────────

int readDistanceCm(uint8_t trig, uint8_t echo)
{
    digitalWrite(trig, LOW);  delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    unsigned long pulse = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
    if (pulse == 0) return 0;  // timeout = open space
    return (int)(pulse / 58UL);
}

// ─── Scan ─────────────────────────────────────────────────────────────────────

void scanOne(int i)
{
    ScanResult&   r = readings[i];
    SensorConfig& s = sensors[i];

    r.prevDist = r.usDist;
    r.usDist   = readDistanceCm(s.trig, s.echo);

    bool usValid = (r.usDist > 0);

    bool blockCondition = usValid && (r.usDist < SCHMITT_BLOCK_CM);
    bool clearCondition = !usValid || (r.usDist > SCHMITT_CLEAR_CM);

    if (!r.blocked)
    {
        if (blockCondition)
        {
            r.blockHits++;
            if (r.blockHits >= BLOCK_CONFIRM)
            {
                r.blocked = true;
                r.blockHits = 0;
                r.clearHits = 0;
            }
        }
        else
        {
            r.blockHits = 0;
        }
    }
    else
    {
        if (clearCondition)
        {
            r.clearHits++;
            if (r.clearHits >= CLEAR_CONFIRM)
            {
                r.blocked = false;
                r.blockHits = 0;
                r.clearHits = 0;
            }
        }
        else
        {
            r.clearHits = 0;
        }
    }
}

void scanAll()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        scanOne(i);
        delay(20);  // let echo die before firing next sensor — prevents crosstalk
    }
}

bool anyBlocked()
{
    for (int i = 0; i < SENSOR_COUNT; i++) if (readings[i].blocked) return true;
    return false;
}

// ─── PD ───────────────────────────────────────────────────────────────────────
int delta(int firstScan, int secondScan)
{
    int delta = firstScan - secondScan;
    int sign  = (delta >= 0) ? 1 : -1;

    return sign * (delta * delta);
}

int PD_Loop(int current, int previous)
{
    int error      = PD_TARGET_CM - current;
    int derivative = delta(previous, current);
    int correction = (DEFAULT_KP * error) + (DEFAULT_KD * derivative);

    Serial.print(F("[PD]    error="));   Serial.print(error);
    Serial.print(F("  deriv="));         Serial.print(derivative);
    Serial.print(F("  correction="));    Serial.println(correction);

    return correction;
}

/*
int PD_Loop(int current, int previous)
{
    int error = current - PD_TARGET_CM;
    int derivative = current - previous;

    int cubicError = error * abs(error) * abs(error);

    int correction = (DEFAULT_KP * cubicError / 25) + (DEFAULT_KD * derivative);

    correction = constrain(correction, -120, 120);

    Serial.print(F("[PD]    error=")); Serial.print(error);
    Serial.print(F("  cubic=")); Serial.print(cubicError);
    Serial.print(F("  deriv=")); Serial.print(derivative);
    Serial.print(F("  correction=")); Serial.println(correction);

    return correction;
}
*/


// ─── I2C ──────────────────────────────────────────────────────────────────────

void i2cSendMotors(uint8_t ld, uint8_t ls, uint8_t rd, uint8_t rs)
{
    Wire.beginTransmission(PILOT_ADDR);
    Wire.write(CMD_MOTORS);
    Wire.write(ld); Wire.write(ls);
    Wire.write(rd); Wire.write(rs);
    uint8_t err = Wire.endTransmission();
    if (err != 0) { Serial.print(F("[I2C]   err=")); Serial.println(err); }
}


// ─── Drive ────────────────────────────────────────────────────────────────────

void driveForward()
{
    Serial.print(F("[MTR]   L=")); Serial.print(BASE_SPEED_L);
    Serial.print(F("  R="));      Serial.println(BASE_SPEED_R);
    i2cSendMotors(0, BASE_SPEED_L, 0, BASE_SPEED_R);
}

void avoidObstacle()
{
    int highBias = 0, highIdx = -1;

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (!readings[i].blocked) continue;
        if (sensors[i].bias > highBias)
        {
            highBias = sensors[i].bias;
            highIdx  = i;
        }
        else if (highIdx != -1 && sensors[i].bias == highBias &&
                 readings[i].usDist < readings[highIdx].usDist)
        {
            highIdx = i;  // tie-break: closest wins
        }
    }

    if (highIdx == -1) return;

    Serial.print(F("[STATE] AVOIDING | ")); Serial.print(sensors[highIdx].label);
    Serial.print(F("  dist="));            Serial.print(readings[highIdx].usDist);
    Serial.println(F("cm"));

    int correction = PD_Loop(readings[highIdx].usDist, readings[highIdx].prevDist);

    uint8_t l, r;
    if (highIdx <= 1)
    {
        // obstacle on right (1 or 3 o'clock) — steer left
        l = (uint8_t)constrain(BASE_SPEED_L - correction, 0, 255);
        r = (uint8_t)constrain(BASE_SPEED_R + correction, 0, 255);
    }
    else
    {
        // obstacle on left (9 or 11 o'clock) — steer right
        l = (uint8_t)constrain(BASE_SPEED_L + correction, 0, 255);
        r = (uint8_t)constrain(BASE_SPEED_R - correction, 0, 255);
    }

    Serial.print(F("[MTR]   L=")); Serial.print(l);
    Serial.print(F("  R="));      Serial.println(r);

    i2cSendMotors(0, l, 0, r);
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    delay(500);
    Wire.begin();

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        pinMode(sensors[i].trig, OUTPUT);
        pinMode(sensors[i].echo, INPUT);
    }

    memset(readings, 0, sizeof(readings));

    Serial.println(F("=== ScoutPDTest ready ==="));
    Serial.print(F("Kp="));     Serial.print(DEFAULT_KP);
    Serial.print(F("  Kd="));   Serial.print(DEFAULT_KD);
    Serial.print(F("  target=")); Serial.print(PD_TARGET_CM);
    Serial.print(F("cm  block<")); Serial.print(SCHMITT_BLOCK_CM);
    Serial.print(F("cm  clear>")); Serial.print(SCHMITT_CLEAR_CM);
    Serial.println(F("cm"));
    Serial.println();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop()
{
    // 1. Scan
    scanAll();

    // 2. Sensor readings
    Serial.print(F("[SENS]  1pm="));  Serial.print(readings[0].usDist);
    Serial.print(F("cm  3pm="));      Serial.print(readings[1].usDist);
    Serial.print(F("cm  9pm="));      Serial.print(readings[2].usDist);
    Serial.print(F("cm  11pm="));     Serial.print(readings[3].usDist);
    Serial.println(F("cm"));

    // 3. Blocked summary
    Serial.print(F("[BLKD]  "));
    bool anyBlock = false;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (readings[i].blocked)
        {
            Serial.print(sensors[i].label);
            Serial.print(F("  "));
            anyBlock = true;
        }
    }
    if (!anyBlock) Serial.print(F("clear"));
    Serial.println();

    // 4. State + act
    currentState = anyBlocked() ? AVOIDING : TRACKING;

    switch (currentState)
    {
        case TRACKING:
            Serial.println(F("[STATE] TRACKING"));
            driveForward();
            break;
        case AVOIDING:
            avoidObstacle();
            break;
    }

    Serial.println(F("────────────────────────────"));
}