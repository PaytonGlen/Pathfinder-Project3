
#include <Wire.h>
#include "Sensors/Mpu6050.h"   // initMpu(), readMpu() — beacon-direction turn uses gyro Z

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
const uint8_t CMD_ARM    = 0x02;   // (handled by Pilot; not used directly here yet)
const uint8_t CMD_TURN   = 0x03;   // dir byte: 0=right, 1=left
const uint8_t CMD_SWEEP  = 0x04;   // tell Pilot to run the BLE beacon sweep

int MILLIS_TIMER = 5;

// ─── Tuning ───────────────────────────────────────────────────────────────────

const uint8_t       BASE_SPEED_L     = 100;
const uint8_t       BASE_SPEED_R     = 102;
const int           SCHMITT_BLOCK_CM = 18;   // triggers blocked when dist drops below this
const int           SCHMITT_CLEAR_CM = 21;   // clears blocked when dist rises above this
const int           PD_TARGET_CM     = 13;   // desired following distance
const int           DEFAULT_KP       = 12;
const int           DEFAULT_KD       = 6;
const unsigned long ECHO_TIMEOUT_US  = 6000;

// ─── Beacon localization / MPU turn ──────────────────────────────────────────

const unsigned long LOCALIZE_TIMEOUT_MS    = 30000;  // if Pilot never returns "done", give up
const unsigned long POLL_INTERVAL_MS       = 250;    // how often to ask Pilot for sweep status
const uint8_t       VERIFY_PEAK_TOL_DEG    = 25;     // verify pass: ±25° of center is OK
const float         GYRO_LSB_PER_DPS       = 131.0f; // MPU6050 default ±250 dps full-scale
const unsigned long MPU_CALIB_SAMPLES      = 200;
const unsigned long MPU_SAMPLE_DELAY_MS    = 2;
// If calibration shows the ESP32-upside-down mount mirrors angle → RSSI,
// flip this to true to negate the turn delta.
const bool          MIRROR_BEACON_ANGLE    = false;

// ─── Data structures ──────────────────────────────────────────────────────────

const int TILT_LEVEL = 160;   // calibrated: arm horizontal

struct SensorConfig
{
    const char* label;
    uint8_t     trig;
    uint8_t     echo;
    uint8_t     bias;
};

struct ScanResult
{
    int           usDist;
    int           prevDist;
    bool          blocked;
    unsigned long blockSince;   // ms at which dist first dropped below BLOCK; 0 = not below
};

enum CarState { TRACKING, AVOIDING };
enum Phase    { LOCALIZING, TURNING, VERIFYING, DRIVING };

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

// Phase machine state
Phase          currentPhase    = LOCALIZING;
unsigned long  phaseStartMs    = 0;
unsigned long  lastPollMs      = 0;
int            bestBeaconAngle = 90;     // 90 = straight ahead; updated when sweep returns
int8_t         lastRssi        = -127;

// MPU yaw state
float          gyroZOffset     = 0.0f;

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



// Per-sensor debounce. Returns true once the sensor has been continuously
// below SCHMITT_BLOCK_CM for at least MILLIS_TIMER ms — gates the blocked flag
// from flapping on transient echoes.
bool CurrentTimePassed(int i)
{
    ScanResult& r = readings[i];
    bool belowBlock = (r.usDist > 0) && (r.usDist < SCHMITT_BLOCK_CM);

    if (belowBlock)
    {
        if (r.blockSince == 0) r.blockSince = millis();
        return (millis() - r.blockSince) >= (unsigned long)MILLIS_TIMER;
    }

    r.blockSince = 0;
    return false;
}

// ─── Scan ─────────────────────────────────────────────────────────────────────

// usSensor reading -> 40
// when reading < 60, the obstacle detected is true
// obstacle detected = true
// if obstacle detected = true for 3 seconds, then make blocked = true

// if (obstacleDetected) && (millis - blockstart > 50)

// unsigned long lastMotionTime = 0; // Stores time of most recent trigger
// bool occupiedState = false;       // Tracks whether sensor is triggered
// unsigned long currentTime = millis();     // Current time
// lastMotionTime = currentTime; // Update last detected motion time
/*

- take reading
- take currentTime

- set State

currentTime = millis;

- if (State == HIGH)
    {
        lastMotionTime = currentTime;    

        if (State && lastMotionTime - currentTime > 50)
            {
                blocked = true;
            }
    }

   
    x = CurrentTime();

    if (x > 5 && condition)


  if (buttonState == HIGH) {
    lastMotionTime = currentTime; // Update last detected motion time

    // Only switch to occupied once until timeout resets it
    if (!occupiedState) {
      occupiedState = true;

      signServo.write(occupiedAngle); // Point to "occupied"

      tone(buzzerPin, 1000); // Play warning tone
      delay(500);            // Sound for 0.5 seconds
      noTone(buzzerPin);     // Stop tone
    }
  }
*/

void scanOne(int i)
{
    ScanResult&   r = readings[i];
    SensorConfig& s = sensors[i];

    r.prevDist = r.usDist;
    r.usDist   = readDistanceCm(s.trig, s.echo);

    bool usValid = (r.usDist > 0);

    // Block: must be below BLOCK_CM and the debounce has elapsed (CurrentTimePassed).
    bool blockCondition = usValid && r.usDist < SCHMITT_BLOCK_CM && CurrentTimePassed(i);

    if (!r.blocked && blockCondition)
    {
        r.blocked = true;
    }
    // Clear: when above CLEAR_CM, reset blocked + debounce timer.
    else if (r.blocked && usValid && r.usDist > SCHMITT_CLEAR_CM)
    {
        r.blocked    = false;
        r.blockSince = 0;
    }
}

void scanAll()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        scanOne(i);
        delay(20);  // let echo die before firing next sensor
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

    // Serial.print(F("[PD]    error="));   
    // Serial.print(error);
    // Serial.print(F("  deriv="));         
    // Serial.print(derivative);
    // Serial.print(F("  correction="));    
    // Serial.println(correction);

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

// Single-byte command (used for CMD_SWEEP).
void i2cSendByte(uint8_t cmd)
{
    Wire.beginTransmission(PILOT_ADDR);
    Wire.write(cmd);
    Wire.endTransmission();
}

// CMD_TURN with direction byte (0=right, 1=left). Pilot holds the turn at
// TURN_SPEED until next CMD_MOTORS arrives, which is why turnByDegrees() ends
// with i2cSendStop().
void i2cSendTurn(uint8_t dir)
{
    Wire.beginTransmission(PILOT_ADDR);
    Wire.write(CMD_TURN);
    Wire.write(dir);
    Wire.endTransmission();
}

void i2cSendStop()
{
    i2cSendMotors(0, 0, 0, 0);
}

// Pulls sweep status from Pilot: [doneFlag, bestAngle, latestRssi].
// Returns false on I²C short-read (no response, or Pilot's onRequest not set up).
bool readSweepStatus(uint8_t &done, uint8_t &bestAngle, int8_t &rssi)
{
    uint8_t got = Wire.requestFrom((uint8_t)PILOT_ADDR, (uint8_t)3);
    if (got < 3) return false;

    done      = Wire.read();
    bestAngle = Wire.read();
    rssi      = (int8_t)Wire.read();
    return true;
}

// ─── MPU helpers ──────────────────────────────────────────────────────────────

// Sample gyro Z with the tank stationary for MPU_CALIB_SAMPLES to get the bias.
// readYawRateDps() subtracts this so a stationary gyro reads ~0 dps.
void calibrateMpu()
{
    long sum = 0;
    for (unsigned long n = 0; n < MPU_CALIB_SAMPLES; n++)
    {
        short ax,ay,az,gx,gy,gz;
        readMpu(ax,ay,az,gx,gy,gz);
        sum += gz;
        delay(MPU_SAMPLE_DELAY_MS);
    }
    gyroZOffset = (float)sum / (float)MPU_CALIB_SAMPLES;
}

float readYawRateDps()
{
    short ax,ay,az,gx,gy,gz;
    readMpu(ax,ay,az,gx,gy,gz);
    return ((float)gz - gyroZOffset) / GYRO_LSB_PER_DPS;
}

// Blocking turn by integrating gyro Z until |heading| reaches |deg|.
// Positive deg = right turn (CMD_TURN dir=0), negative = left (dir=1).
void turnByDegrees(int deg)
{
    if (deg == 0) return;

    bool rightTurn = (deg > 0);
    i2cSendTurn(rightTurn ? 0 : 1);

    float target  = fabs((float)deg);
    float heading = 0.0f;
    unsigned long lastMs = millis();

    while (fabs(heading) < target)
    {
        unsigned long now = millis();
        float dt = (now - lastMs) / 1000.0f;
        lastMs = now;
        float dps = readYawRateDps();
        heading += dps * dt;
        delay(MPU_SAMPLE_DELAY_MS);
    }

    i2cSendStop();
    Serial.print(F("[MPU]   turn done. heading=")); Serial.println(heading);
}

// ─── Phase handlers ──────────────────────────────────────────────────────────

void handleLocalizing()
{
    if (millis() - phaseStartMs > LOCALIZE_TIMEOUT_MS)
    {
        Serial.println(F("[BEACON] Localize timeout — falling through to DRIVING"));
        currentPhase = DRIVING;
        phaseStartMs = millis();
        return;
    }

    if (millis() - lastPollMs < POLL_INTERVAL_MS) return;
    lastPollMs = millis();

    uint8_t done = 0, ang = 90;
    int8_t  rssi = -127;
    if (!readSweepStatus(done, ang, rssi)) return;  // Pilot not ready yet

    lastRssi = rssi;
    if (done)
    {
        bestBeaconAngle = ang;
        Serial.print(F("[BEACON] sweep done. bestAngle=")); Serial.print(bestBeaconAngle);
        Serial.print(F("°  rssi=")); Serial.println(lastRssi);
        currentPhase = TURNING;
        phaseStartMs = millis();
    }
}

void handleTurning()
{
    int deltaDeg = bestBeaconAngle - 90;          // +ve = right of center
    if (MIRROR_BEACON_ANGLE) deltaDeg = -deltaDeg;

    Serial.print(F("[TURN] target delta=")); Serial.print(deltaDeg); Serial.println(F("°"));

    turnByDegrees(deltaDeg);

    // Ask Pilot for a confirmation sweep, then move into VERIFYING.
    i2cSendByte(CMD_SWEEP);
    currentPhase = VERIFYING;
    phaseStartMs = millis();
}

void handleVerifying()
{
    if (millis() - phaseStartMs > LOCALIZE_TIMEOUT_MS)
    {
        Serial.println(F("[VERIFY] timeout — engaging DRIVING anyway"));
        currentPhase = DRIVING;
        return;
    }

    if (millis() - lastPollMs < POLL_INTERVAL_MS) return;
    lastPollMs = millis();

    uint8_t done = 0, ang = 90;
    int8_t  rssi = -127;
    if (!readSweepStatus(done, ang, rssi)) return;

    if (done)
    {
        int err = abs((int)ang - 90);
        Serial.print(F("[VERIFY] peak=")); Serial.print(ang);
        Serial.print(F("° err="));         Serial.println(err);

        if (err <= (int)VERIFY_PEAK_TOL_DEG)
        {
            Serial.println(F("[VERIFY] Heading good. Engaging DRIVING."));
            currentPhase = DRIVING;
        }
        else
        {
            Serial.println(F("[VERIFY] Off-center — re-turning."));
            bestBeaconAngle = ang;
            currentPhase    = TURNING;
            phaseStartMs    = millis();
        }
    }
}


// ─── Drive ────────────────────────────────────────────────────────────────────

void driveForward()
{
    // Serial.print(F("[MTR]   L=")); Serial.print(BASE_SPEED_L);
    // Serial.print(F("  R="));      Serial.println(BASE_SPEED_R);
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

    // Serial.print(F("[STATE] AVOIDING | ")); Serial.print(sensors[highIdx].label);
    // Serial.print(F("  dist="));            Serial.print(readings[highIdx].usDist);
    // Serial.println(F("cm"));

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

    // Serial.print(F("[MTR]   L=")); Serial.print(l);
    // Serial.print(F("  R="));      Serial.println(r);

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

    // MPU init + calibrate. Tank MUST be stationary during calibrateMpu().
    initMpu();
    Serial.println(F("[MPU] calibrating gyro offset (hold still)..."));
    calibrateMpu();
    Serial.print  (F("[MPU] gyroZOffset=")); Serial.println(gyroZOffset);

    // Kick off Pilot's beacon localization sweep.
    Serial.println(F("[BEACON] requesting sweep..."));
    i2cSendByte(CMD_SWEEP);
    phaseStartMs = millis();
    lastPollMs   = 0;

/*
    Serial.println(F("=== ScoutPDTest ready ==="));
    Serial.print(F("Kp="));     Serial.print(DEFAULT_KP);
    Serial.print(F("  Kd="));   Serial.print(DEFAULT_KD);
    Serial.print(F("  target=")); Serial.print(PD_TARGET_CM);
    Serial.print(F("cm  block<")); Serial.print(SCHMITT_BLOCK_CM);
    Serial.print(F("cm  clear>")); Serial.print(SCHMITT_CLEAR_CM);
    Serial.println(F("cm"));
    Serial.println();
    */
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop()
{
    // Phase machine: only DRIVING falls through to the original obstacle-avoidance body.
    switch (currentPhase)
    {
        case LOCALIZING: handleLocalizing(); return;
        case TURNING:    handleTurning();    return;
        case VERIFYING:  handleVerifying();  return;
        case DRIVING:    /* fall through to original loop body below */ break;
    }

    // ─── Original DRIVING-phase body (unchanged) ─────────────────────────────

    // 1. Scan
    scanAll();

    /*
    // 2. Sensor readings
    Serial.print(F("[SENS]  1pm="));  Serial.print(readings[0].usDist);
    Serial.print(F("cm  3pm="));      Serial.print(readings[1].usDist);
    Serial.print(F("cm  9pm="));      Serial.print(readings[2].usDist);
    Serial.print(F("cm  11pm="));     Serial.print(readings[3].usDist);
    Serial.println(F("cm"));
    */

    // 3. Blocked summary
    // Serial.print(F("[BLKD]  "));
    bool anyBlock = false;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (readings[i].blocked)
        {
            // Serial.print(sensors[i].label);
            // Serial.print(F("  "));
            anyBlock = true;
        }
    }
    // if (!anyBlock) Serial.print(F("clear"));
    // Serial.println();

    // 4. State + act
    currentState = anyBlocked() ? AVOIDING : TRACKING;

    switch (currentState)
    {
        case TRACKING:
            // Serial.println(F("[STATE] TRACKING"));
            driveForward();
            break;
        case AVOIDING:
            avoidObstacle();
            break;
    }

    // Serial.println(F("────────────────────────────"));
}