// Pilot Arduino — receives motor, arm, and turn commands from Scout over I2C.
// CMD_MOTORS (0x01): left speed, right speed
// CMD_ARM    (0x02): pan angle
// CMD_TURN   (0x03): direction (0 = right, 1 = left)
//
// Everything inlined into one file — Arduino IDE does not compile .cpp files
// in subdirectories, so the Config/Motors/Arm files are kept for reference only.

#include <Wire.h>
#include <Servo.h>

// ─── Pins ─────────────────────────────────────────────────────────────────────
// This is for the TB6612FNG motor drive

const byte PIN_STBY       = 3;
const byte PIN_PWMA       = 5;   // right track PWM
const byte PIN_PWMB       = 6;   // left track PWM
const byte PIN_AIN1       = 7;   // right track direction
const byte PIN_BIN1       = 8;   // left track direction
const byte PIN_SERVO_PAN  = 10;
const byte PIN_SERVO_TILT = 11;

// needs pins for the US Sensor
const byte US_Trig = 13;
const byte US_Echo = 12;

const int      SCHMITT_BLOCK_CM    = 13;
const int      SCHMITT_CLEAR_CM    = 20;

// ─── Drive Config ─────────────────────────────────────────────────────────────

const bool RIGHT_FORWARD_HIGH = true;
const bool LEFT_FORWARD_HIGH  = true;

// ─── I2C + Command Config ─────────────────────────────────────────────────────

const uint8_t MY_I2C_ADDRESS = 0x08;

const uint8_t CMD_MOTORS = 0x01;
const uint8_t CMD_ARM    = 0x02;
const uint8_t CMD_TURN   = 0x03;
const uint8_t       TURN_SPEED       = 150;
const unsigned long ECHO_TIMEOUT_US  = 2900;  // ~50cm — matches ScoutTest

// ─── Arm Constants ────────────────────────────────────────────────────────────

const int PAN_CENTER = 50;
const int TILT_LEVEL = 160;

// ─── Servo Instances ──────────────────────────────────────────────────────────

static Servo panServo;
static Servo tiltServo;

// ─── Motor Helpers ────────────────────────────────────────────────────────────

void initMotors()
{
    pinMode(PIN_STBY, OUTPUT);
    pinMode(PIN_PWMA, OUTPUT);
    pinMode(PIN_PWMB, OUTPUT);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    analogWrite(PIN_PWMA, 0);
    analogWrite(PIN_PWMB, 0);
    digitalWrite(PIN_STBY, LOW);
}

void enableMotors()  { digitalWrite(PIN_STBY, HIGH); }
void stopMotors()    { analogWrite(PIN_PWMA, 0); analogWrite(PIN_PWMB, 0); }

void setRightTrack(bool forward, uint8_t speed)
{
    digitalWrite(PIN_AIN1, (forward == RIGHT_FORWARD_HIGH) ? HIGH : LOW);
    analogWrite(PIN_PWMA, speed);
}

void setLeftTrack(bool forward, uint8_t speed)
{
    digitalWrite(PIN_BIN1, (forward == LEFT_FORWARD_HIGH) ? HIGH : LOW);
    analogWrite(PIN_PWMB, speed);
}

// ─── Arm Helpers ──────────────────────────────────────────────────────────────

void initScanArm()
{
    panServo.attach(PIN_SERVO_PAN);
    tiltServo.attach(PIN_SERVO_TILT);
    panServo.write(PAN_CENTER);
    tiltServo.write(TILT_LEVEL);
}

void panTo(int angle) { panServo.write(angle); }

// ─── Command Buffer (written in ISR, read in loop) ────────────────────────────

volatile uint8_t pendingCmd  = 0xFF;
volatile uint8_t pendingArg1 = 0;
volatile uint8_t pendingArg2 = 0;
volatile uint8_t pendingArg3 = 0;
volatile uint8_t pendingArg4 = 0;
volatile bool    cmdReady    = false;

// ─── I2C Receive Handler (ISR — no Serial, no delay) ──────────────────────────

void onReceive(int numBytes)
{
    if (!Wire.available()) return;
    pendingCmd  = Wire.read();
    pendingArg1 = Wire.available() ? Wire.read() : 0;
    pendingArg2 = Wire.available() ? Wire.read() : 0;
    pendingArg3 = Wire.available() ? Wire.read() : 0;
    pendingArg4 = Wire.available() ? Wire.read() : 0;
    cmdReady    = true;
}


void processCommand(uint8_t cmd, uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
    if (cmd == CMD_MOTORS)
    {
        // a1=leftDir, a2=leftSpeed, a3=rightDir, a4=rightSpeed
        // dir: 0=forward, 1=reverse
        bool leftFwd  = (a1 == 0);
        bool rightFwd = (a3 == 0);
        setLeftTrack(leftFwd,  a2);
        setRightTrack(rightFwd, a4);
        Serial.print(F("[MOTORS] L="));
        Serial.print(leftFwd ? F("+") : F("-")); Serial.print(a2);
        Serial.print(F("  R="));
        Serial.print(rightFwd ? F("+") : F("-")); Serial.println(a4);
    }
    else if (cmd == CMD_TURN)
    {
        // Scout owns turn timing — Pilot sets tracks and holds until next CMD_MOTORS
        bool right = (a1 == 0);
        if (right) {
            setLeftTrack(true,  TURN_SPEED);
            setRightTrack(false, TURN_SPEED);
        } else {
            setRightTrack(true,  TURN_SPEED);
            setLeftTrack(false, TURN_SPEED);
        }
        Serial.print(F("[TURN] "));
        Serial.println(right ? F("RIGHT") : F("LEFT"));
    }
    else if (cmd == CMD_ARM)
    {
        panTo(a1);
        Serial.print(F("[ARM] angle="));
        Serial.println(a1);
    }
    else
    {
        Serial.print(F("[?] Unknown cmd=0x"));
        Serial.println(cmd, HEX);
    }
}

int readDistanceCm(uint8_t trig, uint8_t echo)
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    unsigned long pulse = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
    if (pulse == 0) return 0;
    return (int)(pulse / 58UL);
}

// Reads the 12 o'clock US sensor on the Pilot.
// Returns true if something is within SCHMITT_BLOCK_CM.
// Stops motors immediately so Scout can issue a turn without crashing.
static bool    crashBlocked = false;
static uint8_t crashCount   = 0;

bool crashAlert()
{
    int dist     = readDistanceCm(US_Trig, US_Echo);
    bool usValid = (dist > 0);

    if (!crashBlocked)
    {
        if (usValid && dist < SCHMITT_BLOCK_CM) crashCount++;
        else crashCount = 0;
        crashBlocked = (crashCount >= 2);
    }
    else
    {
        crashBlocked = usValid && (dist < SCHMITT_CLEAR_CM);
        if (!crashBlocked) crashCount = 0;
    }

    if (crashBlocked)
    {
        stopMotors();
        Serial.print(F("[CRASH] dist="));
        Serial.println(dist);
    }

    return crashBlocked;
}

// ─── Setup / Loop ──────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);

    initMotors();
    enableMotors();
    initScanArm();

    Wire.begin(MY_I2C_ADDRESS);
    Wire.onReceive(onReceive);

    Serial.println(F("=== Pilot ready — waiting for Scout ==="));
    Serial.println(F("I2C: 0x08  |  115200 baud"));
    Serial.println(F("---------------------------------------"));

    pinMode(US_Trig, OUTPUT);
    pinMode(US_Echo, INPUT);
}

void loop()
{
    // Crash sensor always checked first — overrides any incoming motor command
    if (crashAlert()) return;

    if (cmdReady)
    {
        uint8_t cmd = pendingCmd;
        uint8_t a1  = pendingArg1;
        uint8_t a2  = pendingArg2;
        uint8_t a3  = pendingArg3;
        uint8_t a4  = pendingArg4;
        cmdReady = false;

        processCommand(cmd, a1, a2, a3, a4);
    }
}
