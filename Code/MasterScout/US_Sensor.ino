/*
 * UltrasonicSensorTest.ino
 * Simple test sketch for 4 ultrasonic sensors.
 * Prints readings constantly over Serial Monitor.
 * Serial Monitor: 115200 baud
 */

// #1 -- 1 o'clock direction
const byte US_Sensor1_Echo_Pin = 32;
const byte US_Sensor1_Trig_Pin = 33;

// #2 -- 3 o'clock direction
const byte US_Sensor2_Echo_Pin = 26;
const byte US_Sensor2_Trig_Pin = 27;

// #3 -- 9 o'clock direction
const byte US_Sensor3_Echo_Pin = 51;
const byte US_Sensor3_Trig_Pin = 53;

// #4 -- 11 o'clock direction
const byte US_Sensor4_Echo_Pin = 50;
const byte US_Sensor4_Trig_Pin = 52;

// 30000 us allows readings up to roughly 5 meters.
// If pulseIn() times out, readDistanceCm() returns -1.
const unsigned long ECHO_TIMEOUT_US = 30000;

int readDistanceCm(byte trig, byte echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);

  digitalWrite(trig, LOW);

  unsigned long pulse = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);

  if (pulse == 0) return -1; // No echo / timeout

  return pulse / 58;
}

void printReading(const char* label, int value)
{
  Serial.print(label);
  Serial.print(": ");

  if (value == -1)
  {
    Serial.print("NO ECHO");
  }
  else
  {
    Serial.print(value);
    Serial.print(" cm");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(US_Sensor1_Trig_Pin, OUTPUT);
  pinMode(US_Sensor1_Echo_Pin, INPUT);

  pinMode(US_Sensor2_Trig_Pin, OUTPUT);
  pinMode(US_Sensor2_Echo_Pin, INPUT);

  pinMode(US_Sensor3_Trig_Pin, OUTPUT);
  pinMode(US_Sensor3_Echo_Pin, INPUT);

  pinMode(US_Sensor4_Trig_Pin, OUTPUT);
  pinMode(US_Sensor4_Echo_Pin, INPUT);

  Serial.println("=== Ultrasonic Sensor Test Ready ===");
}

void loop()
{
  int us1 = readDistanceCm(US_Sensor1_Trig_Pin, US_Sensor1_Echo_Pin);
  delay(50);

  int us2 = readDistanceCm(US_Sensor2_Trig_Pin, US_Sensor2_Echo_Pin);
  delay(50);

  int us3 = readDistanceCm(US_Sensor3_Trig_Pin, US_Sensor3_Echo_Pin);
  delay(50);

  int us4 = readDistanceCm(US_Sensor4_Trig_Pin, US_Sensor4_Echo_Pin);
  delay(50);

  printReading("1 o'clock", us1);
  Serial.print(" | ");

  printReading("3 o'clock", us2);
  Serial.print(" | ");

  printReading("9 o'clock", us3);
  Serial.print(" | ");

  printReading("11 o'clock", us4);
  Serial.println();

  delay(250);
}
