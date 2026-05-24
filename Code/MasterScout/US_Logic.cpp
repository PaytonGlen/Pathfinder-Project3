/* 
    This will be where the code that processes the ultrasonic sensor logic lives.
*/

#include <IrSensor.h>
#include <Pins.h>
#include <NetworkVars.h>


int readDistanceCm(SensorDirection& s) {
  digitalWrite(s.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(s.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(s.trigPin, LOW);

  unsigned long pulseDuration = pulseIn(s.echoPin, HIGH, ECHO_TIMEOUT_US);
  if (pulseDuration == 0) return 0;
  return pulseDuration / 58;
}