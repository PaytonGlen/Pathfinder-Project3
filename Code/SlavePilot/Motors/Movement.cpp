// Shared movement helper implementations.

#include "Movement.h"
#include "MotorControl.h"
#include "../Config/DriveConfig.h"

void driveForward(unsigned long durationMs) {
  setRightTrack(true, RIGHT_DRIVE_SPEED);
  setLeftTrack(true, LEFT_DRIVE_SPEED);
  delay(durationMs);
  stopMotors();
}

void reverse(unsigned long durationMs) {
  setRightTrack(false, RIGHT_DRIVE_SPEED);
  setLeftTrack(false, LEFT_DRIVE_SPEED);
  delay(durationMs);
  stopMotors();
}

// This might have an issue where the speeds continuosly decrease as new obstacles are found without returning to the original speed
/*

if (oneoclk_dtc == True || threeoclk_dtc == True || nineoclk_dtc == True || tenoclk_dtc == True || 
(fiveoclk_dtc == True && fiveoclk_dist < 50) || (sevenoclk_dtc ==True && sevenoclk_dist < 50)
{
right_Turnspd= K*((1/oneoclk_dist) + (1/threeoclk_dist) + (1/fiveoclk_dist))
left_TurnSpd = K*((1/sevenoclk_dist) + (1/nineoclk_dist) + (1/tenoclk_dist))
RIGHT_DRIVE_SPEED = RIGHT_DRIVE_SPEED - left_Turnspd
RIGHT_DRIVE_SPEED = RIGHT_DRIVE_SPEED - right_Turnspd
}

*/



/*void turnRight(unsigned long durationMs) {
  setLeftTrack(true, TURN_SPEED);
  setRightTrack(false, TURN_SPEED);
  delay(durationMs);
  stopMotors();
}

void turnLeft(unsigned long durationMs) {
  setLeftTrack(false, TURN_SPEED);
  setRightTrack(true, TURN_SPEED);
  delay(durationMs);
  stopMotors();
}
*/
