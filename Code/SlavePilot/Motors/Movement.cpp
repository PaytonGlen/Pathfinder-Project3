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
if (oneoclk_dtc == true || threeoclk_dtc == true || nineoclk_dtc == true || tenoclk_dtc == true || (fiveoclk_dtc == True && fiveoclk_dist < 50) || (sevenoclk_dtc ==True && sevenoclk_dist < 50))
{
right_Turnspd= K*((1.0/oneoclk_dist) + (1.0/threeoclk_dist) + (1.0/fiveoclk_dist))
left_TurnSpd = K*((1.0/sevenoclk_dist) + (1.0/nineoclk_dist) + (1.0/tenoclk_dist))
}
if (twelveoclk_dtc == true) //A obstacle directly ahead
{
  if (oneoclk_dtc == True && tenoclk_dtc == false) //Left turn so long as left is the only option
  {
  starttime_rturn=millis();
  pivoting_right = true;
    if (millis-startime_rturn < 1500)
    {
    RIGHT_DRIVE_SPEED = 800;
    LEFT_DRIVE_SPEED = -793;
    }
    else
    {
    pivoting_right = false;
  }
  if (tenoclk == True) // Turns right when there is an opening to the right whether one is available to the left or not
  starttime_lturn = millis();
  pivoting_left = true;
  }
    if (millis-starttime_lturn < 1500)
    {
    RIGHT_DRIVE_SPEED = -800;
    LEFT_DRIVE_SPEED = 793;
    }
    else
    {
    pivoting_left = false;
    }
  }
if (right_turnspd > left_turnspd-(left_turnspd*0.05) && righ_turnspd < left_turnspd+(left_turnspd*0.05)
{
left_turnspd = right_turnspd
}
RIGHT_DRIVE_SPEED = RIGHT_DRIVE_SPEED - left_Turnspd
RIGHT_DRIVE_SPEED = RIGHT_DRIVE_SPEED - right_Turnspd
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
