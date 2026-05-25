#include <Pins.h>
#include <NetworkVars.h>
#include <IrSensor.h>
#include <Calculations.h>

struct SensorDirection
{
    // points to the first char of the string (array of char's)
    const char* Label; 

    // each grouping will have a trigger pin, echo pin, and IR pin.
    int trigPin;
    int echoPin;
    int irPin;
};

// make an array of all of the sensor groups to iterate through them easily
SensorDirection Sensors[6] = 
{
    { "1 o'clock", US_Sensor1_Trig_Pin, US_Sensor1_Echo_Pin, IR_Sensor1_Pin},
    { "3 o'clock", US_Sensor2_Trig_Pin, US_Sensor2_Echo_Pin, IR_Sensor2_Pin},
    { "5 o'clock", US_Sensor3_Trig_Pin, US_Sensor3_Echo_Pin, IR_Sensor3_Pin},
    { "7 o'clock", US_Sensor4_Trig_Pin, US_Sensor4_Echo_Pin, IR_Sensor4_Pin},
    { "9 o'clock", US_Sensor5_Trig_Pin, US_Sensor5_Echo_Pin, IR_Sensor5_Pin},
    { "11 o'clock", US_Sensor6_Trig_Pin, US_Sensor6_Echo_Pin, IR_Sensor6_Pin}
};

// create a structure to hold the result of the scans.
// The goal should be to make an array of these scan results, or maybe a struct
struct ScanResult {
  int  usDist;     // shoulder US distance in cm (0 means timeout = far)
  int  irRaw;      // raw IR analogRead (0-1023, higher = closer)
  bool blocked;    // true if both sensors see something close. Redundancy to keep system from seeing ghosts
  int prevDist;    // this variable needs to hold the previous scan
};

// returns a scan result struct. Accepts a struct as argument
ScanResult SensorDetect(SensorDirection& s, ScanResult& prev)
{
    // create a struct within this function to hold values
    ScanResult r;

    r.prevDist = prev.usDist;  // grab last scan's distance
    r.usDist = readDistanceCm(s);  // now take the new reading

    r.irRaw = analogRead(s.irPin);

    bool usBlocked = (r.usDist > 0 && r.usDist < CLEAR_DISTANCE_CM);
    bool irBlocked = (r.irRaw > IR_OBSTACLE_THRESHOLD);

    r.blocked = usBlocked && irBlocked;

    // return the newly made struct with the new data
    return r;
};

void objectDetected(byte direction, int usDist, int irRaw, int prevDist);
void ScanAll(SensorDirection Sensors[], ScanResult readings[], int count);

// when drivingForward(), make sure to call this with count = 2
void direction(ScanResult readings[], int count)
{
    for (int i = 0; i < count; i++) {
    if (readings[i].blocked) {
        // something is close in direction i — react
        objectDetected(i, readings[i].usDist, readings[i].irRaw, readings[i].prevDist);
    }
  }
};

void objectDetected(byte direction, int usDist, int irRaw, int prevDist)
{
    // theres a problem that this function only receives one scan
    int firstScan;  // this should take the average
    int secondScan; // this should take the average of the second scan

    // these are to hold the correction obtained from running calculateSpeeds
    int case1;
    int case2;
    int case3;
    int case4;
    int case5;

    // needs to do something when object is detected at 11 o clock
    // so it should be called like: objectedDetected(11 o clock)
    
    switch (direction)
    {
        case 0:     // this is 1 o'clock direction
            calculateSpeeds();
            break;
        case 1:
            calculateSpeeds();
            break;
        case 2:
            LEFT_MOTOR_SPEED;
            RIGHT_MOTOR_SPEED;
            break;
        case 3:
            LEFT_MOTOR_SPEED;
            RIGHT_MOTOR_SPEED;
            break;
        case 4:
            LEFT_MOTOR_SPEED;
            RIGHT_MOTOR_SPEED;
            break;
        case 5:
            LEFT_MOTOR_SPEED;       // keep at same speed
            RIGHT_MOTOR_SPEED;      // reduce right motor by 25% - turn right
            break;
    }
};

void ScanAll(SensorDirection Sensors[], ScanResult readings[], int count)
{
    for (int i = 0; i < count; i++)
    {
        readings[i] = SensorDetect(Sensors[i], readings[i]);

        /*
        readings[0] = SensorDetect(Sensors[0], readings[0])
        SensorDetect returns a ScanResult structure. { usDist, irRaw, blocked, prevDist}
        */
    }
};

