// ObjectDetect.h
// Function library to detetc objects using a HC-SR04 
// ultrasonic sensor.
// It is optimized for location soda cans in the 6can competition
//
// Mike Williamson 2/28/2023
// MRW 3/12/2023 State machine reworked

#ifndef _ObjectDetect_h
#define _ObjectDetect_h

#include <Arduino.h>
#include <PCA9685.h>  // servo driver
#include <HC_SR04.h>

#define PI 3.14159265

#define SENSOR2SERVO_OFFSET 3.5
#define SERVO2ROBOTX_OFFSET 4.5
#define SERVO2ROBOTY_OFFSET 10.0

// detect object states
enum detObjStates{
  detObj_Init0,
  detObj_Init1,
  detObj_ObjStart,  // look for start of object in scan
  detObj_ObjScan,   // look for start of object in scan
  detObj_ObjEnd,    // look for end of object in scan
  detObj_enum       // number of detObj enums  
};

enum {
rightSensor = 0,
leftSensor  = 1
};


class ObjectDetect
{
public:
PCA9685 *pca9685_p;
HC_SR04_BASE *sonicSensors_p;


// globals obj det
struct {
// working per object data - persistant
int detObjState;
int nonObjSeqCnt;
int objSeqCnt;
int objDegStart;
int objDegLast;
int objDegAcc;
int objDistMin;
bool objScanStart;
bool objScanEnd;

// final object info after scan
bool objDetected;
int objDeg;
int objDist;

} objDetVars[2]; // 0=right sensor, 1=left sensor

int wallLocX;
int wallLocY;
int robotLocX; 
int robotLocY; 

// The servoUsPreDeg was calibrated at 0 and 180 Degree
// The right servo seems to have a significant offset at 90
// indicating that the 1500 + and - may not be equal
// Maybe even a non-linear Us to Deg conversion function since that is used most?
// TODO: Try a 0 to 90 and 90 to 180 calibration to get 90 Deg better?
// TODO: Should I only calibrate 0 to 90
int servoZeroNomUs = 1500; // Zero is tweaked with horizontal angle screw
float rightServoUsPerDeg = 10.25; // calibrated trial and error
float leftServoUsPerDeg  = 10.05; // calibrated trial and error
int rightSensorServo = 8;
int leftSensorServo = 12;
int sensorError = 25; // accuracy - repeatability of sensors at the walls at an angle

// the Left and Right sensors move in tandem
int scanDegFwd = -3;
int scanDegRev = 90;
int scanDegInc =  1;
int scanDeg;
int scanDegLast = 0;

// used by both sensor state machines
//bool objIntersectsAtScanPt0 = false; // assume false
bool scanMax2Min = true;
char charMatrix[41][20]; // [x][y]


void initSonicSensors(PCA9685 *pca9685_ptr, HC_SR04_BASE *sonicSensors_ptr, int wallAbsX, int wallAbsY);
int servoDeg2Us(int servo, int degree);
void limitDistance(int &distance);
void positionRangeSensors(int scanDeg);
void rangePolar2Xy(int deg, int dCm, int &xCm, int &yCm);
void scan2Rob(bool sensorL, int scanDeg, int scanDist, 
              int &robDeg, int &robDist, int &robX, int &robY);
void detectObjects(bool scanEnd, int rl, int scanDeg, int scanDist, int x, int y);
bool getObject(int &moveDeg, int &moveDist, int &xCm, int &yCm);

void scanRange(int robotX, int robotY);

void initCharMatrix();
void xyCm2CharMatrix(int xCm, int yCm, char marker);
void printCharMatrix();
void printRangeSensors(int degR, int distR, int xR, int yR, int degL, int distL, int xL, int yL);
void printWhileScanning(int degR, int distR, int xR, int yR, int degL, int distL, int xL, int yL);

};


#endif //_OBJECTDETECT_H_