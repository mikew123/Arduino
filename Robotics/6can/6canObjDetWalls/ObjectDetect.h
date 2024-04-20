// ObjectDetect.h
// Function library to detetc objects using a HC-SR04 
// ultrasonic sensor.
// It is optimized for location soda cans in the 6can competition
//
// Mike Williamson 2/28/2023

#ifndef _ObjectDetect_h
#define _ObjectDetect_h

#include <Arduino.h>
#include <PCA9685.h>  // servo driver
#include <HC_SR04.h>

#define PI 3.14159265

#define SENSOR2SERVO_OFFSET 2.5
#define SERVO2ROBOTX_OFFSET 4.5
#define SERVO2ROBOTY_OFFSET 6.0

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
int detObjState;
int nonObjSeqCnt;
int objSeqCnt0;
int objDegStart0;
int objDegLast0;
int objDistMin0;
bool objDetected;

// TODO multiple objects?
int objDeg;
int objDist;
} objDetVars[2]; // 0=right sensor, 1=left sensor

int wallLocX;
int wallLocY;
int robotLocX; 
int robotLocY; 

int servoZeroNomUs = 1500;
float servoUsPerDeg = 10.2; // calibrated trial and error
int rightSensorServo = 8;
int leftSensorServo = 12;

// the Left and Right sensors move in tandem
int scanDegFwd = -3;
int scanDegRev = 90;
int scanDegInc =  1;
int scanDeg;
int scanDegLast = 0;

// used by both sensor state machines
bool objIntersectsAtScanPt0 = false; // assume false
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