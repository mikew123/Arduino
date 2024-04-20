// 6canSensorLowLRObjDet.ino

// Adding IMU to offset direction drift while driving
// also to control the rotation to be precise +-90 or +-180 degrees
// MRW 2/9/2023 moved lots of code to libraries to help manage code size
// MRW 2/12/2023 Added HC-SR04 range sensor 
// MRW 2/16/2023 Moved sensors low and on left right of body
// MRW 2/20/2023 Started working on objectr detection

#include <Arduino.h>

// My libraries
#include <NanoBleLed.h>
#include <NanoBleSensors.h>
#include <MecanumWheels.h>

// Libraries from Arduino
#include <HC_SR04.h>
#include <PCA9685.h>  // servo driver
#include <mbed.h>  // rtos threads and semaphores
#include <string>

// Instance my libraries 
// TODO: can some of these be move to the library?
NanoBleLed led;
NanoBleSensors sensors;
MecanumWheels wheels;

// Instance servo controller
PCA9685 pca9685;
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ

// Instance and Map range sensor trigger and echo to BLE Sense pins
//HC_SR04<A6> sensor(A7); // right sensor with echo A6 and trigger A7 pins
HC_SR04_BASE *Slaves[] = { new HC_SR04<A3>() };
HC_SR04<A6> sonicSensors(A7, Slaves, 1);


// prints values to serial port after initialization
rtos::Thread serialHandlingThread;
void serialHandling();

// Create RTOS threads
// TODO: How to move these to my sensor library files?
// TODO: Should some be combined into a single thread
// Magnometer sensor
rtos::Thread MAGSensorHandlingThread;
void MAGSensorHandling() {sensors.MAGSensorHandling();}
// Accellerometer and Gyroscope sensors
rtos::Thread AGSensorHandlingThread;
void AGSensorHandling() {sensors.AGSensorHandling();}
// Temp sensor
rtos::Thread HTSSensorHandlingThread;
void HTSSensorHandling() {sensors.HTSSensorHandling();}


bool serialOK = 0; // assume serial port is OK until time out waiting
void initSerial()
{
  Serial.begin(9600);
  int i;
  for(i=0; !Serial; i++) {if (i >= 5000) break; delay(1);}
  if (i < 5000) serialOK=1;  
  if(serialOK) Serial.println("initSerial OK");  
  
}

void initPca9685(){
  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);
}

void initSonicSensors()
{
//  sensor.beginAsync();  
  sonicSensors.beginAsync();
//  sonicSensors.startAsync(0);

  // init sensor servoss to point straight and level
  pca9685.setChannelServoPulseDuration(8, 1500); // right
  pca9685.setChannelServoPulseDuration(12, 1500);// left
}

int motion; // global for printing
float speed;


int motions[] = {
  stop,
  fwd,
  rev,
  left,
  right,
  fwd_left,
  rev_right,
  rev_left,
  fwd_right
};

void motionLeds(int motion)
{
    // LED have different color for each motion, some duplicates
    switch(motion) {
    case stop:      led.rgbLed(1,0,0);break; //red
    case fwd:       led.rgbLed(0,1,0);break; // green
    case rev:       led.rgbLed(0,0,1);break; // blue
    case right:     led.rgbLed(1,1,0);break;
    case left:      led.rgbLed(1,0,1);break;
    case fwd_right: led.rgbLed(1,0,0);break;
    case fwd_left:  led.rgbLed(0,1,0);break;
    case rev_right: led.rgbLed(0,1,1);break;
    case rev_left:  led.rgbLed(1,1,1);break;
    case cwise:     led.rgbLed(0,1,0);break;
    case ccwise:    led.rgbLed(0,0,1);break;
    }  
}

void setup()
{
  initSerial();

  led.initLed();
  led.ledOn();
  led.rgbLed(1,1,1); // white

  initPca9685();
  initSonicSensors();

  wheels.initWheels(&pca9685);
  sensors.initSensors(serialOK);   

  // Start threads
  // TODO: How to move these to my sensor library files?
  MAGSensorHandlingThread.start(MAGSensorHandling);
  AGSensorHandlingThread.start(AGSensorHandling);
  HTSSensorHandlingThread.start(HTSSensorHandling);

  serialHandlingThread.start(serialHandling);

  motion  = stop;
  motionLeds(motion);
  // calibrate IMU AG when stopped
  sensors.calibrateAG = motion==stop;

  led.ledOff();
}



// Range sensor functions
int servoZeroNomUs = 1500;
float servoUsPerDeg = 10.2; // calibrated trial and error

int rightSensorServo = 8;
int leftSensorServo = 12;

// the Left and Right sensors move in tandem
int scanDegFwd = -3;
int scanDegRev = 90;
int scanDegInc =  1;
int scanDeg;

int numSamples=1; // number of samples per servo angle


int servoDeg2Us(int servo, int degree)
{
  int usec = servoZeroNomUs + (servoUsPerDeg * degree) ;
  return(usec);
}

// position ultrasonic range sensors
// sensors mounted on the side right(8) 0 to +180, left(12) 0 to -180
// servos are -90 to +90
int scanDegLast = 0;
//int leftDegLast = 0;
void positionRangeSensors(int scanDeg)
{

  // adjust degrees to conform to servo degrees
  scanDeg -= 90;

  // move the range sensors 1 degree at a time to the desired position
  for(int deg = scanDegLast;  
     ((scanDeg<scanDegLast)?(scanDeg<=deg):(scanDeg>=deg));
     (((scanDeg<scanDegLast)?(deg--):(deg++))))
  {
    pca9685.setChannelServoPulseDuration(rightSensorServo, servoDeg2Us(rightSensorServo, -deg)); // right
    pca9685.setChannelServoPulseDuration(leftSensorServo,  servoDeg2Us(leftSensorServo,  +deg)); // left
    delay(5); // slow down movement, less jerky ???
  }
  scanDegLast = scanDeg;

}

void printRangeSensors(int degR, int distR, int xR, int yR, int degL, int distL, int xL, int yL)
{

  Serial.print(" Right: ");
  Serial.print(degR); Serial.print(", ");
  Serial.print(distR); Serial.print(", ");
  Serial.print(xR); Serial.print(", ");
  Serial.print(yR); Serial.print("");
  Serial.print(" Left: ");
  Serial.print(degL); Serial.print(", ");
  Serial.print(distL); Serial.print(", ");
  Serial.print(xL); Serial.print(", ");
  Serial.print(yL); Serial.print("\n");
}

#define PI 3.14159265
void rangePolar2Xy(int deg, int dCm, int &xCm, int &yCm)
{
  yCm = dCm * cos(deg*PI/180);
  xCm = dCm * sin(deg*PI/180);
}
// translate scanned degrees,distance to robot degrees,distance
// The robot reference point is the front center of the robot chassis
// The scanned reference point is at the sensor
// Physical fixed offsets:
#define SENSOR2SERVO_OFFSET 2.5
#define SERVO2ROBOTX_OFFSET 4.5
#define SERVO2ROBOTY_OFFSET 6.0
void scan2Rob(bool sensorL, int scanDeg, int scanDist, 
              int &robDeg, int &robDist, int &robX, int &robY)
{
  double X;
  double Y;
  double D;
  if(sensorL) // Scan using left sensor  (+5 to -185)
    X = ((scanDist + SENSOR2SERVO_OFFSET) * sin(scanDeg*PI/180)) - SERVO2ROBOTX_OFFSET;
  else // scan using right sensor (-5 to +185)
    X = ((scanDist + SENSOR2SERVO_OFFSET) * sin(scanDeg*PI/180)) + SERVO2ROBOTX_OFFSET;

  Y = ((scanDist + SENSOR2SERVO_OFFSET) * cos(scanDeg*PI/180)) - SERVO2ROBOTY_OFFSET;
  D = sqrt(X*X + Y*Y);
  if((scanDeg>45) && (scanDeg<120))
    robDeg = acos(Y/D)*180/PI;
  else if((scanDeg<-45) && (scanDeg>-120))
    robDeg = -1*(acos(Y/D)*180/PI);
  else
    // TODO: calc >120 properly (sin valid for -90 to +90)
    robDeg = asin(X/D)*180/PI;
  robDist = D;  
  robX = X;
  robY = Y;
}


char charMatrix[41][20]; // [x][y]
// initialize to space chars (blank)
void initCharMatrix()
{
  for(int x=0; x<41; x++)
  {
    for(int y=0; y<20; y++)
    {
      charMatrix[x][y] = '-';
    }
  }
}

void xyCm2CharMatrix(int xCm, int yCm, char marker)
{
  //x -400 to +400 cm to 0 to 40 matrix table  
  //each x table entry is 20 cm, x -10 to 10 is table entry x=20 (0)
  int x = ((xCm + 400)/20.0 + 0.5);
  int y = (yCm/20.0);
  charMatrix[x][y] = marker;
}

void printCharMatrix()
{
  
  for(int y=19; y>=0; y--)
  {
    for(int x=0; x<41; x++)
    {
      Serial.print(charMatrix[x][y]);
      Serial.print(' ');
    }
    Serial.print('\n');
  }
  // sensor/robot position
  Serial.println("                                        ^"); // sensor position
  
}

// Limit distance to range of sensor
void limitDistance(int &distance)
{
  if(distance > 395) distance = 395;
}


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

// used by both sensor state machines
bool objIntersectsAtScanPt0 = false; // assume false

// Send coordinates in sequence, 
// reliable scan dist = 1M and walls at a out 1M 
// the object will be sensed for about 30 degrees of scan
// but not between 45 and 120 degrees, the wall echo overpowers
// determine center of scan degrees
// the object scan starts at first distance < max (about 100 cm)
// and ends at next distance < max (about 100 cm)
// the object scan must have at least 5 points
// After the first object is detected no other objects are considered
// If an object is sensed on both sensors at start or end of scan
// the scans intersect so extend the scan detection 
// set global detected object degree and distance (min dist)
// 2nd arg selects 0=right, 1=left
// 1st arg used to determine last scan point - see if 2 sensor scans intersect at end
void detectObjects(bool scanEnd, int rl, int scanDeg, int scanDist, int x, int y)
{

  bool objScanPt0 = false;
  
  switch(objDetVars[rl].detObjState)
  {
  case detObj_Init0:
    objDetVars[rl].objDetected = false;
    // both sensors at 1st scan point
    objScanPt0 = true;
    objIntersectsAtScanPt0 = false;
    // drop through to init1
  case detObj_Init1:
    objDetVars[rl].nonObjSeqCnt = 0;
    objDetVars[rl].detObjState = detObj_ObjStart;
    // drop through to start
  case detObj_ObjStart: // look for start of object in scan
    if(scanDist<95 && x<95 && y<95)
    { // consider data point for object
      objDetVars[rl].objSeqCnt0 = 1;
      objDetVars[rl].objDegStart0 = scanDeg;
      objDetVars[rl].objDegLast0 = scanDeg;
      objDetVars[rl].objDistMin0 = scanDist;
      objDetVars[rl].detObjState = detObj_ObjScan;
      // determine if both sensors det same obj at 1st scan point
      if(objScanPt0 && (objDetVars[rl?0:1].detObjState == detObj_ObjScan))
      { // first scan data point of other sensor has possible object
        // test to see if 1st scan dist of the objects are at the same distance
//        if(abs(objDetVars[rl?0:1].objDistMin0 - objDetVars[rl].objDistMin0) < 10)
//        {
          objIntersectsAtScanPt0 = true;
//        }
      }
    }
    objScanPt0 = false;
    break;

// TODO: improve combining RL scans when first or last scan point appears to be nonobject
  case detObj_ObjScan: // process object data points while scanning   
    objDetVars[rl].objDetected = false; // assume no object is detected yet
    if(scanDist<95 && x<95 && y<95)
    { // consider for object
      objDetVars[rl].nonObjSeqCnt = 0;
      if(abs(scanDist - objDetVars[rl].objDistMin0) < 10)  
      { // distance to min is small, should be the same object 
        if(scanDist < objDetVars[rl].objDistMin0) 
        { // save minimum object distance point during scan
          objDetVars[rl].objDistMin0 = scanDist;
        }
        // keep degrees of scan, in case it is the last point        
        objDetVars[rl].objDegLast0 = scanDeg;
        objDetVars[rl].objSeqCnt0++;
        // determine if object is detected at the end of scan
        if((scanEnd == true) && (objDetVars[rl].objSeqCnt0 >= 5))
        {      
          objDetVars[rl].objDetected = true;
        }
      }
      // else different 2nd object??
    }
    else 
    { // not an object - assume end of object scan after more than 5 consecutive nonobject
      objDetVars[rl].nonObjSeqCnt++;
      if(objDetVars[rl].nonObjSeqCnt > 5) 
      {
        // test for possible object intersect at Pt0 start of scan
        if((objIntersectsAtScanPt0 == true) && (objDetVars[rl?0:1].objDetected == true)
          && (objDetVars[rl].objSeqCnt0 + objDetVars[rl?0:1].objSeqCnt0 >= 10))
        { // will combine both sensor points
          objDetVars[rl].objDetected = true;  
        }
        else if(objDetVars[rl].objSeqCnt0 >= 5)
        { // Enough consecutive object scan counts for valid detect ???
          objDetVars[rl].objDetected = true;
        }
        else
        { // Attempt to detect an object again
          objDetVars[rl].detObjState = detObj_Init1;
//          objIntersectsAtScanPt0 = false;
        }
      }
      else if((scanEnd == true) && (objDetVars[rl].objSeqCnt0 >= 5))
      { // end of scan and seems like an object has been detected
        objDetVars[rl].objDetected = true;
      }
    }


    if(objDetVars[rl].objDetected == true)
    {
      if((objIntersectsAtScanPt0 == true) && (objDetVars[rl?0:1].objDetected == true))
      { // both sensors have detected objects, now combine
        // determine mid scan degrees, consider fwd and rev scan direction
        if(objDetVars[rl].objDegLast0 > objDetVars[rl?0:1].objDegLast0) 
        {
          objDetVars[rl].objDeg = objDetVars[rl].objDegLast0 -
            ((objDetVars[rl].objDegLast0 - objDetVars[rl?0:1].objDegLast0)/2);
        }
        else 
        {
          objDetVars[rl].objDeg = objDetVars[rl?0:1].objDegLast0 -
            ((objDetVars[rl?0:1].objDegLast0 - objDetVars[rl].objDegLast0)/2);
        }
        // use minimum distance as the detected object distance
        if(objDetVars[rl].objDistMin0 <= objDetVars[rl?0:1].objDistMin0)
        {
          objDetVars[rl].objDist = objDetVars[rl].objDistMin0;
          objDetVars[rl?0:1].objDetected = false; // remove other sensor object det
        }
        else
        {
          objDetVars[rl?0:1].objDist = objDetVars[rl?0:1].objDistMin0;
          objDetVars[rl].objDetected = false; // remove other sensor object det
        }
        // stop looking when object is found
        objDetVars[rl].detObjState = detObj_ObjEnd;
      }
      else if((scanEnd == true) && (objDetVars[rl?0:1].objDetected == true))
      { // combine sensors at end of scan
        // determine mid scan degrees, consider fwd and rev scan direction
        if(objDetVars[rl].objDegStart0 > objDetVars[rl?0:1].objDegStart0) 
        {
          objDetVars[rl].objDeg = objDetVars[rl].objDegStart0 -
            ((objDetVars[rl].objDegStart0 - objDetVars[rl?0:1].objDegStart0)/2);
        }
        else 
        {
          objDetVars[rl].objDeg = objDetVars[rl?0:1].objDegStart0 -
            ((objDetVars[rl?0:1].objDegStart0 - objDetVars[rl].objDegStart0)/2);
        }
        // use minimum distance as the detected object distance
        if(objDetVars[rl].objDistMin0 <= objDetVars[rl?0:1].objDistMin0)
        {
          objDetVars[rl].objDist = objDetVars[rl].objDistMin0;
          objDetVars[rl?0:1].objDetected = false; // remove other sensor object det
        }
        else
        {
          objDetVars[rl?0:1].objDist = objDetVars[rl?0:1].objDistMin0;
          objDetVars[rl].objDetected = false; // remove other sensor object det
        }
        // stop looking when object is found
        objDetVars[rl].detObjState = detObj_ObjEnd;
        
      }      
      else
      { // not combining both scans after nonobject after object
//          objIntersectsAtScanPt0 = false;
        // determine mid scan degrees, consider fwd and rev scan direction
        if(objDetVars[rl].objDegLast0 > objDetVars[rl].objDegStart0) 
        {
          objDetVars[rl].objDeg = objDetVars[rl].objDegStart0 + 
                  ((objDetVars[rl].objDegLast0 - objDetVars[rl].objDegStart0)/2);
        }
        else 
        {
          objDetVars[rl].objDeg = objDetVars[rl].objDegStart0 - 
                  ((objDetVars[rl].objDegStart0 - objDetVars[rl].objDegLast0)/2);
        }
        // use minimum distance as the detected object distance
        objDetVars[rl].objDist = objDetVars[rl].objDistMin0;
        // stop looking when object is found
        objDetVars[rl].detObjState = detObj_ObjEnd;
      }
    }
    break;

  case detObj_ObjEnd: // stay here after an object detected
      //TODO:  multiple objects detected per sensor?
      break;
  }

}

// scan using ultrasonic sensors
// exit when scan is complete
bool scanMax2Min = true;
void scanRange()
{
  int xR, yR;
  int xL, yL;
  int scanDistR;
  int scanDistL;
  int robDegR, robDistR;
  int robDegL, robDistL;

  // init object detect state for 1st state
  objDetVars[rightSensor].detObjState = detObj_Init0;
  objDetVars[leftSensor].detObjState = detObj_Init0;

  int scanCntDwn = abs(scanDegRev - scanDegFwd)/scanDegInc;

  for(scanDeg = (scanMax2Min ? scanDegRev : scanDegFwd); 
                (scanMax2Min ? (scanDeg>=scanDegFwd) : (scanDeg<=scanDegRev)); 
      scanDeg += (scanMax2Min ? -scanDegInc : scanDegInc))
  {
    positionRangeSensors(scanDeg);
    sonicSensors.startAsync(0);
delay(25); // makes measurement more stable???
    while (!sonicSensors.isFinished()) {delay(25); Serial.print(".");}

    scanDistR = sonicSensors.getDist_cm(rightSensor); // right sensor
    scanDistL = sonicSensors.getDist_cm(leftSensor); // left sensor

    limitDistance(scanDistR);
    limitDistance(scanDistL);

    scan2Rob(rightSensor,  +scanDeg, scanDistR, robDegR, robDistR, xR, yR);
    scan2Rob(leftSensor,   -scanDeg, scanDistL, robDegL, robDistL, xL, yL);

    // object detection algoritm
    //TODO: impliment Left scan processing, sets XY
    detectObjects(scanCntDwn==0, rightSensor, robDegR, robDistR, xR, yR);
    detectObjects(scanCntDwn==0, leftSensor,  robDegL, robDistL, xL, yL);
    scanCntDwn--;

    xyCm2CharMatrix(xR, yR, 'R');       
    xyCm2CharMatrix(xL, yL, 'L');    

Serial.print(objIntersectsAtScanPt0);
Serial.print("R "); Serial.print(objDetVars[rightSensor].detObjState); 
Serial.print(','); Serial.print(objDetVars[rightSensor].objDetected); Serial.print("; ");   
Serial.print("L "); Serial.print(objDetVars[leftSensor].detObjState); 
Serial.print(','); Serial.print(objDetVars[leftSensor].objDetected); Serial.print(": ");   
    printRangeSensors(robDegR, robDistR, xR, yR, robDegL, robDistL, xL, yL);
    
  }
  scanMax2Min = ! scanMax2Min;

  // TODO: consider both sensor objects
  // add closest object
  if(objDetVars[rightSensor].objDetected == true)
  {
    rangePolar2Xy(objDetVars[rightSensor].objDeg, objDetVars[rightSensor].objDist, xR, yR);
    xyCm2CharMatrix(xR,yR,'@');       
  }
  else if(objDetVars[leftSensor].objDetected == true)
  {
    rangePolar2Xy(objDetVars[leftSensor].objDeg, objDetVars[leftSensor].objDist, xL, yL);
    xyCm2CharMatrix(xL,yL,'@');       
  }

}


void loop()
{
  // stop motion before starting heading calibration while stopped
  motion = stop;
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, motion, speed, sensors.heading);
  delay(50);
  sensors.calibrateAG = true; // calibrate while stopped

  // Scan for object positions
  initCharMatrix();
  scanRange();
  printCharMatrix();
//delay(5000);
  
  // move toward closest object
  int moveDeg;
  int moveDist;
  int xCm,yCm;
  int runTimeMs;
  unsigned long startMillis;  

  // if both sensors detected use minimum distance detected
  if(((objDetVars[rightSensor].objDetected == true) && (objDetVars[leftSensor].objDetected != true)) 
      || (((objDetVars[rightSensor].objDetected == true) && (objDetVars[leftSensor].objDetected == true))
            && ((objDetVars[rightSensor].objDist <= objDetVars[leftSensor].objDist))))
  {
Serial.println("objDetVars[rightSensor].objDetected == true");
    moveDeg  = objDetVars[rightSensor].objDeg;
    moveDist = objDetVars[rightSensor].objDist;
  }
  else if(((objDetVars[leftSensor].objDetected == true) && (objDetVars[rightSensor].objDetected != true)) 
            || (((objDetVars[leftSensor].objDetected == true) && (objDetVars[rightSensor].objDetected == true))
                  && ((objDetVars[leftSensor].objDist <= objDetVars[rightSensor].objDist))))
  {
Serial.println("objDetVars[leftSensor].objDetected == true");
    moveDeg  = objDetVars[leftSensor].objDeg;
    moveDist = objDetVars[leftSensor].objDist;
  }
  else
  {
    moveDeg  = 0;
    moveDist = 0;
  }  
  
  // convert deg dist to XY
  rangePolar2Xy(moveDeg, moveDist, xCm, yCm);
  
printRangeSensors(moveDeg, moveDist, xCm, yCm, 0,0,0,0);


  // travel to the can
  sensors.calibrateAG = false; // stop calibration while moving
  speed = 1.0;
  float calSecPerCm = 1.0/25; // calibrated for 25 cm/sec

  // move in X axis
  if(xCm>=0) motion = right; else motion = left;
  runTimeMs = ((motion==right)?+1:-1)*(1000L*xCm)*calSecPerCm;
Serial.print("Move X "); Serial.print(xCm); 
Serial.print(" cm For "); Serial.print(runTimeMs); Serial.println(" msec");
  startMillis = millis();
  while((millis() - startMillis) < runTimeMs)
  {   
    for (int wheel=0;wheel<4;wheel++)
    {
//      wheels.WheelDrive(wheel, motion, speed, sensors.heading);
    }
    delay(50);
  }
  
  // move in Y axis
  if(yCm>=0) motion = fwd; else motion = rev;
  sensors.calibrateAG = (motion == stop);
  runTimeMs = ((motion==fwd)?+1:-1)*(1000L*yCm)*calSecPerCm;
Serial.print("Move Y "); Serial.print(yCm); 
Serial.print(" cm For "); Serial.print(runTimeMs); Serial.println(" msec");
  startMillis = millis();
  while((millis() - startMillis) < runTimeMs)
  {   
    for (int wheel=0;wheel<4;wheel++)
    {
//      wheels.WheelDrive(wheel, motion, speed, sensors.heading);
    }
    delay(50);
  }

}

// print various global variables
void serialHandling()
{
  float t;
  float initialTemperature = sensors.initialTemperature;
  float currentTemperature;
  if(serialOK) while(1)
  {

/*
//    currentTemperature = sensors.currentTemperature;    
//    t = currentTemperature - initialTemperature;
//    Serial.print(t);
//    Serial.print('\t');
    Serial.print(motion/10.0); // t
    Serial.print('\t');
    Serial.print(sensors.magY);  // magnometer primary signal
    Serial.print('\t');
    Serial.print(sensors.heading);
    Serial.print('\t');
    Serial.print(100*sensors.headingDriftRate,6);
    Serial.print('\n');
*/
    delay(500);
  }
}
