// 6canDetStatesReworkOK.ino

// Adding IMU to offset direction drift while driving
// also to control the rotation to be precise +-90 or +-180 degrees
// MRW 2/9/2023 moved lots of code to libraries to help manage code size
// MRW 2/12/2023 Added HC-SR04 range sensor 
// MRW 2/16/2023 Moved sensors low and on left right of body
// MRW 2/20/2023 Started working on objectr detection
// MRW 2/28/2023 Moved object detection code to ObjectDetect lib
// MRW 3/2/2023 Detect walls relative to robot as to not detect them
// MRW 3/5/2023 Worked on finding can, measuring goal, tweak X Y movement calibration
// MRW 3/11/2023 obj det states seems to need major reworking - saved code
// MRW 3/13/2023 works OK (1 can) saving again
// MRW 3/13/2023 Horrible detection with 4 cans

#include <Arduino.h>

// My libraries
#include <NanoBleLed.h>
#include <NanoBleSensors.h>
#include <MecanumWheels.h>

#include "ObjectDetect.h"

// Libraries from Arduino
#include <HC_SR04.h>
#include <PCA9685.h>  // servo driver
#include <mbed.h>  // rtos threads and semaphores
#include <rtos.h>
#include <string>

// Instance my libraries 
// TODO: can some of these be move to the library?
NanoBleLed led;
NanoBleSensors bleSensors;
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

ObjectDetect objdet;

// prints values to serial port after initialization
rtos::Thread serialHandlingThread;
void serialHandling();

// Create RTOS threads
// TODO: How to move these to my sensor library files?
// TODO: Should some be combined into a single thread
// Magnometer sensor
rtos::Thread MAGSensorHandlingThread;
void MAGSensorHandling() {bleSensors.MAGSensorHandling();}
// Accellerometer and Gyroscope sensors
rtos::Thread AGSensorHandlingThread;
void AGSensorHandling() {bleSensors.AGSensorHandling();}
// Temp sensor
rtos::Thread HTSSensorHandlingThread;
void HTSSensorHandling() {bleSensors.HTSSensorHandling();}


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


// returns msec per cm to move
float calSecPerCmX(int motion, float speed)
{
float secPerCm;
  // Calibration is different depending on the motion and speed to move

  // fully charged battery
  switch(motion)
  {
    // image.pngbat 6.7V
  case right:
    if(speed == 0.5) secPerCm = (2.2/speed)*(1.0/25);
    else             secPerCm = (1.1/speed)*(1.0/25);
    break;
  case left:
  default:
    if(speed == 0.5) secPerCm = (2.0/speed)*(1.0/25);
    else             secPerCm = (1.15/speed)*(1.0/25);
    break;
  }

/* low battery  
  switch(motion)
  {
  case right:
    if(speed == 0.5) secPerCm = (2.3/speed)*(1.0/25);
    else             secPerCm = (1.2/speed)*(1.0/25);
    break;
  case left:
  default:
    if(speed == 0.5) secPerCm = (2.1/speed)*(1.0/25);
    else             secPerCm = (1.15/speed)*(1.0/25);
    break;
  }
*/
  return(secPerCm);
}

float calSecPerCmY(int motion, float speed) 
{
float secPerCm;
  // Calibration is different depending on the motion and speed to move

  // fully charged battery
  switch(motion)
  {
  case fwd:
    if(speed == 0.5) secPerCm = (1.5/speed)*(1.0/25);
    else             secPerCm = (1.0/speed)*(1.0/25);
    break;
  case rev:
  default:
    if(speed == 0.5) secPerCm = (1.5/speed)*(1.0/25);
    else             secPerCm = (1.0/speed)*(1.0/25);
    break;
  }

/* low battery
  switch(motion)
  {
  case fwd:
    if(speed == 0.5) secPerCm = (1.4/speed)*(1.0/25);
    else             secPerCm = (1.2/speed)*(1.0/25);
    break;
  case rev:
  default:
    if(speed == 0.5) secPerCm = (1.4/speed)*(1.0/25);
    else             secPerCm = (1.1/speed)*(1.0/25);
    break;
  }
*/
  return(secPerCm);
}

int clawOpenUsec = 1000;
int clawCloseUsec = 2050;
int rClawTrim = 0;
int lClawTrim = -50;
bool clawIsOpen = false;
void openClaws(int openPct = 100)
{
  // set claw servos to open percent position
  int rClawUs = clawCloseUsec - ((clawCloseUsec-clawOpenUsec)*(openPct/100.0));
  int lClawUs = 1500 + (1500 - rClawUs);
  pca9685.setChannelServoPulseDuration( 9, rClawUs + rClawTrim);
  pca9685.setChannelServoPulseDuration(13, lClawUs + lClawTrim);
  clawIsOpen = true;
}

void closeClaws()
{
  // set claw servos to open position slowly
  if(clawIsOpen == true)
  {
    for(int rClawUs = clawOpenUsec; rClawUs <= clawCloseUsec; rClawUs+=2)
    {
      int lClawUs = 1500 + (1500 - rClawUs);

      pca9685.setChannelServoPulseDuration( 9, rClawUs + rClawTrim);
      pca9685.setChannelServoPulseDuration(13, lClawUs + lClawTrim);
      delay(1);
    }
  }
  clawIsOpen = false;
}

int wallAbsX=105; // wall X offset from any Y, R L equal, walls parrallel
int wallAbsY=122; // goals are at center of these walls at X=0
int robStartX=0;  // robot starting location X,Y = (0,-50)
int robStartY=-50; 

int robotRefX, robotRefY;
int robotX, robotY;
int canX, canY;


// Mearure Left and right wall distance at 90 degree sensor angle
// if valid robotX is updated and true is returned
bool  measRobotX(int &robotX)
{
  bool validMeas;
  objdet.positionRangeSensors(90);
  sonicSensors.startAsync(0);
delay(25); // makes measurement more stable???
  while (!sonicSensors.isFinished()) {delay(25); Serial.print(".");}

  int wallRX = sonicSensors.getDist_cm(rightSensor); // right sensor +X
  int wallLX = -sonicSensors.getDist_cm(leftSensor); // left sensor -X

  // add robot sensor offset
  wallRX += SERVO2ROBOTX_OFFSET;
  wallLX -= SERVO2ROBOTX_OFFSET;

  int wallDist = wallRX - wallLX;
  if(abs(wallDist - 2*wallAbsX) < 20) 
  { // it is within 10%
    robotX = wallAbsX - wallRX;
    validMeas = true;
  }
  else validMeas = false;
  
Serial.print("90 Deg Wall L,R = ");Serial.print(wallLX);Serial.print(", ");Serial.print(wallRX);
if(validMeas){
Serial.println(" valid RobotX measurement");
}
else {
Serial.println(" invalid RobotX measurement");

}
Serial.print("Measure robot XY = ");Serial.print(robotX);Serial.print(", ");Serial.println(robotY);

  return(validMeas);
}

void moveX(int xCm)
{
  float speed;
  int motion;  
  int runTimeMs;
  unsigned long startMillis;  

  // move in X axis
  if(xCm>=0) motion = right; else motion = left;
  if(abs(xCm)>20) speed = 1.0;
  else speed = 0.5;
  runTimeMs = ((motion==right)?+1:-1)*(1000L*xCm)*calSecPerCmX(motion, speed);
Serial.print("Move X "); Serial.print(xCm); 
Serial.print(" cm For "); Serial.print(runTimeMs); Serial.println(" msec");
  startMillis = millis();
  while((millis() - startMillis) < runTimeMs)
  {   
    for (int wheel=0;wheel<4;wheel++)
    {
      wheels.WheelDrive(wheel, motion, speed, bleSensors.heading);
    }
    delay(10); // manage rate of wheel upate which controls movement direction PI loop filter
  }

}

void moveY(int yCm, bool senseGoal = false)
{
  float speed;
  int motion;  
  int runTimeMs;
  unsigned long startMillis;  

int goalSensorDeg = 95;
int goalSensorDist = 95;
// TODO: goalSensorDistMin ?

  if(senseGoal == true) 
  { // set sensors to look at X wall and start range measure
    // look back slightly to not detect the Y wall edge until closer
    objdet.positionRangeSensors(goalSensorDeg);
    sonicSensors.startAsync(0);    
  }

  // move in Y axis
  if(yCm>=0) motion = fwd; else motion = rev;
  if(abs(yCm)>20) speed = 1.0;
  else speed = 0.5;
  runTimeMs = ((motion==fwd)?+1:-1)*(1000L*yCm)*calSecPerCmY(motion, speed);
Serial.print("Move Y "); Serial.print(yCm); 
Serial.print(" cm For "); Serial.print(runTimeMs); Serial.println(" msec");
  startMillis = millis();
  while((millis() - startMillis) < runTimeMs)
  {   
    for (int wheel=0;wheel<4;wheel++)
    {
      wheels.WheelDrive(wheel, motion, speed, bleSensors.heading);
    }

    // detect goal opening by read sensors at 90 degrees pointing to wall X
    // Goal is detected when the sensors both detect a big change from expected wall position
    // NOTE: 2 cans could cause this, maybe it needs to be more sophisticated/sensitive 
    if((senseGoal == true) && sonicSensors.isFinished())
    {
      int wallRX = sonicSensors.getDist_cm(rightSensor); // right sensor +X
      int wallLX = -sonicSensors.getDist_cm(leftSensor); // left sensor -X
      // add robot sensor offset
      wallRX += SERVO2ROBOTX_OFFSET;
      wallLX -= SERVO2ROBOTX_OFFSET;
      sonicSensors.startAsync(0);  
Serial.print("Goal X L,R = "); Serial.print(wallLX);Serial.print(", "); Serial.println(wallRX);
      if(wallRX<goalSensorDist && wallLX>-goalSensorDist)
      {
Serial.print("Goal detected, stop Y movement");
        break; // exit while millis loop
      }
    }  
    delay(10); // manage rate of wheel upate which controls movement direction PI loop filter
  }
 
}

// Move in X then y, optionally detect the goal entrance and set Robot Y
void moveXY(int xCm, int yCm, bool senseGoal = false)
{
  moveX(xCm);
  moveY(yCm, senseGoal);
  // TODO: Measure actual time and trim amount of XY movement?
  // estimate robot XY movement
  robotX += xCm;
  robotY += yCm;
  
  // Stop movement
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, stop, 0, bleSensors.heading);

}


void moveYX(int yCm, int xCm)
{
  moveY(yCm);
  moveX(xCm);
  // TODO: Measure actual time and trim amount of XY movement?
  // TODO: Measure actual time and trim amount of XY movement?
  // estimate robot XY movement
  robotX += xCm;
  robotY += yCm;

  // Stop movement
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, stop, 0, bleSensors.heading);

}

void initLedRG()
{
//  pinMode(D2, OUTPUT);
//  pinMode(D3, OUTPUT);
}

void ledRG(int red, int grn)
{
//  digitalWrite(D2, red);
//  digitalWrite(D3, grn);
}
void setup()
{
  initLedRG();
  initSerial();

  led.initLed();
  led.ledOn();
  led.rgbLed(1,1,1); // white

  initPca9685();

  ledRG(1, 0);

  openClaws();

  // initialize robot position
  robotRefX = 0;
  robotRefY = robStartY;
  robotX = robotRefX;
  robotY = robotRefY;

  objdet.initSonicSensors(&pca9685, &sonicSensors, &bleSensors, wallAbsX, wallAbsY);

while(0){ // calibrate range sensor positioning
  objdet.positionRangeSensors(90);
  delay(20000);
  objdet.positionRangeSensors(0);
  delay(20000);
  objdet.positionRangeSensors(180);
  delay(20000);

}

  wheels.initWheels(&pca9685);
  bleSensors.initSensors(serialOK);   

  // Start threads
  // TODO: How to move these to my sensor library files?
  MAGSensorHandlingThread.start(MAGSensorHandling);
  AGSensorHandlingThread.start(AGSensorHandling);
  HTSSensorHandlingThread.start(HTSSensorHandling);

  serialHandlingThread.start(serialHandling);

  bleSensors.calibrateAG = false;
  
  led.ledOff();
  ledRG(1, 1);

Serial.print("Init robot XY = "); Serial.print(robotX); Serial.print(","); Serial.println(robotY);
} 

// Robot start scan XY locations
typedef struct XY{
  int X;
  int Y;
};

// Robot scan positions to use
#define numRobotScanXY 12
int robotScanXYidx = 0;
XY robotScanXY[numRobotScanXY] = 
{
{  0,-50}, {  0,  0}, {  0, 50}, {  0,-50},
{-90,-50}, {-90,  0}, {-90, 50}, {  0,-50},
{ 90,-50}, { 90,  0}, { 90, 50}, {  0,  0}
};

// main loop states
enum {
  loopState_gotoScanXY,
  loopState_Scan4Can,
  loopState_Move2Can,
//  loopState_UpdateXY,
  loopState_GrabCan,
  loopState_Can2Goal,
  loopState_Done
};

// Start loop go to location and scan for objects (cans)
int loopState = loopState_gotoScanXY;
// variables used between states
int canDeg, canDist;
int canXcm,canYcm;
int goalXcm, goalYcm;
bool objectDetected;
int movX, movY;
void loop()
{

  switch(loopState)
  {
  case loopState_gotoScanXY:
    ledRG(1, 1);

Serial.print("Go to scan location idx,x,y ");Serial.print(robotScanXYidx);Serial.print(", ");
Serial.print(robotScanXY[robotScanXYidx].X);Serial.print(", ");
Serial.print(robotScanXY[robotScanXYidx].Y);Serial.print("\n");

    movX = robotScanXY[robotScanXYidx].X - robotX;
    movY = robotScanXY[robotScanXYidx].Y - robotY;
    moveXY(movX, movY);
    measRobotX(robotX) == false;

    loopState =  loopState_Scan4Can;
    break; //case loopState_gotoScanXY  

  case loopState_Scan4Can:  
    ledRG(1, 1);

    // TODO: is this stop movement redundant?
    // stop motion before starting heading calibration while stopped to scan
    for (int wheel=0;wheel<4;wheel++)
      wheels.WheelDrive(wheel, stop, 0, bleSensors.heading);
    delay(10); // wait for motion to stop for cal

    bleSensors.calibrateAG = true; // calibrate while stopped

    // Scan for object positions, input current robot location for wall ignore
    objdet.scanRange(robotX, robotY);

    bleSensors.calibrateAG = false; // stop calibration after scan

    objectDetected = objdet.getObject(canDeg, canDist, canXcm, canYcm);

    if(objectDetected == true)
    {
      ledRG(0, 1); // Green

if(1){
Serial.print("canDeg "); Serial.print(canDeg); Serial.print(", ");
Serial.print("canDist ");Serial.print(canDist);Serial.print(", ");
Serial.print("canXcm "); Serial.print(canXcm); Serial.print(", ");
Serial.print("canYcm "); Serial.print(canYcm); Serial.print("\n");
}

      // TODO: Use LIDAR to get close to can to grab 
      if(canDist>5  || abs(canDeg)>45) loopState = loopState_Move2Can;
      else loopState =  loopState_GrabCan;
    }
    else
    {
  Serial.println("Object not detected in last scan, goto new scan location");
      robotScanXYidx++;
      if(robotScanXYidx < numRobotScanXY) loopState = loopState_gotoScanXY;
      else loopState = loopState_Done;
    }
    break; // case loopState_Scan

  case loopState_Move2Can:
//         if(canXcm>+5 && canYcm<5) moveYX(-10, +10); // Can is on the left side, move to back of can
//    else if(canXcm<-5 && canYcm<5) moveYX(-10, -10); // Can is on the right side, move to back of can
         if(canYcm<5 && abs(canXcm)>5) moveYX(-10, canXcm); // Can is on the R or L side, move to back of can
//    else if(canYcm<5 && canXcm<-5) moveYX(-10, canXcm); // Can is on the left side, move to back of can
    else moveXY(canXcm, canYcm); 

//    loopState = loopState_UpdateXY;  
//    break; // case loopState_Move2Can

//  case loopState_UpdateXY:
  measRobotX(robotX);
Serial.print("UpdateXY robot XY = "); Serial.print(robotX); Serial.print(","); Serial.println(robotY);

    loopState = loopState_Scan4Can;
    break; // case loopState_UpdateXY

  case loopState_GrabCan:
    // TODO: Verify can was grabbed with lidar and maybe color
    closeClaws();    
    measRobotX(robotX);
    loopState = loopState_Can2Goal;
    break; // case loopState_GrabCan

  case loopState_Can2Goal:
    // calc movement to goal
    goalXcm = -robotX; // goto center X=0
    goalYcm = wallAbsY - robotY + 50; // try to go past goal
    // detect the goal walls while moving to goal and set robotY coordinate
    moveXY(goalXcm, goalYcm, true); 
    robotY = wallAbsY; // stopped when goal (wall Y) detected 

Serial.print("At goal robot XY = "); Serial.print(robotX); Serial.print(","); Serial.println(robotY);

    // go 10cm more into goal area to ensure can is in
    moveXY(0, 10);

    // Open claws enought to release can but not disturb other cans
    openClaws(20);
    
    // backup out of the goal and measure the robotX coordinate
    moveXY(0, -35);
    measRobotX(robotX);

    // Fully open claws
    openClaws(100);

    loopState = loopState_gotoScanXY;
    break; // case loopState_Can2Goal

  case loopState_Done:
    ledRG(1, 0); // red

    delay(1000);
    break; // case loopState_Done
     
  } // end switch loopState

  delay(1); // give control to other threads immediately

}

// print various global variables
void serialHandling()
{
  float t;
  float initialTemperature = bleSensors.initialTemperature;
  float currentTemperature;

 
  if(serialOK) while(1)
  {

/*
//    currentTemperature = bleSensors.currentTemperature;    
//    t = currentTemperature - initialTemperature;
//    Serial.print(t);
//    Serial.print('\t');
    Serial.print(motion/10.0); // t
    Serial.print('\t');
    Serial.print(bleSensors.magY);  // magnometer primary signal
    Serial.print('\t');
    Serial.print(bleSensors.heading);
    Serial.print('\t');
    Serial.print(100*bleSensors.headingDriftRate,6);
    Serial.print('\n');
*/
    delay(500);
  }
}
