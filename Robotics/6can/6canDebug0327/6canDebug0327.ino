// 6canCanDetLidar.ino

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
// MRW 3/16/2023 Use LIDAR for final can approach before grabbing
// MRW 3/25/2023 Only used erial if OK, also misc tweaks
// MRW 3/25/2023 Fix goal 
// MRW 3/27/2023 Fix more movement bugs/issues to mak it faster
// MRW 3/28/2023 correct X offset after moveScanXY and done
// MRW 3/28/2023 remove serial handling thread
// MRW 3/29/2023 Replaced RR wheel motor - calibrated zero - had to tweak
// MRW 3/29/2023 Removed slow MAG sensor thread - combined with AG sensor which is faster

#include <Arduino.h>

// My libraries
#include <NanoBleLed.h>
#include <NanoBleSensors.h>
#include <MecanumWheels.h>

#include "ObjectDetect.h"

// Libraries from Arduino
#include <HC_SR04.h>
#include <PCA9685.h>  // servo driver
#include <INA219.h>

#include <Wire.h>
#include <mbed.h>  // rtos threads and semaphores
#include <rtos.h>
#include <string>

// Instance my libraries 
// TODO: can some of these be move to the library?
NanoBleLed led;
NanoBleSensors bleSensors;
MecanumWheels wheels;


TwoWire *wire_ptr = &Wire;
INA219 ina219(0x41, wire_ptr);
// INA219 and PCA9698 shace the I2C bus Wire
// Instance servo controller
PCA9685 pca9685;
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ

// Instance and Map range sensor trigger and echo to BLE Sense pins
//HC_SR04<A6> sensor(A7); // single sensor with echo A6 and trigger A7 pins
HC_SR04_BASE *Slaves[] = { new HC_SR04<A3>() }; // left sensor
HC_SR04<A6> sonicSensors(A7, Slaves, 1); // right sensor

ObjectDetect objdet;

// Create RTOS threads
// TODO: How to move these to my sensor library files?
// Accellerometer and Gyroscope sensors and MAG sensor
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

float getBusVoltage()
{ 
  float busV = 0;
  int cnt=0;
  while((busV < 6.0) || (busV > 7.5))
  {
    if((cnt>0) && (serialOK))
    {
      Serial.print("busV is bad "); Serial.println(busV);   
    }
    
    busV = ina219.getBusVoltage();
    delay(1);        
    cnt++;    
  }
  return(busV);
}

void initIna219() {
  if (!ina219.begin() )
  {
    if(serialOK) Serial.println("could not connect. Fix and Reboot");
  }

  ina219.setMaxCurrentShunt(2.5, 0.002);

}

void initPca9685(){
//  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupSingleDevice(*wire_ptr,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);
}

int canCount = 0; // Number of cans put into the goal area

int wallAbsX=111; // wall X offset from any Y, R L equal, walls parallel
int wallAbsY=122; // goals are at center of these walls at X=0
int robStartX=0;  // robot starting location X,Y = (0,-50)
int robStartY=-50; 

int robotRefX, robotRefY;
int robotX, robotY;
int canX, canY;

// motion travel calibrations
// OLD
/*
float xCalRight1p0 = 1.1;
float xCalRight0p5 = 2.2;
float xCalLeft1p0  = 1.15;
float xCalLeft0p5  = 2.0;

float yCalFwd1p0 = 1.0;
float yCalFwd0p5 = 1.5;
float yCalRev1p0 = 1.0;
float yCalRev0p5 = 1.5;
*/

// New formula secPerCm(V) = _A + _B*V
float calSecPerCmLeft1p0_A  = 0.10345;
float calSecPerCmRight1p0_A = 0.10300;
float calSecPerCmLeft0p5_A  = 0.77250;
float calSecPerCmRight0p5_A = 1.10840;

float calSecPerCmLeft1p0_B  = -0.00850;
float calSecPerCmRight1p0_B = -0.00850;
float calSecPerCmLeft0p5_B  = -0.09300;
float calSecPerCmRight0p5_B = -0.14000;

float calSecPerCmRev1p0_A  = 0.09640;
float calSecPerCmFwd1p0_A  = 0.08080;
float calSecPerCmRev0p5_A  = 0.69695;
float calSecPerCmFwd0p5_A  = 0.68497;

float calSecPerCmRev1p0_B  = -0.0080;
float calSecPerCmFwd1p0_B  = -0.0060;
float calSecPerCmRev0p5_B  = -0.0870;
float calSecPerCmFwd0p5_B  = -0.0880;

// returns calibrated sec per cm to move
float secPerCmX(float volts, int motion, float speed)
{
float secPerCm;
  // Calibration move sec/cm is different depending on the voltage, motion and speed

  switch(motion)
  {

  case right:
    if(speed == 1.0) secPerCm = calSecPerCmRight1p0_A + (volts * calSecPerCmRight1p0_B);
    else             secPerCm = calSecPerCmRight0p5_A + (volts * calSecPerCmRight0p5_B);
    break;

  case left:
  default:
    if(speed == 1.0) secPerCm = calSecPerCmLeft1p0_A + (volts * calSecPerCmLeft1p0_B);
    else             secPerCm = calSecPerCmLeft0p5_A + (volts * calSecPerCmLeft0p5_B);
    break;

  }

  return(secPerCm);
}

float secPerCmY(float volts, int motion, float speed) 
{
float secPerCm;
  // Calibration is different depending on the motion and speed to move

  switch(motion)
  {

  case fwd:
    if(speed == 1.0) secPerCm = calSecPerCmFwd1p0_A + (volts * calSecPerCmFwd1p0_B);
    else             secPerCm = calSecPerCmFwd0p5_A + (volts * calSecPerCmFwd0p5_B);

// There seems to be an error when calibrated last time, attempt tweak
secPerCm *= (45.0/50.0);

    break;

  case rev:
  default:
    if(speed == 1.0) secPerCm = calSecPerCmRev1p0_A + (volts * calSecPerCmRev1p0_B);
    else             secPerCm = calSecPerCmRev0p5_A + (volts * calSecPerCmRev0p5_B);
    break;

  }

  return(secPerCm);
}

float cmPerSecX(float volts, int motion, float speed)
{
  float cmPerSec;
  switch(motion)
  {
  case right:
    cmPerSec = 1.0/(secPerCmX(volts, motion, speed));
    break;

  case left:
  default:
    cmPerSec = -1.0/(secPerCmX(volts, motion, speed));
    break;
  }

  return(cmPerSec);
}

float yCmPerSec(float volts, int motion, float speed)
{
  float cmPerSec;
  switch(motion)
  {
  case fwd:
    cmPerSec = 1.0/(secPerCmY(volts, motion, speed));
    break;

  case rev:
  default:
    cmPerSec = -1.0/(secPerCmY(volts, motion, speed));
    break;
  }

  return(cmPerSec);
}

int clawOpenUsec = 1000;
int clawCloseUsec = 2050;
int rClawTrim = 0;
int lClawTrim = -50;
int clawOpenedUsec = 0;
void openClaws(int openPct = 100, bool quickly = false)
{
  if(quickly)
  { // open quickly - suddenly
    int rClawUs = clawCloseUsec - ((clawCloseUsec-clawOpenUsec)*(openPct/100.0));
    int lClawUs = 1500 + (1500 - rClawUs);

    pca9685.setChannelServoPulseDuration( 9, rClawUs + rClawTrim);
    delay(1); // Let last cmd finsh?
    pca9685.setChannelServoPulseDuration(13, lClawUs + lClawTrim);
    delay(1); // Let last cmd finsh?

    clawOpenedUsec = rClawUs;   
  }
  else
  { // open slowly
    // claw servos to open percent position
    int rClaw2OpenUsec = clawCloseUsec - ((clawCloseUsec-clawOpenUsec)*(openPct/100.0));

    for(int rClawUs = clawOpenedUsec; rClawUs >= rClaw2OpenUsec; rClawUs-=2)
    {  
      int lClawUs = 1500 + (1500 - rClawUs);
      pca9685.setChannelServoPulseDuration( 9, rClawUs + rClawTrim);
      delay(1); // Let last cmd finsh?
      pca9685.setChannelServoPulseDuration(13, lClawUs + lClawTrim);
      delay(1); // Let last cmd finsh?
    }

    clawOpenedUsec = rClaw2OpenUsec;
  }
}

void closeClaws()
{
  // claw servos to close slowly
  for(int rClawUs = clawOpenedUsec; rClawUs <= clawCloseUsec; rClawUs+=2)
  {
    int lClawUs = 1500 + (1500 - rClawUs);

    pca9685.setChannelServoPulseDuration( 9, rClawUs + rClawTrim);
    delay(1); // Let last cmd finsh?
    pca9685.setChannelServoPulseDuration(13, lClawUs + lClawTrim);
    delay(1);
  }

  clawOpenedUsec = clawCloseUsec;
}


// Mearure Left and right wall distance at 90 degree sensor angle
// if valid robotX is updated and true is returned
bool  measRobotX(int &robotX)
{
  bool validMeas;
  objdet.positionRangeSensors(90);
  sonicSensors.startAsync(0);
delay(25); // makes measurement more stable???
  while (!sonicSensors.isFinished()) {delay(25); if(serialOK) Serial.print(".");}

  int wallRX = sonicSensors.getDist_cm(rightSensor); // right sensor +X
  int wallLX = -sonicSensors.getDist_cm(leftSensor); // left sensor -X

  // add robot center to sensor offsets, at 90 deg they simply add
  wallRX += SENSOR2SERVO_OFFSET + SERVO2ROBOTX_OFFSET;
  wallLX -= SENSOR2SERVO_OFFSET + SERVO2ROBOTX_OFFSET;

  int wallDist = wallRX - wallLX;
  if(abs(wallDist - 2*wallAbsX) < 20) 
  { // it is within 10%
    robotX = wallAbsX - wallRX;
    validMeas = true;
  }
  else validMeas = false;

if(serialOK) {
Serial.print("90 Deg Wall L,R = ");Serial.print(wallLX);Serial.print(", ");Serial.print(wallRX);
  if(validMeas){
  Serial.println(" valid RobotX measurement");
  }
  else {
  Serial.println(" invalid RobotX measurement");

  }
Serial.print("Measure robot XY = ");Serial.print(robotX);Serial.print(", ");Serial.println(robotY);
}

  return(validMeas);
}

void stopMovement()
{
  for (int wheel=0;wheel<4;wheel++)
  {
    wheels.WheelDrive(wheel, stop, 0, bleSensors.heading);
  }    
}

void moveX(int &xCm, bool lidar = false)
{
  float busV;
  float speed;
  int motion;  
  unsigned long runTimeMs;
  unsigned long startMillis;  
  unsigned long millisElapsed;

  int proximity;
  int lastProx = 1000; // more than detector can output;
  
  
  // move in X axis
  if(xCm>=0) motion = right; else motion = left;  
  if(lidar) speed = 0.5;  
  else if(abs(xCm)>20) speed = 1.0;
  else speed = 0.5;

  busV = getBusVoltage(); delay(1);
  runTimeMs = (1000.0*abs(xCm))*secPerCmX(busV, motion, speed);

if(serialOK) {
Serial.print("Move X "); Serial.print((motion==right)?"right ":"left "); Serial.print(xCm); 
Serial.print(" cm For "); Serial.print(runTimeMs); Serial.print(" msec");
Serial.print(", busV = "); Serial.println(busV);
}
  startMillis = millis();
  millisElapsed = 0;
  while(millisElapsed < runTimeMs)
  {   
    for (int wheel=0;wheel<4;wheel++)
    {
      wheels.WheelDrive(wheel, motion, speed, bleSensors.heading);
    }
    
    if(lidar == true)
    {
      bleSensors.readAPDSSensor(proximity);
if(serialOK) {
Serial.print("moveX LIDAR "); Serial.print(proximity); Serial.print(", ");
Serial.print("millis = "); Serial.print(millis() - startMillis); Serial.print("\n");
}
      if((proximity - lastProx) > 5)       
      {
        // TODO: estimate X travel based on time elapsed, use to estimate robot XY
        millisElapsed = (millis() - startMillis);    
        break; // exit while time
      }

      lastProx = proximity;      
    }

    delay(10); // manage rate of wheel upate which controls movement direction PI loop filter
    millisElapsed = (millis() - startMillis);    
  }
  // estimate the amount of X traveled
  busV = getBusVoltage(); delay(1);
  xCm = (millisElapsed/1000.0) * cmPerSecX(busV, motion, speed);
  stopMovement();
}

void moveY(int &yCm, bool senseGoal = false, bool lidar = false)
{
  float speed;
  int motion;  
  unsigned long runTimeMs;
  unsigned long startMillis;  
  unsigned long millisElapsed;

  int proximity;

int goalSensorDeg = 95;
int goalWidthMax  = 108 + 10;
int goalWidthMin  = 108 - 10;
int goalSenseTolY = 50;



  if(senseGoal == true) 
  { // set sensors to look at X wall and start range measure
    // look back slightly to not detect the Y wall edge until closer
    objdet.positionRangeSensors(goalSensorDeg);
    sonicSensors.startAsync(0);    
  }

  // move in Y axis
  if(yCm>=0) motion = fwd; else motion = rev;
  yCm = abs(yCm);
  if(lidar) speed = 0.5;
  else if(abs(yCm)>20) speed = 1.0;
  else speed = 0.5;

  float busV = getBusVoltage();
  runTimeMs = (1000.0*abs(yCm))*secPerCmY(busV, motion, speed);
  
if(serialOK) {
Serial.print("Move Y "); Serial.print((motion==fwd)?"fwd ":"rev "); Serial.print(yCm); 
Serial.print(" cm For "); Serial.print(runTimeMs); Serial.print(" msec");
Serial.print(", busV = "); Serial.println(busV);
}
  startMillis = millis();
  millisElapsed = 0;
  while(millisElapsed < runTimeMs)
  {   
    for (int wheel=0;wheel<4;wheel++)
    {
      wheels.WheelDrive(wheel, motion, speed, bleSensors.heading);
    }

    if(lidar == true)
    {
      bleSensors.readAPDSSensor(proximity);
      
if(serialOK) {
Serial.print("moveY LIDAR "); Serial.print(proximity); Serial.print(", ");
Serial.print("millis = "); Serial.print(millis() - startMillis); Serial.print("\n");
}
      if(proximity < 2) 
      {
        //estimate Y travel based on time elapsed, use to estimate robot XY
        millisElapsed = (millis() - startMillis);    
        break; // exit while time
      }
    }
    // detect goal opening by read sensors at 90 degrees pointing to wall X
    // Goal is detected when the sensors both detect a big change from expected wall position
    // NOTE: 2 cans could cause this, maybe it needs to be more sophisticated/sensitive 
    else if((senseGoal == true) && sonicSensors.isFinished())
    {
      int wallRX = sonicSensors.getDist_cm(rightSensor); // right sensor +X
      int wallLX = -sonicSensors.getDist_cm(leftSensor); // left sensor -X
      // add robot sensor offset
      wallRX += SERVO2ROBOTX_OFFSET;
      wallLX -= SERVO2ROBOTX_OFFSET;
      sonicSensors.startAsync(0);  
      
if(serialOK) {
Serial.print("senseGoal X L,R = "); Serial.print(wallLX);Serial.print(", "); Serial.println(wallRX);
}
      int goalWidthMeas = wallRX - wallLX; 
      if(( goalWidthMeas < goalWidthMax) && (goalWidthMeas > goalWidthMin))
      { // Goal X width verified
        // estimate the amount of Y traveled
        millisElapsed = (millis() - startMillis);    
        yCm = (millisElapsed/1000.0) * yCmPerSec(busV, motion, speed);
        int estRobotY = robotY + yCm;
        if(abs(wallAbsY - estRobotY) < goalSenseTolY)
        { // close to expected goal Y location
        
if(serialOK) {
Serial.println("Goal detected, stop Y movement");
}
          break; // exit while millis loop
        }
      }
    }  
    delay(10); // manage rate of wheel upate which controls movement direction PI loop filter
    millisElapsed = (millis() - startMillis);    
  } // end while millis loop

  millisElapsed = (millis() - startMillis);    
  stopMovement();

  // estimate the amount of Y traveled
  busV = getBusVoltage();
  yCm = (millisElapsed/1000.0) * yCmPerSec(busV, motion, speed);

}

// Move in X then y, optionally detect the goal entrance and set Robot Y
void moveXY(int xCm, int yCm, bool senseGoal = false, bool lidar = false)
{
  int lidarXcm = 0;
  moveX(xCm, lidar);

  if(lidar) 
  {
    lidarXcm = xCm>=0?-2:+2;
    moveX(lidarXcm); // a bit of correction for overshoot???
  }

  moveY(yCm, senseGoal, lidar);

  // estimate robot XY movement
  robotX += (xCm + lidarXcm);
  robotY += yCm;
  
}


void moveYX(int yCm, int xCm)
{
  moveY(yCm);
  moveX(xCm);

  // estimate robot XY movement
  robotX += xCm;
  robotY += yCm;

}

void move360(void)
{

  for (int wheel=0;wheel<4;wheel++) 
  {
    wheels.WheelDrive(wheel, cwise, 2, 0);
  }

  delay(2600); // rotate about 360 at speed = 2

  stopMovement();
}


// move to a side wall
// return wall distance 
int calMove(float speed, int motion)
{
  unsigned long startMillis;  
  unsigned long millisElapsed;
  int wallDist;
  int wallDistStop;

  sonicSensors.startAsync(0); // start new sensor measurement
  startMillis = millis();
  millisElapsed = 0;

  while(millisElapsed < 40000) 
  { // will break exit when close to wall
    // update wheels motion and heading 
    for (int wheel=0;wheel<4;wheel++)
    {
      wheels.WheelDrive(wheel, motion, speed, bleSensors.heading);
    }

    if(sonicSensors.isFinished())
    {
      switch(motion)
      {
      case right:
        wallDist = sonicSensors.getDist_cm(rightSensor) 
          + SENSOR2SERVO_OFFSET + SERVO2ROBOTX_OFFSET; // right sensor +X wall
        break;

      case left:
        wallDist = -sonicSensors.getDist_cm(leftSensor) 
          - SENSOR2SERVO_OFFSET - SERVO2ROBOTX_OFFSET; // left sensor -X wall
        break;

      case fwd:
        wallDist = sonicSensors.getDist_cm(rightSensor) 
          + SENSOR2SERVO_OFFSET - SERVO2ROBOTY_OFFSET; // right sensor +X wall
        break;

      case rev:
        wallDist = -sonicSensors.getDist_cm(leftSensor) 
          - SENSOR2SERVO_OFFSET - SERVO2ROBOTY_OFFSET; // left sensor -X wall
        break;
      }
      
//Serial.println(wallDist);

      switch(motion)
      {
      case right:
      case left:
        wallDistStop = 20;
        break;

    case fwd:
        wallDistStop = 10;
        break;

    case rev:
        wallDistStop = 30;
        break;

      }

      if(abs(wallDist) <  wallDistStop)
      { // close to the left -X wall
        stopMovement();
        break; // exit while loop
      }
      else 
      {
        sonicSensors.startAsync(0); // start new sensor measurement
      }
    }

    delay(10); // slow down for wheel loop filter operation
    millisElapsed = millis() - startMillis;
  };

  return(wallDist);
} // end calMove()

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
  initIna219();

  ledRG(1, 0);

  openClaws(100, true);

  // initialize robot position
  robotRefX = 0;
  robotRefY = robStartY;
  robotX    = robotRefX;
  robotY    = robotRefY;

  objdet.initObjectDetect(serialOK, &pca9685, &sonicSensors, &bleSensors, wallAbsX, wallAbsY);

  wheels.initWheels(&pca9685);
  bleSensors.initSensors(serialOK);   
  bleSensors.calibrateAG = false;

  // Start threads
  // TODO: How to move these to my sensor library files?
  AGSensorHandlingThread.start(AGSensorHandling);
  HTSSensorHandlingThread.start(HTSSensorHandling);
  
  led.ledOff();
  ledRG(1, 1);

if(serialOK) {
Serial.print("Init robot XY = "); Serial.print(robotX); Serial.print(","); Serial.println(robotY);

Serial.print("PCA9685 V = "); Serial.print(ina219.getBusVoltage(), 3);
Serial.print("\t");
Serial.print("PCA9685 mA = "); Serial.print(ina219.getCurrent_mA(), 3);
Serial.print("\n");
}

} // end setup()

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
  loopState_Move2CanLidar,
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
int proximity;


//***********************************************************
//***********************************************************
void loop()
{

if(serialOK) {
Serial.print("PCA9685 V = "); Serial.print(ina219.getBusVoltage(), 3);
Serial.print("\t");
Serial.print("PCA9685 mA = "); Serial.print(ina219.getCurrent_mA(), 3);
Serial.print("\n");
}


while(0){ // calibrate range sensor positioning
  objdet.positionRangeSensors(90);
  delay(20000);
  objdet.positionRangeSensors(0);
  delay(20000);
  objdet.positionRangeSensors(180);
  delay(20000);
  objdet.positionRangeSensors(0, -180);
  delay(20000);

}

if(0)
{ // get data to calibrate sec/cm vs voltage and left/right motion at speed = 1.0
  // (each motion has different motor strengths and friction)
  // place sensors at 90 being able to see the X walls on each side
  // travel motion left and right until close to walls for maximum 
  // distance and time which provides most accurate calibration
  // print out voltage and travel distance and seconds
  // Manually stop the infinate loop when enough data is collected
  // This will be used to determine a math formula for secPerCm(Voltage, Motion)

  // Do we need to calibrate for different speeds? 1.0 and 0.5?

  // 0 Move to -X wall until close, save measured distance to -X wall and current time
  float speed;
  int motion;  

  unsigned long startMillis;  
  float travelTimeSec;
  int travelDistCm;
  int wallDistA, wallDistB;
  float busV;


  speed = 1.0;
  speed = 0.5;
  motion = left;
  motion = rev;

  // point sensors to walls
  // when left-right robot is parallel to walls
  // when fwd-rev robot is facing walls
  if(motion == left) objdet.positionRangeSensors(90, -90);
  else               objdet.positionRangeSensors(0, -180);

  wallDistA = calMove(speed, motion);
//while(1);
  // reverse direction
  motion = (motion==left)? right : fwd;

  while(1) {
    // 1 Move to +X wall until close, save measured distance to +X wall and current time
    startMillis = millis();    
    wallDistB = calMove(speed, motion);

    travelTimeSec = (millis() - startMillis)/1000.0;
    travelDistCm = (2*wallAbsX) - abs(wallDistB - wallDistA);
    busV = getBusVoltage();
    
if(serialOK) {
    // 2 Print Voltage, travel direction, distance traveled, amount of travel time
    Serial.print(busV); Serial.print(",\t");
    Serial.print(speed); Serial.print(",\t");
    Serial.print(motion); Serial.print(",\t");
    Serial.print(travelDistCm); Serial.print(",\t");
    Serial.print(travelTimeSec); Serial.print("\n");
}

    // calibrate IMU while stopped a while
    bleSensors.calibrateAG = true;
    delay(10000);
    bleSensors.calibrateAG = false;

    // Repeat 1 and 2 but moving to other wall
    wallDistA = wallDistB;
    // reverse direction
    switch(motion){
    case right: motion = left;  break;
    case left: motion  = right; break;
    case fwd: motion   = rev;   break;
    case rev: motion   = fwd;   break;
    }
  }
}
else
{ // 6 Can routine
  switch(loopState)
  {


  //***************************
  case loopState_gotoScanXY:

if(serialOK) {
  Serial.println("loopState_gotoScanXY");
}

if(serialOK) {
Serial.print("Go to scan location idx,x,y ");Serial.print(robotScanXYidx);Serial.print(", ");
Serial.print(robotScanXY[robotScanXYidx].X);Serial.print(", ");
Serial.print(robotScanXY[robotScanXYidx].Y);Serial.print("\n");
}

    movX = robotScanXY[robotScanXYidx].X - robotX;
    movY = robotScanXY[robotScanXYidx].Y - robotY;
    moveYX(movY, movX);

    // zero X offset
    {
    int x = robotX;
    measRobotX(robotX);
    moveXY(x-robotX, 0);
    }
    
    loopState =  loopState_Scan4Can;
    break; //case loopState_gotoScanXY  


  //***************************
  case loopState_Scan4Can:  
if(serialOK) {
  Serial.println("loopState_Scan4Can");
}

    bleSensors.readAPDSSensor(proximity);
    if(proximity < 50)
    { // Lidar detects can close and in front
      canXcm = 0;
      canYcm = 10;
      objectDetected = true;
      loopState = loopState_Move2CanLidar;
    }
    else
    { // proximity is not triggered, scan with sonar
      // TODO: is this stop movement redundant?
      // stop motion before starting heading calibration while stopped to scan
      stopMovement();

      bleSensors.calibrateAG = true; // calibrate while stopped

      // Scan for object positions, input current robot location for wall ignore
      objdet.scanRange(robotX, robotY);

      bleSensors.calibrateAG = false; // stop calibration after scan

      objectDetected = objdet.getObject(canDeg, canDist, canXcm, canYcm);

      if(objectDetected == true)
      {
        ledRG(0, 1); // Green

if(serialOK) {
Serial.print("canDeg "); Serial.print(canDeg); Serial.print(", ");
Serial.print("canDist ");Serial.print(canDist);Serial.print(", ");
Serial.print("canXcm "); Serial.print(canXcm); Serial.print(", ");
Serial.print("canYcm "); Serial.print(canYcm); Serial.print("\n");
}

        loopState = loopState_Move2Can;
      }
      else
      {

if(serialOK) {
  Serial.println("Object not detected in last scan, goto new scan location");
}

        robotScanXYidx++;
        if(robotScanXYidx < numRobotScanXY) loopState = loopState_gotoScanXY;
        else robotScanXYidx = 0; // not all cans found, start over!!
      }
    }
    break; // case loopState_Scan


  //***************************
  case loopState_Move2Can:

if(serialOK) {
  Serial.println("loopState_Move2Can");
}

    bleSensors.readAPDSSensor(proximity);

if(serialOK) {
Serial.print("Move2Can LIDAR "); Serial.print(proximity); Serial.print("\n");
}

    if(((abs(canDeg) <= 85) && (canYcm<5) && (abs(canXcm)<10)) || (proximity<5)) 
    { // move back and to the side to scan using lidar
      int yCm = canYcm -2;
      int xCm = (canXcm>=0)? -2:2;
      moveYX(yCm, xCm); // Can is close, move a bit toward back of can
      loopState = loopState_Move2CanLidar;
    }
    else if(abs(canDeg) > 85)
    { // backup before going to location
      int yCm = -15;
      int xCm = 0;
      moveYX(yCm, xCm);      
      xCm = canXcm;
      yCm = canYcm +15;
      moveXY(xCm, yCm);
      loopState = loopState_Scan4Can;
    }
    else
    { // go to can location
      if(canYcm > 1) moveXY(canXcm, canYcm); 
      else           moveYX(canYcm -3, canXcm);
      loopState = loopState_Scan4Can;
    }

    measRobotX(robotX);

if(serialOK) {
Serial.print("Move2Can robot XY = "); Serial.print(robotX); Serial.print(","); Serial.println(robotY);
}

    break; // case loopState_Move2Can


  //***************************
  case loopState_Move2CanLidar:

if(serialOK) {
  Serial.println("loopState_Move2CanLidar");
}

    // Move robot in X L or R until can center is detected using LIDAR
    if(canXcm >= 0) moveXY(+30,10,false,true); // move at most X=30, Y=10 cm
    else            moveXY(-30,10,false,true);
    measRobotX(robotX);
 
    bleSensors.readAPDSSensor(proximity);
    if(proximity<3) loopState = loopState_GrabCan;
    else            loopState = loopState_Scan4Can;
    break; // case loopState_Move2CanLidar


  //***************************
  case loopState_GrabCan:

if(serialOK) {
  Serial.println("loopState_GrabCan");
}

    // TODO: Verify can was grabbed with lidar and maybe color
    closeClaws();    
    measRobotX(robotX);
    loopState = loopState_Can2Goal;

//loopState = loopState_Done; // DEBUG stop
    break; // case loopState_GrabCan

  //***************************
  case loopState_Can2Goal:
    // calc movement to goal
    measRobotX(robotX);
    
    goalXcm = -robotX; // goto center X=0
    goalYcm = wallAbsY - robotY + 50; // try to go past goal
    // detect the goal walls while moving to goal and set robotY coordinate
    moveXY(goalXcm, goalYcm, true); 
    robotY = wallAbsY; // stopped when goal (wall Y) detected 
    measRobotX(robotX);
    
if(serialOK) {
Serial.print("At goal robot XY = "); Serial.print(robotX); Serial.print(","); Serial.println(robotY);
}

    // go 10cm more into goal area to ensure can is in
    moveXY(0, 10);

    // Open claws enough to release can but not disturb other cans
    openClaws(20);
    
    // backup out of the goal and measure the robotX coordinate
    moveXY(0, -35);
    measRobotX(robotX);

    // Fully open claws
    openClaws(100, true);

    // correct any X offset
    {
    int xCm = -robotX;
    moveXY(xCm,0);
    }
    
    canCount++;
    if(canCount < 6)    
      loopState = loopState_gotoScanXY;
    else
      loopState = loopState_Done;    
    break; // case loopState_Can2Goal


  //***************************
  case loopState_Done:

if(serialOK) {
  Serial.println("loopState_Done");
}
    ledRG(1, 0); // red

    // should move to center (0,0)
    moveXY(-robotX, -robotY);

    // zero X offset
    measRobotX(robotX);
    moveXY(-robotX, 0);

    // have robot do a 360 to celebrate
    move360();

    while(1) delay(1000);
    break; // case loopState_Done
     
  } // end switch loopState

  delay(1); // give control to other threads immediately

} // end else 6 Can routine

} // end loop()
