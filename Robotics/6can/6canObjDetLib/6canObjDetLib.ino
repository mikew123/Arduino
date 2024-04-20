// 6canSensorLowLRObjDet.ino

// Adding IMU to offset direction drift while driving
// also to control the rotation to be precise +-90 or +-180 degrees
// MRW 2/9/2023 moved lots of code to libraries to help manage code size
// MRW 2/12/2023 Added HC-SR04 range sensor 
// MRW 2/16/2023 Moved sensors low and on left right of body
// MRW 2/20/2023 Started working on objectr detection
// MRW 2/28/2023 Moved object detection code to ObjectDetect lib


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

ObjectDetect objdet;

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


int motion;


int wallAbsX=110; // wall X offset from any Y, R L equal, walls parrallel
int wallAbsY=122; // goals are at center of these walls at X=0
int robStartX=0;  // robot starting location X,Y = (0,-50)
int robStartY=-50; 

int robotRefX, robotRefY;
int robotX, robotY;
int canX, canY;

void setup()
{
  int motion;

  initSerial();

  led.initLed();
  led.ledOn();
  led.rgbLed(1,1,1); // white

  initPca9685();

  // initialize robot position
  robotRefX = 0;
  robotRefY = robStartY;
  robotX = robotRefX;
  robotY = robotRefY;

  objdet.initSonicSensors(&pca9685, &sonicSensors, wallAbsX, wallAbsY);
  wheels.initWheels(&pca9685);
  sensors.initSensors(serialOK);   

  // Start threads
  // TODO: How to move these to my sensor library files?
  MAGSensorHandlingThread.start(MAGSensorHandling);
  AGSensorHandlingThread.start(AGSensorHandling);
  HTSSensorHandlingThread.start(HTSSensorHandling);

  serialHandlingThread.start(serialHandling);

  sensors.calibrateAG = false;
  
  led.ledOff();
} 

// Mearure Left and right wall distance at 90 degree sensor angle
// if valid robotX is updated and true is returned
bool  measRobotX(int &robotX)
{
  bool validMeas;
  int scanDeg = 90;
  objdet.positionRangeSensors(scanDeg);
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
  
Serial.print("90 Deg Wall R,L = ");Serial.print(wallRX);Serial.print(",");Serial.print(wallLX);

if(validMeas){Serial.print("  Robot X = ");Serial.println(robotX);}
else {Serial.println(" invalid measurement");}

  return(validMeas);
}

void loop()
{
  int motion;
  float speed;

  // stop motion before starting heading calibration while stopped
  motion = stop;
  speed = 0;
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, motion, speed, sensors.heading);
  delay(50);
  sensors.calibrateAG = true; // calibrate while stopped

  // Scan for object positions
  objdet.scanRange(robotX, robotY);
  
  // move toward closest object
  int moveDeg;
  int moveDist;
  int xCm,yCm;
  int runTimeMs;
  unsigned long startMillis;  
  bool objectDetected;

  objectDetected = objdet.getObject(moveDeg, moveDist, xCm, yCm);

  if(objectDetected == true)
  {
  objdet.printRangeSensors(moveDeg, moveDist, xCm, yCm, 0,0,0,0);


    // travel to the can
    sensors.calibrateAG = false; // stop calibration while moving
    speed = 1.0;
    float calSecPerCm = 1.0/25; // calibrated for 25 cm/sec at speed==1.0

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
        wheels.WheelDrive(wheel, motion, speed, sensors.heading);
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
        wheels.WheelDrive(wheel, motion, speed, sensors.heading);
      }
      delay(50);
    }
  }
  else
  {
Serial.println("Object not detected in last scan");
  }

  // TODO: make direct robot X measurement using RL sensors at 90
  if(measRobotX(robotX) == false) robotX += xCm;
  robotY += yCm;
Serial.print("Robot XY = "); Serial.print(robotX); Serial.print(","); Serial.println(robotY);

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
