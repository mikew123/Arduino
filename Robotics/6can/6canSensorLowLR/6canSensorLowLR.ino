// sg90MotorRangeDemo.ino

// Adding IMU to offset direction drift while driving
// also to control the rotation to be precise +-90 or +-180 degrees
// MRW 2/9/2023 moved lots of code to libraries to help manage code size
// MRW 2/12/2023 Added HC-SR04 range sensor 
// MRW 2/16/2023 Moved sensors low and on left right of body

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

int closeScanDeg;
int closeScanDist;

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

void printRangeSensors(int scanDeg, int distR, int xR, int yR,int distL, int xL, int yL)
{
/*
       if(distance <  50) {led.rgbLed(1,1,1); delay(100);} // white
  else if(distance < 100) {led.rgbLed(0,1,0); delay(10);}// green  
  else if(distance < 200) {led.rgbLed(0,0,1); delay(1);}// blue  
  else led.rgbLed(1,0,0); // red
*/
  Serial.print(scanDeg); Serial.print(", ");
  Serial.print(" Right: ");
  Serial.print(distR); Serial.print(", ");
  Serial.print(xR); Serial.print(", ");
  Serial.print(yR); Serial.print(",");
  Serial.print(" Left: ");
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

// average x and y for detected objects
float xAvg, yAvg;
void detectObject(int scanDeg, int distance, int x, int y)
{
/*
  int objDistDiff;
  if(objSeqCnt==0) 
  {
    xAvg = 0;
    yAvg = 0;
    deg0 = rightDeg;
    dist0 = distance
    x0 = x;
    y0 = y;    
  }

  if(distance<95 && x<95 && y<95)
  { // consider for object
    if(objSeqCnt>0)
    {
      disDiff = abs(distance - dist0);
      



    lastObjDist = distance;
    } else {
      objSeqCnt = 1;
    }    
  } else { // do not consider, reset object detect data
    objSeqCnt=0;
  }
*/
}

// scan using ultrasonic sensors
// exit when scan is complete
bool scanMax2Min = true;
void scanRange()
{
  closeScanDist=395;
  closeScanDeg=0;
  int xR, yR;
  int xL, yL;

  for(scanDeg = (scanMax2Min ? scanDegRev : scanDegFwd); 
                (scanMax2Min ? (scanDeg>=scanDegFwd) : (scanDeg<=scanDegRev)); 
      scanDeg += (scanMax2Min ? -scanDegInc : scanDegInc))
  {
    positionRangeSensors(scanDeg);
    sonicSensors.startAsync(0);
delay(25); // makes measurement more stable???
    while (!sonicSensors.isFinished()) {delay(25); Serial.print(".");}
    int distanceR = sonicSensors.getDist_cm(0);
    int distanceL = sonicSensors.getDist_cm(1);
    limitDistance(distanceR);
    limitDistance(distanceL);

    if(distanceR<closeScanDist) {
      closeScanDist = distanceR;
      closeScanDeg = scanDeg;
Serial.print(closeScanDeg); Serial.print(" @ "); Serial.println(closeScanDist);
    }

    rangePolar2Xy(scanDeg, distanceR, xR, yR);
    xyCm2CharMatrix(xR, yR, 'R');       
    rangePolar2Xy(-scanDeg, distanceL, xL, yL);
    xyCm2CharMatrix(xL, yL, 'L');       
    printRangeSensors(scanDeg, distanceR, xR, yR, distanceL, xL, yL);
    
    detectObject(scanDeg, distanceR, xR, yR);
  }
  scanMax2Min = ! scanMax2Min;

  // add closest object
  rangePolar2Xy(closeScanDeg, closeScanDist, xR, yR);
  xyCm2CharMatrix(xR,yR,'@');       

}


void loop()
{
  // stop motion before calibrating heading
  motion = stop;
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, motion, speed, sensors.heading);
delay(5000);

  sensors.calibrateAG = (motion == stop);
  // Scan for object positions
  initCharMatrix();
  scanRange();
  printCharMatrix();
  delay(5000);
  
  // move toward closest object
  int xCm,yCm;
  int runTimeMs;
  unsigned long startMillis;  
  rangePolar2Xy(closeScanDeg, closeScanDist, xCm, yCm);
printRangeSensors(closeScanDeg, closeScanDist, xCm, yCm, 0,0,0);
  int xCmOff=+8; // side sensor offset from center
  int yCmOff=-10; // side center offset from front + 10cm to not hit the can
  xCm+=xCmOff;
  yCm+=yCmOff;
printRangeSensors(closeScanDeg, closeScanDist, xCm, yCm, 0,0,0);


  // travel to the can
  speed = 1.0;
  float calSecPerCm = 1.0/25; // calibrated for 25 cm/sec

  // move in X axis
  if(xCm>=0) motion = right; else motion = left;
  sensors.calibrateAG = (motion == stop);
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
