// sg90MotorRangeDemo.ino

// Adding IMU to offset direction drift while driving
// also to control the rotation to be precise +-90 or +-180 degrees
// MRW 2/9/2023 moved lots of code to libraries to help manage code size
// MRW 2/12/2023 Added HC-SR04 range sensor 

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
HC_SR04<A6> sensor(A7);   // sensor with echo A6 and trigger A7 pins


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

void initRangeSensor()
{
  sensor.beginAsync();  
  // init sensor servoss to point straight and level
  pca9685.setChannelServoPulseDuration(8, 1500); // horizontal
  pca9685.setChannelServoPulseDuration(12, 1500);// vertical
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
  initRangeSensor();

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
int servoZeroUs = 1500;
float usPerDeg = 900/90.0; // calibrated trial and error

int horSensorServo = 8;
int verSensorServo = 12;

int horDegMin = -90;
int horDegMax =  90;
int horDegInc = 1;
int horDeg;

int verDegList[] = {0};
int verDeg;

int closeDeg;
int closeDist;

int servoDeg2Us(int degree)
{
  int usec;
  usec = servoZeroUs + (usPerDeg * degree);
  return usec;
}

// position ultrasonic range sensors
int horDegLast = 0;
int verDegLast = 0;
void positionRangeSensors(int horDeg, int verDeg)
{
//  pca9685.setChannelServoPulseDuration(horSensorServo, servoDeg2Us(-horDeg)); // horizontal
//  pca9685.setChannelServoPulseDuration(verSensorServo, servoDeg2Us(-verDeg)); // vertical

  // move the range sensors 1 degree at a time to the desired position
  for(int deg = horDegLast;  
     ((horDeg<horDegLast)?(horDeg<=deg):(horDeg>=deg));
     (((horDeg<horDegLast)?(deg--):(deg++))))
  {
    pca9685.setChannelServoPulseDuration(horSensorServo, servoDeg2Us(-deg)); // horizontal
    delay(5); // slow down movement, less jerky
  }
  horDegLast = horDeg;

  for(int deg = verDegLast;  
     ((verDeg<verDegLast)?(verDeg<=deg):(verDeg>=deg));
     (((verDeg<verDegLast)?(deg--):(deg++))))
  {
    pca9685.setChannelServoPulseDuration(verSensorServo, servoDeg2Us(-deg)); // vertical
    delay(5); // slow down movement, less jerky
  }
  verDegLast = verDeg;

}

void printRangeSensors(int horDeg, int verDeg, int distance, int x, int y)
{
       if(distance <  30) {led.rgbLed(1,1,1); delay(1000);} // white
  else if(distance < 150) {led.rgbLed(0,1,0); delay(100);}// green  
  else if(distance < 300) {led.rgbLed(0,0,1); delay(10);}// blue  
  else led.rgbLed(1,0,0); // red

  Serial.print(verDeg); Serial.print(", ");
  Serial.print(horDeg); Serial.print(", ");
  Serial.print(distance); Serial.print(", ");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(-50); Serial.print(", ");
  Serial.print(350); Serial.print("\n");

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
void limitDistance(int verDeg, int &distance)
{
  if(distance > 395) distance = 395;
}

// scan using ultrasonic sensors
// exit when scan is complete
bool scanMax2Min = true;
void scanRange()
{
  closeDist=400;
  closeDeg=0;
  int x, y;

  for(int verDeg : verDegList) {
    for(horDeg = (scanMax2Min ? horDegMax : horDegMin); 
                 (scanMax2Min ? (horDeg>=horDegMin) : (horDeg<=horDegMax)); 
       horDeg += (scanMax2Min ? -horDegInc : horDegInc))
    {
      positionRangeSensors(horDeg, verDeg);
//      delay(10); // wait for motion jerk to settle
      sensor.startAsync(0);
      while (!sensor.isFinished()) delay(1);
      int distance = sensor.getDist_cm();
      limitDistance(verDeg, distance);
      if(distance<closeDist) {
        closeDist = distance;
        closeDeg = horDeg;
      }
      rangePolar2Xy(horDeg, distance, x, y);
      xyCm2CharMatrix(x,y,'X');       
      printRangeSensors(horDeg, verDeg, distance, x, y);
    }
    scanMax2Min = ! scanMax2Min;
  }
  // add closest object
  rangePolar2Xy(closeDeg, closeDist, x, y);
  xyCm2CharMatrix(x,y,'@');       

}


void loop()
{
  // stop motion before calibrating heading
  motion = stop;
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, motion, speed, sensors.heading);
  sensors.calibrateAG = (motion == stop);
  // Scan for object positions
  initCharMatrix();
  scanRange();
  printCharMatrix();
  delay(10000);
  
  // move toward closest object
  int xCm,yCm;
  int runTimeMs;
  unsigned long startMillis;  
  rangePolar2Xy(closeDeg, closeDist, xCm, yCm);
  speed = 1.0;

  // move in X axis
  if(xCm>=0) motion = right; else motion = left;
  sensors.calibrateAG = (motion == stop);
  runTimeMs = ((motion==right)?+1:-1)*(1000L*xCm)/25; // calibrated for 25 cm/sec
  Serial.print("Move X "); Serial.print(xCm); 
  Serial.print(" cm For "); Serial.print(runTimeMs); Serial.println(" msec");
  startMillis = millis();
  while((millis() - startMillis) < runTimeMs)
  {   
    for (int wheel=0;wheel<4;wheel++){
      wheels.WheelDrive(wheel, motion, speed, sensors.heading);
      delay(50);
    } // end for servos
  }
  
  // move in Y axis
  if(yCm>=0) motion = fwd; else motion = rev;
  sensors.calibrateAG = (motion == stop);
  runTimeMs = ((motion==fwd)?+1:-1)*(1000L*yCm)/25; // calibrated for 25 cm/sec
  startMillis = millis();
  while((millis() - startMillis) < runTimeMs)
  {   
    for (int wheel=0;wheel<4;wheel++){
      wheels.WheelDrive(wheel, motion, speed, sensors.heading);
      delay(50);
    } // end for servos
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
