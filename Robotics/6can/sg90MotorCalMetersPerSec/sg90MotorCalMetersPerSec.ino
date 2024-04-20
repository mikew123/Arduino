// sg90MotorCalMetersPerSec.ino

// Adding IMU to offset direction drift while driving
// also to control the rotation to be precise +-90 or +-180 degrees
// MRW 2/9/2023 moved lots of code to libraries to help manage code size
// MRW 2/14/2023 modified for meeter per second calibrationReads
//     goes fwd, rev, left, right for 4 seconds with a long wait between
//     the wheels library is manually updated after measure distance traveled
//     the target is 250cm/sec at speed = 1.0, 1 meter per motion run

#include <Arduino.h>

// My libraries
#include <NanoBleLed.h>
#include <NanoBleSensors.h>
#include <MecanumWheels.h>

// external modules
#include <PCA9685.h>  // servo driver

#include <mbed.h>  // rtos threads and semaphores
#include <string>

NanoBleLed led;
NanoBleSensors sensors;
MecanumWheels wheels;

PCA9685 pca9685;
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ

bool serialOK = 0; // assume serial port is OK until time out waiting
void initSerial()
{
  Serial.begin(9600);
  int i;
  for(i=0; !Serial; i++) {if (i >= 5000) break; delay(1);}
  if (i < 5000) serialOK=1;  
  if(serialOK) Serial.println("initSerial OK");  
  
}

// prints values to serial port after initialization
rtos::Thread serialHandlingThread;

// TODO: How to move these to my sensor library files?
// Magnometer sensor
rtos::Thread MAGSensorHandlingThread;
void MAGSensorHandling() {sensors.MAGSensorHandling();}
// Accellerometer and Gyroscope sensors
rtos::Thread AGSensorHandlingThread;
void AGSensorHandling() {sensors.AGSensorHandling();}
// Temp sensor
rtos::Thread HTSSensorHandlingThread;
void HTSSensorHandling() {sensors.HTSSensorHandling();}

void initPca9685(){
  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);
}

void setup()
{
  initSerial();

  led.initLed();
  led.ledOn();
  led.rgbLed(1,1,1); // white

  initPca9685();
  wheels.initWheels(&pca9685);
  sensors.initSensors(serialOK);   

  // Start threads
  // TODO: How to move these to my sensor library files?
  MAGSensorHandlingThread.start(MAGSensorHandling);
  AGSensorHandlingThread.start(AGSensorHandling);
  HTSSensorHandlingThread.start(HTSSensorHandling);

  serialHandlingThread.start(serialHandling);

  led.rgbLed(0,0,0);
  led.ledOff();

}


int motions[] = {
//  fwd,
//  rev,
  left,
  right /*,
  fwd_left,
  rev_right,
  rev_left,
  fwd_right */
};

int motion; // global for printing
int updateHeadingDelay = 100; // update heading every 100ms
int Tdelay = 4000; // run each motion for 4 sec, target 1 meter
int speed = 1.0;

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

void loop()
{
  // to use motions list the variable must be like int m, motion will not work!
  for(int m : motions) {
    motion  = m;

    motionLeds(motion);

    // calibrate IMU AG when stopped
    sensors.calibrateAG = motion==stop;

//    int TdelayAccum=0;
    unsigned long startMillis = millis();

//    while(TdelayAccum<Tdelay){   
    while((millis() - startMillis) < Tdelay)
    {   
      for (int wheel=0;wheel<4;wheel++){
        wheels.WheelDrive(wheel, motion, speed, sensors.heading);
      } // end for servos
      delay(updateHeadingDelay);
//      TdelayAccum+=updateHeadingDelay; // update heading rate
    } // end while Tdelay 
    
    motion = stop;
    for (int wheel=0;wheel<4;wheel++)
      wheels.WheelDrive(wheel, motion, speed, sensors.heading);
    sensors.calibrateAG = motion == stop;

    delay(20000);  // 20 seconds to mrk position
  } // end for motions
}

// print various global variables 500ms rate
void serialHandling()
{
  if(serialOK) while(1)
  {
    Serial.print(motion/10.0);
    Serial.print('\t');
    Serial.print(sensors.magY);  // magnometer primary signal
    Serial.print('\t');
    Serial.print(sensors.heading);
    Serial.print('\t');
    Serial.print(100*sensors.headingDriftRate,6);
    Serial.print('\n');

    delay(500);
  }
}
