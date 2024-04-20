/*
  Arduino LSM9DS1 - Simple Magnetometer

  This example reads the magnetic field values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.

  MRW 2/3/2023 Added initial offset, sum xyz and servo activation

*/

#include <Arduino_LSM9DS1.h>
float ix, iy, iz;

#include <PCA9685.h>  // servo driver
PCA9685 pca9685;
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ


void initPca9685(){
  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(ix, iy, iz);
  }

  initPca9685();

}

int loopCnt=0;
int drv = 1520;

void loop() {
  float x, y, z;
  float xyz;

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    x-=ix;
    y-=iy;
    z-=iz;

    xyz = x+y+z;
    
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
//    Serial.print('\t');
 //   Serial.print(xyz);
    Serial.print('\n');


    loopCnt++;

  }

  if(loopCnt==100)
  {
    loopCnt=0;
    if(drv==1560)      drv=1520;
    else if(drv==1520) drv=1480;
    else if(drv==1480) drv=1560;
    
    for(int servo=0;servo<16;servo++)
      pca9685.setChannelServoPulseDuration(servo, drv);

  }
}
