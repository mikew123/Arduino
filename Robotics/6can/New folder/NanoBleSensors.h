// NanoBleSensors.h

// Library to control sensors on Arduino Nano BLE
// Mike Williamson 2/8/2023

#ifndef _NanoBleSensors_h
#define _NanoBleSensors_h

#include <Arduino.h>
// Nano BLE sensors
#include <Arduino_APDS9960.h> // light
#include <Arduino_HTS221.h>   // temperature
#include <Arduino_LSM9DS1.h>  // motion
#include <MadgwickAHRS.h>     // motion filter

#include <mbed.h>  // rtos

class NanoBleSensors
{
public:
  int readMAGSensor(float &x, float &y, float &z);
  int readAGSensorFiltered(float &pitch, float &roll, float &heading);
  int readHTSSensor();

  void initSensors(bool serialOK); 
  void MAGSensorHandling();
  void AGSensorHandling();
  void HTSSensorHandling();

  float initialTemperature;
  float currentTemperature;
  float iMagX, iMagY, iMagZ;
  float magX=0, magY=0, magZ=0;

  float initialHeading;
  float headingDriftRate;
  float heading;

  bool calibrateAG;

private:
  rtos::Semaphore sensorSerialSemaphore {1};
  Madgwick imuFilter {};
};


#endif