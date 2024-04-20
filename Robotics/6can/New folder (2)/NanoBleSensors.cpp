// NanoBleSensors.cpp

// Library to control the sensors on NanoBLE
// Uses the mbed OS semaphores, threads, etc
// Mike Williamson 2/8/2023

#include "NanoBleSensors.h"


int NanoBleSensors::readMAGSensor(float &x, float &y, float &z)
{

  // wait for gyroscope data ready to read
  sensorSerialSemaphore.acquire();
  while (IMU.magneticFieldAvailable() == 0)
  { 
    sensorSerialSemaphore.release();
    delay(1);
    sensorSerialSemaphore.acquire();
  }
  IMU.readMagneticField(x, y, z);
  sensorSerialSemaphore.release();

}

// read IMU accellerator and gyro sensors and 
// filter out roll pitch and heading
// uses the sensor serial port semiphore
int NanoBleSensors::readAGSensorFiltered(float &pitch, float &roll, float &heading)
{
  float ax, ay, az;
  float gx, gy, gz;
  // wait for gyroscope data ready to read
  sensorSerialSemaphore.acquire();
  while (IMU.gyroscopeAvailable() == 0)
  { 
    sensorSerialSemaphore.release();
    delay(1);
    sensorSerialSemaphore.acquire();
  }
  IMU.readGyroscope(gx, gy, gz);
  // assume accel is avialable same time as gyro
  IMU.readAcceleration(ax, ay, az);
  sensorSerialSemaphore.release();

  // update the filter, which computes orientation
  // imuFilter.updateIMU(gx, gy, gz, ax, ay, az);
  // the sensor is mounted vertically around the y axis
  // switch the -X and Z axis sensor values
  imuFilter.updateIMU(gz, gy, -gx, az, ay, -ax);
  roll    = imuFilter.getRoll();
  pitch   = imuFilter.getPitch();
  heading = imuFilter.getYaw();
}

int NanoBleSensors::readHTSSensor()
{
  sensorSerialSemaphore.acquire();
  currentTemperature = HTS.readTemperature();
  sensorSerialSemaphore.release();  
}

void NanoBleSensors::initSensors(bool serialOK)
{
// local variables
float GyroSampleRate;

  if(serialOK) Serial.println("initSensors"); 

  if (!APDS.begin()) {
    if(serialOK) Serial.println("Error initializing APDS9960 sensor!");
    while(1);
  }
  if(serialOK) Serial.println("APDS sensor OK"); 

  if (!HTS.begin()) {
    if(serialOK) Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  if(serialOK) Serial.println("HTS sensor OK"); 

  if (!IMU.begin()) {
    if(serialOK) {
      if(serialOK) Serial.println("Failed to initialize IMU!");
      while(1);
    }
  }
  if(serialOK) Serial.println("IMU sensor OK"); 

  // Accelerometer and Gyroscope sensors
  GyroSampleRate = IMU.gyroscopeSampleRate();
  imuFilter.begin(GyroSampleRate);

  //TODO: Capture current heading offset and calibrate drift
  // Why is there heading drift?
  initialHeading = 180.0; // used to zero the initial heading
  headingDriftRate = 0;
 // headingDriftRate += -0.0060; // manual cal

  if(serialOK) Serial.println("Calibrating IMU");
  // calibrate heading  drift rate by averaging 2000 samples
  // run sensor for 10 reads first to "warm" it up
  // this should take about 20 seconds
  int calibrationReads = 2000;
  float p, r, heading;
  float last_heading;
  for(int i=0; i<10; i++) readAGSensorFiltered(p,r,heading);
  last_heading = heading;
  for(int i=0; i<calibrationReads; i++)
  {
    readAGSensorFiltered(p,r,heading);
    headingDriftRate += (heading-last_heading);
    last_heading = heading;
   }
  headingDriftRate = headingDriftRate/calibrationReads;
  initialHeading = heading;

  // Magnometer sensor
  float MagSampleRate = IMU.magneticFieldSampleRate();
  float x,y,z;
  // Get initial magnometer reading for offset
  // Average 20 reads
  for(int i=0;i<20;i++)
  {
    readMAGSensor(x,y,z);
    iMagX+=x;
    iMagY+=y;
    iMagZ+=z;
  }
  iMagX = iMagX/20;
  iMagY = iMagY/20;
  iMagZ = iMagZ/20;

  //TODO: Capture temperature during init
  initialTemperature = HTS.readTemperature();


  if(serialOK)
  {
    Serial.print("Gyroscope sample rate = ");
    Serial.print(GyroSampleRate);
    Serial.println(" Hz");
    Serial.print("Gyroscope initial heading = ");
    Serial.println(initialHeading);
    Serial.print("Gyroscope drift rate = ");
    Serial.println(headingDriftRate,4);

    Serial.print("Magnometer sample rate = ");
    Serial.print(MagSampleRate);
    Serial.println(" Hz");

    Serial.print("Temperature = ");
    Serial.print(initialTemperature);
    Serial.println(" °C");

    delay(5000);
  }
}


void NanoBleSensors::HTSSensorHandling()
{
  while(1){
    // get current temperature
    readHTSSensor();
    delay(1000);
  }

}

void NanoBleSensors::MAGSensorHandling()
{
float x,y,z;
  while(1)
  {
    readMAGSensor(x, y, z);
    // offset initial sensor reading
    y-=iMagY;
     // filter magnometer Y which is used as compus 
    float A = 0.01;
    magY = A*y + (1-A)*magY;
    // unfiltered
    magX=x;
    magZ=z;
 }
 delay(1);
}


void NanoBleSensors::AGSensorHandling()
{

  float heading_l=0;
  float prevRawHeading = 0;
  float headingDiff;
  float A = 0.001;
  int count=0;
  bool prevCal=false;
  float pitch, roll, rawHeading;
  float stopHeadingOffset=0;
  float headingDrift=0;

  while(1)
  {
    readAGSensorFiltered(pitch, roll, rawHeading);
    // adjust heading drift rate while motion is stopped
    if(calibrateAG) if(count>0)
    {
      headingDiff = rawHeading-prevRawHeading;
      if(headingDiff<0.1 && headingDiff>-0.1)
        headingDriftRate = A*(headingDiff) +(1-A)*headingDriftRate;
    }

    if(prevCal != calibrateAG)
    {
      // use the magnometer to correct heading 
      stopHeadingOffset += heading_l - magY;
    }
    prevCal = calibrateAG;

    // heading drift based on rate of drift
    headingDrift+=headingDriftRate;
    // NOTE: is this correct wraping?
    if(headingDrift<=-360.0) headingDrift+=360.0;    
//    if(headingDrift>=+360.0) headingDrift-=360.0; 

    // heading is 0 initialy with a range of +-180
    heading_l  = rawHeading;
    heading_l -= stopHeadingOffset;
    heading_l -= (initialHeading+headingDrift);
    // NOTE: is this correct wraping?
    if(heading_l> +180.0) heading_l-=360.0;
    if(heading_l<=-180.0) heading_l+=360.0;

    // update public heading variable
    heading = heading_l;
    prevRawHeading = rawHeading;
    count++;
  }
}


