#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* From example sensorapi
  This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to SCL pin (analog 5 on Arduino UNO)
   Connect SDA to SDA pin (analog 4 on Arduino UNO)
   Connect VDD to 3-5V DC (depending on your board's logic level)
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers

   3/22/2014 MRW - Copied from examples for Robo24ImuTest to develop code
                    before adding to the TOF8x8x3_ROS2 code
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_HZ (50)
#define BNO055_SAMPLERATE_DELAY_MS (1000/BNO055_SAMPLERATE_HZ)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

int32_t last_millis = 0;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{

//  Wire.setClock(1000000);
//  Wire.setSDA(0);
//  Wire.setSCL(1);
//  Wire.setTimeout(1000);
  
  Serial.begin(1000000);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin( OPERATION_MODE_IMUPLUS ))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

//  delay(1000);

  // /* Display some basic information on this sensor */
  // displaySensorDetails();

  // /* Optional: Display current status */
  // displaySensorStatus();

  bno.setExtCrystalUse(true);

  last_millis = millis();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  while((millis()-last_millis)<BNO055_SAMPLERATE_DELAY_MS);
  last_millis = millis();

  /* Get a new sensor event */
  sensors_event_t angVelocityData, linearAccelData;
  bool a = bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bool v = bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Quaternion quat = bno.getQuat();

  Serial.print("IMU ");
  Serial.print(millis());Serial.print(" ");
  Serial.print(angVelocityData.acceleration.x,4); Serial.print(" ");
  Serial.print(angVelocityData.acceleration.y,4); Serial.print(" ");
  Serial.print(angVelocityData.acceleration.z,4); Serial.print(" ");
  Serial.print(linearAccelData.acceleration.x,4); Serial.print(" ");
  Serial.print(linearAccelData.acceleration.y,4); Serial.print(" ");
  Serial.print(linearAccelData.acceleration.z,4); Serial.print(" ");
  Serial.print(quat.w(), 4); Serial.print(" ");
  Serial.print(quat.x(), 4); Serial.print(" ");
  Serial.print(quat.y(), 4); Serial.print(" ");
  Serial.print(quat.z(), 4); Serial.println(""); // end line

  // printEvent(&angVelocityData);
  // printEvent(&linearAccelData);

  // // Quaternion data
  // Serial.print("qW: ");
  // Serial.print(quat.w(), 4);
  // Serial.print(" qX: ");
  // Serial.print(quat.x(), 4);
  // Serial.print(" qY: ");
  // Serial.print(quat.y(), 4);
  // Serial.print(" qZ: ");
  // Serial.print(quat.z(), 4);
  // Serial.print("\n");

  // /* Optional: Display calibration status */
  // displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
//  Serial.println("");

  /* Wait the specified delay before requesting nex data */
//  delay(BNO055_SAMPLERATE_DELAY_MS);

}




// void printEvent(sensors_event_t* event) {
//   double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//   if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//     Serial.print("Accl:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else if (event->type == SENSOR_TYPE_ORIENTATION) {
//     Serial.print("Orient:");
//     x = event->orientation.x;
//     y = event->orientation.y;
//     z = event->orientation.z;
//   }
//   else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
//     Serial.print("Mag:");
//     x = event->magnetic.x;
//     y = event->magnetic.y;
//     z = event->magnetic.z;
//   }
//   else if (event->type == SENSOR_TYPE_GYROSCOPE) {
//     Serial.print("AngVel:");
//     x = event->gyro.x;
//     y = event->gyro.y;
//     z = event->gyro.z;
//   }
//   else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
//     Serial.print("Rot:");
//     x = event->gyro.x;
//     y = event->gyro.y;
//     z = event->gyro.z;
//   }
//   else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//     Serial.print("LinAcc:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else if (event->type == SENSOR_TYPE_GRAVITY) {
//     Serial.print("Gravity:");
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else {
//     Serial.print("Unk:");
//   }

//   Serial.print("\tx= ");
//   Serial.print(x);
//   Serial.print(" |\ty= ");
//   Serial.print(y);
//   Serial.print(" |\tz= ");
//   Serial.println(z);
// }

// /**************************************************************************/
// /*
//     Displays some basic information on this sensor from the unified
//     sensor API sensor_t type (see Adafruit_Sensor for more information)
// */
// /**************************************************************************/
// void displaySensorDetails(void)
// {
//   sensor_t sensor;
//   bno.getSensor(&sensor);
//   Serial.println("------------------------------------");
//   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//   Serial.print  ("Type:         "); Serial.println(sensor.type);
//   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
//   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
//   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
//   Serial.println("------------------------------------");
//   Serial.println("");
//   delay(500);
// }

// /**************************************************************************/
// /*
//     Display some basic info about the sensor status
// */
// /**************************************************************************/
// void displaySensorStatus(void)
// {
//   /* Get the system status values (mostly for debugging purposes) */
//   uint8_t system_status, self_test_results, system_error;
//   system_status = self_test_results = system_error = 0;
//   bno.getSystemStatus(&system_status, &self_test_results, &system_error);

//   /* Display the results in the Serial Monitor */
//   Serial.println("");
//   Serial.print("System Status: 0x");
//   Serial.println(system_status, HEX);
//   Serial.print("Self Test:     0x");
//   Serial.println(self_test_results, HEX);
//   Serial.print("System Error:  0x");
//   Serial.println(system_error, HEX);
//   Serial.println("");
//   delay(500);
// }

// /**************************************************************************/
// /*
//     Display sensor calibration status
// */
// /**************************************************************************/
// void displayCalStatus(void)
// {
//   /* Get the four calibration values (0..3) */
//   /* Any sensor data reporting 0 should be ignored, */
//   /* 3 means 'fully calibrated" */
//   uint8_t system, gyro, accel, mag;
//   system = gyro = accel = mag = 0;
//   bno.getCalibration(&system, &gyro, &accel, &mag);

//   /* The data should be ignored until the system calibration is > 0 */
//   Serial.print("\t");
//   if (!system)
//   {
//     Serial.print("! ");
//   }

//   /* Display the individual values */
//   Serial.print("Sys:");
//   Serial.print(system, DEC);
//   Serial.print(" G:");
//   Serial.print(gyro, DEC);
//   Serial.print(" A:");
//   Serial.print(accel, DEC);
//   Serial.print(" M:");
//   Serial.print(mag, DEC);
// }

