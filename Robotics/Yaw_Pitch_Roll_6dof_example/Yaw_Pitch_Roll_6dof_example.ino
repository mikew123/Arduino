/************************************************************************************
 * 	
 * 	Name    : Comp6DOF_n0m1 Library Example: Yaw, Pitch, Roll                       
 * 	Author  : Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com                       
 * 	Date    : Feb 27th 2012                                    
 * 	Version : 0.1                                              
 * 	Notes   : Arduino Library for compass tilt compensation and hard iron offset 
 *
 ***********************************************************************************/

// MRW 2/6/2023 Modified to use LSM9D IMU, outputs are not good!!!
//#include <I2C.h>
//#include <MMA8453_n0m1.h>
#include <Comp6DOF_n0m1.h>
//#include <HMC5883L.h> // Reference the HMC5883L Compass Library

// creates IMU class for sensors on the nano 33 BLE board
#include <Arduino_LSM9DS1.h>

//MMA8453_n0m1 accel;
Comp6DOF_n0m1 sixDOF;
//HMC5883L compass;


void setup()
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial Started");

/*
  accel.dataMode(true, 2); //enable highRes 10bit, 2g range [2g,4g,8g]

  compass = HMC5883L(); // Construct a new HMC5883 compass.

  error = compass.SetScale(1.3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
*/

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized");

  /* hard iron offset finder
   int doneoffset =0;
   while (doneoffset ==0)
   {
   // Should indicate with a blink right here
   int donecombo = 0;
   while ( donecombo == 0 )   // load tuning array 
   {  
   delay (50);
   MagnetometerRaw  raw = compass.ReadRawAxis();
   donecombo = sixDOF.deviantSpread (raw.XAxis, raw.YAxis, raw.ZAxis);
   }
   
   doneoffset = sixDOF.calOffsets();
   }
   */

}

float mx, my, mz, ax, ay, az;
int imx, imy, imz, iax, iay, iaz;

void loop()
{
//float mx, my, mz, ax, ay, az;
//int imx, imy, imz, iax, iay, iaz;

  // poll sensors for new data
//  accel.update();
//  MagnetometerRaw raw = compass.ReadRawAxis();

  while (IMU.accelerationAvailable() == 0)
  { 
    delay(1);
  }
  IMU.readAcceleration(ax, ay, az);

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  

  //NOTE my conversions must not be correct, the output is garbage
  // convert float to integer for library
  // acceleromoter integer is +/2g 10 bits
  iax = 1024*(ax/2);
  iay = 1024*(ay/2);
  iaz = 1024*(az/2);
  // magnetometer integer range is +-1.3Ga gain = 1090? 
  imx = 1090*mx;
  imy = 1090*my;
  imz = 1090*mz;

  // offset compass by hard iron
  // raw.XAxis -= 40;
  // raw.YAxis -= 100;
  // raw.ZAxis -= 350;

  //enter compass data and accel data for calculation
//  sixDOF.compCompass(raw.XAxis, raw.YAxis, raw.ZAxis, accel.x(), accel.y(), accel.z(), false);
  sixDOF.compCompass(imx, imy, imz, iax, iay, iaz, false);

  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az-1.0); Serial.print(", ");
  Serial.print(mx/100.0); Serial.print(", ");
  Serial.print(my/100.0); Serial.print(", ");
  Serial.print(mz/100.0); Serial.print("\n");

  Serial.print ("  Roll: ");
  Serial.print (sixDOF.roll()/100); 
  Serial.print ("  Pitch: ");
  Serial.print (sixDOF.pitch()/100);      
  Serial.print ("  Yaw: ");
  Serial.print (sixDOF.yaw()/100);   
  Serial.println ("");
  }
  
  delay(1000);

}




