/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

// Create 3 sensors Left, Front, Right
SparkFun_VL53L5CX myImagerL;
VL53L5CX_ResultsData measurementDataL; // Result data class structure, 1356 byes of RAM
SparkFun_VL53L5CX myImagerF;
VL53L5CX_ResultsData measurementDataF; // Result data class structure, 1356 byes of RAM
SparkFun_VL53L5CX myImagerR;
VL53L5CX_ResultsData measurementDataR; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

#define pinLED 8

#define pinSDA 0
#define pinSCL 1

#define pinLPL 2
#define pinLPF 3
#define pinLPR 4

#define i2cAddrD 0x29 /* default */
#define i2cAddrL 0x2A /* Left sensor */
#define i2cAddrF 0x2B /* Front sensor */
#define i2cAddrR 0x2C /* Right sensor */

// init sensor LPn pins
void initSensorLpPins() {
  pinMode(pinLPL, OUTPUT);
  digitalWrite(pinLPL, LOW);
  pinMode(pinLPF, OUTPUT);
  digitalWrite(pinLPF, LOW);
  pinMode(pinLPR, OUTPUT);
  digitalWrite(pinLPR, LOW);
}

void configSensorAddrs() {
  digitalWrite(pinLPL, HIGH);
  myImagerL.setAddress(i2cAddrL);
  digitalWrite(pinLPL, LOW);

  digitalWrite(pinLPF, HIGH);
  myImagerF.setAddress(i2cAddrF);
  digitalWrite(pinLPF, LOW);

  digitalWrite(pinLPR, HIGH);
  myImagerR.setAddress(i2cAddrR);
  digitalWrite(pinLPR, LOW);

  // Enable i2c on all 3 sensors
  digitalWrite(pinLPL, HIGH);
  digitalWrite(pinLPF, HIGH);
  digitalWrite(pinLPR, HIGH);
}
void initSensors() {
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImagerL.begin() == false) {
    Serial.println(F("Left Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  if (myImagerF.begin() == false) {
    Serial.println(F("Front Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  if (myImagerR.begin() == false) {
    Serial.println(F("Right Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }

  // assume all sensors are configured the same
  myImagerL.setResolution(8*8); //Enable all 64 pads
  imageResolution = myImagerL.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  Serial.println("All 3 TOF sensors initialized OK");
}

void setup()
{

  Serial.begin(115200);
  delay(1000);
  Serial.println("VL53L5CX TOF Imager");


  // initialize i2c addresses of TOF sensors
  initSensorLpPins();

  Wire.setSDA(pinSDA);
  Wire.setSCL(pinSCL);
  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(1000000); //Sensor has max I2C freq of 400kHz 
  
  while(1) {}
  
  configSensorAddrs();

  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, HIGH);

  initSensors();

  myImagerF.startRanging();
}

void loop()
{
  //Poll sensor for new data
  if (myImagerF.isDataReady() == true)
  {
    if (myImagerF.getRangingData(&measurementDataF)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        {
          //Serial.print("\t");
          int d = measurementDataF.distance_mm[x + y]/10;
          char f[3];
          sprintf(f,"%2d ", d);
          int s = measurementDataF.target_status[x + y]; 
          if (s==5  || s==9 ) Serial.print(f);
          else                Serial.print("-- "); 
        }
        Serial.println();
      }
      Serial.println();
    }
  }

  delay(5); //Small delay between polling
}
