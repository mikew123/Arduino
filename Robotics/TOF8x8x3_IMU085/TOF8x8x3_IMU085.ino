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

  mrw 1/30/2024 controls 3 sensors and prints array on 8 lines
*/

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;
#define BNO08X_INT  -1
#define BNO08X_RST  -1
#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

// Create 3 sensors Left, Front, Right
SparkFun_VL53L5CX myImagerL;
SparkFun_VL53L5CX myImagerF;
SparkFun_VL53L5CX myImagerR;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output
int sharpenerPct = 30; // default 5%
int integrationTimeMsec = 20; // default 5ms

#define pinTofLED 8

#define pinSDA 0
#define pinSCL 1

#define pinTofLPL 2 /* active low */
#define pinTofLPF 3
#define pinTofLPR 4
#define pinTofRST 5 /* active high */
#define pinTofPwrEn 7 /* power down LOW */

#define i2cImuAddr  0x28 /* IMU */
#define i2cTofAddrD 0x29 /* default TOF */
#define i2cTofAddrL 0x2A /* Left TOF  */
#define i2cTofAddrF 0x2B /* Front TOF */
#define i2cTofAddrR 0x2C /* Right TOF */

#define rangingFreqHz 10
#define resolution (8*8) /*8*8 or 4*4 */

// attempt to code a text image pixel with a char
char pix[5] = {255,176,177,178,219};


// init sensor LPn pins
void initSensorPins() {
  pinMode(pinTofLPL, OUTPUT);
  digitalWrite(pinTofLPL, LOW);
  pinMode(pinTofLPF, OUTPUT);
  digitalWrite(pinTofLPF, LOW);
  pinMode(pinTofLPR, OUTPUT);
  digitalWrite(pinTofLPR, LOW);
  pinMode(pinTofRST, OUTPUT);
  digitalWrite(pinTofRST, LOW);
  pinMode(pinTofPwrEn, OUTPUT);
  digitalWrite(pinTofPwrEn, HIGH);

}

void initSensor(SparkFun_VL53L5CX *myImager, int addr, int pinTofLP) {
  // enable I2C on this sensor
  digitalWrite(pinTofLP, HIGH);
  if (myImager->begin() == false) {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  myImager->setAddress(addr);
  myImager->setRangingFrequency(rangingFreqHz);
  myImager->setResolution(resolution); // 8*8 or 4*4
  myImager->setSharpenerPercent(sharpenerPct);
  myImager->setIntegrationTime(integrationTimeMsec);
}

void initSensors() {
  Serial.println("Initializing sensor boards. This can take up to 10s. Please wait.");

  // power cycle to reset default i2c address
  digitalWrite(pinTofPwrEn, LOW);
  delay(100);
  digitalWrite(pinTofPwrEn, HIGH);
  delay(100);

  initSensor(&myImagerR, i2cTofAddrR, pinTofLPR);
  initSensor(&myImagerF, i2cTofAddrF, pinTofLPF);
  initSensor(&myImagerL, i2cTofAddrL, pinTofLPL);

  Serial.println("TOF sensors initialized OK");
}

void setup()
{
  pinMode(pinTofLED, OUTPUT);
  digitalWrite(pinTofLED, HIGH);

  Serial.begin(2000000);
  delay(1000);
  Serial.println("VL53L5CX 3X TOF 8x8 Imager");


  // init gpio to sensor LPn pins
  initSensorPins();

  // Init I2C Wire interface
  Wire.setSDA(pinSDA);
  Wire.setSCL(pinSCL);
  Wire.begin(); //This resets to 100kHz I2C
//  Wire.setClock(1000000); //Sensor IMU has max I2C freq of 400kHz 

  // Wire.setClock(100000); //For BNO085
  // //if (myIMU.begin(BNO08X_ADDR) == false) {  // Setup without INT/RST control (Not Recommended)
  // if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
  //   Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
  //   while (1);
  // }
  // Serial.println("BNO08x found!");

  Wire.setClock(1000000); //Sensor IMU has max I2C freq of 400kHz 

  initSensors();

  // assume all sensors are configured the same
  imageResolution = myImagerF.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImagerR.startRanging();
  myImagerF.startRanging();
  myImagerL.startRanging();

}

uint8_t status[3][64];
uint16_t distance_mm[3][64];
uint16_t sigma[3][64];
uint8_t reflect[3][64];
bool dataRdy[3] = {false,false,false};
uint8_t reflVal = 25;
uint16_t sigmVal = 10;

// returns data ready
bool getSensorData(SparkFun_VL53L5CX *myImager, int n) {
bool rdy = false;
  //Poll sensor for new data
  if (myImager->isDataReady() == true)
  {
    if (myImager->getRangingData(&measurementData)) //Read distance data into array
    {
      for(int i=0; i<imageResolution; i++) {
        status[n][i] =  measurementData.target_status[i];
        distance_mm[n][i] =  measurementData.distance_mm[i];
        sigma[n][i] = measurementData.range_sigma_mm[i];
        reflect[n][i] = measurementData.reflectance[i];
      }
      rdy = true;
    }
  }
  delay(5); //?? Small delay between polling ??
  return(rdy);
}

enum {
  serialMode_ROS2,
  serialMode_PLOT
};

int serialMode = serialMode_PLOT;

void loop()
{
  if (Serial.available() > 0) {
    // read the incoming string:
    String incomingString = Serial.readStringUntil('\n');

    int n1, n2, n3;
    char str[100];
    int val;

    n1 = sscanf(incomingString.c_str(), "MODE %s", &str);
    n2 = sscanf(incomingString.c_str(), "REFL %d", &val);
    n3 = sscanf(incomingString.c_str(), "SIGM %d", &val);

    if(n1==1 && strncmp("ROS2", str, 4)==0) serialMode = serialMode_ROS2;
    if(n1==1 && strncmp("PLOT", str, 4)==0) serialMode = serialMode_PLOT;
    if(n2==1) reflVal = val;
    if(n3==1) sigmVal = val;

  }

  if(!dataRdy[0]) dataRdy[0] = getSensorData(&myImagerL, 0);
  if(!dataRdy[1]) dataRdy[1] = getSensorData(&myImagerF, 1);
  if(!dataRdy[2]) dataRdy[2] = getSensorData(&myImagerR, 2);

  // Send frame of sensor data when all 3 sets are ready
  if (dataRdy[0]&dataRdy[1]&dataRdy[2]) {
    //The ST library returns the data transposed from zone mapping shown in datasheet
    int xmax = imageWidth - 1;
    int ymax = imageWidth * xmax;
    if(serialMode == serialMode_ROS2) {
      Serial.print("TOF8x8x3 ");
      for (int y = 0 ; y <= ymax ; y += imageWidth) { // 0, 64, 128
        for(int n=0;n<3;n++) { // 0, 1, 2
          for (int x = 0 ; x <= xmax ; x++) { // 0 to 63
            uint16_t g = sigma[n][x+y];
            uint16_t r = reflect[n][x+y];
            int s = status[n][x + y];
            if (s==5 && g<sigmVal && r>reflVal) {
              int d = distance_mm[n][x + y];
              Serial.print(d); Serial.print(" ");
            }
            else Serial.print("-1 ");
          }
        }
      }
    }
    else {
      for (int y = 0 ; y <= ymax ; y += imageWidth) { // 0,8,16
        for(int n=0;n<3;n++) { // 0,1,2
          for (int x = 0 ; x <= xmax ; x++) { // 0 to 7
            uint16_t g = sigma[n][x+y];
            uint16_t r = reflect[n][x+y];
            int s = status[n][x + y];
            if (s==5 && g<sigmVal && r>reflVal) {
              int d = distance_mm[n][x + y];
              // }
              char c = '.';
              if(d<=100)       c = '0';
              else if(d<=200)  c = '1';
              else if(d<=500)  c = '2';
              else if(d<=1000) c = '3';
              else if(d<=1500) c = '4';
              else if(d<=2000) c = '5';
              else if(d<=2500) c = '6';
              else if(d<=3000) c = '7';
              else if(d<=3500) c = '8';
              else             c = '.';
              Serial.print(c);
            }
            //else Serial.print("--- "); 
            else Serial.print(" "); 
          }
        }
        Serial.println();      
      }
    }

    if(serialMode == serialMode_ROS2) Serial.println();  
    else {
      Serial.print("|      |   ^^   |      |");
      Serial.println();
    }
    dataRdy[0] = false;
    dataRdy[1] = false;
    dataRdy[2] = false;
  }
}
