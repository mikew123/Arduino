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

// Create 3 sensors Left, Front, Right
SparkFun_VL53L5CX myImagerL;
SparkFun_VL53L5CX myImagerF;
SparkFun_VL53L5CX myImagerR;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output
int sharpenerPct = 20; // default 5%
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

  Serial.begin(115200);
  delay(1000);
  Serial.println("VL53L5CX TOF Imager");


  // init gpio to sensor LPn pins
  initSensorPins();

  Wire.setSDA(pinSDA);
  Wire.setSCL(pinSCL);
  Wire.begin(); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor IMU has max I2C freq of 400kHz 

  initSensors();

  // assume all sensors are configured the same
  imageResolution = myImagerF.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImagerR.startRanging();
  myImagerF.startRanging();
  myImagerL.startRanging();

}

int status[3][64];
int distance_mm[3][64];
bool dataRdy[3] = {false,false,false};

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
      }
      rdy = true;
    }
  }
  delay(5); //Small delay between polling
  return(rdy);
}

void loop()
{

  if(!dataRdy[0]) dataRdy[0] = getSensorData(&myImagerL, 0);
  if(!dataRdy[1]) dataRdy[1] = getSensorData(&myImagerF, 1);
  if(!dataRdy[2]) dataRdy[2] = getSensorData(&myImagerR, 2);

  if (dataRdy[0]&dataRdy[1]&dataRdy[2]) {
    //The ST library returns the data transposed from zone mapping shown in datasheet
    //Pretty-print data with increasing y, decreasing x to reflect reality
    //Changed to list 3 sensors readings Left to Right
    int xmax = imageWidth - 1;
    int ymax = imageWidth * xmax;
    for (int y = 0 ; y <= ymax ; y += imageWidth) {
      for(int n=0;n<3;n++) {
//        for (int x = xmax ; x >= 0 ; x--) {
        for (int x = 0 ; x <= xmax ; x++) {
          int s = status[n][x + y]; // 
          if (s==5  || s==9 ) {
//            int d = distance_mm[n][x + y]/600;
            int d = distance_mm[n][x + y];
            //char f[3];
            //sprintf(f,"%3d ", d);
            //Serial.print(f);
            // switch(d) { // 255,176,177,178,219
            // case 0: Serial.print("#"); break;
            // case 1: Serial.print("@"); break;
            // case 2: Serial.print("="); break;
            // case 3: Serial.print("+"); break;
            // case 4: Serial.print("-"); break;
            // default:  Serial.print("."); break;
            // }
            char c = '.';
            if(d<=100)       c = '0';
            else if(d<=200)  c = '1';
            else if(d<=500)  c = '2';
            else if(d<=1000) c = '3';
            else if(d<=1500) c = '4';
            else if(d<=2000) c = '5';
            else             c = '.';
            Serial.print(c);
          }
          //else Serial.print("--- "); 
          else Serial.print(" "); 
        }
      }
      Serial.println();
    }
    Serial.println("|       |  ^^  |       |");
    
    dataRdy[0] = false;
    dataRdy[1] = false;
    dataRdy[2] = false;
  }
}
