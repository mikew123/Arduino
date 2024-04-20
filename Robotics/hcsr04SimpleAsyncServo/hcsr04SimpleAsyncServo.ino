// HC_SR04 - https://github.com/bjoernboeckle/HC_SR04
// Copyright © 2022, Björn Böckle
// MIT License

// MRW 2/8/2023 using Arduino Nano BLE Sense pins A7, A6
// MRW 2/8/2023 added 2 servos to rotate and tilt the sensor

#include <HC_SR04.h>
#include <PCA9685.h>  // servo driver

// Map trigger and echo to BLE Sense pins
HC_SR04_BASE *Slaves[] = { new HC_SR04<A3>() };
HC_SR04<A6> sonicMaster(A7, Slaves, 1);
//HC_SR04<A6> sensor(A7);   // sensor with echo A6 and trigger A7 pins
int distanceR;
int distanceL;

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

void rgbLed(bool r,bool g,bool b){
  digitalWrite(LEDR,!r);
  digitalWrite(LEDG,!g);
  digitalWrite(LEDB,!b);
}
void ledOn()  {digitalWrite(LED_BUILTIN, HIGH);}
void ledOff() {digitalWrite(LED_BUILTIN, LOW);}
void initLed(){pinMode(LED_BUILTIN, OUTPUT);}


void setup() {
  Serial.begin(9600);

  sonicMaster.beginAsync();
  sonicMaster.startAsync(0);
//  sensor.beginAsync();  
//  sensor.startAsync(0);        // start first measurement
 
  initPca9685();
  // init sensors to point straight and level
  pca9685.setChannelServoPulseDuration(8, 1500); // Right
  pca9685.setChannelServoPulseDuration(12, 1500);// Left
//  while(1); // hold zero positions to mount sensor arms to motors
}

int servoZeroUs = 1500;
float usPerDeg = 10.0;

int servoDeg2Us(int degree)
{
  int usec;
  usec = servoZeroUs + (usPerDeg * degree);
  return usec;
}


int horDegMin = -75;
int horDegMax =  75;
int horDegInc = 1;
int horDeg = 0;
/*
int verDegMin = -25;
int verDegMax =  0;
int verDegInc = 10;
int verDeg = 0;
*/
// ruturns true when direction changes
bool adjHorDeg()
{
  bool changeDir = false;
  horDeg+=horDegInc;
  
  if(horDeg>horDegMax) 
  {
    horDeg = horDegMax;
    horDegInc = -horDegInc;
    changeDir = true;
  }
  if(horDeg<horDegMin) 
  {
    horDeg = horDegMin;
    horDegInc = -horDegInc;
    changeDir = true;
  }

  pca9685.setChannelServoPulseDuration(8, servoDeg2Us(horDeg)); // Right
  pca9685.setChannelServoPulseDuration(12, servoDeg2Us(-horDeg)); // Left
  delay(1);
  return(changeDir);
}
/*
// ruturns true when direction changes
bool adjVerDeg()
{
  bool changeDir = false;
  verDeg+=verDegInc;
  if(verDeg>verDegMax) 
  {
    verDeg = verDegMax;
    verDegInc = -verDegInc;
    changeDir = true;
  }
  if(verDeg<verDegMin) 
  {
    verDeg = verDegMin;
    verDegInc = -verDegInc;
    changeDir = true;
  }

  pca9685.setChannelServoPulseDuration(12, servoDeg2Us(-verDeg)); // vertical
  delay(1);
  return(changeDir);
}
*/

void loop() {
  adjHorDeg();
//  if(adjHorDeg()) adjVerDeg(); // adjust vertical when horizontal direction changes

  delay(50); // 

  if (sonicMaster.isFinished()) {
//  if (sensor.isFinished()) {
    distanceR = sonicMaster.getDist_cm(0);
    distanceL = sonicMaster.getDist_cm(1);
    sonicMaster.startAsync(0);
//    distance = sensor.getDist_cm();
//    sensor.startAsync(0);
 
         if(distanceR <  30) {rgbLed(1,1,1); delay(1000);} // white
    else if(distanceR < 150) {rgbLed(0,1,0); delay(100);}// green  
    else if(distanceR < 300) {rgbLed(0,0,1); delay(10);}// blue  
    else rgbLed(1,0,0); // red

//    Serial.print(verDeg); Serial.print(", ");
    Serial.print(horDeg); Serial.print(", ");
    Serial.print(distanceR); Serial.print(", ");
    Serial.print(distanceL); Serial.print(", ");
    Serial.print(-50); Serial.print(", ");
    Serial.print(350); Serial.print("\n");
    delay(10);
  }
  
}
