#include <Arduino.h>
#include <PCA9685.h>  // servo driver
#include <Wire.h>

PCA9685 pca9685;
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ

TwoWire *wire_ptr = &Wire;

void initPca9685(){
//  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupSingleDevice(*wire_ptr,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);
}
bool serialOK = 0; // assume serial port is OK until time out waiting
void initSerial()
{
  Serial.begin(9600);
  int i;
  for(i=0; !Serial; i++) {if (i >= 5000) break; delay(1);}
  if (i < 5000) serialOK=1;  
  if(serialOK) Serial.println("initSerial OK");   
}

void wheelDrive(int FR=5000, int FL=0, int RR=0, int RL=0)
{
  int frus, flus, rrus, rlus;
  if(serialOK){
    Serial.print("wheelDrive ");
    Serial.print(FR); Serial.print(" ");
    Serial.print(FL); Serial.print(" ");
    Serial.print(RR); Serial.print(" ");
    Serial.print(RL); Serial.print("\n");
  }
  if(FR==5000) {frus = 0; flus=0; rrus=0; rlus=0;}
  else{
    frus = 1520 + FR;
    flus = 1520 + FL;
    rrus = 1520 + RR;
    rlus = 1520 + RL;
  }

  pca9685.setChannelServoPulseDuration(5,frus);
  pca9685.setChannelServoPulseDuration(1,flus);
  pca9685.setChannelServoPulseDuration(4,rrus);
  pca9685.setChannelServoPulseDuration(0,rlus);
}

void setup() {
  // put your setup code here, to run once:
  initPca9685();  
  initSerial();  
  wheelDrive();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  wheelDrive(50, -50, 50, -50);
  delay(5000);
  wheelDrive(-50, 50, -50, 50);
  delay(5000);
  wheelDrive();
}
