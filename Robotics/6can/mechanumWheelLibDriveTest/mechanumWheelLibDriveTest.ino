#include <Arduino.h>
#include <PCA9685.h>  // servo driver
#include <Wire.h>
#include <MecanumWheels.h>

TwoWire *wire_ptr = &Wire;
MecanumWheels wheels;

PCA9685 pca9685;
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ

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

void setup() {
  // put your setup code here, to run once:
  initSerial();  
  initPca9685();  
  wheels.initWheels(&pca9685);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, stop, 0, 0);
  delay(5000);
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, fwd, 1, 0);
  delay(5000);
  for (int wheel=0;wheel<4;wheel++)
    wheels.WheelDrive(wheel, rev, 1, 0);
  delay(5000);
}
