#include <Arduino.h>
#include <PCA9685.h>

PCA9685 pca9685;

const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ


void setup()
{
 
  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);

  // all 4 wheels at "stop" un-trimmed
  pca9685.setChannelServoPulseDuration(0, 1500);
  pca9685.setChannelServoPulseDuration(1, 1500);
  pca9685.setChannelServoPulseDuration(4, 1500);
  pca9685.setChannelServoPulseDuration(5, 1500);

  // set sensor servos to mid point zero position
  pca9685.setChannelServoPulseDuration(8, 1500);
  pca9685.setChannelServoPulseDuration(12,1500);

  // set claw servos to mid point zero position
  pca9685.setChannelServoPulseDuration(9, 1500);
  pca9685.setChannelServoPulseDuration(13,1500);

 }


void loop()
{
}
