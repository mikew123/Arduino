#include <Arduino.h>
#include <PCA9685.h>

PCA9685 pca9685;

const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ


void setup()
{
  Serial.begin(9600);

  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);

 }

void setWheelMotorUsec(unsigned int uSec)
{
  Serial.print("uSec = "); Serial.println(uSec);
  pca9685.setChannelServoPulseDuration(0, uSec);
  pca9685.setChannelServoPulseDuration(1, uSec);
  pca9685.setChannelServoPulseDuration(4, uSec);
  pca9685.setChannelServoPulseDuration(5, uSec);

}

void loop()
{

  Serial.println("All 4 wheels at 'stop- un-trimmed ");
  setWheelMotorUsec(1500);
  delay(20000);
  
  Serial.println("All 4 wheels inc +- 5 uSec around 1500 ");
  for(int us=0; us <= 50; us += 5)
  {
    setWheelMotorUsec(1500 +us);
    delay(10000);
    setWheelMotorUsec(1500 -us);
    delay(10000);
  }

  Serial.println("END");
}
