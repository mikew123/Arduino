#include <Arduino.h>
#include <PCA9685.h>

#include "Constants.h"


PCA9685 pca9685;


int servo_angle_degrees[16];
int servo_angle_degrees_increment[16];
int loop_delay = 10;
bool led = HIGH;

int setServoDegrees (int servo_num, int angle_degrees); 

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);


  pca9685.setupSingleDevice(Wire,constants::device_address);

  pca9685.setupOutputEnablePin(constants::output_enable_pin);
  pca9685.enableOutputs(constants::output_enable_pin);

//  pca9685.setToServoFrequency();
  pca9685.setToFrequency(constants::frequency);

  for (int i=0; i<16; i++)
  {
    servo_angle_degrees_increment[i] = 1;
    servo_angle_degrees[i] = constants::servo[i].servo_degrees_nom;
    setServoDegrees(i, servo_angle_degrees[i]); 
    delay(constants::servoWriteDelay);   
  }

  digitalWrite(LED_BUILTIN, led);

}

void loop()
{
if(1) 
{  
  for (int i=0;  i<16; i=i+1)
  {

    servo_angle_degrees[i] += servo_angle_degrees_increment[i];
    if (setServoDegrees(i, servo_angle_degrees[i]) != 0) 
    {
      servo_angle_degrees_increment[i] = -servo_angle_degrees_increment[i];
      // toggle LED when servo 0 hits either limit
      if (i==0)
      {
        led = !led;
        digitalWrite(LED_BUILTIN, led);
      }
    }
  }
}

  delay(loop_delay);
}

/* 
  setServoDegrees()
  servo_num - 0 to 15
  angle_degrees - limited to the  min and max of the servo
  if the configured polarity of servo == 0 then servo is not controlled
  return - 0=normal, -1=min limited, +1=max limited
*/
int setServoDegrees (int servo_num, int angle_degrees) 
{
  PCA9685::DurationMicroseconds width_min = constants::servo[servo_num].servo_pulse_duration_min;
  PCA9685::DurationMicroseconds width_max = constants::servo[servo_num].servo_pulse_duration_max;
  PCA9685::DurationMicroseconds width_range = width_max - width_min;
  PCA9685::DurationMicroseconds width_mid = width_min + width_range/2;
  PCA9685::DurationMicroseconds width = (width_max-width_min)/2;
  int servo_polarity = constants::servo[servo_num].servo_polarity;
  int return_code = 0;
  int deg = angle_degrees;

  // Limit min max phase degrees
  if (deg < constants::servo[servo_num].servo_degrees_min)
  {
    deg = constants::servo[servo_num].servo_degrees_min;
    return_code = -1;
  } 
  else if (deg > constants::servo[servo_num].servo_degrees_max)
  {
    deg = constants::servo[servo_num].servo_degrees_max;
    return_code = 1;
  }

  // Convert phase degrees to pulse width microseconds
  // assumes +-180 max-min servo range
  // can reverse angle as needed
  if (servo_polarity == 1)
  {
    width = width_mid + ((width_range/2)*(deg/180.0));
    pca9685.setChannelServoPulseDuration(servo_num, width);
    delay(constants::servoWriteDelay);
  } 
  else if (servo_polarity == -1)
  {
    width = width_mid - ((width_range/2)*(deg/180.0));
    pca9685.setChannelServoPulseDuration(servo_num, width);
    delay(constants::servoWriteDelay);
 }
  // else server not enabled

  return return_code;
}
