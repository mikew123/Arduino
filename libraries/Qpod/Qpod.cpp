// ----------------------------------------------------------------------------
// Qpod.cpp
//
//
// Author: Mike Williamson
// License: public domain
// ----------------------------------------------------------------------------
#include "Qpod.h"

/*
  iniitalize servo driver etc
*/
void Qpod::init() {
  setupSingleDevice(Wire,device_address);
  setupOutputEnablePin(output_enable_pin);
  enableOutputs(output_enable_pin);
  setToFrequency(frequency);
}

/* 
  setServoDegrees()
  servo_num - 0 to 15
  angle_degrees - limited to the  min and max of the servo
  if the configured polarity of servo == 0 then servo is not controlled
  return - 0=normal, -1=min limited, +1=max limited
*/
int Qpod::setServoDegrees (int servo_num, int angle_degrees) 
{

  PCA9685::DurationMicroseconds width_min = QpodGlob::servo[servo_num].servo_pulse_duration_min;
  PCA9685::DurationMicroseconds width_max = QpodGlob::servo[servo_num].servo_pulse_duration_max;
  PCA9685::DurationMicroseconds width_range = width_max - width_min;
  PCA9685::DurationMicroseconds width_mid = width_min + width_range/2;
  PCA9685::DurationMicroseconds width = (width_max-width_min)/2;
  int servo_polarity = QpodGlob::servo[servo_num].servo_polarity;
  int return_code = 0;
  int deg = angle_degrees;

  // Limit min max phase degrees
  if (deg < QpodGlob::servo[servo_num].servo_degrees_min)
  {
    deg = QpodGlob::servo[servo_num].servo_degrees_min;
    return_code = -1;
  } 
  else if (deg > QpodGlob::servo[servo_num].servo_degrees_max)
  {
    deg = QpodGlob::servo[servo_num].servo_degrees_max;
    return_code = 1;
  }

  // Convert phase degrees to pulse width microseconds
  // assumes +-90 max-min servo range
  // can reverse angle as needed
  if (servo_polarity == 1)
  {
    width = width_mid + ((width_range/2)*(deg/90.0));
    setChannelServoPulseDuration(servo_num, width);
    delay(servoWriteDelay);
  } 
  else if (servo_polarity == -1)
  {
    width = width_mid - ((width_range/2)*(deg/90.0));
    setChannelServoPulseDuration(servo_num, width);
    delay(servoWriteDelay);
  }
  // else server not enabled - not implimented

  return return_code;
}

// Control walking speed by changing delay after each servo write
// returns -1 when delay time is limited
int Qpod::incWalkSpeed()
{
	if(servoWriteDelay>0) {servoWriteDelay--; return 0;}
	else return -1;	
}
int Qpod::decWalkSpeed()
{
	if(servoWriteDelay<20) {servoWriteDelay++; return 0;}
	else return -1;
}

void Qpod::disableServos()
{
	for (int i=0; i<16; i++)
	{
		setChannelServoPulseDuration(i, 0);
	}
}
