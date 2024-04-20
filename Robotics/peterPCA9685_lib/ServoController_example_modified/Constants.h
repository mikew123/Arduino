// ----------------------------------------------------------------------------
// Constants.h
//
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Arduino.h>
#include <PCA9685.h>


namespace constants
{

extern const PCA9685::DeviceAddress device_address;
extern const PCA9685::Pin output_enable_pin;
extern const PCA9685::Frequency frequency; // servo pulse rate
extern const int servoWriteDelay; //delay this amount after servo write

typedef struct
{
  PCA9685::DurationMicroseconds servo_pulse_duration_min; // at -180
  PCA9685::DurationMicroseconds servo_pulse_duration_max; // at +180
  int servo_polarity; // +-1 used to reverse the servo angle for example left vs right legs, 0=unused
  int servo_degrees_min; // restrict movement between 180 to -180
  int servo_degrees_max; // restrict movement between 180 to -180
  int servo_degrees_nom; // angle after reset
} servo_struct;
extern const servo_struct servo[16];

}
#endif
