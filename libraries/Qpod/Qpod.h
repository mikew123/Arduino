// ----------------------------------------------------------------------------
// Qpod.h
//
// This library abstracts the servo width functions and uses angle in degrees
// It also has range data for each servo to calibrate the servo motion
// Since the servo data struct could not be instanced in the Qpod class
// the data structure is in global memmory in the QpodGlob namespace
//
// The external servo module is managed my the PCA9685 library
// https://github.com/janelia-arduino/PCA9685
// 
// The external servo controller is the Zio 16 Servo Controller using the PCA9685
// Other modules using PCA9685 could be used instead
// 
// Author: Mike Williamson
// License: public domain
// ----------------------------------------------------------------------------
#ifndef QPOD_H
#define QPOD_H

#include <Arduino.h>
#include <PCA9685.h>

namespace QpodGlob
{
// Created array of servo structures in global
// For some reason i could not access it when created in the Class
struct servo_struct
{
  PCA9685::DurationMicroseconds servo_pulse_duration_min; // at -90
  PCA9685::DurationMicroseconds servo_pulse_duration_max; // at +90
  int servo_polarity; // +-1 used to reverse the servo angle for example left vs right legs, 0=unused
  int servo_degrees_min; // restrict movement between 90 to -90
  int servo_degrees_max; // restrict movement between 90 to -90
  int servo_degrees_nom; // angle after reset
};
// set nominal to standing position
static constexpr servo_struct servo[16] = {
/*servo min us max pol  min deg max  nom
/* 0*/ {500,  2300, -1, -90,     45, -45}, /* left rear body*/
/* 1*/ {500,  2300,  1, -60,     90, -45}, /* left rear mid*/
/* 2*/ {500,  2300,  1, -90,     45, -45}, /* left rear tip*/
/* 3*/ {0, 0, 0, 0, 0, 0},
/* 4*/ {500,  2300,  1, -90,     45, -45}, /* right rear joint 1*/
/* 5*/ {500,  2300, -1, -60,     90, -45},
/* 6*/ {500,  2300, -1, -90,     45, -45},
/* 7*/ {0, 0, 0, 0, 0, 0},
/* 8*/ {500,  2300,  1, -45,     90,  45}, /* right front joint 1*/
/* 9*/ {500,  2300,  1, -60,     90, -45},
/*10*/ {500,  2300,  1, -90,     45, -45},
/*11*/ {0, 0, 0, 0, 0, 0},
/*12*/ {500,  2300, -1, -45,     90,  45}, /* left front joint 1*/
/*13*/ {500,  2300, -1, -60,     90, -45},
/*14*/ {500,  2300, -1, -90,     45, -45},
/*15*/ {0, 0, 0, 0, 0, 0}
};

}

// Qaudrapod servo control
// Uses the library https://github.com/janelia-arduino/PCA9685 
class Qpod : private PCA9685 {

private:
  static constexpr PCA9685::DeviceAddress device_address = 0x40;
  static constexpr PCA9685::Pin output_enable_pin = 2;
  static constexpr PCA9685::Frequency frequency = 50; // servo pulse rate
//  static constexpr int servoWriteDelay = 2; //msec delay this amount after servo write
  int servoWriteDelay = 2; //msec delay this amount after servo write

public:  
  // iniitalize PCA9685
  void init();
  
  // set servo angle in degrees
  int setServoDegrees (int servo_num, int angle_degrees);
  
  // Control walking speed by changing delay after each servo write
  // returns -1 when delay time is limited
  int incWalkSpeed();
  int decWalkSpeed();

// Disable servos 
void disableServos();

};

#endif
