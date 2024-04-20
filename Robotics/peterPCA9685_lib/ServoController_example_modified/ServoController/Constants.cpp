// ----------------------------------------------------------------------------
// Constants.cpp
//
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Constants.h"


namespace constants
{
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ
const int servoWriteDelay = 1;

const servo_struct servo[16] = {
  /* 0*/ 500, 2300, -1, -180, 110, 0, /* right rear body*/
  /* 1*/ 500, 2300,  1, -120, 180, 0, /* right rear mid*/
  /* 2*/ 500, 2300,  1, -180,  90, 0, /* right rear tip*/
  /* 3*/ 0, 0, 0, 0, 0, 0,
  /* 4*/ 500, 2300,  1, -180, 110, 0, /* left rear joint 1*/
  /* 5*/ 500, 2300, -1, -120, 180, 0,
  /* 6*/ 500, 2300, -1, -180,  90, 0,
  /* 7*/ 0, 0, 0, 0, 0, 0,
  /* 8*/ 500, 2300,  1, -110, 180, 0, /* left front joint 1*/
  /* 9*/ 500, 2300, -1, -120, 180, 0,
  /*10*/ 500, 2300, -1, -180,  90, 0,
  /*11*/ 0, 0, 0, 0, 0, 0,
  /*12*/ 500, 2300, -1, -110, 180, 0, /* right front joint 1*/
  /*13*/ 500, 2300,  1, -120, 180, 0,
  /*14*/ 500, 2300,  1, -180,  90, 0,
  /*15*/ 0, 0, 0, 0, 0, 0
  };


}
