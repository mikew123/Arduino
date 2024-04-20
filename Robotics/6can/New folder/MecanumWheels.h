// MecanumWheels.h

// Library to control Mecanum wheels on a robot
// 65mm dia Mecanum wheel is 20.4cm circumference
// SG90 at 120rpm can drive 24.5M/min = 0.41M/sec?
// Mike Williamson 2/9/2023

#ifndef _MecanumWheels_h
#define _MecanumWheels_h

#include <Arduino.h>
#include <PCA9685.h>  // servo driver

// TODO: can enum be put in the class?
// otherwise append wheels_ to each motion enum
enum Wheelmotions_e {
stop=0,
fwd=1,
rev=2,
right=3,
left=4,
fwd_right=5,
fwd_left=6,
rev_right=7,
rev_left=8,
cwise=9,
ccwise=10,
numMotions=11
};

class MecanumWheels
{

public:
  // TODO: pass servo channels at init
  void initWheels(PCA9685 *pca9685_ptr);
  void WheelDrive(int wheel, int motion, float speed, float heading);

private:

  typedef struct WheelConfigStruct{
    int chan; // PCA9685 servo driver channel
    int pol;  // polarity of servo control for fwd motion
    int stop_trim; // trim stop offset from stop_nom
    int min; // min offset from stop_trim to overcome frictions for fwd/rev motion
    int motion[numMotions]; // Wheelmotions_e one for each motion
  };

  WheelConfigStruct WheelConfig[4];

  // PI filter integrators for each motion - Needed for each motion???
  // this offsets friction and motor imbalances which can be different 
  // for each motion
  typedef struct MotionConfigStruct{
    float motionPiIntegral; // wheel PI loop filter intergrals
    float motionCal; // calibrate for 250m/s at speed=1.0
  };

  MotionConfigStruct MotionConfig[numMotions];
  
  int Sdelay = 1; // delay after servo write, is it needed?

  float piPropCoeff = 2; // Proportional coefficient
  float piIntCoeff = 0.1; // Integral coefficient
  int integralLimit=50; 
  int headingCorrectionLimit=50;

  // Servo/wheel/motor
//  int width_max = 2500;
//  int width_min =  500;
  int stop_nom  = 1500;

  PCA9685 *pca9685_p;

};

#endif