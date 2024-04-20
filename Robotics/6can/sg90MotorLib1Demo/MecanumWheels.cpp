// MecanumWheels.cpp

// Library to control Mecanum wheels on a robot
// 65mm dia Mecanum wheel is 20.4cm circumference
// SG90 at 120rpm can drive 24.5M/min = 0.41M/sec?
// Mike Williamson 2/9/2023

#include "MecanumWheels.h"



void MecanumWheels::initWheels(PCA9685 *pca9685_ptr)
{
  pca9685_p = pca9685_ptr;  
  // Movement
  // Motor 1 front left
  WheelConfig[0].chan = 1; // servo channel
  WheelConfig[0].pol   = +1; // for +val fwd motion
  WheelConfig[0].stop_trim = +15; // trim stop
  WheelConfig[0].min   = 40;
   // Motor 2 front right
  WheelConfig[1].chan = 5;
  WheelConfig[1].pol   = -1; // for +val fwd motion
  WheelConfig[1].stop_trim = +25; // trim stop
  WheelConfig[1].min   = 40;
  // Motor 3 rear left
  WheelConfig[2].chan = 0;
  WheelConfig[2].pol   = +1; // for +val fwd motion
  WheelConfig[2].stop_trim = +20; // trim stop
  WheelConfig[2].min   = 40;
  // Motor 4 rear right
  WheelConfig[3].chan = 4;
  WheelConfig[3].pol       = -1; // for +val fwd motion
  WheelConfig[3].stop_trim = +15; // trim stop
  WheelConfig[3].min   = 40;

  // motions
  //0
  WheelConfig[0].motion[stop] = 0;
  WheelConfig[1].motion[stop] = 0;
  WheelConfig[2].motion[stop] = 0;
  WheelConfig[3].motion[stop] = 0;
  //1
  WheelConfig[0].motion[fwd] = +1;
  WheelConfig[1].motion[fwd] = +1;
  WheelConfig[2].motion[fwd] = +1;
  WheelConfig[3].motion[fwd] = +1;
  //2
  WheelConfig[0].motion[rev] = -1;
  WheelConfig[1].motion[rev] = -1;
  WheelConfig[2].motion[rev] = -1;
  WheelConfig[3].motion[rev] = -1;
  //3
  WheelConfig[0].motion[right] = +1;
  WheelConfig[1].motion[right] = -1;
  WheelConfig[2].motion[right] = -1;
  WheelConfig[3].motion[right] = +1;
  //4
  WheelConfig[0].motion[left] = -1;
  WheelConfig[1].motion[left] = +1;
  WheelConfig[2].motion[left] = +1;
  WheelConfig[3].motion[left] = -1;
  //5  2Xspeed since only 2 wheels used
  WheelConfig[0].motion[fwd_right] = +2;
  WheelConfig[1].motion[fwd_right] =  0;
  WheelConfig[2].motion[fwd_right] =  0;
  WheelConfig[3].motion[fwd_right] = +2;
  //6
  WheelConfig[0].motion[fwd_left] =  0;
  WheelConfig[1].motion[fwd_left] = +2;
  WheelConfig[2].motion[fwd_left] = +2;
  WheelConfig[3].motion[fwd_left] =  0;
  //7
  WheelConfig[0].motion[rev_right] =  0;
  WheelConfig[1].motion[rev_right] = -2;
  WheelConfig[2].motion[rev_right] = -2;
  WheelConfig[3].motion[rev_right] =  0;
  //8
  WheelConfig[0].motion[rev_left] = -2;
  WheelConfig[1].motion[rev_left] =  0;
  WheelConfig[2].motion[rev_left] =  0;
  WheelConfig[3].motion[rev_left] = -2;
  //9
  WheelConfig[0].motion[cwise] = +1;
  WheelConfig[1].motion[cwise] = -1;
  WheelConfig[2].motion[cwise] = +1;
  WheelConfig[3].motion[cwise] = -1;
  //10
  WheelConfig[0].motion[ccwise] = -1;
  WheelConfig[1].motion[ccwise] = +1;
  WheelConfig[2].motion[ccwise] = -1;
  WheelConfig[3].motion[ccwise] = +1;

};

// the concept is to keep the robot heading in same direction as at init
// using an IMU to correct the heading as it drifts from init
void MecanumWheels::WheelDrive(int wheel, int motion, float speed, float heading)
{    
  int dir  = WheelConfig[wheel].motion[motion];
  int chan = WheelConfig[wheel].chan;
  int pol  = WheelConfig[wheel].pol;
  int drvMin  = WheelConfig[wheel].min;
  int stopVal = stop_nom + WheelConfig[wheel].stop_trim;
  int drv  = stopVal + speed*pol*dir*drvMin;

  // NOTE: does each motion need its own PI loop filter?
  // To correct for heading drift (should be zero cal at init)
  // add heading drift offset to all wheels to cause rotation
  // limit speed correction and integral
  // uses a Proportional Integral loop filter
  float integral = motionPiIntegral[motion];
  float proportional;
  float headingCorrection;
  switch(motion){
    case fwd:
    case rev:
    case left:
    case right:
    case fwd_left:
    case rev_right:
    case rev_left:
    case fwd_right:
      // loop filter
      integral += piIntCoeff*heading;        
      if(integral>+integralLimit) integral=+integralLimit;
      if(integral<-integralLimit) integral=-integralLimit;
      proportional = piPropCoeff*heading;        
      headingCorrection = proportional + integral;
      if(headingCorrection>+headingCorrectionLimit) headingCorrection=+headingCorrectionLimit;
      if(headingCorrection<-headingCorrectionLimit) headingCorrection=-headingCorrectionLimit;
      
      switch(wheel){
        case 0: // wheel 1 front left
          drv += headingCorrection*pol*speed; 
          break;
        case 1: // wheel 2 front right 
          drv -= headingCorrection*pol*speed; 
          break;
        case 2: // wheel 3 rear left
          drv += headingCorrection*pol*speed; 
          break;
        case 3: // wheel 4 rear right 
          drv -= headingCorrection*pol*speed; 
          break;
      }
      break;

    // no drive loop filter on other motions
    default: 
      break;                
  }

  ((PCA9685)(*pca9685_p)).setChannelServoPulseDuration(chan, drv);
  // save new integral value of loop filter
  motionPiIntegral[motion] = integral;

  delay(Sdelay);

}
