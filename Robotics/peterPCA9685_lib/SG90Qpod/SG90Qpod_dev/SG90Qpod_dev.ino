// ----------------------------------------------------------------------------
// SG90Qpod_dev sketch
//
// cycles the joints through range of motion to adjust range parameters
//
// Author: Mike Williamson
// License: public domain
// ----------------------------------------------------------------------------

#include <Arduino.h>
#include <Qpod.h>

Qpod qpod;


int servo_angle_degrees[16];
int servo_angle_degrees_increment[16];
int loop_delay = 10;
bool led = HIGH;

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  //  initialize servo driver etc
  qpod.init();

  // initialize each servo joint angle to its nominal 
  for (int i=0; i<16; i++)
  {
    servo_angle_degrees_increment[i] = 1; // start pos servo sweep
    servo_angle_degrees[i] = QpodGlob::servo[i].servo_degrees_nom;
    qpod.setServoDegrees(i, servo_angle_degrees[i]); 
  }

  digitalWrite(LED_BUILTIN, led);
  delay(10000); // wait 10 seconds
}

void loop()
{
  for (int i=0;  i<16; i=i+1)
  {

    servo_angle_degrees[i] += servo_angle_degrees_increment[i];
    if (qpod.setServoDegrees(i, servo_angle_degrees[i]) != 0) 
    { // servo angle exceeded the min or max range limits
      servo_angle_degrees_increment[i] = -servo_angle_degrees_increment[i];
      // toggle LED when servo 0 hits either limit
      if (i==0)
      {
        led = !led;
        digitalWrite(LED_BUILTIN, led);
      }
    }
  }

  delay(loop_delay);
}
