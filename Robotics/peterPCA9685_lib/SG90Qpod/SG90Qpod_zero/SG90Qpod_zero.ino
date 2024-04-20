// ----------------------------------------------------------------------------
// SG90Qpod_zero sketch
//
// sets the joints to 0 degrees to adjust joint attachment
// 
// Author: Mike Williamson
// License: public domain
// ----------------------------------------------------------------------------

#include <Arduino.h>
#include <Qpod.h>

Qpod qpod;

int servo_angle_degrees[16];
int servo_angle_degrees_increment[16];
int loop_delay = 500;
bool led = HIGH;

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  //  initialize servo driver etc
  qpod.init();

  // initialize joint servos to 0 degree angle
  for (int i=0; i<16; i++)
  {
    servo_angle_degrees_increment[i] = 1;
    servo_angle_degrees[i] = 0;
    qpod.setServoDegrees(i, servo_angle_degrees[i]); 
  }

  digitalWrite(LED_BUILTIN, led);

}

void loop()
{
  led = !led;
  digitalWrite(LED_BUILTIN, led);
  delay(loop_delay);
}

