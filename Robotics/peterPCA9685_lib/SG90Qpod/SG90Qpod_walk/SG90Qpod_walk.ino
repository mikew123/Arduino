// ----------------------------------------------------------------------------
// SG90Qpod_walk sketch
//
// have the Quadrapod walk simple dragging tips a bit
//
// Author: Mike Williamson
// License: public domain
// ----------------------------------------------------------------------------

#include <Arduino.h>
#include <Qpod.h>

Qpod qpod;


int servo_angle_degrees[16];
int servo_angle_degrees_increment[16];
bool led = HIGH;

int walk_sequence_cnt = 14;
int walk_sequence[14][16] = 
{
//  left rear         right rear        right front       left front      
  {  0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // standing all legs front tips on ground
  {-45, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // sweep legs to back to walk forward
  // move each leg forward 1 at a time
  // right front servos 8,9,10
  {-45, -55, -45, 0, -45, -55, -45, 0,   0, -55, -40, 0,   0, -55, -45, 0}, // lift
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -40, 0,   0, -55, -45, 0}, // forward
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // lower
  // left front servos 12,13,14
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,   0, -55, -40, 0}, // lift
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,  45, -55, -40, 0}, // forward
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lower
  // right rear servos 4,5,6
  {-45, -55, -45, 0, -45, -55, -40, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lift
  {-45, -55, -45, 0,   0, -55, -40, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // forward
  {-45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lower
  // left rear leg servos 0,1,2 (leg 0)
  {-45, -55, -40, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lift
  {  0, -55, -40, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // forward
  {  0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}  // lower
};

enum leg_sel {all=-1, lr=0, rr=4, rf=8, lf=12};
enum leg_mode {fw, bk, up, dn};
enum leg_updn {u=0,d=1};

// servo positions for moving tip of leg up and down
// mid out servos   up{md   ot} dn{md   ot}
int leg_updn[2][2] = {{-35,-45},  {-55,-45}};

// servo positions for moving leg forward and backward
// inner servo      ft{fw   bk} rr{fw   bk}   
int leg_fwbk[2][2] = {{45, -25},  {25, -45}};

void move_leg(leg_sel sel, int servos[3])
{
  switch(sel)
  {
    case all:

      break;
  }
}

void move_legs(leg_sel sel, leg_mode mode)
{
  int servos[3] = {0,0,0}; 
  switch(mode)
  {
    case fw: // forward motion

//      move_leg(sel, leg_splay, leg_updn[dn]);
      break;
    
  }
}

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
  for (int seq=0; seq<walk_sequence_cnt; seq++)
  {
    int servos_active = 16;
    while(servos_active != 0)
    {
      servos_active = 16;
      for (int servo=0;  servo<16; servo++)
      {
        if(servo_angle_degrees[servo] == walk_sequence[seq][servo])
        {
          servos_active--; // sequence stops when all servos have moved to their seq position
        }
        else
        {
          if(servo_angle_degrees[servo] < walk_sequence[seq][servo])
          {
            servo_angle_degrees[servo]++;
          }
          else if(servo_angle_degrees[servo] > walk_sequence[seq][servo])
          {
            servo_angle_degrees[servo]--;
          }
          qpod.setServoDegrees(servo, servo_angle_degrees[servo]); 
        }
      }
    }
    // toggle LED every sequence
    led = !led;
    digitalWrite(LED_BUILTIN, led);
    delay(0); // sfter each sequence
  }

  delay(0); // after all sequences
};
