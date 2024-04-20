
#define _PWM_LOGLEVEL_        0
#include "RP2040_PWM.h"
#include "pio_encoder.h"

// FreeRTOS and pio_encoder are not compatible 
// code fails to run and usb locked up when encoders are enabled

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

//SemaphoreHandle_t encodersMutex;


//#define PI 3.14159265

// Motor PWM frequency
float frequency = 5000;

// WHEEL 
#define WHEEL_RADUIS_M (0.080/2)
#define WHEEL_CIRCUMFERENCE_M (2*PI*WHEEL_RADUIS_M)
#define WHEEL_ENCODER_COUNT_PER_ROTATION (48*20.4086666)
#define WHEEL_DIST0_RADIUS_M (0.160)

// ODOM POD
#define ODOM_RADUIS_M (0.048/2)
#define ODOM_CIRCUMFERENCE_M (2*PI*ODOM_RADUIS_M)
#define ODOM_ENCODER_COUNT_PER_ROTATION (2000.0)
#define ODOM_DIST0_RADIUS_M (0.120)

//#define PIN0 0
//#define PIN1 1
//#define PIN8 8
//#define PIN8 9
//#define PIN8 26
//#define PIN8 27
//#define PIN8 28
//#define PIN8 29

// Wheel motor odometry pins
#define PIN_MOTOR_ODOM_RA 2
#define PIN_MOTOR_ODOM_RB 3
#define PIN_MOTOR_ODOM_LA 4
#define PIN_MOTOR_ODOM_LB 5

// Wheel motor driver pins
#define PIN_MOTOR_L1      6
#define PIN_MOTOR_L2      7
#define PIN_MOTOR_R1      14
#define PIN_MOTOR_R2      15


// Odometry Pod odometry pins
#define PIN_POD_ODOM_X1 10
#define PIN_POD_ODOM_X2 11
#define PIN_POD_ODOM_Y1 12
#define PIN_POD_ODOM_Y2 13

//creates pwm instance for motor drive
RP2040_PWM* PWM_MOTOR_L1;
RP2040_PWM* PWM_MOTOR_L2;
RP2040_PWM* PWM_MOTOR_R1;
RP2040_PWM* PWM_MOTOR_R2;

// Odometry encoder encoders (Input even pin of encoder pin pairs)
PioEncoder encoder[4] = {PIN_MOTOR_ODOM_RA, PIN_MOTOR_ODOM_LA, PIN_POD_ODOM_X1, PIN_POD_ODOM_Y1};
enum {
  encoderR_e = 0, // right wheel
  encoderL_e = 1, // left wheel
  encoderX_e = 2, // X direction Odom Pod
  encoderY_e = 3  // Y direction Odom Pod (angle)
};

int encoderPolarity[4] = {1,-1,-1,1};
int32_t lastEncoderCount[4] = {0,0,0,0};
int32_t currEncoderCount[4] = {0,0,0,0};
int16_t diffEncoderCount[4] = {0,0,0,0};
double currTravelMeters[4] = {0,0,0,0};
double diffTravelMeters[4] = {0,0,0,0};
double velocityMPS[4] = {0,0,0,0};
uint32_t currTimeUs = 0;
uint32_t lastTimeUs = 0;
uint32_t diffTimeUs = 0;

void ResetEncoderValues() {
  for(int enc=0; enc<4; enc++) {
    currTimeUs = micros();
    lastTimeUs = currTimeUs;
    diffTimeUs = 0;
    lastEncoderCount[enc] = 0;
    currEncoderCount[enc] = 0;
    diffEncoderCount[enc] = 0;
    currTravelMeters[enc] = 0;
    diffTravelMeters[enc] = 0;
    velocityMPS[enc] = 0;
  }
}

// This is executed as a thread loop
// A semiphore protects the time and count values
void updateEncoderCountsTask( void * pvParameters ) {
  // TickType_t xLastWakeTime = xTaskGetTickCount();

  // for( ;; )
  // {
  //   vTaskDelayUntil( &xLastWakeTime, 10 ); // every 10 msec

  //   if( xSemaphoreTake( encodersMutex , ( TickType_t ) 10 ) == pdTRUE ) {
      currTimeUs = micros();
      for(int enc=0; enc<4; enc++) {
//        currEncoderCount[enc] = encoder[enc].getCount() * encoderPolarity[enc];
      }
  //     xSemaphoreGive( encodersMutex );
  //   } else {
  //     //Getting Mutex failed
  //   }
  // }
}


void InitEncoders() {
  for(int enc=0; enc<4; enc++) {
//    encoder[enc].begin();
//    encoder[enc].reset();
  }

  // encodersMutex = xSemaphoreCreateMutex();

  // xTaskCreate(
  //   updateEncoderCountsTask,       /* Function that implements the task. */
  //   "encodersTask",          /* Text name for the task. */
  //   8196,      /* Stack size in words, not bytes. */
  //   NULL, /*parameters*/
  //   1,/* Priority at which the task is created. */
  //   NULL );      /* Used to pass out the created task's handle. */

  ResetEncoderValues();

}

void UpdateEncoders() {

  updateEncoderCountsTask(NULL);

  diffTimeUs = currTimeUs - lastTimeUs;
  lastTimeUs = currTimeUs;

  for(int enc=0; enc<4; enc++) {
    diffEncoderCount[enc] = currEncoderCount[enc] - lastEncoderCount[enc];
    lastEncoderCount[enc] = currEncoderCount[enc];
    if((enc==encoderR_e) || (enc==encoderL_e)) {
      currTravelMeters[enc] = (currEncoderCount[enc]/WHEEL_ENCODER_COUNT_PER_ROTATION)*WHEEL_CIRCUMFERENCE_M;
      diffTravelMeters[enc] = (diffEncoderCount[enc]/WHEEL_ENCODER_COUNT_PER_ROTATION)*WHEEL_CIRCUMFERENCE_M;
      velocityMPS[enc] = diffTravelMeters[enc]/(1e-6*diffTimeUs);
    } else {
      currTravelMeters[enc] = (currEncoderCount[enc]/ODOM_ENCODER_COUNT_PER_ROTATION)*ODOM_CIRCUMFERENCE_M;
      diffTravelMeters[enc] = (diffEncoderCount[enc]/ODOM_ENCODER_COUNT_PER_ROTATION)*ODOM_CIRCUMFERENCE_M;
      velocityMPS[enc] = diffTravelMeters[enc]/(1e-6*diffTimeUs);
    }
  }
}

void InitMotorDrivers(){
  // initialize to stop-braked (100PCT FOR STOP BRAKED)
  PWM_MOTOR_L1 = new RP2040_PWM(PIN_MOTOR_L1, frequency, 100);
  PWM_MOTOR_L2 = new RP2040_PWM(PIN_MOTOR_L2, frequency, 100);
  PWM_MOTOR_R1 = new RP2040_PWM(PIN_MOTOR_R1, frequency, 100);
  PWM_MOTOR_R2 = new RP2040_PWM(PIN_MOTOR_R2, frequency, 100);
}

// +pct = fwd, -pct = rev, 0pct = stop-braked
void MotorDrivePct(int pctL, int pctR) {
  pctL = -pctL; // opposite rotation as Right Wheel
  // Limit to +-100 pct
  if(pctL>100) pctL=100;
  if(pctL<-100) pctL=-100;
  if(pctL>=0) {
    // Motor Left Forward
    PWM_MOTOR_L1->setPWM(PIN_MOTOR_L1, frequency, 100);
    PWM_MOTOR_L2->setPWM(PIN_MOTOR_L2, frequency, 100-pctL);
  } else {
    // Motor Left Reverse
     PWM_MOTOR_L1->setPWM(PIN_MOTOR_L1, frequency, 100+pctL);
     PWM_MOTOR_L2->setPWM(PIN_MOTOR_L2, frequency, 100);
  }

  pctR = +pctR;
  // Limit to +-100 pct
  if(pctR>100) pctR=100;
  if(pctR<-100) pctR=-100;
  if(pctR>=0) {
    // Motor Right Forward
    PWM_MOTOR_R1->setPWM(PIN_MOTOR_R1, frequency, 100);
    PWM_MOTOR_R2->setPWM(PIN_MOTOR_R2, frequency, 100-pctR);
  } else {
    // Motor Right Reverse
    PWM_MOTOR_R1->setPWM(PIN_MOTOR_R1, frequency, 100+pctR);
    PWM_MOTOR_R2->setPWM(PIN_MOTOR_R2, frequency, 100);
  }

}

// Set motor drive in velocity Meters Per Second
// Wheel encoders are used to dynamically control the motor drive percent
// Right wheel in meters per second
// Left wheel in meters per second
void WheelDriveVel(float R_Mps=0, float L_Mps = 0) {

}

// Set Robot drive in velocity X direction and Z rotation
// Convert the direction and rotation velocities into R + L wheel velocities
// This should be compatible with ROS2 Twist/Velcmd values.
// X direction in meters per second
// Z (rotation) in radians per second
void RobotDriveVel(float X_Mps=0, float Z_Rps = 0) {

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(155200);
  Serial.println("Setup Robot 2024 motor/encoder test");

  InitEncoders();

  InitMotorDrivers();
  MotorDrivePct(0, 0);

}


void loop() {

  MotorDrivePct(50, +20); // L=50 R=20 FWD ABOUT 3FT CIRCLE CW

  delay(1000);

// TODO: Redesign using a thread to read encoders.

  UpdateEncoders();

  Serial.print("T:");  Serial.print(1e-6*currTimeUs);
  Serial.print(", DT:");  Serial.print(1e-6*diffTimeUs);
  Serial.print(", R:");  Serial.print(diffEncoderCount[encoderR_e]);
  Serial.print(", L:");Serial.print(diffEncoderCount[encoderL_e]);
  Serial.print(", X:");Serial.print(diffEncoderCount[encoderX_e]);
  Serial.print(", Y:");Serial.print(diffEncoderCount[encoderY_e]);
  Serial.print(", RV:");Serial.print(velocityMPS[encoderR_e]);
  Serial.print(", LV:");Serial.print(velocityMPS[encoderL_e]);
  Serial.print("\r\n");
}
