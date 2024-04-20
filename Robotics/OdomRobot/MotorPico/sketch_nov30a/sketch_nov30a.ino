
#define _PWM_LOGLEVEL_        0
#include "RP2040_PWM.h"
#include "pio_encoder.h"

float frequency = 20000;
float dutyCycle = 0;

// Wheel motor driver pins
#define PIN_ML_IN1      14
#define PIN_ML_IN2      15

// Wheel motor odometry pins
#define PIN_MOTOR_ODOM_1 2
#define PIN_MOTOR_ODOM_2 3

// Odometry Pod odometry pins
#define PIN_POD_ODOM_1 0
#define PIN_POD_ODOM_2 1

//creates pwm instance
RP2040_PWM* PWM_ML_IN1;
RP2040_PWM* PWM_ML_IN2;

// encoder is connected to IN1 and IN2
//PioEncoder encoder(PIN_POD_ODOM_1); 
PioEncoder encoder(PIN_MOTOR_ODOM_1); 

int lastEncoderCount = 0;
int currEncoderCount = 0;
int diffEncoderCount = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(15200);
  Serial.println("Setup");

  PWM_ML_IN1 = new RP2040_PWM(PIN_ML_IN1, frequency, 100);
  PWM_ML_IN2 = new RP2040_PWM(PIN_ML_IN2, frequency, 100);

  encoder.begin();

}

void loop() {
  dutyCycle = 10;
  PWM_ML_IN2->setPWM(PIN_ML_IN2, frequency, 100);
  PWM_ML_IN1->setPWM(PIN_ML_IN1, frequency, dutyCycle);
  delay(5000);
  currEncoderCount = encoder.getCount();
  diffEncoderCount = currEncoderCount - lastEncoderCount;
  lastEncoderCount = currEncoderCount;
  Serial.println(diffEncoderCount);

  dutyCycle = 90;
  PWM_ML_IN1->setPWM(PIN_ML_IN1, frequency, dutyCycle);
  delay(5000);
  currEncoderCount = encoder.getCount();
  diffEncoderCount = currEncoderCount - lastEncoderCount;
  lastEncoderCount = currEncoderCount;
  Serial.println(diffEncoderCount);

  dutyCycle = 10;
  PWM_ML_IN1->setPWM(PIN_ML_IN1, frequency, 100);
  PWM_ML_IN2->setPWM(PIN_ML_IN2, frequency, dutyCycle);
  delay(5000);
  currEncoderCount = encoder.getCount();
  diffEncoderCount = currEncoderCount - lastEncoderCount;
  lastEncoderCount = currEncoderCount;
  Serial.println(diffEncoderCount);

  dutyCycle = 90;
  PWM_ML_IN2->setPWM(PIN_ML_IN2, frequency, dutyCycle);
  delay(5000);
  currEncoderCount = encoder.getCount();
  diffEncoderCount = currEncoderCount - lastEncoderCount;
  lastEncoderCount = currEncoderCount;
  Serial.println(diffEncoderCount);

  PWM_ML_IN1->setPWM(PIN_ML_IN1, frequency, 100);
  PWM_ML_IN2->setPWM(PIN_ML_IN2, frequency, 100);
  encoder.reset();
  lastEncoderCount = 0;
  delay(5000);
  currEncoderCount = encoder.getCount();
  diffEncoderCount = currEncoderCount - lastEncoderCount;
  lastEncoderCount = currEncoderCount;
  Serial.println(diffEncoderCount);

}
