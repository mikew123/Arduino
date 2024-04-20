/*
 * Ultrasonic Simple
 * Prints the distance read by an ultrasonic sensor in
 * centimeters. They are supported to four pins ultrasound
 * sensors (liek HC-SC04) and three pins (like PING)))
 * and Seeed Studio sensors).
 *
 * The circuit:
 * * Module HR-SC04 (four pins) or PING))) (and other with
 *   three pins), attached to digital pins as follows:
 * ---------------------    --------------------
 * | HC-SC04 | Arduino |    | 3 pins | Arduino |
 * ---------------------    --------------------
 * |   Vcc   |   5V    |    |   Vcc  |   5V    |
 * |   Trig  |   12    | OR |   SIG  |   13    |
 * |   Echo  |   13    |    |   Gnd  |   GND   |
 * |   Gnd   |   GND   |    --------------------
 * ---------------------
 * Note: You do not obligatorily need to use the pins defined above
 * 
 * By default, the distance returned by the read()
 * method is in centimeters. To get the distance in inches,
 * pass INC as a parameter.
 * Example: ultrasonic.read(INC)
 *
 * created 3 Apr 2014
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 23 Jan 2017
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 03 Mar 2017
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 11 Jun 2018
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 *
 * This example code is released into the MIT License.
 */

// MRW 2/6/2023 using Arduino Nano BLE Sense pins A7, A6
// MRW 2/7/2023 added 2 servos to rotate and tilt the sensor

#include <Ultrasonic.h>
#include <PCA9685.h>  // servo driver

// Map trigger and echo to BLE Sense pins
Ultrasonic ultrasonic(A7, A6);
int distance;

PCA9685 pca9685;
const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ
void initPca9685(){
  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);
}

void rgbLed(bool r,bool g,bool b){
  digitalWrite(LEDR,!r);
  digitalWrite(LEDG,!g);
  digitalWrite(LEDB,!b);
}
void ledOn()  {digitalWrite(LED_BUILTIN, HIGH);}
void ledOff() {digitalWrite(LED_BUILTIN, LOW);}
void initLed(){pinMode(LED_BUILTIN, OUTPUT);}


void setup() {
  Serial.begin(9600);
  initPca9685();
  // init sensors to point straight and level
  pca9685.setChannelServoPulseDuration(8, 1500); // horizontal
  pca9685.setChannelServoPulseDuration(12, 1500);// vertical
//  while(1); // hold zero positions to mount sensor arms to motors
}

int servoZeroUs = 1500;
float usPerDeg = 1000/90.0;

int servoDeg2Us(int degree)
{
  int usec;
  usec = servoZeroUs + (usPerDeg * degree);
  return usec;
}


int horDegMin = -75;
int horDegMax =  75;
int horDegInc = 1;
int horDeg = 0;

int verDegMin = -25;
int verDegMax =  0;
int verDegInc = 10;
int verDeg = 0;

// ruturns true when direction changes
bool adjHorDeg()
{
  bool changeDir = false;
  horDeg+=horDegInc;
  
  if(horDeg>horDegMax) 
  {
    horDeg = horDegMax;
    horDegInc = -horDegInc;
    changeDir = true;
  }
  if(horDeg<horDegMin) 
  {
    horDeg = horDegMin;
    horDegInc = -horDegInc;
    changeDir = true;
  }

  pca9685.setChannelServoPulseDuration(8, servoDeg2Us(horDeg)); // horizontal
  delay(1);
  return(changeDir);
}
// ruturns true when direction changes
bool adjVerDeg()
{
  bool changeDir = false;
  verDeg+=verDegInc;
  if(verDeg>verDegMax) 
  {
    verDeg = verDegMax;
    verDegInc = -verDegInc;
    changeDir = true;
  }
  if(verDeg<verDegMin) 
  {
    verDeg = verDegMin;
    verDegInc = -verDegInc;
    changeDir = true;
  }

  pca9685.setChannelServoPulseDuration(12, servoDeg2Us(-verDeg)); // vertical
  delay(1);
  return(changeDir);
}


void loop() {

  if(adjHorDeg()) adjVerDeg(); // adjust vertical when horizontal direction changes

  delay(10); // wait for motion jerk to settle

  distance = ultrasonic.read();
  
  if(distance < 20) {rgbLed(1,1,1); delay(1000);} // white
  else if(distance < 150) rgbLed(0,1,0); // green  
  else if(distance < 300) rgbLed(0,0,1); // blue  
  else rgbLed(1,0,0); // red

  Serial.print(verDeg); Serial.print(", ");
  Serial.print(horDeg); Serial.print(", ");
  Serial.print(distance); Serial.print(", ");
  Serial.print(-50); Serial.print(", ");
   Serial.print(350); Serial.print("\n");
   delay(10);
  
}
