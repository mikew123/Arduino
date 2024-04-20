#include <Arduino.h>
#include <PCA9685.h>

// 65mm dia Mecanum wheel is 20.4cm circumference
// SG90 at 120rpm can drive 24.5M/min = 0.41M/sec?
// Metal gear SGM90 is rated for 50rpm = 10.2M/min = 0.17M/sec

PCA9685 pca9685;

const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ

int servo_num = 0;
int width_max = 2500;
int width_min = 500;

int fwd_max = 500; //2500;
int rev_max = -500; //500;
int stop_nom = 1500;

typedef struct WheelConfigStruct{
  int chan; // servo driver channel
  int pol;  // polarity for fwd motion
  int stop_trim; // trim stop offset from stop_nom
  int min; // min offset from stop_trim for fwd/rev motion
};

WheelConfigStruct WheelConfig[4];

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);

  // Motor 1 front left
  WheelConfig[0].chan = 1;
  WheelConfig[0].pol   = +1; // for +val fwd motion
  WheelConfig[0].stop_trim = +20; // trim stop
  WheelConfig[0].min   = 35;
   // Motor 2 front right
  WheelConfig[1].chan = 5;
  WheelConfig[1].pol   = -1; // for +val fwd motion
  WheelConfig[1].stop_trim = +25; // trim stop
  WheelConfig[1].min   = 35;
  // Motor 3 rear left
  WheelConfig[2].chan = 0;
  WheelConfig[2].pol   = +1; // for +val fwd motion
  WheelConfig[2].stop_trim = +20; // trim stop
  WheelConfig[2].min   = 30;
  // Motor 4 rear right
  WheelConfig[3].chan = 4;
  WheelConfig[3].pol       = -1; // for +val fwd motion
  WheelConfig[3].stop_trim = +15; // trim stop
  WheelConfig[3].min   = 40;

  digitalWrite(LEDR,HIGH);
  digitalWrite(LEDG,HIGH);
  digitalWrite(LEDB,HIGH);
}

int Tdelay = 5000;
float scale = 1.0;

void loop()
{
  for (int servo=0;servo<4;servo++){
    int dir = 0; // stop
    int chan = WheelConfig[servo].chan;
    int pol  = WheelConfig[servo].pol;
    int min  = WheelConfig[servo].min;
    int stop = stop_nom + WheelConfig[servo].stop_trim;
    int drv  = stop + scale*pol*dir*min;
    pca9685.setChannelServoPulseDuration(chan, drv);
  }
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LEDR,LOW);
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDB,HIGH);
    delay(Tdelay);

  for (int servo=0;servo<4;servo++){
    int dir = +1; // fwd
    int chan = WheelConfig[servo].chan;
    int pol  = WheelConfig[servo].pol;
    int min  = WheelConfig[servo].min;
    int stop = stop_nom + WheelConfig[servo].stop_trim;
    int drv  = stop + scale*pol*dir*min;
    pca9685.setChannelServoPulseDuration(chan, drv);
   }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LEDR,HIGH);
    digitalWrite(LEDG,LOW);
    digitalWrite(LEDB,HIGH);
    delay(Tdelay);

  for (int servo=0;servo<4;servo++){
    int dir = 0; // stop
    int chan = WheelConfig[servo].chan;
    int pol  = WheelConfig[servo].pol;
    int min  = WheelConfig[servo].min;
    int stop = stop_nom + WheelConfig[servo].stop_trim;
    int drv  = stop + scale*pol*dir*min;
    pca9685.setChannelServoPulseDuration(chan, drv);
   }

    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LEDR,LOW);
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDB,HIGH);
    delay(Tdelay);

  for (int servo=0;servo<4;servo++){
    int dir = -1; // rev
    int chan = WheelConfig[servo].chan;
    int pol  = WheelConfig[servo].pol;
    int min  = WheelConfig[servo].min;
    int stop = stop_nom + WheelConfig[servo].stop_trim;
    int drv  = stop + scale*pol*dir*min;
    pca9685.setChannelServoPulseDuration(chan, drv);
  }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LEDR,HIGH);
    digitalWrite(LEDG,HIGH);
    digitalWrite(LEDB,LOW);
    delay(Tdelay);

}
