// NanoBleLed.h

// Library to control LEDs on Arduino Nano BLE
// Mike Williamson 2/8/2023

#ifndef NanoBleLed_h
#define NanoBleLed_h

#include <Arduino.h>

class NanoBleLed
{
public:
  void initLed();

  void ledOn();
  void ledOff();

  void rgbLed(bool r,bool g,bool b);
};

#endif