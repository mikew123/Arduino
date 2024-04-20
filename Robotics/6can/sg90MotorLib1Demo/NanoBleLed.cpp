// NanoBleLed.cpp

// Library to control LEDs on NanoBLE
// Mike Williamson 2/8/2023

#include "NanoBleLed.h"

void NanoBleLed::initLed(){pinMode(LED_BUILTIN, OUTPUT);}

void NanoBleLed::ledOn()  {digitalWrite(LED_BUILTIN, HIGH);}
void NanoBleLed::ledOff() {digitalWrite(LED_BUILTIN, LOW);}

void NanoBleLed::rgbLed(bool r,bool g,bool b)
{
  digitalWrite(LEDR,!r);
  digitalWrite(LEDG,!g);
  digitalWrite(LEDB,!b);
}
