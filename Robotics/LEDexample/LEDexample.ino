/*
  LED

  This example creates a BLE peripheral with service that contains a
  characteristic to control an LED.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>


bool serialOK = 1; // assume OK untill time out waiting
void initSerial()
{
  Serial.begin(9600);
  for(int i=0; !Serial; i++) {if (i == 10000){serialOK=0; break;}}
}

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void  initBLEservice()
{
  // begin initialization
  if (!BLE.begin()) {
    if(serialOK) {Serial.println("starting BLE failed!");}

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("LED");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
}

const int ledPin = LED_BUILTIN; // pin to use for the LED

// RGB led pins
 #define RED 22     
 #define BLUE 24     
 #define GREEN 23

void setRgbLed(bool r, bool g, bool b)
{
  // The LED pins seem to be inverted: low=ON
  digitalWrite(RED,   !r);
  digitalWrite(BLUE,  !b);
  digitalWrite(GREEN, !g);
}

void initRgbLed()
{
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  setRgbLed(0,0,0);
}

void setup() {
  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  initRgbLed();
  setRgbLed(1,0,0); // RED

  initSerial();

  initBLEservice();

  setRgbLed(0,0,1); // BLUE
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    setRgbLed(0,1,0); // GREEN

    if(serialOK) {Serial.print("Connected to central: ");}
    // print the central's MAC address:
    if(serialOK) {Serial.println(central.address());}

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value()) {   // any value other than 0
          if(serialOK) {Serial.println("LED on");}
          digitalWrite(ledPin, HIGH);         // will turn the LED on
        } else {                              // a 0 value
          if(serialOK) {Serial.println(F("LED off"));}
          digitalWrite(ledPin, LOW);          // will turn the LED off
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
