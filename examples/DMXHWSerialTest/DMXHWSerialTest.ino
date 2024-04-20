/*
 DMXHWSerailTest
  
 Software serial multiple serial test

 Receives from the DMX hardware serial, sends to software serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This original example code is in the public domain.

 11/17/2021
 Modified by Mike Williamson to output data on serial port
 which is connected to a Raspberry pi to control lights
 The Leonardo Arduino board was used for test
 Currently using Home Assistant to bridge to Zigbee lights

 */
 
#include <SoftwareSerial.h>

#include <DMXSerial.h>

// This test receives ch 0 to 6 and send string out USB serial:

const int startChannel = 1;
byte *rxbuffer_p;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  DMXSerial.init(DMXReceiver);

  rxbuffer_p = DMXSerial.getBuffer();
  
  // set some default values
  DMXSerial.write(1, 201);
  DMXSerial.write(2, 202);
  DMXSerial.write(3, 203);
  DMXSerial.write(4, 204);
  DMXSerial.write(5, 205);
  DMXSerial.write(6, 206);

}

void loop() { // run over and over
  // process up to 10 frames per second max (100ms)
  delay(100);
  
  // Calculate how long no data bucket was received
  unsigned long lastPacket = DMXSerial.noDataSince();

  byte ch1 = 101;
  byte ch2 = 102;
  byte ch3 = 103;
  byte ch4 = 104;
  byte ch5 = 105;
  byte ch6 = 106;

  if (lastPacket < 5000) {
    ch1 = rxbuffer_p[1];
    ch2 = rxbuffer_p[2];
    ch3 = rxbuffer_p[3];
    ch4 = rxbuffer_p[4];
    ch5 = rxbuffer_p[5];
    ch6 = rxbuffer_p[6];
  } else {
    ch1 = ch1+1;
    ch2 = ch2+1;
    ch3 = ch3+1;
    ch4 = ch4+1;
    ch5 = ch5+1;
    ch6 = ch6+1;
  }
  
//  Serial.println("100,32,64,128");
//  Serial.println(startChannel);
//  Serial.println(lastPacket);
  
  Serial.print(ch1); Serial.print(",");
  Serial.print(ch2); Serial.print(",");
  Serial.print(ch3); Serial.print(",");
  Serial.print(ch4); Serial.print(",");
  Serial.print(ch5); Serial.print(",");
  Serial.print(ch6); Serial.println("");
    
}
