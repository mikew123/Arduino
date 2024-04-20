// HC_SR04 - https://github.com/bjoernboeckle/HC_SR04
// Copyright © 2022, Björn Böckle
// MIT License

#include <Arduino.h>
#include <HC_SR04.h>

HC_SR04<A2> sensorT(A7);   // sensor with echo A6 and trigger A7 pins
HC_SR04<A3> sensorR(A1);   // sensor with echo A6 and trigger A7 pins
HC_SR04<A6> sensorL(A0);   // sensor with echo A6 and trigger A7 pins

void setup() { 
  Serial.begin(9600); 

  sensorL.beginAsync();  
  sensorR.beginAsync();  
  sensorT.beginAsync();  
}

// main loop function
void loop() {

  sensorR.startAsync(0);
  while (!sensorR.isFinished()) delay(0);
delay(2);

  sensorL.startAsync(100000);
  while (!sensorL.isFinished()) delay(0);
delay(2);

  sensorT.startAsync(100000);
  while (!sensorT.isFinished()) delay(0);
delay(2);

  Serial.print("L ");Serial.print(sensorL.getDist_cm());
  Serial.print(", R ");Serial.print(sensorR.getDist_cm());
  Serial.print(", T ");Serial.println(sensorT.getDist_cm());

  delay(100);
}

