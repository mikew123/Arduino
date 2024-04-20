// HC_SR04 - https://github.com/bjoernboeckle/HC_SR04
// Copyright © 2022, Björn Böckle
// MIT License

#include <Arduino.h>
#include <HC_SR04.h>

HC_SR04<A2> sensorT(A7);   // Top   sensor with echo A6 and trigger A7 pins
HC_SR04<A3> sensorR(A1);   // Right sensor with echo A6 and trigger A7 pins
HC_SR04<A6> sensorL(A0);   // Left  sensor with echo A6 and trigger A7 pins

void setup() {  
  Serial.begin(9600);  
  sensorL.begin(); 
  sensorR.begin(); 
  sensorT.begin(); 
  }

void loop() {
  sensorL.startMeasure();
  Serial.print("L ");Serial.print(sensorL.getDist_cm());
  sensorR.startMeasure();
  Serial.print(", R ");Serial.print(sensorR.getDist_cm());
  sensorT.startMeasure();
  Serial.print(", T ");Serial.println(sensorT.getDist_cm());

  delay(100);
}

