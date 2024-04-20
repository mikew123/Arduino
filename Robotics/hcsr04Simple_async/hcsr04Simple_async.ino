// HC_SR04 - https://github.com/bjoernboeckle/HC_SR04
// Copyright © 2022, Björn Böckle
// MIT License

#include <Arduino.h>
#include <HC_SR04.h>

//HC_SR04<2> sensor(8);   // sensor with echo and trigger pin
HC_SR04<A6> sensor(A7);   // sensor with echo A6 and trigger A7 pins

void setup() { 
  Serial.begin(9600); 
  sensor.beginAsync();  
  sensor.startAsync(100000);        // start first measurement
}

// main loop function
int count = 0;
void loop() {
  if (sensor.isFinished()) {
    Serial.print(count); Serial.print(", ");
    Serial.println(sensor.getDist_cm());
//    delay(1000);

    sensor.startAsync(100000);
    count = 0;
  }

  // do something usefull
  count++;
}

