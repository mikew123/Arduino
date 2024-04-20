// HC_SR04 - https://github.com/bjoernboeckle/HC_SR04
// Copyright © 2022, Björn Böckle
// MIT License

#include <Arduino.h>
#include <HC_SR04.h>

HC_SR04_BASE *Slaves[] = { new HC_SR04<A3>() };
HC_SR04<A6> sonicMaster(A7, Slaves, 1);

void setup() { 
  Serial.begin(9600); 
  sonicMaster.beginAsync();
  sonicMaster.startAsync(0);
}

// main loop function
int count = 0;
void loop() {
  if (sonicMaster.isFinished()) {
    int distanceR = sonicMaster.getDist_cm(0);
    int distanceL = sonicMaster.getDist_cm(1);

    Serial.print(count); 
    Serial.print(", "); Serial.print(distanceR);
    Serial.print(", "); Serial.println(distanceL);

    sonicMaster.startAsync(10000);
        count = 0;
  }

  // do something usefull
  Serial.print(".");
  count++;
  delay(10);
}

