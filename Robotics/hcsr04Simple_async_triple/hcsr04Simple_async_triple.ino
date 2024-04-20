// HC_SR04 - https://github.com/bjoernboeckle/HC_SR04
// Copyright © 2022, Björn Böckle
// MIT License

#include <Arduino.h>
#include <HC_SR04.h>

HC_SR04_BASE *Slaves[] = { new HC_SR04<A3>(),
                           new HC_SR04<A2>()};
HC_SR04<A6> sonicMaster(A7, Slaves, 2);

void setup() { 
  Serial.begin(9600); 
  sonicMaster.beginAsync();
  sonicMaster.startAsync(0);
}

// main loop function
int count = 0;
void loop() {
  if (sonicMaster.isFinished()) {
    int distanceL = sonicMaster.getDist_cm(0);
    int distanceR = sonicMaster.getDist_cm(1);
    int distanceT = sonicMaster.getDist_cm(2);

    Serial.print(count); 
    Serial.print(", R "); Serial.print(distanceR);
    Serial.print(", L "); Serial.print(distanceL);
    Serial.print(", T "); Serial.println(distanceT);

    sonicMaster.startAsync(0);
    count = 0;
  }

  // do something useful
  Serial.print(".");
  count++;
  delay(50); // seems like 25 msec is minimum to not restart
  
  if(count>=10)
  {
    Serial.println("\n Restart Async ");
    sonicMaster.startAsync(0);
    count = 0;
  }
}

