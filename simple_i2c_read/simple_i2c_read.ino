#include <Wire.h>

//int i2cAddr = 0x29; // VL53L5CX
//int i2cAddr = 0x28; // BNO055
int i2cAddr = 0x4A; // BNO085

void setup() {
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();        
  Wire.setClock(400000);
  Wire.setTimeout(1000);
  Serial.begin(9600);       // Start serial for output
}

void loop() {
    Wire.requestFrom(i2cAddr, 4);    // Request 4 bytes from slave device
delay(100);
    // Slave may send less than requested
    while(Wire.available()) {
        int d = Wire.read();    // Receive a byte
        Serial.print(d,HEX);Serial.print(",");         // Print the byte
    }
    Serial.println(";");
    delay(500);
}
