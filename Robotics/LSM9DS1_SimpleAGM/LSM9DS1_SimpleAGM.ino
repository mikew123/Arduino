/*
  Arduino LSM9DS1 - Combined Simple Acc,Gyr,Mag

  Mikew
*/

#include <Arduino_LSM9DS1.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  // Magnometer has slowest update rate
  if (IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(ax, ay, az);
//    if (IMU.gyroscopeAvailable()) 
      IMU.readGyroscope(gx, gy, gz);
//    if (IMU.accelerationAvailable())
      IMU.readMagneticField(mx, my, mz);

    Serial.print("A: ");
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(ay);
    Serial.print(", ");
    Serial.print(az);

    Serial.print("\tG: ");
    Serial.print(gx);
    Serial.print(", ");
    Serial.print(gy);
    Serial.print(", ");
    Serial.print(gz);

    Serial.print("\tM: ");
    Serial.print(mx);
    Serial.print(", ");
    Serial.print(my);
    Serial.print(", ");
    Serial.println(mz);

    delay(1000);
  }
}
