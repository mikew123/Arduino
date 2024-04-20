/******************************************************************
  @file       fusionAHRS.ino
  @brief      Demonstrates the updated Madgwick Filter called Fusion
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.1.0
  Date:        15/12/22

  1.0.0 Original Release.           22/02/22
  1.1.0 Added NONE fusion option.   25/05/22
  2.0.0 Changed Repo & Branding     15/12/22
  2.0.1 Invert Gyro Values PR       24/12/22
  2.1.0 Updated Fusion Library      30/12/22

******************************************************************/

#include <ReefwingAHRS.h>

LSM9DS1 imu;
EulerAngles angles;

int loopFrequency = 0;
const long displayPeriod = 1000;
//const long displayPeriod = 15000; // 15 sec for long term plot
unsigned long previousMillis = 0;

float declinationAngle;

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Positive magnetic declination - Sydney, AUSTRALIA
//  declinationAngle = 12.717; // Sydney Austarlia
  declinationAngle = 14.0; // Portland Oregon USA
//  imu.setDeclination(12.717);
  imu.setDeclination(declinationAngle);

  imu.setFusionAlgorithm(SensorFusion::FUSION);
  imu.setFusionPeriod(0.01f);   // Estimated sample period = 0.01 s = 100 Hz
//  imu.setFusionGain(0.5);       // Default Fusion Filter Gain - try 7.5 for a much quicker response
  imu.setFusionGain(7.5);       // Default Fusion Filter Gain - try 7.5 for a much quicker response

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 

  // Average of Portland calibration results
	imu.loadAccBias(-0.0295105, 0.02026375, -0.00950625);
	imu.loadGyroBias(-1.5420915, 0.074768, 0.20187375);
	imu.loadMagBias(-0.24807725, 0.34835825, 0.00927725);

    imu.start();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void loop() {
  //  Check for new IMU data and update angles
  angles = imu.update();
  // The FUSION mode does not compute the heading variable
  // heading = yaw - declenation angle
  angles.heading = angles.yaw - declinationAngle;
  //  Wait for new sample - 7 ms delay provides a 100Hz sample rate / loop frequency
  delay(7);  

  //  Display sensor data every displayPeriod, non-blocking.
  if (millis() - previousMillis >= displayPeriod) {

if(1){   
    Serial.print("Roll: ");
    Serial.print(angles.roll);
    Serial.print("\tPitch: ");
    Serial.print(angles.pitch);
    Serial.print("\tYaw: ");
    Serial.print(angles.yaw);
    Serial.print("\tHeading: ");
    Serial.print(angles.heading);
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

} else {
    // print heading for serial plotter
    Serial.print(angles.heading); Serial.print(" ");
    Serial.print(angles.yaw); Serial.print("\n");
}
    
    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}