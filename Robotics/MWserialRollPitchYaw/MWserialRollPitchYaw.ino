/******************************************************************
  @file       serialRollPitchYaw.ino
  @brief      Print roll, pitch, yaw and heading angles
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

  This sketch is congigured to work with the MADGWICK, MAHONY,
  CLASSIC and COMPLEMENTARY Sensor Fusion options. Set the 
  algorithm that you wish to use with:

  imu.setFusionAlgorithm(SensorFusion::MADGWICK);

  If you want to test the FUSION (Madgwick v2) algoritm, then use
  the fusionAHRS example sketch or add the FUSION specific
  configuration to this sketch.

******************************************************************/

#include <ReefwingAHRS.h>

LSM9DS1 imu;
EulerAngles angles;

int loopFrequency = 0;
const long displayPeriod = 15*1000L;
unsigned long previousMillis = 0;

float declinationAngle = 14.0; // Portland

//SensorFusion algo = SensorFusion::COMPLEMENTARY;
//SensorFusion algo = SensorFusion::MADGWICK;
SensorFusion algo = SensorFusion::FUSION;
//SensorFusion algo = SensorFusion::MAHONY;
//SensorFusion algo = SensorFusion::CLASSIC;
//SensorFusion algo = SensorFusion::NONE;

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Positive magnetic declination - Sydney, AUSTRALIA
//  imu.setDeclination(12.717);
  imu.setDeclination(declinationAngle);
  imu.setFusionAlgorithm(algo);
//  imu.setFusionAlgorithm(SensorFusion::COMPLEMENTARY);
//  imu.setFusionAlgorithm(SensorFusion::MADGWICK);
//  imu.setFusionAlgorithm(SensorFusion::FUSION);
//  imu.setFusionAlgorithm(SensorFusion::MAHONY);
//  imu.setFusionAlgorithm(SensorFusion::CLASSIC);
//  imu.setFusionAlgorithm(SensorFusion::NONE);

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 

    //  Paste your calibration bias offset HERE
    //  This information comes from the testAndCalibrate.ino 
    //  sketch in the library examples sub-directory.
//    imu.loadAccBias(0.070862, -0.046570, -0.016907);
//    imu.loadGyroBias(0.800018, 0.269165, -0.097198);
//    imu.loadMagBias(-0.192261, -0.012085, 0.118652);

    // Average of Portland calibration results
//    imu.loadAccBias(-0.0295105, 0.02026375, -0.00950625);
//    imu.loadGyroBias(-1.5420915, 0.074768, 0.20187375);
//    imu.loadMagBias(-0.24807725, 0.34835825, 0.00927725);

  // Portland 6/19/2023
	imu.loadAccBias(-0.026306, -0.092834, -0.006042);
	imu.loadGyroBias(-1.547699, 0.089722, 0.216827);
	imu.loadMagBias(-0.214355, 0.242920, -0.035278);

    //  This sketch assumes that the LSM9DS1 is already calibrated, 
    //  If so, start processing IMU data. If not, run the testAndCalibrate 
    //  sketch first.
    imu.start();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }
}

void loop() {
  //  Check for new IMU data and update angles
  angles = imu.update();

  // heading is not calc in lib for FUSION algo
  if(algo == SensorFusion::FUSION) angles.heading = angles.yaw - declinationAngle;

  //  Display sensor data every displayPeriod, non-blocking.
  if (millis() - previousMillis >= displayPeriod) {
Serial.flush();

    //  Uncomment to DEBUG raw sensor data:
    //  SensorData data = imu.rawData();
    //  Serial.print("ax = "); Serial.print(1000*data.ax);  
    //  Serial.print(" ay = "); Serial.print(1000*data.ay); 
    //  Serial.print(" az = "); Serial.print(1000*data.az); Serial.println(" mg");
    //  Serial.print("gx = "); Serial.print( data.gx, 2); 
    //  Serial.print(" gy = "); Serial.print( data.gy, 2); 
    //  Serial.print(" gz = "); Serial.print( data.gz, 2); Serial.println(" deg/s");
    //  Serial.print("mx = "); Serial.print(1000*data.mx ); 
    //  Serial.print(" my = "); Serial.print(1000*data.my ); 
    //  Serial.print(" mz = "); Serial.print(1000*data.mz ); Serial.println(" mG");

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