// sg90IMUDemo
// The IMU was aded and calibrated but not used to correct for
// physical direction drift from motor imballances

#include <Arduino.h>

// Nano BLE sensors
#include <Arduino_APDS9960.h> // light
#include <Arduino_HTS221.h>   // temp
#include <Arduino_LSM9DS1.h>  // motion
#include <MadgwickAHRS.h>     // motion filter

// external modules
#include <PCA9685.h>  // servo driver

#include <mbed.h>  // rtos
#include <string>

// 65mm dia Mecanum wheel is 20.4cm circumference
// SG90 at 120rpm can drive 24.5M/min = 0.41M/sec?

Madgwick imuFilter;

PCA9685 pca9685;

const PCA9685::DeviceAddress device_address = 0x40;
const PCA9685::Pin output_enable_pin = 2;
const PCA9685::Frequency frequency = 50; // HZ

bool serialOK = 0; // assume serial port is OK until time out waiting
void initSerial()
{
  Serial.begin(9600);
  int i;
  for(i=0; !Serial; i++) {if (i >= 5000) break; delay(1);}
  if (i < 5000) serialOK=1;  
//  if(serialOK) Serial.println(i);
  if(serialOK) Serial.println("initSerial OK");  
  
}

// prints values to serial port after initialization
rtos::Thread serialHandlingThread;

// used to manage sensor serial bus 
rtos::Semaphore sensorSerialSemaphore(1);

rtos::Thread IMUSensorHandlingThread;
float initialHeading;    
float headingDriftRate;
float headingDrift=0;
float heading;
//float roll, pitch;

rtos::Thread HTSSensorHandlingThread;
float initialTemperature;
float currentTemperature;

// read IMU sensors and filter out roll pitch and heading
// uses the sensor serial port semiphore
int readIMUSensorFiltered(float &pitch, float &roll, float &heading)
{
  float ax, ay, az;
  float gx, gy, gz;
    // wait for gyroscope data ready to read
    sensorSerialSemaphore.acquire();
    while (IMU.gyroscopeAvailable() == 0)
    { 
      sensorSerialSemaphore.release();
      delay(1);
      sensorSerialSemaphore.acquire();
    }
    IMU.readGyroscope(gx, gy, gz);
    // assume accel is avialable same time as gyro
    IMU.readAcceleration(ax, ay, az);
    sensorSerialSemaphore.release();

    // update the filter, which computes orientation
    // imuFilter.updateIMU(gx, gy, gz, ax, ay, az);
    // the sensor is mounted vertically around the y axis
    // switch the -X and Z axis sensor values
    imuFilter.updateIMU(gz, gy, -gx, az, ay, -ax);
    roll    = imuFilter.getRoll();
    pitch   = imuFilter.getPitch();
    heading = imuFilter.getYaw();
}

void initSensors()
{
  if(serialOK) Serial.println("initSensors"); 

  if (!APDS.begin()) {
    if(serialOK) Serial.println("Error initializing APDS9960 sensor!");
    while(1);
  }
  if(serialOK) Serial.println("APDS sensor OK"); 

  if (!HTS.begin()) {
    if(serialOK) Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  if(serialOK) Serial.println("HTS sensor OK"); 

  if (!IMU.begin()) {
    if(serialOK) {
      if(serialOK) Serial.println("Failed to initialize IMU!");
      while(1);
    }
  }
  if(serialOK) Serial.println("IMU sensor OK"); 

  float GyroSampleRate = IMU.gyroscopeSampleRate();
  imuFilter.begin(GyroSampleRate);

  //TODO: Capture current heading offset and calibrate drift
  // Why is there heading drift?
  initialHeading = 180.0;    
  headingDriftRate = 0;
 // headingDriftRate += -0.0060; // manual cal

  if(serialOK) Serial.println("Calibrating IMU");
  // calibrate heading  drift rate by averaging 2000 samples
  // run sensor for 10 reads first to "warm" it up
  // this should take about 20 seconds
  int calibrationReads = 2000;
  float p, r, heading;
  float last_heading;
  for(int i=0; i<10; i++) readIMUSensorFiltered(p,r,heading);
  last_heading = heading;
  for(int i=0; i<calibrationReads; i++)
  {
    readIMUSensorFiltered(p,r,heading);
    headingDriftRate += (heading-last_heading);
    last_heading = heading;
   }
  headingDriftRate = headingDriftRate/calibrationReads;
  initialHeading = heading;

  //TODO: Capture temperature during init
  initialTemperature = HTS.readTemperature();


  if(serialOK)
  {
    Serial.print("Gyroscope sample rate = ");
    Serial.print(GyroSampleRate);
    Serial.println(" Hz");
    Serial.print("Gyroscope initial heading = ");
    Serial.println(initialHeading);
    Serial.print("Gyroscope drift rate = ");
    Serial.println(headingDriftRate,4);
    Serial.print("Temperature = ");
    Serial.print(initialTemperature);
    Serial.println(" °C");

    delay(5000);
  }

}

void IMUSensorHandling()
{
  float ax, ay, az;
  float gx, gy, gz;
   while(1){

    // wait for gyroscope data ready to read
    sensorSerialSemaphore.acquire();
    while (IMU.gyroscopeAvailable() == 0)
    { 
      sensorSerialSemaphore.release();
      delay(1);
      sensorSerialSemaphore.acquire();
    }
    IMU.readGyroscope(gx, gy, gz);
    // assume accel is avialable same time as gyro
    IMU.readAcceleration(ax, ay, az);
    sensorSerialSemaphore.release();

    // update the filter, which computes orientation
    // imuFilter.updateIMU(gx, gy, gz, ax, ay, az);
    // the sensor is mounted vertically around the y axis
    // switch the -X and Z axis sensor values
    imuFilter.updateIMU(gz, gy, -gx, az, ay, -ax);
//    roll    = imuFilter.getRoll();
//    pitch   = imuFilter.getPitch();
    heading = imuFilter.getYaw();

    // heading is 0 initialy with a range of +-180
    headingDrift+=headingDriftRate;
    if(headingDrift<=-360.0) headingDrift+=360.0;    
    heading-=(initialHeading+headingDrift);
    if(heading> +180.0) heading-=360.0;
    if(heading<=-180.0) heading+=360.0;
  }
}

void HTSSensorHandling()
{
   while(1){
    // get current temperature
    sensorSerialSemaphore.acquire();
    currentTemperature = HTS.readTemperature();
    sensorSerialSemaphore.release();

    delay(1000);
   }

}

// Servo/wheel/motor
int width_max = 2500;
int width_min =  500;
int stop_nom  = 1500;

enum Wheelmotions_e {
stop=0,
fwd=1,
rev=2,
right=3,
left=4,
fwd_right=5,
fwd_left=6,
rev_right=7,
rev_left=8,
cwise=9,
ccwise=10
};


typedef struct WheelConfigStruct{
  int chan; // servo driver channel
  int pol;  // polarity for fwd motion
  int stop_trim; // trim stop offset from stop_nom
  int min; // min offset from stop_trim for fwd/rev motion
  int motion[11]; // Wheelmotions_e
};

WheelConfigStruct WheelConfig[4];

void initWheels(){
  // Movement
  // Motor 1 front left
  WheelConfig[0].chan = 1;
  WheelConfig[0].pol   = +1; // for +val fwd motion
  WheelConfig[0].stop_trim = +15; // trim stop
  WheelConfig[0].min   = 40;
   // Motor 2 front right
  WheelConfig[1].chan = 5;
  WheelConfig[1].pol   = -1; // for +val fwd motion
  WheelConfig[1].stop_trim = +20; // trim stop
  WheelConfig[1].min   = 40;
  // Motor 3 rear left
  WheelConfig[2].chan = 0;
  WheelConfig[2].pol   = +1; // for +val fwd motion
  WheelConfig[2].stop_trim = +20; // trim stop
  WheelConfig[2].min   = 40;
  // Motor 4 rear right
  WheelConfig[3].chan = 4;
  WheelConfig[3].pol       = -1; // for +val fwd motion
  WheelConfig[3].stop_trim = +15; // trim stop
  WheelConfig[3].min   = 40;

  // motions
  //0
  WheelConfig[0].motion[stop] = 0;
  WheelConfig[1].motion[stop] = 0;
  WheelConfig[2].motion[stop] = 0;
  WheelConfig[3].motion[stop] = 0;
  //1
  WheelConfig[0].motion[fwd] = +1;
  WheelConfig[1].motion[fwd] = +1;
  WheelConfig[2].motion[fwd] = +1;
  WheelConfig[3].motion[fwd] = +1;
  //2
  WheelConfig[0].motion[rev] = -1;
  WheelConfig[1].motion[rev] = -1;
  WheelConfig[2].motion[rev] = -1;
  WheelConfig[3].motion[rev] = -1;
  //3
  WheelConfig[0].motion[right] = +1;
  WheelConfig[1].motion[right] = -1;
  WheelConfig[2].motion[right] = -1;
  WheelConfig[3].motion[right] = +1;
  //4
  WheelConfig[0].motion[left] = -1;
  WheelConfig[1].motion[left] = +1;
  WheelConfig[2].motion[left] = +1;
  WheelConfig[3].motion[left] = -1;
  //5
  WheelConfig[0].motion[fwd_right] = +1;
  WheelConfig[1].motion[fwd_right] =  0;
  WheelConfig[2].motion[fwd_right] =  0;
  WheelConfig[3].motion[fwd_right] = +1;
  //6
  WheelConfig[0].motion[fwd_left] =  0;
  WheelConfig[1].motion[fwd_left] = +1;
  WheelConfig[2].motion[fwd_left] = +1;
  WheelConfig[3].motion[fwd_left] =  0;
  //7
  WheelConfig[0].motion[rev_right] =  0;
  WheelConfig[1].motion[rev_right] = -1;
  WheelConfig[2].motion[rev_right] = -1;
  WheelConfig[3].motion[rev_right] =  0;
  //8
  WheelConfig[0].motion[rev_left] = -1;
  WheelConfig[1].motion[rev_left] =  0;
  WheelConfig[2].motion[rev_left] =  0;
  WheelConfig[3].motion[rev_left] = -1;
  //9
  WheelConfig[0].motion[cwise] = +1;
  WheelConfig[1].motion[cwise] = -1;
  WheelConfig[2].motion[cwise] = +1;
  WheelConfig[3].motion[cwise] = -1;
  //10
  WheelConfig[0].motion[ccwise] = -1;
  WheelConfig[1].motion[ccwise] = +1;
  WheelConfig[2].motion[ccwise] = -1;
  WheelConfig[3].motion[ccwise] = +1;

};

void RGB(bool r,bool g,bool b){
  digitalWrite(LEDR,!r);
  digitalWrite(LEDG,!g);
  digitalWrite(LEDB,!b);
}

void initPca9685(){
  pca9685.setupSingleDevice(Wire,device_address);
  pca9685.setupOutputEnablePin(output_enable_pin);
  pca9685.enableOutputs(output_enable_pin);
  pca9685.setToFrequency(frequency);
}

void ledOn()  {digitalWrite(LED_BUILTIN, HIGH);}
void ledOff() {digitalWrite(LED_BUILTIN, LOW);}
void initLed(){pinMode(LED_BUILTIN, OUTPUT);}

void setup()
{
  initSerial();

  initLed();
  ledOn();
  RGB(1,1,1);

  initPca9685();
  initWheels();
  initSensors();

  IMUSensorHandlingThread.start(IMUSensorHandling);
  HTSSensorHandlingThread.start(HTSSensorHandling);
  serialHandlingThread.start(serialHandling);

  RGB(0,0,0);
  ledOff();

}

void serialHandling()
{
  float t;
  if(serialOK) while(1)
  {
    t = currentTemperature - initialTemperature;
    Serial.print(t);
    Serial.print('\t');
    Serial.print(heading);
//        Serial.print('\t');
//        Serial.print(pitch);
//        Serial.print('\t');
//       Serial.print(roll);
    Serial.print('\n');

    delay(500);
  }
}


int motions[] = {
  stop,
  fwd,
  left,
  rev,
  right,
  fwd_right,
  fwd_left,
  rev_left,
  rev_right,
  cwise,
  ccwise
};
/*
int motions[] = {
  stop,
  fwd,
  rev
};
*/
int Sdelay = 1;
int Tdelay = 5000;
float speed = 1;


void loop()
{
  for(int motion : motions) {

    for (int servo=0;servo<4;servo++){
      int dir  = WheelConfig[servo].motion[motion];
      int chan = WheelConfig[servo].chan;
      int pol  = WheelConfig[servo].pol;
      int min  = WheelConfig[servo].min;
      int stop = stop_nom + WheelConfig[servo].stop_trim;
      int drv  = stop + speed*pol*dir*min;
      pca9685.setChannelServoPulseDuration(chan, drv);
      delay(Sdelay);
    }

    switch(motion) {
      case stop:      RGB(1,0,0);break; //red
      case fwd:       RGB(0,1,0);break; // green
      case rev:       RGB(0,0,1);break; // blue
      case right:     RGB(1,1,0);break;
      case left:      RGB(1,0,1);break;
      case fwd_right: RGB(1,0,0);break;
      case fwd_left:  RGB(0,1,0);break;
      case rev_right: RGB(0,1,1);break;
      case rev_left:  RGB(1,1,1);break;
      case cwise:     RGB(0,1,0);break;
      case ccwise:    RGB(0,0,1);break;
    }

    delay(Tdelay);
  }
  
}
