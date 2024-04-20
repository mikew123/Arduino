// sg90MotorDemo
// Adding IMU to offset direction drift while driving
// also to control the rotation to be precise +-90 or +-180 degrees

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

// globals

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
ccwise=10,
numMotions=11
};


typedef struct WheelConfigStruct{
  int chan; // servo driver channel
  int pol;  // polarity for fwd motion
  int stop_trim; // trim stop offset from stop_nom
  int min; // min offset from stop_trim for fwd/rev motion
  // one for each motion
  int motion[numMotions]; // Wheelmotions_e
};

WheelConfigStruct WheelConfig[4];

// PI filter integrators for each motion
// this offsets friction and motor imballances which can be different 
// for each motion
float motionPiIntegral[numMotions]; // wheel PI loop filter intergral


int Sdelay = 1;
int Tdelay = 10000;
float speed = 1;

float piPropCoeff = 2; // Proportional coefficient
float piIntCoeff = 0.1; // Integral coefficient
int integralLimit=50; 
int headingCorrectionLimit=50;

int dir;
int chan;
int pol;
int drvMin;
int stopVal;
int drv;
int motion;

// wheel drive
//float integral = 0;
float proportional;
float headingCorrection;

float stopHeadingOffset=0;
float rawHeading;

float magX, magY, magZ;

// Servo/wheel/motor
int width_max = 2500;
int width_min =  500;
int stop_nom  = 1500;

// prints values to serial port after initialization
rtos::Thread serialHandlingThread;

// used to manage sensor serial bus 
rtos::Semaphore sensorSerialSemaphore(1);

// Magnometer sensor
rtos::Thread MAGSensorHandlingThread;
float iMagX, iMagY, iMagZ;

// Accellerometer and Gyroscope sensors
rtos::Thread AGSensorHandlingThread;
float initialHeading;    
float headingDriftRate;
float headingDrift=0;
float heading;
float roll, pitch;

rtos::Thread HTSSensorHandlingThread;
float initialTemperature;
float currentTemperature;

// read IMU accellerator and gyro sensors and 
// filter out roll pitch and heading
// uses the sensor serial port semiphore
int readMAGSensor(float &x, float &y, float &z)
{

  // wait for gyroscope data ready to read
  sensorSerialSemaphore.acquire();
  while (IMU.magneticFieldAvailable() == 0)
  { 
    sensorSerialSemaphore.release();
    delay(1);
    sensorSerialSemaphore.acquire();
  }
  IMU.readMagneticField(x, y, z);
  sensorSerialSemaphore.release();

}

// read IMU accellerator and gyro sensors and 
// filter out roll pitch and heading
// uses the sensor serial port semiphore
int readAGSensorFiltered(float &pitch, float &roll, float &heading)
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

  // Accelerometer and Gyroscope sensors
  float GyroSampleRate = IMU.gyroscopeSampleRate();
  imuFilter.begin(GyroSampleRate);

  //TODO: Capture current heading offset and calibrate drift
  // Why is there heading drift?
  initialHeading = 180.0; // used to zero the initial heading
  headingDriftRate = 0;
 // headingDriftRate += -0.0060; // manual cal

  if(serialOK) Serial.println("Calibrating IMU");
  // calibrate heading  drift rate by averaging 2000 samples
  // run sensor for 10 reads first to "warm" it up
  // this should take about 20 seconds
  int calibrationReads = 2000;
  float p, r, heading;
  float last_heading;
  for(int i=0; i<10; i++) readAGSensorFiltered(p,r,heading);
  last_heading = heading;
  for(int i=0; i<calibrationReads; i++)
  {
    readAGSensorFiltered(p,r,heading);
    headingDriftRate += (heading-last_heading);
    last_heading = heading;
   }
  headingDriftRate = headingDriftRate/calibrationReads;
  initialHeading = heading;

  // Magnometer sensor
  float MagSampleRate = IMU.magneticFieldSampleRate();
  float x,y,z;
  // Get initial magnometer reading for offset
  // Average 20 reads
  for(int i=0;i<20;i++)
  {
    readMAGSensor(x,y,z);
    iMagX+=x;
    iMagY+=y;
    iMagZ+=z;
  }
  iMagX = iMagX/20;
  iMagY = iMagY/20;
  iMagZ = iMagZ/20;

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

    Serial.print("Magnometer sample rate = ");
    Serial.print(MagSampleRate);
    Serial.println(" Hz");

    Serial.print("Temperature = ");
    Serial.print(initialTemperature);
    Serial.println(" °C");

    delay(5000);
  }

}

void MAGSensorHandling()
{
float x,y,z;
  while(1)
  {
    readMAGSensor(x, y, z);
    // offset initial sensor reading
    y-=iMagY;
     // filter magnometer Y which is used as compus 
    float A = 0.01;
    magY = A*y + (1-A)*magY;
 }
 delay(1);
}

void AGSensorHandling()
{
  float heading_l=0;
  float prevRawHeading = 0;
  float headingDiff;
  float A = 0.001;
  int count=0;
  int prevMotion=-1;
  while(1)
  {
    readAGSensorFiltered(pitch, roll, rawHeading);
    // adjust heading drift rate while motion stop
    if(motion == stop) if(count>0)
    {
      headingDiff = rawHeading-prevRawHeading;
      if(headingDiff<0.1 && headingDiff>-0.1)
        headingDriftRate = A*(headingDiff) +(1-A)*headingDriftRate;
    }
    if(prevMotion != motion)
    {
      if(prevMotion == stop)
      { // exited stop
        // use the magnometer to correct heading 
          stopHeadingOffset += heading_l - magY;
      }
      prevMotion = motion;
    }

    // heading drift based on rate of drift
    headingDrift+=headingDriftRate;
    // NOTE: is this correct wraping?
    if(headingDrift<=-360.0) headingDrift+=360.0;    
//    if(headingDrift>=+360.0) headingDrift-=360.0; 

    // heading is 0 initialy with a range of +-180
    heading_l  = rawHeading;
    heading_l -= stopHeadingOffset;
    heading_l -= (initialHeading+headingDrift);
    // NOTE: is this correct wraping?
    if(heading_l> +180.0) heading_l-=360.0;
    if(heading_l<=-180.0) heading_l+=360.0;

    // update global heading variable
    heading = heading_l;
    prevRawHeading = rawHeading;
    count++;
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
  WheelConfig[1].stop_trim = +25; // trim stop
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
  //5  2Xspeed since only 2 wheels used
  WheelConfig[0].motion[fwd_right] = +2;
  WheelConfig[1].motion[fwd_right] =  0;
  WheelConfig[2].motion[fwd_right] =  0;
  WheelConfig[3].motion[fwd_right] = +2;
  //6
  WheelConfig[0].motion[fwd_left] =  0;
  WheelConfig[1].motion[fwd_left] = +2;
  WheelConfig[2].motion[fwd_left] = +2;
  WheelConfig[3].motion[fwd_left] =  0;
  //7
  WheelConfig[0].motion[rev_right] =  0;
  WheelConfig[1].motion[rev_right] = -2;
  WheelConfig[2].motion[rev_right] = -2;
  WheelConfig[3].motion[rev_right] =  0;
  //8
  WheelConfig[0].motion[rev_left] = -2;
  WheelConfig[1].motion[rev_left] =  0;
  WheelConfig[2].motion[rev_left] =  0;
  WheelConfig[3].motion[rev_left] = -2;
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

  MAGSensorHandlingThread.start(MAGSensorHandling);
  AGSensorHandlingThread.start(AGSensorHandling);
  HTSSensorHandlingThread.start(HTSSensorHandling);
  serialHandlingThread.start(serialHandling);

  RGB(0,0,0);
  ledOff();

}

/*
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
*/
int motions[] = {
  stop,
  fwd,
  rev,
  left,
  right,
  fwd_left,
  rev_right,
  rev_left,
  fwd_right
};

void loop()
{

  for(int m : motions) {
    motion  = m;
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

    int TdelayAccum=0;
    float integral;
    while(TdelayAccum<Tdelay){   
      for (int servo=0;servo<4;servo++){
        dir  = WheelConfig[servo].motion[motion];
        chan = WheelConfig[servo].chan;
        pol  = WheelConfig[servo].pol;
        drvMin  = WheelConfig[servo].min;
        stopVal = stop_nom + WheelConfig[servo].stop_trim;
        drv  = stopVal + speed*pol*dir*drvMin;

        // NOTE: does each servo need its own PI loop filter calc?
        // To correct for heading drift (should be zero)
        // add heading drift offset to front wheels 0 and 1
        // limit speed correction and integral
        // use a PI filter
        integral = motionPiIntegral[motion];
        switch(motion){
          case fwd:
          case rev:
          case left:
          case right:
          case fwd_left:
          case rev_right:
          case rev_left:
          case fwd_right:
            // loop filter
            integral += piIntCoeff*heading;        
            if(integral>+integralLimit) integral=+integralLimit;
            if(integral<-integralLimit) integral=-integralLimit;
            proportional = piPropCoeff*heading;        
            headingCorrection = proportional + integral;
            if(headingCorrection>+headingCorrectionLimit) headingCorrection=+headingCorrectionLimit;
            if(headingCorrection<-headingCorrectionLimit) headingCorrection=-headingCorrectionLimit;
            
            switch(servo){
              case 0: // wheel 1 front left
                drv += headingCorrection*pol*speed; 
                break;
              case 1: // wheel 2 front right 
                drv -= headingCorrection*pol*speed; 
                break;
              case 2: // wheel 3 rear left
                drv += headingCorrection*pol*speed; 
                break;
              case 3: // wheel 4 rear right 
                drv -= headingCorrection*pol*speed; 
                break;
            }
            break;

          default: 
            break;                
        }
        pca9685.setChannelServoPulseDuration(chan, drv);
        // save modified integral value of loop filter
        motionPiIntegral[motion] = integral;

        delay(Sdelay);
      } // end for servos

      delay(100);
      TdelayAccum+=100; // update correction 100ms

    } // end while Tdelay 
  } // end for motions

}

// print various global variables
void serialHandling()
{
  float t;
  if(serialOK) while(1)
  {
    t = currentTemperature - initialTemperature;
    Serial.print(motion/10.0); // t
    Serial.print('\t');
    Serial.print(magY);  // magnometer primary signal
    Serial.print('\t');
    Serial.print(heading);
//    Serial.print('\t');
//    Serial.print(rawHeading); // pitch
//        Serial.print('\t');
//       Serial.print(roll);
//    Serial.print('\t');
//    Serial.print(stopHeadingOffset);
    Serial.print('\t');
    Serial.print(100*headingDriftRate,6);
//    Serial.print('\t');
//    Serial.print(headingDrift);
//    Serial.print('\t');
//    Serial.print(headingCorrection);
//    Serial.print('\t');
//    Serial.print(proportional);
//    Serial.print('\t');
//    Serial.print(integral);
    Serial.print('\n');

    delay(500);
  }
}
