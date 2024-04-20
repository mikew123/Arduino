// ----------------------------------------------------------------------------
// SG90Qpod_walk_bt_rtos_prox sketch
//
// have the Quadrapod walk simple
// Use Bluetooth to control
// Use mbed RTOS to handle commands from BLE APP
// Use proximity data for autonomous mode
//
// The Quadrapod servo motors are controlled by an external PCA9685 module 
// using I2C. The PCA9685 library is used in the Qpod library
//   
// The external servo controller is the Zio 16 Servo Controller using the PCA9685
// Other modules using PCA9685 could be used instead
//
// Author: Mike Williamson
// License: public domain
// ----------------------------------------------------------------------------

#include <Arduino.h>

#include <string>
//string s;

#include <mbed.h>
//using namespace mbed;
//using namespace rtos;
rtos::Thread pollBLEThread;
rtos::Thread proximityHandlingThread;

#include <Qpod.h>
// Create qpod instance
Qpod qpod;

// Sensors
#include <Arduino_APDS9960.h>

// Bluetooth
#include <ArduinoBLE.h>
BLEService ctrlService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE Control Service
// BLE  data Read Write Characteristic - for APP to send control data to Arduino
BLEByteCharacteristic ctrlDataRW("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// BLE walk state string characteristic 20 char+eol from Arduino to APP
char walkState[21] = "12345678901234567890";
BLECharacteristic walkStateRW("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify, walkState);
BLEDevice central;



enum walkModes {stopped, forward, reverse, rotateLeft, rotateRight, zeroServos};
walkModes walk_mode = stopped;
bool automationEnabled = false;

int walk_lcr = 0; // init center walk, -N=left, +N=right
int walk_mode_seq_counter = 0;

bool serialOK = 0; // assume OK until time out waiting
void initSerial()
{
  Serial.begin(9600);
  int i;
  for(i=0; !Serial; i++) {if (i >= 5000) break;}
  if (i < 5000) serialOK=1;  
//  if(serialOK) Serial.println(i);
}

// RGB led pins
 #define RED 22     
 #define BLUE 24     
 #define GREEN 23

void setRgbLed(bool r, bool g, bool b)
{
  // The LED pins seem to be inverted: low=ON
  digitalWrite(RED,   !r);
  digitalWrite(BLUE,  !b);
  digitalWrite(GREEN, !g);
}

void initRgbLed()
{
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  setRgbLed(0,0,0);
}


int servo_angle_degrees[16];
int servo_angle_degrees_increment[16];
bool led = HIGH;

int forward_sequence_cnt = 14;
int forward_sequence[14][16] = 
{
//  left rear         right rear        right front       left front      
  {  0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // standing all legs front tips on ground
  {-45, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // sweep legs to back to walk forward
  // move each leg forward 1 at a time
  // right front servos 8,9,10
  {-45, -55, -45, 0, -45, -55, -45, 0,   0, -55, -35, 0,   0, -55, -45, 0}, // lift
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -35, 0,   0, -55, -45, 0}, // forward
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // lower
  // left front servos 12,13,14
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,   0, -55, -35, 0}, // lift
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,  45, -55, -35, 0}, // forward
  {-45, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lower
  // right rear servos 4,5,6
  {-45, -55, -45, 0, -45, -55, -35, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lift
  {-45, -55, -45, 0,   0, -55, -35, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // forward
  {-45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lower
  // left rear leg servos 0,1,2 (leg 0)
  {-45, -55, -35, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lift
  {  0, -55, -35, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // forward
  {  0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}  // lower
};

int reverse_sequence_cnt = 14;
int reverse_sequence[14][16] = 
{
//  left rear         right rear        right front       left front      
  {-45, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // standing all legs front tips on ground
  {  0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // sweep legs to back to walk forward
  // move each leg forward 1 at a tim
  // right front servos 8,9,10
  {  0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -35, 0,  45, -55, -45, 0}, // lift
  {  0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -35, 0,  45, -55, -45, 0}, // forward
  {  0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // lower
  // left front servos 12,13,14
  {  0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -35, 0}, // lift
  {  0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -35, 0}, // forward
  {  0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // lower
  // right rear servos 4,5,6
  {  0, -55, -45, 0,   0, -55, -35, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // lift
  {  0, -55, -45, 0, -45, -55, -35, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // forward
  {  0, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // lower
  // left rear leg servos 0,1,2 (leg 0)
  {  0, -55, -35, 0, -45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // lift
  {-45, -55, -35, 0, -45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // forward
  {-45, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}  // lower
};

int rotate_left_sequence_cnt = 14;
int rotate_left_sequence[14][16] = 
{
//  left rear         right rear        right front       left front      
  {-45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // standing all legs front tips on ground
  {  0, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // sweep legs left to rotate right
  // move each leg 1 at a time
  // right front servos 8,9,10
  {  0, -55, -45, 0, -45, -55, -45, 0,   0, -55, -35, 0,  45, -55, -45, 0}, // lift
  {  0, -55, -45, 0, -45, -55, -45, 0,  45, -55, -35, 0,  45, -55, -45, 0}, // forward
  {  0, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,  45, -55, -45, 0}, // lower
  // left front servos 12,13,14
  {  0, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,  45, -55, -35, 0}, // lift
  {  0, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,   0, -55, -35, 0}, // forward
  {  0, -55, -45, 0, -45, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // lower
  // right rear servos 4,5,6
  {  0, -55, -45, 0, -45, -55, -35, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // lift
  {  0, -55, -45, 0,   0, -55, -35, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // forward
  {  0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // lower
  // left rear leg servos 0,1,2
  {  0, -55, -35, 0,   0, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // lift
  {-45, -55, -35, 0,   0, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // forward
  {-45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}  // lower
};

int rotate_right_sequence_cnt = 14;
int rotate_right_sequence[14][16] = 
{
//  left rear         right rear        right front       left front      
  {  0, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // standing all legs front tips on ground
  {-45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0,   0, -55, -45, 0}, // sweep legs left to rotate right
  // move each leg 1 at a time
  // right front servos 8,9,10
  {-45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -35, 0,   0, -55, -45, 0}, // lift
  {-45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -35, 0,   0, -55, -45, 0}, // forward
  {-45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0}, // lower
  // left front servos 12,13,14
  {-45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0,   0, -55, -35, 0}, // lift
  {-45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -35, 0}, // forward
  {-45, -55, -45, 0,   0, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // lower
  // right rear servos 4,5,6
  {-45, -55, -45, 0,   0, -55, -35, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // lift
  {-45, -55, -45, 0, -45, -55, -35, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // forward
  {-45, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // lower
  // left rear leg servos 0,1,2
  {-45, -55, -35, 0, -45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // lift
  {  0, -55, -35, 0, -45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0}, // forward
  {  0, -55, -45, 0, -45, -55, -45, 0,   0, -55, -45, 0,  45, -55, -45, 0}  // lower
};

void sendWalkStateRW(char * str)
{
  walkStateRW.writeValue(str,sizeof(str));
}

void  initBLEservice()
{
  // begin initialization
  if (!BLE.begin()) {
    if(serialOK) Serial.println("starting BLE failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("QPOD");
  BLE.setAdvertisedService(ctrlService);

  // add the characteristics to the service
  ctrlService.addCharacteristic(ctrlDataRW);
  ctrlService.addCharacteristic(walkStateRW);

  // add service
  BLE.addService(ctrlService);

  // set the initial value for the characeristics
  ctrlDataRW.writeValue(0);
  sendWalkStateRW("QPOD init");

  // start advertising
  BLE.advertise();

  central = BLE.central();

  pollBLEThread.start(pollBLE);

  if(serialOK) Serial.println("BLE QPOD Peripheral");
}

void initSensors()
{
  if (!APDS.begin()) {
    if(serialOK) Serial.println("Error initializing APDS9960 sensor!");
  }
  proximityHandlingThread.start(proximityHandling);
}

void proximityHandling()
{
  int readCnt = 0;
  int valueSum = 0;
  int proximity = 0;
  int proximityAvg = 0;
  
  if(serialOK) Serial.println("proximityHandling Tread started");

  while(1)
  {
    valueSum = 0;
    for(readCnt=0;readCnt<100;readCnt++)
    {
      // wait for a proximity reading is available
      while (APDS.proximityAvailable() == 0) delay(1);

      // read the proximity
      // - 0   => close
      // - 255 => far
      // - -1  => error
      proximity = APDS.readProximity();
      valueSum += proximity;       
    }
    
    if(readCnt==100) 
    {
      proximityAvg = valueSum/readCnt;
      if(serialOK) Serial.println(proximityAvg);
      if(automationEnabled && (walk_mode==forward))
      {
        if(proximityAvg<220)
        {
          walk_mode = reverse;
          sendWalkStateRW("Auto reverse");
          walk_mode_seq_counter = 0;
        }
      }
    }
    
    // Automation state machine
    // when proximity changes walk mode to non-forward
    switch(walk_mode)
    {
      case reverse:
        if(walk_mode_seq_counter >= random(1,4)) 
        {
          if(random(0,2)==0) walk_mode = rotateRight;
          else walk_mode = rotateLeft;
          sendWalkStateRW("Auto rotateRight");
          walk_mode_seq_counter = 0;
        }       
        break;
      
      case rotateRight:
      case rotateLeft:
        if(walk_mode_seq_counter >= random(3,8)) 
        {
          walk_mode = forward;
          sendWalkStateRW("Auto forward");
          walk_mode_seq_counter = 0;
        }       
        break;
    }

    delay(1);
  }
}

void pollBLE()
{
  int value;
  
  if(serialOK) Serial.println("pollBLE Tread started");

  while(1)
  {  
     if(central.connected())
    {
      if (ctrlDataRW.written()) 
      {
        value = ctrlDataRW.value();
        if(serialOK) {Serial.print("ctrlDataRW = ");Serial.println(value);}
        switch (value)
        {
          case 0: // Stop Qpod movement
            automationEnabled = false;
            if(serialOK) Serial.println("Stop");
            walk_mode = stopped;
            sendWalkStateRW("Stop");
            break;
          case 1: // Forward Qpod movement
            automationEnabled = false;
            if(serialOK) Serial.println("Forward");
            walk_mode = forward;
            sendWalkStateRW("Forward");
            break;
          case 2: // Reverse Qpod movement
            automationEnabled = false;
            if(serialOK) Serial.println("Reverse");
            walk_mode = reverse;
            sendWalkStateRW("Reverse");
            break;
          case 3: // Rotate Left Qpod movement
            automationEnabled = false;
            if(serialOK) Serial.println("Rotate Left");
            walk_mode = rotateLeft;
            sendWalkStateRW("Rotate Left");
            break;
          case 4: // Rotate Right Qpod movement
            automationEnabled = false;
            if(serialOK) Serial.println("Rotate right");
            walk_mode = rotateRight;
            sendWalkStateRW("Rotate right");
            break;
          case 5: // Zero joint angles for adjusting servo mounts
            automationEnabled = false;
            if(serialOK) Serial.println("Zero servos");
            // zero each servo joint angle 
            for (int i=0; i<16; i++)
            {
              servo_angle_degrees_increment[i] = 1; // start pos servo sweep
              servo_angle_degrees[i] = 0;
              qpod.setServoDegrees(i, servo_angle_degrees[i]); 
            }
            walk_mode = zeroServos;
            sendWalkStateRW("Zero servos");
            break;

          case 13: // Faster Qpod movement speed
            if(serialOK) Serial.println("Faster");
            if(qpod.incWalkSpeed()==-1) {if(serialOK) Serial.println("Limited");}          
            break;
          case 14: // Slower Qpod movement speed
            if(serialOK) Serial.println("Slower");
            if(qpod.decWalkSpeed()==-1) {if(serialOK) Serial.println("Limited");}
            break;
          case 15: // adjust walk left wards
            if(serialOK) Serial.println("Left");
            if(walk_lcr>-30) 
            {
              walk_lcr--;
            }
            else if(serialOK) Serial.println("Limited");
            break;
          case 16: // reset walk adjust to center
            if(serialOK) Serial.println("Center");
            walk_lcr=0;
            break;
          case 17: // Slower Qpod movement speed
            if(serialOK) Serial.println("Right");
            if(walk_lcr<30) 
            {
              walk_lcr++;
            }
            else if(serialOK) Serial.println("Limited");
            break;


          case 20: // Turn automation ON
            automationEnabled = true; //default
            switch(walk_mode)
            {
              case forward:     sendWalkStateRW("Auto forward");     break;
              case reverse:     sendWalkStateRW("Auto reverse");     break;
              case rotateRight: sendWalkStateRW("Auto rotateRight"); break;
              case rotateLeft:  sendWalkStateRW("Auto rotateLeft");  break;
              default: automationEnabled = false; break; // walk_mode not automate
            }
            break;
        }
            if(serialOK && automationEnabled) Serial.println("Automation turned ON");
      }
    }
    delay(1);
  }
}

void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  initRgbLed();
  setRgbLed(1,0,0); // RED

  initSerial();

  initBLEservice();

  initSensors();
  
  //  initialize servo driver etc
  qpod.init();


  setRgbLed(0,0,1); // BLUE
}


void loop()
{
  // listen for BLE peripherals to connect:
//  BLEDevice central = BLE.central();
  central = BLE.central();

  // if a central is connected to peripheral:
  if (central) 
  {
    setRgbLed(0,1,0); // GREEN when connected

    // initialize each servo joint angle to its nominal 
    for (int i=0; i<16; i++)
    {
      servo_angle_degrees_increment[i] = 1; // start pos servo sweep
      servo_angle_degrees[i] = QpodGlob::servo[i].servo_degrees_nom;
      qpod.setServoDegrees(i, servo_angle_degrees[i]); 
    }

    if(serialOK) Serial.print("Connected to central: ");
    // print the central's MAC address:
    if(serialOK) Serial.println(central.address());
    // while the central is still connected to peripheral:
    while (central.connected()) 
    {
      int walk_seq_cnt;
      int (* walk_seq)[16]; // pointer to 2 dim sequence array
      switch(walk_mode) 
      {
        case forward:
          walk_seq = forward_sequence;
          walk_seq_cnt = forward_sequence_cnt;
          break;
        case reverse:
          walk_seq = reverse_sequence;
          walk_seq_cnt = reverse_sequence_cnt;
          break;
        case rotateRight:
          walk_seq = rotate_right_sequence;
          walk_seq_cnt = rotate_right_sequence_cnt;
          break;
        case rotateLeft:
          walk_seq = rotate_left_sequence;
          walk_seq_cnt = rotate_left_sequence_cnt;
          break;
        default:
          walk_seq = NULL;
          walk_seq_cnt = 0;
          break;          
      }
      for (int seq=0; seq<walk_seq_cnt; seq++)
      {
        int servos_active = 16;
        while(servos_active != 0)
        {
          servos_active = 16;
          for (int servo=0;  servo<16; servo++)
          {
            int walk_servo = walk_seq[seq][servo];

            if(walk_mode == forward)
            {
              if((walk_lcr<0) && (servo==12)&&((seq==1)||(seq==2)||(seq==3)||(seq==4)||(seq==5)))
              { // adjust walk towards the left
                walk_servo += -walk_lcr; 
              }
              if((walk_lcr>0) && (servo==8)&&((seq==1)||(seq==2)))
              { // adjust walk towards the right
                walk_servo += walk_lcr; 
              }
            }

            if(servo_angle_degrees[servo] == walk_servo)
            {
              servos_active--; // sequence stops when all servos have moved to their seq position
            }
            else
            {
              if(servo_angle_degrees[servo] < walk_servo)
              {
                servo_angle_degrees[servo]++;
              }
              else if(servo_angle_degrees[servo] > walk_servo)
              {
                servo_angle_degrees[servo]--;
              }

              qpod.setServoDegrees(servo, servo_angle_degrees[servo]); 

            }
          }

        }
        // toggle LED every sequence
        led = !led;
        digitalWrite(LED_BUILTIN, led);
      }
      // count number of walk mode complete sequences occur
      walk_mode_seq_counter++;

    } // end while BLE connected
    // when the central disconnects, print it out:
    if(serialOK) Serial.print(F("Disconnected from central: "));
    if(serialOK) Serial.println(central.address());
  } // end when BLE connected

  qpod.disableServos();  
  setRgbLed(0,0,1); // BLUE when disconnected

  delay(0); // call delay to insure task switch cleanly, dont know if it helps
};

