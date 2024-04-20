//#include <core_build_options.h>
//#include <swRTC.h>


/***************************************************
  This creates an alarm system which sends a SMS message
  and sounds an alarm

  The FeatherFONA module is used
Open up the serial console on the Arduino at 115200 baud to interact with FONA
 ****************************************************/

#define SMSON 1
//#define SMSON 0

#define DEBUG

#define FONA_RX 9
#define FONA_TX 8
#define FONA_RST 4
#define FONA_RI 7

#define LED_PIN 13
#define DOOR_PIN 11
#define KEY_PIN 10
#define ALARM_PIN 12

#define MEASV_PIN A0

#define SENDTO "2147293119"

#define RTCINTERVAL (60*60)

#define OFFTIMEOUT   (24*60*60) /* ARM in 1 day */
#define RESETTIMEOUT (5*60) /* ARM in 5 minutes */
#define FONATIMEOUT  (15*60) /* shut off FONA after 15 minutes in ARM */
#define ALARMTIMEOUT (30*60) /* Re-arm after 30 minutes of ALARM ON*/

#define FONA_OFF_TIME (5*60)

#include <Adafruit_FONA.h>
// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

// We default to using software if(serialUp) Serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

#include "StopWatch.h"
StopWatch wakeupSW(StopWatch::SECONDS);
StopWatch timeoutSW(StopWatch::SECONDS);
StopWatch doorOpenSW(StopWatch::SECONDS);
StopWatch fonaOffSW(StopWatch::SECONDS);

int ledPin = LED_PIN;
int ledVal = 0; // LED OFF

int doorPin = DOOR_PIN;
int doorVal = 0; // All doors closed

int keyPin = KEY_PIN;
int keyVal = 1; // de-asserted

int alarmPin = ALARM_PIN;
int alarmVal = 0; // OFF

long waitWakeup = 0;



// this is the SMS number to send messages to
char sendto[16] = SENDTO;

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

void flushSerial();

uint8_t type;

// FONA rx buffer
char fonaRxBuffer[50];

int serialUp = 0;

int lastOnoff = 1; // assume ON


int fonaStrIx = 0;

// Time fields
struct {
  int yr;
  int mo;
  int day;
  int hr;
  int min;
  int sec;
  int tz;
} myT;


enum alarmStates {OFF, RESET, ARMED, ALARM} ;
const char *alarmStateStr[4] = {"OFF", "RESET", "ARMED", "ALARM"};
                                
int alarmState = RESET;
int alarmStateLast = RESET;
char timeBuffer[23];
  
String tString = "16/07/09,21:51:22-20";
String lstr;

// this is a large buffer for replies
char replybuffer[255];
//char smsMsg[250];
//char msg[256];
//char str[100];
//char sstr[50];
//char smsMsg[500];
char msg[300];
char str[200];
char sstr[100];

//////////////////////////////////////////////////
// helper functions prototypes

// Check the barn door switches
// doorVal=1 when open
void checkDoorSwitch();

// sound alarm for time in msec
// time = -1 leaves alarm ON
void soundAlarm(int timeMs);

// Assert KEY pin on FONA to power up or power down
void keyAssert();
  
// Turn FONA ON or OFF and verify
// onoff=1 turn on, onoff=0 turn OFF
int onoffFona(int onoff);

// Initial connect to FONA module like in testcase
void connectFona();

// send string to FONA
void sendFonaStr(char *str);

// read complete line string from FONA
// return > 0 when end of line char is detected
int fonaReadLine();


// extract date and time from +CCLK string
// Uses global myT struct
// return -1 if invalid
int timeStr2myT(char* tStr);


// Read RTC time into global timeBuffer
void readTime();

// reset the timeout stop watch
void restartTimeoutSW();

// reset the door open stop watch
void restartDoorOpenSW();

// Read RTC and start wakeup timer
// return number of secs to wait
long rdTimeRestartWakeupSWWakeupSW(long interval);

// Send SMS with time
void sendTimeSms();

// send SMS with time appended to string
void sendSmsPlusTime(const char* str);

void sendAlarmOnSms();
void sendAlarmOffSms();
void sendAlarmResetSms();
void sendAlarmArmedSms();

// Send SMS with current status
void sendStatusSms ();

// delete SMS 
void deleteSmsMsg(uint8_t smsn);

// return SMS msg #
//uint8_t getSmsMsg(void);

// Set the time in the RTC in FONA
void setRTCTime(char *timeStr);

// Process any SMS messages 
void processSmsMsg();
char* readSmsMsg(uint8_t smsn);

uint16_t readnumber();

// Measure voltage on A0 pin
// Returns mV
int16_t measv();

// Turn FONA OFF after timer reaches Time
int fonaOffTime = FONA_OFF_TIME;
void fonaOffCheck();
void fonaOffReset();

//////////////////////////////////////
// SETUP
/////////////////////////////////////
void setup() {

  ledVal = 0;
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
  digitalWrite(ledPin, ledVal);   // LED OFF
  
  keyVal = 1;
  pinMode(keyPin, OUTPUT);
  digitalWrite(keyPin, keyVal);   // KEY DE-ASSERTED
  
  alarmVal = 0;
  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin, alarmVal);   // ALARM OFF
  
  pinMode(doorPin, INPUT_PULLUP);

  soundAlarm(250);
  
#ifdef DEBUG
  Serial.begin(115200);
  delay(1000);
  serialUp = Serial;
  delay(1000);
#endif

  if(serialUp)Serial.println(F("Start setup"));

  // Turn FONA on and perform initial connect
  connectFona();
    
  // Read time and Start StopWatch for next FONA wakeup
  waitWakeup = rdTimeRestartWakeupSWWakeupSW(RTCINTERVAL);
  restartTimeoutSW();

  alarmState = RESET;
  
  // command line prefix for serial debug
  if(serialUp) Serial.print(F("\nFONA>"));

  // FONA is ON after setup
}

//////////////////////////////////////
// LOOP
/////////////////////////////////////
void loop() {
  // re-establish serial interface if disconnnected
#ifdef DEBUG
  Serial.begin(115200);
  serialUp = Serial;
#endif

  // echo messages from FONA
  if(fonaReadLine()) {
    if(serialUp) Serial.println(fonaRxBuffer);
    // TODO: parse fona strings
  }
  
  // Check the barn door switches
  // doorVal=1 when open
  checkDoorSwitch();
  
  // Wake up periodically to process sms messages 
  // and send calibrate CPU time from RTC
  wakeupPeriodically();

  // Process SMS messages (if FONA is on)
  processSmsMsg();
  
  // ALARM state machine
  alarmStateMachine();

  // Check for characters on serial port for debugging
#ifdef DEBUG
  serialInterfaceDebug();
#endif

  // Process ALARM state changes
  processAlarmStateChanges();

  // if on then turn off after timer elapses
  fonaOffCheck();
 
  delay(100);
  
} // end loop

///////////////////////////////////////////
// Check the barn door switches
// doorVal=1 when open
void checkDoorSwitch(){
  // Get door switch status 1 = door open
  doorVal = digitalRead(doorPin);
  digitalWrite(ledPin, doorVal);   // RED LED = door switch
  if(doorVal==0) restartDoorOpenSW();
  
}

///////////////////////////////////////////////////////////////
// ALARM state machine
void alarmStateMachine() {
  // ALARM state machine
  switch(alarmState) {
  case RESET: {
    if(timeoutSW.elapsed() > RESETTIMEOUT) {
      alarmState = ARMED;
    }
      break;      
  } // end state == RESET
  
  case OFF: {
    if(timeoutSW.elapsed() > OFFTIMEOUT) {
      alarmState = ARMED;
    }
    break;
  } // end state = OFF
    
  case ARMED: {
    // ALARM when door is open for 3 seconds
    // This helps reduce false alarms
    if(doorOpenSW.elapsed() >= 3) {
      alarmState = ALARM;
    }
    break;
  }// end state == ARMED
    
  case ALARM: {
    if(timeoutSW.elapsed() > ALARMTIMEOUT) {
      alarmState = ARMED;
    }
    break;
  } // end state == ALARM
 } // end case
}

///////////////////////////////////////////////////////////////
 // Wake up periodically to process sms messages 
 // and send calibrate CPU time from RTC
void wakeupPeriodically() {
  if(wakeupSW.elapsed() > waitWakeup) {
    if(serialUp) Serial.println(F("Waked up and processing SMS"));  

    // power UP FONA
    onoffFona(1);
    restartFonaOffSW(FONA_OFF_TIME);

    delay(5000);

    // Read time and Start StopWatch for next read
    // Time parsed into myT structure
    waitWakeup = rdTimeRestartWakeupSWWakeupSW(RTCINTERVAL);
      
    // Send current time SMS message once a day
//    readTime();
//    timeStr2myT(timeBuffer);
    if(serialUp) {Serial.print(F("myT.hr =")); Serial.println(myT.hr);} 
//    if(myT.hr == 7+12) { // 7PM
    if(myT.hr == 13) {
        sendStatusSms();     
    }
    else {
      // process up to 10 SMS messages
      for(int i=0; i<10; i++) processSmsMsg();
      delay(5000);
      
      // power DOWN FONA
      onoffFona(0);
      delay(5000);
    } 
    
    // Reset door open timer
    restartDoorOpenSW();
  }
}

/////////////////////////////////////////////////////////////////////////
// Process ALARM state changes
void processAlarmStateChanges() {
//  char str[100] = "";
  str[0]=0; //""
  if(alarmState != alarmStateLast) {
        
    soundAlarm(500);
    strcat(str, alarmStateStr[alarmStateLast]);
    strcat(str, "->");
    strcat(str, alarmStateStr[alarmState]);
    if(serialUp) Serial.println(str);
        
    switch(alarmState) {
      case OFF: {       
        soundAlarm(0);
        // power up FONA
        onoffFona(1); 
        restartFonaOffSW(FONA_OFF_TIME);
        delay(5000);      
        readTime();
        sendAlarmOffSms();                          
        // set timeout
        restartTimeoutSW();
        break;
      }
      case RESET: {
        soundAlarm(0);
        // power up FONA
        onoffFona(1);  
        restartFonaOffSW(FONA_OFF_TIME);;     
        delay(5000);
        readTime();
        sendAlarmResetSms();                          
        // set timeout
        restartTimeoutSW();
        break;
      }
     case  ARMED: {        
        soundAlarm(0);
        // power up FONA
        onoffFona(1);  
        restartFonaOffSW(FONA_OFF_TIME);     
        delay(5000);            
        readTime();
        sendAlarmArmedSms(); 
        // set timeout
        restartTimeoutSW();
        break;
      }
      case ALARM: {
        // Turn on ALARM indefinately
        soundAlarm(-1);
        // power up FONA, get time and send alarm text message
        onoffFona(1);
        restartFonaOffSW(FONA_OFF_TIME);
        delay(5000);
        // Send current time SMS message
        readTime();
        sendAlarmOnSms();      
        // Leave FONA on
        restartTimeoutSW();
        break;
      }
    } // end case alarmState
    
    if(serialUp) Serial.print(F("\nFONA>"));
    restartDoorOpenSW();
    
    alarmStateLast = alarmState;
  } // end if alarmState != alarmStateLast
}

/////////////////////////////////////////////////////////
// Process any SMS messages 
void processSmsMsg() {
return;
  if(lastOnoff==0 || SMSON==0) return;
//    if(serialUp) Serial.println(F("Processing SMS messages"));  

  if(fona.getNumSMS() == 0) return; // no messages to process
  char* smsMsg; 
  int8_t smsn;
  
  if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
    smsn = 0; // zero indexed
  } else {
    smsn = 1;  // 1 indexed
  }

  for ( ; smsn <= 10; smsn++) {
    smsMsg = readSmsMsg(smsn);
    if(smsMsg != 0) {
      // if the length is zero, its a special case where the index number is higher
      if (strlen(smsMsg) == 0) {
        if(serialUp) Serial.println(F("[empty slot]"));
      }
      else  {
        if(serialUp) Serial.println(F("Processing SMS message:"));  
        if(serialUp) {Serial.print(smsn);Serial.print(':');Serial.println(smsMsg);}
        // convert message to lower case to not be case sensitive
        for(int i=0;i<strlen(smsMsg);i++) smsMsg[i] = tolower(smsMsg[i]);
        if(strcmp(smsMsg, "alarm reset")==0) {
          if(serialUp) Serial.println(F("Reseting alarm"));
          alarmState = RESET;
        }
        else if(strcmp(smsMsg, "alarm off")==0) {
          if(serialUp) Serial.println(F("Turning alarm OFF"));
          alarmState = OFF;
        }
        else if(strcmp(smsMsg, "alarm on")==0) {
          if(serialUp) Serial.println(F("Turning alarm ON"));
          alarmState = ALARM;
        }
        else if(strcmp(smsMsg, "alarm arm")==0) {
          if(serialUp) Serial.println(F("Arming alarm"));          
          alarmState = ARMED;
        }
        else if(strcmp(smsMsg, "status")==0) {
          if(serialUp) Serial.println(F("Sending status"));
          readTime(); 
          sendStatusSms();
        }         
        else if(strncmp(smsMsg, "set time ", 9)==0) {
          if(serialUp) Serial.println(F("Setting time"));          
          if((smsMsg[9] == '"') && (smsMsg[9+21] == '"')) {
            setRTCTime(&smsMsg[9]);
          }
          else {
            if(serialUp) Serial.println(F("Bad time string"));
            if(serialUp) Serial.println(smsMsg);                  
          }
        }
        else {
          if(serialUp) Serial.println(F("Unrecognized sms cmd"));                
        }
        deleteSmsMsg(smsn);
        // Keep radio alive
        restartFonaOffSW(FONA_OFF_TIME); 
        delay(1000);
        break; 
      }
    }
    delay(100);   
    restartDoorOpenSW();      
  }
}


////////////////////////////////////////////////////////////////////////
// Serial interface debug
void serialInterfaceDebug(void) {
  // get rx char for command
  if(serialUp) if(Serial.available()){
    char command = Serial.read();
  
    switch (command) {

      case 'x': {
        alarmState = RESET;
        break;
      }
      
      case 'o': {
        alarmState = OFF;
        break;
      }
      
      case 'a': {
        alarmState = ALARM;
        break;
      }
      
      case 'q': {
        alarmState = ARMED;
        break;
      }
      
      case 'e': {
        if(serialUp) Serial.println(wakeupSW.elapsed());        
        break;
      }
      
      case 'C': {
        onoffFona(1);
        break;
      }
      
      case 'D': {  
        onoffFona(0);
        break;
      }
      
      case 's': {
        readTime();
        sendStatusSms();
        break;
      }
      
      case 'T': { // send time text message
        readTime();
        sendTimeSms();     
        break;
      }
      case 't': {
        // read the time
        fona.getTime(timeBuffer, 23);  // make sure replybuffer is at least 23 bytes!
        if(serialUp) Serial.print(F("Time = ")); if(serialUp) Serial.println(timeBuffer);
          break;
      }
      
      case 'N': {
        // read the number of SMS's!
        int8_t smsnum = fona.getNumSMS();
        if (smsnum < 0) {
          if(serialUp) Serial.println(F("Could not read # SMS"));
        } else {
          if(serialUp) Serial.print(smsnum);
          if(serialUp) Serial.println(F(" SMS's on SIM card!"));
        }
        break;
      }
            
      case 'R': {
        // read all SMS
        int8_t smsnum = fona.getNumSMS();
        uint16_t smslen;
        int8_t smsn;

        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          smsn = 0; // zero indexed
          smsnum--;
        } else {
          smsn = 1;  // 1 indexed
        }

        // read at most 10 slots
        for (uint8_t n=smsn ; n <= 10; n++) {
          char* smsMsg = readSmsMsg(n);

          if(smsMsg != 0) {
            // if the length is zero, its a special case where the index number is higher
            // so increase the max we'll look at!
            if (strlen(smsMsg) == 0) {
              if(serialUp) Serial.println(F("[empty slot]"));
              smsnum++;
              smsn++;
            }
            else if(serialUp) {
              Serial.print(F("***** SMS #")); Serial.print(n);
              Serial.print(" ("); Serial.print(strlen(smsMsg)); Serial.println(F(") bytes *****"));
              Serial.println(smsMsg);
              Serial.println(F("*****"));
              if(++smsn > smsnum) break;
            }
          }
        }
        break;
      }

      case 'r': { // read SMS message
        // read an SMS
        flushSerial();
        if(serialUp) Serial.print(F("Read #"));
        uint8_t smsn = readnumber();
        char* smsMsg = readSmsMsg(smsn);

        if(smsMsg !=0) {
          if(serialUp) Serial.print(F("***** SMS #")); Serial.print(smsn);
          if(serialUp) Serial.print(" ("); Serial.print(strlen(smsMsg)); Serial.println(F(") bytes *****"));
          if(serialUp) Serial.println(smsMsg);
          if(serialUp) Serial.println(F("*****"));
        }
        break;
        
      }

    case 'd': {
        // delete an SMS
        flushSerial();
        if(serialUp) Serial.print(F("Delete #"));
        uint8_t smsn = readnumber();

        if(serialUp) Serial.print(F("\n\rDeleting SMS #")); Serial.println(smsn);
        if (fona.deleteSMS(smsn)) {
          if(serialUp) Serial.println(F("OK!"));
        } else {
         if(serialUp)  Serial.println(F("Couldn't delete"));
        }
        break;
      }

      case 'X': { // Set Time 
//        char str[50];
        flushSerial();
        readline(str, 50, 0);
        Serial.println(str);
        setRTCTime(str);
        flushSerial();
        break;
      }
        
      case 'A': {
        // read the ADC
        uint16_t adc;
        if (! fona.getADCVoltage(&adc)) {
          Serial.println(F("Failed to read ADC"));
        } else {
          Serial.print(F("ADC = ")); Serial.print(adc); Serial.println(F(" mV"));
        }
        break;
      }

      case 'w': { // wake up by clearing time out
        waitWakeup = 0;
        break;
      }

      case 'v': { // measure voltage
        uint16_t v = measv();
        Serial.print(F("Voltage = "));
        Serial.println(v);
        break;
      }

      case 'p': { // Process a SMS message
        processSmsMsg();
        break;
      }
      case 10: { // LF
        if(serialUp) Serial.print(F("\nFONA>"));
        break;
      }

      default: if(serialUp) {
        Serial.println();
        Serial.println("x: alarm RESET");
        Serial.println("o: alarm OFF");
        Serial.println("a: alarm ALARM");
        Serial.println("q: alarm ARMED");
        Serial.println("e: elapsed time value");
        Serial.println("C: fona ON");
        Serial.println("D: fona OFF");
        Serial.println("s: send status SMS");
        Serial.println("T: send time text message");
        Serial.println("t: read the time");
        Serial.println("N: read the number of SMS's!");
        Serial.println("R: read all SMS");
        Serial.println("r: read SMS message");
        Serial.println("d: delete an SMS");
        Serial.println("X: set time");
        Serial.println("A: read ADC");
        Serial.println("w: wake up by clearing timeout");
        Serial.println("v: read Voltage");
        Serial.println("p: process a sms message");
        break;
      }
    } // end case command  

    restartDoorOpenSW();
  }
}


//////////////////////////////////////////////////////////////////////////////////////
// Turn FONA OFF after timer reaches Time
void fonaOffCheck(void){
  if(lastOnoff == 1) {
    if(fonaOffSW.elapsed() >= fonaOffTime) {
      onoffFona(0);
    }
  }
}
void restartFonaOffSW(int t){
  fonaOffSW.reset();
  fonaOffSW.start();
  fonaOffTime = t;
}


//////////////////////////////////////////////////////////////////////////////////////
// helper functions

// read an SMS message
// returns pointer or 0 if no message
char* readSmsMsg(uint8_t smsn) {
        if(serialUp) Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);

        // Retrieve SMS sender address/phone number.
        if (! fona.getSMSSender(smsn, replybuffer, 250)) {
          if(serialUp) Serial.println("Failed!");
          return 0;
        }
        if(serialUp) Serial.print(F("FROM: ")); Serial.println(replybuffer);
        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          if(serialUp) Serial.println("Failed!");
          return 0;
        }

        return replybuffer;
}
// Measure voltage on A0 pin
// Returns mV
int16_t measv() {
  float measurev;
  measurev = analogRead (MEASV_PIN);
  measurev = analogRead (MEASV_PIN); // discard first meas
  measurev *= (100.0+39.0)/39.0;   // 100K(V) - 39K(GND)
  measurev *= 3.3; // VREF
  measurev *= 1000.0/1024.0; // convert to mV
  measurev *= 1.0082; // manual calibration
  
  return measurev;
}

// sound alarm for time in msec
// time = -1 leaves alarm ON
void soundAlarm(int timeMs) {
  alarmVal = 1;
  digitalWrite(alarmPin, alarmVal);   // Alarm ON
  if(timeMs >=0 ) {
    delay(timeMs);
    alarmVal = 0;
    digitalWrite(alarmPin, alarmVal);   // Alarm OFF
  }
}

// Assert KEY pin on FONA to power up or power down
void keyAssert() {
//  ledVal = 1;
//  digitalWrite(ledPin, ledVal);   // sets the LED on
  keyVal = 0;
  digitalWrite(keyPin, keyVal);   // sets the KEY pin low
  delay(5000);
  keyVal = 1;
  digitalWrite(keyPin, keyVal);   // sets the KEY pin high
//  ledVal = 0;// waits for a second
//  digitalWrite(ledPin, ledVal);   // sets the LED off
  delay(1000);
}

// Turn FONA ON or OFF and verify
// onoff=1 turn on, onoff=0 turn OFF
int onoffFona(int onoff) {
  int i;
  int stat;
  int8_t rssi;
  if (onoff==1) {
    if(serialUp) Serial.println(F("Turning FONA ON"));
  } else {
    if(serialUp) Serial.println(F("Turning FONA OFF"));    
  }

  // Use last onoff state to enable toggle if new onoff is different
  // This can speed up ON OFF switches when last state is correct
  if(onoff != lastOnoff) keyAssert();
  lastOnoff = onoff;
  
  for(i=0; i<4; i++) {
    fonaSerial->begin(4800);
    stat = fona.begin(*fonaSerial);
    if( onoff==1) { // turn ON
      if (stat != 0) {
        if(SMSON==0) break;
        else {
          for (int j=0;j<10;j++) {
            rssi = get_rssi();
            if(serialUp) Serial.println(rssi);
            if(rssi > -100) break;
            else delay(5000);
          }
          if(rssi > -100) break;
        } 
      }
      else keyAssert();
    } else { // TURN OFF
      if (stat == 0 ) break;
      else keyAssert();
    }
  }

  if(serialUp) Serial.print(F("stat = "));
  if(serialUp) Serial.println(stat);
      
  if ((onoff==1) && (stat==0)) {
      if(serialUp) Serial.println(F("Couldn't turn ON FONA"));
      while (1);
  } else if ((onoff==0) && (stat !=0)) {
      if(serialUp) Serial.println(F("Couldn't turn OFF FONA"));
      while (1);
  }
}

// Initial connect to FONA module like in testcase
void connectFona() {
  if(serialUp) Serial.println(F("FONA basic test"));
  if(serialUp) Serial.println(F("Initializing....(May take many seconds)"));

  onoffFona(1); // turn ON
  restartFonaOffSW(FONA_OFF_TIME);
  
  type = fona.type();
  if(serialUp) Serial.println(F("FONA is OK"));
  if(serialUp) Serial.print(F("Found "));
  switch (type) {
    case FONA800L:
      if(serialUp) Serial.println(F("FONA 800L")); break;
    case FONA800H:
      if(serialUp) Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      if(serialUp) Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      if(serialUp) Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      if(serialUp) Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      if(serialUp) Serial.println(F("FONA 3G (European)")); break;
    default: 
      if(serialUp) Serial.println(F("???")); break;
  }
  
  // Print module IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    if(serialUp) Serial.print("Module IMEI: "); if(serialUp) Serial.println(imei);
  }
  // flush serial rx buff and FONA tx buff
  flushSerial();
  while (fona.available()) {
    if(serialUp) Serial.write(fona.read());
    else fona.read(); // discard
  }

}

// send string to FONA
void sendFonaStr(char *str) {
  int n = strlen(str);
  if(n>0 && n<100) {
    for(int i=0; i<n; i++) {
      fona.write(str[i]);
//      delay(100);
//      if (fona.available()) {
//        Serial.write(fona.read());
//
    }
    fona.write('\n');
    delay(5000);
  }
}

// read complete line string from FONA
// return > 0 when end of line char is detected
int fonaReadLine() {
  char c = 0;
  if(fonaStrIx>0){
    if(fonaRxBuffer[fonaStrIx-1] == 0x0D) {
      fonaStrIx = 0;
      fonaRxBuffer[fonaStrIx] = 0;
    }
  }
  if( fona.available()){
    c = fona.read();
//    if(serialUp) Serial.print(c);
//    if(serialUp) Serial.print(c);if(serialUp) Serial.print("#");if(serialUp) Serial.print(c, HEX);if(serialUp) Serial.print("#");
    fonaRxBuffer[fonaStrIx++] = c;
    fonaRxBuffer[fonaStrIx] = 0;
  }
  return (c == 0x0D);
}

// extract date and time from +CCLK string
// Uses global myT struct
// return -1 if invalid
// "16/07/09,21:51:22-20"
int timeStr2myT(char* tStr) {
//  tString = tStr;
//  String tString = "16/07/09,21:51:22-20";
 Serial.println(tString);
//  for(int i=0; i<23; i++) tString.setCharAt(i,tStr[i]);
  Serial.println("timeStr2myT");
 Serial.println(tStr);
 Serial.println(tString);
//  String lstr;
  if(tString.length() != 22) return -1;
  // Year
  lstr = tString.substring(1,3);
  myT.yr = lstr.toInt();
  // Month
  lstr = tString.substring(4,6);
  myT.mo = lstr.toInt();
  // Day
  lstr = tString.substring(7,9);
  myT.day = lstr.toInt();
  // Hour
  lstr = tString.substring(10,12);
  myT.hr = lstr.toInt();
  // Minute
  lstr = tString.substring(13,15);
  myT.min = lstr.toInt();
  // Second
  lstr = tString.substring(16,18);
  myT.sec = lstr.toInt();
  // Timezone
  lstr = tString.substring(19,21);
  myT.tz = lstr.toInt();

  return 0;
}

// Read RTC time into global timeBuffer
void readTime(){
    // read the time from FONA RTC
    fona.getTime(timeBuffer, 23);  // make sure replybuffer is at least 23 bytes!
    if(serialUp) Serial.print(F("Time = ")); if(serialUp) Serial.println(timeBuffer);
}

// reset the timeout stop watch
void restartTimeoutSW() {
  if(serialUp) Serial.println(F("Resetting restartTimeoutSW"));  
  timeoutSW.reset();
  timeoutSW.start();  
}

// reset the door open stop watch
void restartDoorOpenSW() {
  doorOpenSW.reset();
  doorOpenSW.start();  
}


// Read RTC and start wakeup timer
// return number of secs to wait
long rdTimeRestartWakeupSWWakeupSW(long interval){
    // readRTC time
    readTime();
    
    wakeupSW.reset();
    wakeupSW.start();
     
    timeStr2myT(timeBuffer);
    if(serialUp) Serial.print(myT.yr);if(serialUp) Serial.print("/");
    if(serialUp) Serial.print(myT.mo);if(serialUp) Serial.print("/");
    if(serialUp) Serial.print(myT.day);if(serialUp) Serial.print(",");
    if(serialUp) Serial.print(myT.hr);if(serialUp) Serial.print(":");
    if(serialUp) Serial.print(myT.min);if(serialUp) Serial.print(":");
    if(serialUp) Serial.print(myT.sec);if(serialUp) Serial.print("-");
    if(serialUp) Serial.println(myT.tz);

    //calc # secs till next interval
    long secs = 0;
    if(interval>0) {
      secs = 1L*myT.sec + 60L*myT.min + 60L*60*myT.hr;
      if(serialUp) Serial.println(secs);
      secs = secs % interval;
      if(serialUp) Serial.println(secs);
      secs = interval - secs;
      if(serialUp) Serial.print(F(" num secs until next event = "));
      if(serialUp) Serial.println(secs);
    }
    
    return secs;
 
}


// Send SMS with time
void sendTimeSms() {

    if(serialUp) Serial.print(F("sendto  ")); if(serialUp) Serial.println(sendto);
    if(serialUp) Serial.print(F("timeBuffer ")); if(serialUp) Serial.println(timeBuffer);

    if(SMSON) if(SMSON) if (!fona.sendSMS(sendto, timeBuffer)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("Sent!"));
    }

}

// send SMS with time appended to string
void sendSmsPlusTime(const char* str) {
//    char msg[256] = "";
    msg[0]=0; // ""
    if(serialUp) Serial.print(F("sendto  ")); if(serialUp) Serial.println(sendto);
    if(serialUp) Serial.print(F("timeBuffer ")); if(serialUp) Serial.println(timeBuffer);

    strcat(msg, str);
    strcat(msg, " at ");
    strcat(msg, timeBuffer);
    if(serialUp) Serial.println(msg);
    
    if(SMSON) if (!fona.sendSMS(sendto, msg)) {
      if(serialUp) Serial.println(F("Failed"));
      //try again
      delay(5000);
      if(SMSON) if (!fona.sendSMS(sendto, msg)) {
        if(serialUp) Serial.println(F("Failed")); 
      }   
    } else {
      if(serialUp) Serial.println(F("Sent!"));
    }  
}


void sendAlarmOnSms() {
  sendSmsPlusTime("ALARM ON");
}

void sendAlarmOffSms() {
  sendSmsPlusTime("ALARM OFF");
}

void sendAlarmResetSms() {
  sendSmsPlusTime("ALARM RESET");
}

void sendAlarmArmedSms() {
  sendSmsPlusTime("ALARM ARMED");
}

int8_t get_rssi(){
  int8_t rssi;
  rssi = fona.getRSSI();
  if      (rssi ==  0) rssi = -115;
  else if (rssi ==  1) rssi = -111;
  else if (rssi == 31) rssi = -52;
  else if ((rssi >= 2) && (rssi <= 30)) {
    rssi = map(rssi, 2, 30, -110, -54);
  }
  return rssi;
}

// Send SMS with current status
void sendStatusSms () {
//  char str[200];
  uint16_t vbat;
  int8_t rssi;
//  char sstr[50];
  
  readTime();
  
  if(serialUp) Serial.println("sendStatusSms");
          
  // alarm status
  strcpy(str, alarmStateStr[alarmState]);
  
  // Litium Ion cell battery
  fona.getBattVoltage(&vbat);
  sprintf(sstr, ", li %d mV", vbat);
  strcat(str, sstr);

  // 12V battery
  vbat = measv();
  sprintf(sstr, ", 12v %d mV", vbat);
  strcat(str, sstr);
  
  // IEM#
//  strcat(str, ", IEM# 1234567899");
  
  // signal level
  rssi = get_rssi();
  sprintf(sstr, ", rssi %d dBm", rssi);
  strcat(str, sstr);

  // time
  if(serialUp) Serial.println(str);        
  sendSmsPlusTime(str);
  
}

// delete SMS 
void deleteSmsMsg(uint8_t smsn) {

      if(serialUp) {Serial.print(F("\n\rDeleting SMS #")); Serial.println(smsn);}
      if (fona.deleteSMS(smsn)) {
        if(serialUp) Serial.println(F("OK!"));
      } else {
        if(serialUp) Serial.println(F("Couldn't delete"));
      }

}

/*
// return SMS msg #
uint8_t getSmsMsg(void) {

// get max sms #
    int8_t smsnum = fona.getNumSMS();
    uint16_t smslen;
    int8_t smsn = 1;

// search and return last sms msg
    smsMsg[0] = 0;

    for ( ; smsn <= smsnum; smsn++) {
        if (fona.readSMS(smsn, smsMsg, 250, &smslen)) break;
        // if no message and the length is zero, 
        // its a special case where the index number is higher
        // so increase the max we'll look at!
        if (smslen == 0) {
          if(serialUp) Serial.println(F("[empty slot]"));
          smsnum++;
          continue;
        }

    } // end smsn loop
  return smsn;
}
*/

// Set the time in the RTC in FONA
void setRTCTime(char *timeStr){
//    char str[50] = "at+cclk=";
    str[0]=0; //""
    strcat(str, "at+cclk=");
    strcat(str, timeStr);
    if(serialUp) Serial.println(str);
    sendFonaStr(str);  
}









void flushSerial() {
  if(serialUp) {
    while(Serial.available()) {
      if(serialUp) Serial.read();
      else break;
    }
  }
}

char readBlocking() {
  if(serialUp){
  while (!Serial.available());
  return Serial.read();
  }
}

uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //if(serialUp) Serial.print(c);
  }
  if(serialUp) Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    if(serialUp) Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //if(serialUp) Serial.println(F("SPACE"));
      break;
    }

    while (Serial.available() && serialUp) {
      Serial.print(">");
      char c =  Serial.read();
      Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //if(serialUp) Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}

