/****************************************************************************************************************************
  ISR_Timers_Array_Simple.ino
  RPi_Pico_ISR_Timer-Impl.h
  For RP2040-based boards such as RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040.
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/RPI_PICO_TimerInterrupt
  Licensed under MIT license

  The RPI_PICO system timer peripheral provides a global microsecond timebase for the system, and generates
  interrupts based on this timebase. It supports the following features:
    • A single 64-bit counter, incrementing once per microsecond
    • This counter can be read from a pair of latching registers, for race-free reads over a 32-bit bus.
    • Four alarms: match on the lower 32 bits of counter, IRQ on match: TIMER_IRQ_0-TIMER_IRQ_3

  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one RPI_PICO timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.
*****************************************************************************************************************************/


// Mike Williamson - modified to only use Interrupr and acheived 10us interrupt interval



// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         1
#define _TIMERINTERRUPT_LOGLEVEL_     4

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer1(1);

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
//#include "RPi_Pico_ISR_Timer.h"
//RPI_PICO_ISR_Timer ISR_timer;


// You have to use longer time here if having problem because Arduino AVR clock is low, 16MHz => lower accuracy.
// Tested OK with 1ms when not much load => higher accuracy.
#define TIMER_INTERVAL_MS 1            

volatile uint32_t startMillis = 0;

volatile uint32_t deltaMillis2s = 0;
volatile uint32_t deltaMillis5s = 0;

volatile uint32_t previousMillis2s = 0;
volatile uint32_t previousMillis5s = 0;

volatile uint32_t isrCount = 0;

bool TimerHandler(struct repeating_timer *t)
{
  (void) t;
  
  static bool toggle  = false;
  static int timeRun  = 0;

  isrCount ++;
//  ISR_timer.run();

  if(isrCount%100000 == 0) doingSomething2s();

  return true;
}

unsigned long isrCountDelta2s = 0;
unsigned long isrCountPrevious2s = 0;

void doingSomething2s()
{
  unsigned long currentMillis  = micros();

  deltaMillis2s    = currentMillis - previousMillis2s;
  previousMillis2s = currentMillis;

  isrCountDelta2s = isrCount - isrCountPrevious2s;
  isrCountPrevious2s = isrCount;
}

void doingSomething5s()
{
  unsigned long currentMillis  = millis();

  deltaMillis5s    = currentMillis - previousMillis5s;
  previousMillis5s = currentMillis;
}


void setup()
{

  Serial.begin(115200);
  while (!Serial);

  Serial.print(F("\nStarting ISR_Timers_Array_Simple on ")); Serial.println(BOARD_NAME);
  Serial.println(RPI_PICO_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));

//  if (ITimer1.attachInterruptInterval(TIMER_INTERVAL_MS * 1000, TimerHandler))
  if (ITimer1.attachInterruptInterval(10L, TimerHandler))
  {
    Serial.print(F("Starting ITimer1 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));

  previousMillis5s = previousMillis2s = micros();  

//  ISR_timer.setInterval(1000L, doingSomething2s);
//  ISR_timer.setInterval(5000L, doingSomething5s);

}


void loop()
{
  // This unadvised blocking task is used to demonstrate the blocking effects onto the execution and accuracy to Software timer
  // You see the time elapse of ISR_Timer still accurate, whereas very unaccurate for Software Timer
  // The time elapse for 2000ms software timer now becomes 3000ms (BLOCKING_TIME_MS)
  // While that of ISR_Timer is still prefect.
  delay(100);

  Serial.print(isrCountDelta2s); Serial.print(": ");
  Serial.print(previousMillis2s);Serial.print(">");Serial.println(deltaMillis2s);
}
