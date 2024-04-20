#include "RP2040_PWM.h"
#include "pio_encoder.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

SemaphoreHandle_t xSemaphore;

/* Task to be created. */
void vTaskCode( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. */
//    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1; // msec period
    unsigned long currTimeUs;
     // Initialise the xLastWakeTime variable with the current time.
     xLastWakeTime = xTaskGetTickCount();

    for( ;; )
    {
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE ) {
          currTimeUs = micros();
          xSemaphoreGive( xSemaphore );
        }

        Serial.print(currTimeUs);
        Serial.println(" Task");

    }
}

//#define STACK_SIZE 256
//#define tskIDLE_PRIORITY 1

/* Function that creates a task. */
void vOtherFunction( void )
{
BaseType_t xReturned;
xTaskHandle xHandle = NULL;

    /* Create a mutex type semaphore. */
    xSemaphore = xSemaphoreCreateMutex();

    /* Create the task, storing the handle. */
    xReturned = xTaskCreate(
                    vTaskCode,       /* Function that implements the task. */
                    "NAME",          /* Text name for the task. */
                    256,      /* Stack size in words, not bytes. */
                    NULL,
                    //( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    NULL );      /* Used to pass out the created task's handle. */

//    if( xReturned == pdPASS )
//    {
//        /* The task was created.  Use the task's handle to delete the task. */
//        vTaskDelete( xHandle );
//    }

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(155200);
  Serial.println("Setup task");
  vOtherFunction();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}
