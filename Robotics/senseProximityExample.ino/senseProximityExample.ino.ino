#include <Arduino_APDS9960.h>

int ledState = LOW;

unsigned long previousMillis = 0;

const long intervalLong = 1000;
const long intervalMed = 500;
const long intervalShort = 100;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor!");
  }

  // set the LEDs pins as outputs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // turn all the LEDs off
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void loop() {
  unsigned long currentMillis = millis();

  // check if a proximity reading is available
  if (APDS.proximityAvailable()) {
    // read the proximity
    // - 0   => close
    // - 255 => far
    // - -1  => error
    int proximity = APDS.readProximity();

    if (proximity > 150) {
      if (currentMillis - previousMillis >= intervalLong) {
        previousMillis = currentMillis;

        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW) {
          ledState = HIGH;
        } else {
          ledState = LOW;
        }

        // set the green LED with the ledState of the variable and turn off the rest
        digitalWrite(LEDG, ledState);
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDB, HIGH);
      }
    }

    else if(proximity > 50 && proximity <= 150){
      if (currentMillis - previousMillis >= intervalMed) {
        previousMillis = currentMillis;

        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW) {
          ledState = HIGH;
        } else {
          ledState = LOW;
        }

        // set the blue LED with the ledState of the variable and turn off the rest
        digitalWrite(LEDB, ledState);
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
      }
    }

    else {
      if (currentMillis - previousMillis >= intervalShort) {
        previousMillis = currentMillis;

        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW) {
          ledState = HIGH;
        } else {
          ledState = LOW;
        }

        // set the blue LED with the ledState of the variable and turn off the rest
        digitalWrite(LEDR, ledState);
        digitalWrite(LEDB, HIGH);
        digitalWrite(LEDG, HIGH);
      }
    }

    // print value to the Serial Monitor
    Serial.println(proximity);
  }
}