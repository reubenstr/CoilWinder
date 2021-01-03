// Coil-Winder
//
// A basic coil winder with a stepper spinning as fast as possible.
//
// No acceleration
// No RPM measurement
// No counts


#include <Arduino.h>

#define PIN_LED_ONBOARD 13
#define PIN_STEPPER_DRIVER_STEP 4
#define PIN_STEPPER_DRIVER_DIR 5
#define PIN_BUTTON_START 2
#define PIN_BUTTON_STOP 3

#define BUTTON_PRESSED_STATE 0

bool enableStepper;

void setup()
{
  Serial.begin(115200);
  Serial.println("Coil-Winder starting up...");

  pinMode(PIN_BUTTON_START, INPUT_PULLUP);
  pinMode(PIN_BUTTON_STOP, INPUT_PULLUP);

  pinMode(PIN_STEPPER_DRIVER_STEP, OUTPUT);
  pinMode(PIN_STEPPER_DRIVER_DIR, OUTPUT);

  enableStepper = false;
}

void loop()
{
  if (digitalRead(PIN_BUTTON_START) == BUTTON_PRESSED_STATE)
  {
    enableStepper = true;
  }

  if (digitalRead(PIN_BUTTON_STOP) == BUTTON_PRESSED_STATE)
  {
    enableStepper = false;
  }

 if (enableStepper)
  {
    delayMicroseconds(750);
    digitalWrite(PIN_STEPPER_DRIVER_STEP, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_STEPPER_DRIVER_STEP, LOW);
  }
}
