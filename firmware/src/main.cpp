// Coil-Winder

#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

#define PIN_STEPPER_DRIVER_STEP 13
#define PIN_STEPPER_DRIVER_DIR 4

#define PIN_MOTOR_PWM 6
#define PIN_MOTOR_IN1 8
#define PIN_MOTOR_IN2 7

#define PIN_BUTTON_START A7
#define PIN_BUTTON_PAUSE A6
#define PIN_BUTTON_STOP A3

#define PIN_HALL_EFFECT_SENSOR 2
#define PIN_SWITCH_STEPPER_INDEX 3

#define PIN_BUZZER 11

#define BUTTON_ACTIVE 0

LiquidCrystal_I2C lcd(0x27, 20, 2);

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPER_DRIVER_STEP, PIN_STEPPER_DRIVER_DIR);

enum State
{
  Startup,
  Ready,
  Winding,
  Pause,
  Stop
};

State state = State::Startup;

volatile int rotationCount;


int windingCount;




bool HomeStepper()
{
  lcd.clear();
  lcd.home();
  lcd.print("Homing stepper");

  stepper.setCurrentPosition(0);
  stepper.moveTo(-2400);

  bool indexSuccessFlag = false;

  while (stepper.distanceToGo() != 0)
  {
    if (digitalRead(PIN_SWITCH_STEPPER_INDEX) == BUTTON_ACTIVE)
    {
      indexSuccessFlag = true;
      break;
    }
    stepper.run();
  }

  return indexSuccessFlag;
}

void CheckControlButtons()
{

  if (analogRead(PIN_BUTTON_START) == 0)
  {
  }
  else if (analogRead(PIN_BUTTON_PAUSE) == 0)
  {
  }
  else if (analogRead(PIN_BUTTON_STOP) == 0)
  {
  }
}

void UpdateLCD()
{
  //lcd.setCursor(0, 1);
  //lcd.print(rotationCount);

  lcd.home();

  enum MenuItem
  {
    WindCount,
    WindSpeed,
    WindDirection,
    IndexSpeed,
    IndexStart,
    IndexStop,

  };

  int menuSelect = int(MenuItem::WindCount);

  if (1) // IN MENU
  {
    if (menuSelect == int(MenuItem::WindCount))
    {
      lcd.print("Winding Count:");
      lcd.setCursor(0, 1);
      lcd.print(windingCount); 

    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
    }
    else if (menuSelect == int(MenuItem::WindDirection))
    {
    }
    else if (menuSelect == int(MenuItem::IndexSpeed))
    {
    }
    else if (menuSelect == int(MenuItem::IndexStart))
    {
    }
    else if (menuSelect == int(MenuItem::IndexStop))
    {
    }
  }
}

void CountRotation()
{
  rotationCount++;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Coil-Winder starting up...");

  pinMode(PIN_BUTTON_START, INPUT);
  pinMode(PIN_BUTTON_PAUSE, INPUT);
  pinMode(PIN_BUTTON_STOP, INPUT);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_HALL_EFFECT_SENSOR, INPUT_PULLUP);
  pinMode(PIN_SWITCH_STEPPER_INDEX, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.printstr("Coil Winder");

  rotationCount = 0;

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(4000);

  //attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_SENSOR), CountRotation, CHANGE);

/*
  if (HomeStepper() == false)
  {
    lcd.clear();
    lcd.home();
    lcd.print("Stepper failed");
    lcd.setCursor(0, 1);
    lcd.print("to home!");

    while (true)
      ;
  }
  */
}

void loop()
{
  CheckControlButtons();

  UpdateLCD();

  //digitalWrite(PIN_BUZZER, !digitalRead(PIN_HALL_EFFECT_SENSOR));
  digitalWrite(PIN_BUZZER, !digitalRead(PIN_SWITCH_STEPPER_INDEX));
}
