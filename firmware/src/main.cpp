// Coil-Winder

#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <JC_Button.h> // https://github.com/JChristensen/JC_Button

#define PIN_STEPPER_DRIVER_STEP 13
#define PIN_STEPPER_DRIVER_DIR 4

#define PIN_MOTOR_PWM 6
#define PIN_MOTOR_IN1 8
#define PIN_MOTOR_IN2 7

#define PIN_BUTTON_START A7
#define PIN_BUTTON_PAUSE A6
#define PIN_BUTTON_STOP A3

#define PIN_BUTTON_MENU_PREVIOUS 5
#define PIN_BUTTON_MENU_NEXT 9
#define PIN_BUTTON_OPTION_DOWN 10
#define PIN_BUTTON_OPTION_UP 12

#define PIN_HALL_EFFECT_SENSOR 2
#define PIN_SWITCH_STEPPER_INDEX 3

#define PIN_BUZZER 11

#define BUTTON_ACTIVE 0

Button buttonMenuPrevious(PIN_BUTTON_MENU_PREVIOUS);
Button buttonMenuNext(PIN_BUTTON_MENU_NEXT);
Button buttonOptionDown(PIN_BUTTON_OPTION_DOWN);
Button buttonOptionUp(PIN_BUTTON_OPTION_UP);

LiquidCrystal_I2C lcd(0x27, 20, 2);

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPER_DRIVER_STEP, PIN_STEPPER_DRIVER_DIR);

enum class State
{
  Startup,
  Standby,
  Winding,
  Pause,
  Stop
};

State state = State::Startup;

volatile int rotationCount;

enum class MenuItem
{
  Home,
  WindCount,
  WindSpeed,
  WindDirection,
  IndexSpeed,
  IndexStart,
  IndexStop,
  Count
};

int windingCount;
int windingSpeed;
int windingDirection;
int indexSpeed;
int indexStart;
int indexStop;

int menuSelect = int(MenuItem::WindCount);

const int buttonToneLengthMs = 20;

/////////////////////////////////////////////////////////////////////////////

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

  static unsigned long startHeldCount;
  static unsigned long pauseHeldCount;
  static unsigned long stopHeldCount;

  if (analogRead(PIN_BUTTON_START) == 0)
  {
    if (millis() - startHeldCount > 1000)
    {
      state = State::Winding;
    }
  }
  else
  {
    startHeldCount = millis();
  }

  if (analogRead(PIN_BUTTON_PAUSE) == 0)
  {
    if (millis() - pauseHeldCount > 1000)
    {
      state = State::Pause;
    }
  }
  else
  {
    pauseHeldCount = millis();
  }

  if (analogRead(PIN_BUTTON_STOP) == 0)
  {
    if (millis() - stopHeldCount > 1000)
    {
      state = State::Stop;
    }
  }
  else
  {
    stopHeldCount = millis();
  }
}

void CheckMenuButtons()
{

  buttonMenuPrevious.read();
  buttonMenuNext.read();
  buttonOptionDown.read();
  buttonOptionUp.read();

  // TODO: check system state, dont allow button presses while winding.

  if (buttonMenuPrevious.wasPressed())
  {
    if (menuSelect == 0)
    {
      menuSelect = int(MenuItem::Count) - 1;
    }
    else
    {
      menuSelect--;
    }

    tone(PIN_BUZZER, 440, buttonToneLengthMs);
  }

  if (buttonMenuNext.wasPressed())
  {
    if (menuSelect == int(MenuItem::Count) - 1)
    {
      menuSelect = 0;
    }
    else
    {
      menuSelect++;
    }
    tone(PIN_BUZZER, 880, buttonToneLengthMs);
  }

  if (buttonOptionDown.wasPressed())
  {
  }

  if (buttonOptionUp.wasPressed())
  {
  }
}

// Print string to LCD with trailing spaces to fill/clear the row.
void PrintLine(int line, const char str[])
{
  if (line == 0)
  {
    lcd.setCursor(0, 0);
  }
  else
  {
    lcd.setCursor(0, 1);
  }

  char buf[32];
  sprintf(buf, "%-16s", str);
  lcd.printstr(buf);
}

void UpdateLCD()
{
  char buf[20];

  lcd.home();

  if (state == State::Standby)
  {
    if (menuSelect == int(MenuItem::Home))
    {
      lcd.printstr("Coil Winder");
      lcd.setCursor(0, 1);
    }
    else if (menuSelect == int(MenuItem::WindCount))
    {
      lcd.printstr("Winding Count:");
      lcd.setCursor(0, 1);
      sprintf(buf, "%u Rotations", windingCount);
      lcd.printstr(buf);
    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
      lcd.printstr("Winding Speed:");
      lcd.setCursor(0, 1);
      sprintf(buf, "%u RPM", windingSpeed);
      lcd.printstr(buf);
    }
    else if (menuSelect == int(MenuItem::WindDirection))
    {
      lcd.printstr("Winding Direction:");
      lcd.setCursor(0, 1);
      if (windingDirection == 0)
        lcd.printstr("CC");
      else if (windingDirection == 1)
        lcd.printstr("CCW");
    }
    else if (menuSelect == int(MenuItem::IndexSpeed))
    {
      lcd.printstr("Index Speed:");
      lcd.setCursor(0, 1);
      sprintf(buf, "%u RPM", indexSpeed);
      lcd.printstr(buf);
    }
    else if (menuSelect == int(MenuItem::IndexStart))
    {
      lcd.printstr("Index Start:");
      lcd.setCursor(0, 1);
      sprintf(buf, "%u In/bottom", indexStart);
      lcd.printstr(buf);
    }
    else if (menuSelect == int(MenuItem::IndexStop))
    {
      lcd.printstr("Index Stop:");
      lcd.setCursor(0, 1);
      sprintf(buf, "%u Inch/bottom", indexStop);
      lcd.printstr(buf);
    }
  }
  else if (state == State::Winding)
  {
    PrintLine(0, "Winding...");
    sprintf(buf, "%u of %u", rotationCount, windingCount);
    PrintLine(1, buf);
  }
  else if (state == State::Pause)
  {
    PrintLine(0, "Winding paused.");
    sprintf(buf, "%u of %u", rotationCount, windingCount);
    PrintLine(1, buf);
  }
  else if (state == State::Stop)
  {
    PrintLine(0, "Winding stoped.");
    sprintf(buf, "%u of %u", rotationCount, windingCount);
    PrintLine(1, buf);
  }
}

void StateController()
{
  static State previousState = State::Pause;

  if (previousState != state)
  {
    previousState = state;
  }

  if (state == State::Startup)
  {
  }
  else if (state == State::Standby)
  {
    // Prepare variables and hardware for winding.

    rotationCount = 0;

    if (windingDirection == 0)
    {
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
    }
    else if (windingDirection == 1)
    {
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
    }
  }
  else if (state == State::Winding)
  {
    analogWrite(PIN_MOTOR_PWM, 255);
  }
  else if (state == State::Pause)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
  }
  else if (state == State::Stop)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
  }
}

// Interrupt callback.
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

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_PWM, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.printstr("Coil Winder");

  rotationCount = 0;

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(4000);

  buttonMenuPrevious.begin();
  buttonMenuNext.begin();
  buttonOptionDown.begin();
  buttonOptionUp.begin();

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_SENSOR), CountRotation, FALLING);

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

  state = State::Standby;
}

void loop()
{
  StateController();

  CheckControlButtons();

  CheckMenuButtons();

  UpdateLCD();
}
