/*
  Coil-Winder



  Notes:

  Indexer lead screw is 2mm travel per turn (0.0787402 inches).
  400

  0.0787402 / 400 = 0.0001968505 per step


*/

#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <JC_Button.h> // https://github.com/JChristensen/JC_Button
#include <TimerOne.h>  // https://playground.arduino.cc/Code/Timer1/

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
  IndexTop,
  IndexBottom,
  Count
};

// Menu configurable variables.
int windingCount;
int windingSpeed;
int windingDirection;
int indexSpeed;
double indexTop = 1.0;
double indexBottom = 0.0;

// Running variables.
double indexPosition = 0.0;

int menuSelect = int(MenuItem::WindCount);

const int buttonToneLengthMs = 20;

enum class Tone
{
  StepperHomeSuccess,
  StepperHomeFailed,
  MenuPrevious,
  MenuNext,
  OptionDecrement,
  OptionIncrement
};

// Indexer lead screw is 2mm travel per turn (0.0787402 inches).
// 0.0787402 / 400 = 0.0001968505 per step
const float stepperTravelPerSteps = 0.0001968505;
const float indexUserIncrement = 0.100;
const float indexMaxPosition = 1.5;

/////////////////////////////////////////////////////////////////////////////

void PlaySound(Tone t)
{
  if (t == Tone::StepperHomeSuccess)
  {
    tone(PIN_BUZZER, 440, 125);
    delay(125);
    tone(PIN_BUZZER, 540, 125);
    delay(125);
    tone(PIN_BUZZER, 640, 125);
  }
  else if (t == Tone::StepperHomeFailed)
  {
    tone(PIN_BUZZER, 440, 125);
    delay(125);
    tone(PIN_BUZZER, 340, 125);
    delay(125);
    tone(PIN_BUZZER, 240, 125);
  }
}

void SetIndexPosition(float pos)
{
  stepper.moveTo(pos / stepperTravelPerSteps);
}

void CheckControlButtons()
{

  static unsigned long startHeldCount;
  static unsigned long pauseHeldCount;
  static unsigned long stopHeldCount;

  if (analogRead(PIN_BUTTON_START) == 0)
  {
    if (millis() - startHeldCount > 100)
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
    if (millis() - pauseHeldCount > 100)
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
    if (millis() - stopHeldCount > 100)
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

    if (menuSelect == int(MenuItem::IndexBottom))
    {
      indexTop -= indexUserIncrement;
      if (indexTop < 0)
        indexTop = 0;

      SetIndexPosition(indexTop);
    }
  }

  if (buttonOptionUp.wasPressed())
  {

    if (menuSelect == int(MenuItem::IndexBottom))
    {
      indexTop += indexUserIncrement;
      if (indexTop > indexMaxPosition)
      {
        indexTop = indexMaxPosition;
      }
      SetIndexPosition(indexTop);
    }
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

  if (state == State::Standby)
  {
    if (menuSelect == int(MenuItem::Home))
    {
      PrintLine(0, "Coil Winder");
      PrintLine(1, "");
    }
    else if (menuSelect == int(MenuItem::WindCount))
    {
      PrintLine(0, "Winding Count:");
      sprintf(buf, "%u Rotations", windingCount);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
      PrintLine(0, "Winding Speed:");
      sprintf(buf, "%u RPM", windingSpeed);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::WindDirection))
    {
      PrintLine(0, "Winding Direction:");
      if (windingDirection == 0)
        PrintLine(1, buf);
      else if (windingDirection == 1)
        PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::IndexSpeed))
    {
      PrintLine(0, "Index Speed:");
      sprintf(buf, "%u RPM", indexSpeed);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::IndexTop))
    {
      PrintLine(0, "Index Top:");
      sprintf(buf, dtostrf(indexTop, 5, 3, "%5.3f"));
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::IndexBottom))
    {
      PrintLine(0, "Index Bottom:");
      sprintf(buf, dtostrf(indexBottom, 5, 3, "%5.3f"));
      PrintLine(1, buf);
    }
  }
  else if (state == State::Winding)
  {
    PrintLine(0, "Winding...");
    //sprintf(buf, "%u of %u", rotationCount, windingCount);
    //PrintLine(1, buf);

    sprintf(buf, "%u of %u", stepper.currentPosition(), stepper.targetPosition());
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

    int posBottom = indexBottom / stepperTravelPerSteps;
    int posTop = indexTop / stepperTravelPerSteps;
    if (stepper.currentPosition() == posTop)
    {
      stepper.moveTo(posBottom);
    }
    else if (stepper.currentPosition() == posBottom)
    {
      stepper.moveTo(posTop);
    }
    else if (!stepper.isRunning())
    {
      stepper.moveTo(posTop);
    }
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

bool HomeStepper()
{
  PrintLine(0, "Homing stepper");

  stepper.setCurrentPosition(0);
  stepper.moveTo(-(indexMaxPosition / stepperTravelPerSteps));

  bool indexSuccessFlag = false;

  while (stepper.distanceToGo() != 0)
  {
    if (digitalRead(PIN_SWITCH_STEPPER_INDEX) == BUTTON_ACTIVE)
    {
      stepper.setCurrentPosition(0);
      stepper.moveTo(200);
      stepper.runToPosition();
      stepper.setCurrentPosition(0);
      indexPosition = 0;
      indexSuccessFlag = true;
      break;
    }
    stepper.run();
  }

  return indexSuccessFlag;
}

// Interrupt callback.
void CountRotation()
{
  rotationCount++;
}

// Timer interupt callback
void TickStepperCallback()
{
  stepper.run();
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

  rotationCount = 0;

  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(8000);

  buttonMenuPrevious.begin();
  buttonMenuNext.begin();
  buttonOptionDown.begin();
  buttonOptionUp.begin();

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_SENSOR), CountRotation, FALLING);

  if (HomeStepper() == false)
  {
    PrintLine(0, "Stepper failed");
    PrintLine(1, "to home!");
    PlaySound(Tone::StepperHomeFailed);
    while (true)
      ;
  }
  else
  {
    PlaySound(Tone::StepperHomeSuccess);
  }

  Timer1.initialize(1000);
  Timer1.attachInterrupt(TickStepperCallback);

  state = State::Standby;
  menuSelect = int(MenuItem::IndexBottom);
}

void loop()
{
  StateController();

  CheckControlButtons();

  CheckMenuButtons();

  UpdateLCD(); // ~52ms processing time.


}
