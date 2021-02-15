/*
  Coil-Winder



  Notes:

  Indexer lead screw is 2mm travel per turn (0.0787402 inches).
  400

  0.0787402 / 400 = 0.0001968505 per step


  TODO:

  No motor rpm detected timeout.

*/

#include <Arduino.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h> // https://github.com/waspinator/AccelStepper
#include <JC_Button.h>    // https://github.com/JChristensen/JC_Button
#include <TimerOne.h>     // https://playground.arduino.cc/Code/Timer1/
#include <PID_v1.h>       // https://github.com/br3ttb/Arduino-PID-Library
#include <RunningMedian.h>

#define PIN_STEPPER_DRIVER_STEP 13
#define PIN_STEPPER_DRIVER_DIR 4

#define PIN_MOTOR_PWM 6
#define PIN_MOTOR_IN1 8
#define PIN_MOTOR_IN2 7

#define PIN_BUTTON_START A7
#define PIN_BUTTON_PAUSE A6
#define PIN_BUTTON_STOP A3

#define PIN_BUTTON_MENU_PREVIOUS A2
#define PIN_BUTTON_MENU_NEXT A1
#define PIN_BUTTON_OPTION_DOWN A0
#define PIN_BUTTON_OPTION_UP 12

#define PIN_HALL_EFFECT_SENSOR 2
#define PIN_SWITCH_STEPPER_INDEX 3

#define PIN_LED_START 11
#define PIN_LED_PAUSE 10
#define PIN_LED_STOP 9

#define PIN_BUZZER 5

#define BUTTON_ACTIVE 0

Button buttonMenuPrevious(PIN_BUTTON_MENU_PREVIOUS);
Button buttonMenuNext(PIN_BUTTON_MENU_NEXT);
Button buttonOptionDecrement(PIN_BUTTON_OPTION_DOWN);
Button buttonOptionIncrement(PIN_BUTTON_OPTION_UP);

LiquidCrystal_I2C lcd(0x27, 20, 2);
const int toggleDisplayDelay = 2000;

AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPER_DRIVER_STEP, PIN_STEPPER_DRIVER_DIR);

enum class State
{
  Standby,
  Winding,
  Pause,
  Stop
};

State state = State::Standby;

enum class MenuItem
{
  Home,
  WindCount,
  WindSpeed,
  WindDirection,
  IndexSpeed,
  IndexTop,
  IndexBottom,
  PlaySounds,
  Count
};

// Menu configurable variables.
int windingCount = 1000;
int windingSpeedRpm = 1000;
int windingDirection;
int indexSpeed;
double indexTop = 0.25;
double indexBottom = 0.0;
bool playSounds = true;

const int windingSpeedRpmMin = 300;
const int windingSpeedRpmMax = 1000;
const int indexSpeedRpmMin = 10;
const int indexSpeedRpmMax = 100;

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
  OptionIncrement,
  RpmReached,
  StoppedWindingSuccess,
  StoppedWinding
};

// Indexer lead screw is 2mm travel per turn (0.0787402 inches).
// 0.0787402 / 400 = 0.0001968505 per step (at 1/2 step microstepping)
const float stepperTravelPerSteps = 0.0001968505;
const float indexUserIncrement = 0.100;
const float indexMaxPosition = 1.5;
const float indexMinPosition = 0;

// Motor
const int motorStartRPM = 250;
const int motorPwmMin = 64;
const int motorPwmMax = 255;
const int motorIncrementPwmDelay = 50; // milliseconds.
volatile int skipReadingsCount;
const int skipReadingsNum = 2;
volatile int rotationCount;

RunningMedian rotationDeltaRunningMedian(20); // For cleaner RPM visualization.

// Motor PID controller.
double pidSetpoint, pidInputRPM, pidOutputPWM;

double Kp = .05, Ki = .075, Kd = .01;

PID motorPID(&pidInputRPM, &pidOutputPWM, &pidSetpoint, Kp, Ki, Kd, DIRECT);

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void PlaySound(Tone t)
{
  if (!playSounds)
  {
    return;
  }

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
  else if (t == Tone::MenuPrevious)
  {
    tone(PIN_BUZZER, 440, 50);
  }
  else if (t == Tone::MenuNext)
  {
    tone(PIN_BUZZER, 660, 50);
  }
  else if (t == Tone::OptionDecrement)
  {
    tone(PIN_BUZZER, 880, 50);
  }
  else if (t == Tone::OptionIncrement)
  {
    tone(PIN_BUZZER, 1100, 50);
  }
  else if (t == Tone::RpmReached)
  {
    tone(PIN_BUZZER, 880, 100);
    delay(200);
    tone(PIN_BUZZER, 880, 100);
  }
  else if (t == Tone::StoppedWindingSuccess)
  {
    tone(PIN_BUZZER, 550, 250);
    delay(275);
    tone(PIN_BUZZER, 440, 250);
    delay(275);
    tone(PIN_BUZZER, 550, 125);
    delay(150);
    tone(PIN_BUZZER, 650, 1000);
    //delay(1000);
  }
  else if (t == Tone::StoppedWinding)
  {
    tone(PIN_BUZZER, 240, 500);
  }
}

int MotorRpm(unsigned long timeDeltaUs)
{
  // Calculate motor RPM from delta time between rotational sensors.
  return (1.0 / (timeDeltaUs * 2.0)) * 1000.0 * 1000.0 * 60.0;
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
    digitalWrite(PIN_LED_START, HIGH);
  }
  else
  {
    digitalWrite(PIN_LED_START, LOW);
    startHeldCount = millis();
  }

  if (analogRead(PIN_BUTTON_PAUSE) == 0)
  {
    if (millis() - pauseHeldCount > 100)
    {
      state = State::Pause;
    }
    digitalWrite(PIN_LED_PAUSE, HIGH);
  }
  else
  {
    digitalWrite(PIN_LED_PAUSE, LOW);
    pauseHeldCount = millis();
  }

  if (analogRead(PIN_BUTTON_STOP) == 0)
  {
    if (millis() - stopHeldCount > 100)
    {
      state = State::Stop;
    }
    digitalWrite(PIN_LED_STOP, HIGH);
  }
  else
  {
    digitalWrite(PIN_LED_STOP, LOW);
    stopHeldCount = millis();
  }
}

void CheckMenuButtons()
{

  buttonMenuPrevious.read();
  buttonMenuNext.read();
  buttonOptionDecrement.read();
  buttonOptionIncrement.read();

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

    PlaySound(Tone::MenuPrevious);
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
    PlaySound(Tone::MenuNext);
  }

  // Option decrement.
  if (buttonOptionDecrement.wasPressed())
  {
    if (menuSelect == int(MenuItem::WindCount))
    {
      if (windingCount > 10)
      {
        windingCount -= 10;
      }
    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
      if (windingSpeedRpm >= windingSpeedRpmMin + 10)
      {
        windingSpeedRpm -= 10;
      }
    }
    else if (menuSelect == int(MenuItem::WindDirection))
    {
      if (windingDirection == 0)
        windingDirection = 1;
      else
        windingDirection = 0;
    }
    else if (menuSelect == int(MenuItem::IndexSpeed))
    {
      if (indexSpeed >= indexSpeedRpmMin + 10)
      {
        indexSpeed -= 10;
      }
    }
    else if (menuSelect == int(MenuItem::IndexTop))
    {
      indexTop -= indexUserIncrement;
      if (indexTop < 0)
        indexTop = 0;
      SetIndexPosition(indexTop);
    }
    else if (menuSelect == int(MenuItem::IndexBottom))
    {
      indexBottom -= indexUserIncrement;
      if (indexBottom < 0)
        indexBottom = 0;
      SetIndexPosition(indexBottom);
    }
    else if (menuSelect == int(MenuItem::PlaySounds))
    {
      if (playSounds)
        playSounds = false;
      else
        playSounds = true;
    }
    PlaySound(Tone::OptionDecrement);
  }

  // Option increment.
  if (buttonOptionIncrement.wasPressed())
  {
    if (menuSelect == int(MenuItem::WindCount))
    {
      windingCount += 10;
    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
      if (windingSpeedRpm <= windingSpeedRpmMax - 10)
      {
        windingSpeedRpm += 10;
      }
    }
    else if (menuSelect == int(MenuItem::WindDirection))
    {
      if (windingDirection == 0)
        windingDirection = 1;
      else
        windingDirection = 0;
    }
    else if (menuSelect == int(MenuItem::IndexSpeed))
    {
      if (indexSpeed <= indexSpeedRpmMax - 10)
      {
        indexSpeed += 10;
      }
    }
    else if (menuSelect == int(MenuItem::IndexTop))
    {
      indexTop += indexUserIncrement;
      if (indexTop > indexMaxPosition)
        indexTop = indexMaxPosition;
      SetIndexPosition(indexTop);
    }
    else if (menuSelect == int(MenuItem::IndexBottom))
    {
      indexBottom += indexUserIncrement;
      if (indexBottom > indexMaxPosition)
        indexBottom = indexMaxPosition;
      SetIndexPosition(indexBottom);
    }
    else if (menuSelect == int(MenuItem::PlaySounds))
    {
      if (playSounds)
        playSounds = false;
      else
        playSounds = true;
    }
    PlaySound(Tone::OptionIncrement);
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
  static unsigned long toggleDisplayMillis = millis();
  static bool toggleDisplay;

  if (millis() - toggleDisplayMillis > toggleDisplayDelay)
  {
    toggleDisplayMillis = millis();
    toggleDisplay = !toggleDisplay;
  }

  if (state == State::Standby)
  {
    if (menuSelect == int(MenuItem::Home))
    {
      PrintLine(0, "Coil Winder");
      PrintLine(1, "");
    }
    else if (menuSelect == int(MenuItem::WindCount))
    {
      PrintLine(0, "Count:");
      sprintf(buf, "%u Rotations", windingCount);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
      PrintLine(0, "Speed:");
      sprintf(buf, "%u RPM", windingSpeedRpm);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::WindDirection))
    {
      PrintLine(0, "Direction:");
      if (windingDirection == 0)
        PrintLine(1, "CC");
      else if (windingDirection == 1)
        PrintLine(1, "CCW");
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
    else if (menuSelect == int(MenuItem::PlaySounds))
    {
      PrintLine(0, "Play sounds:");
      if (playSounds)
        PrintLine(1, "Yes");
      else
        PrintLine(1, "No");
    }
  }
  else if (state == State::Winding)
  {
    PrintLine(0, "Winding...");
    if (toggleDisplay)
      sprintf(buf, "%u of %u", rotationCount, windingCount);
    else
    {
      sprintf(buf, "%u RPM", MotorRpm(rotationDeltaRunningMedian.getAverage()));
    }

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

void StateWinding(bool firstRunFlag)
{
  static unsigned long motorIncrementPwmMillis = 0;
  static bool accelerationPhase = true;
  static bool playRpmReachedToneFlag;

  int posBottom = indexBottom / stepperTravelPerSteps;
  int posTop = indexTop / stepperTravelPerSteps;

  // Output debug motor values.
  static unsigned long start = millis();
  if (millis() - start > 250)
  {
    start = millis();
    char buf[128];
    sprintf(buf, "Detected (avg.) RPM: %4i | Set Point RPM: %4i | Final RPM: %4i | Motor PWM: %3i", MotorRpm(rotationDeltaRunningMedian.getAverage()), int(pidSetpoint), windingSpeedRpm, int(pidOutputPWM));
    Serial.println(buf);
  }

  // Prepare variables and hardware for winding.
  if (firstRunFlag)
  {
    rotationCount = 0;
    motorIncrementPwmMillis = millis();
    accelerationPhase = true;
    skipReadingsCount = 0;
    rotationDeltaRunningMedian.clear();
    pidSetpoint = motorStartRPM;
    playRpmReachedToneFlag = true;

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

    analogWrite(PIN_MOTOR_PWM, motorPwmMin);
  }

  // Accelerate motor until target RPM is reached.
  if (accelerationPhase)
  {
    if (millis() - motorIncrementPwmMillis > motorIncrementPwmDelay)
    {
      motorIncrementPwmMillis = millis();

      const int setPointRpmIncrement = 5;
      if (pidSetpoint <= windingSpeedRpm - setPointRpmIncrement)
      {
        pidSetpoint += setPointRpmIncrement;
      }
      else
      {
        accelerationPhase = false;
      }
    }
  }

  // Play tone once when motor RPM reaches target RPM.
  if (playRpmReachedToneFlag)
  {
    if (MotorRpm(rotationDeltaRunningMedian.getAverage()) >= windingSpeedRpm)
    {
      playRpmReachedToneFlag = false;
      PlaySound(Tone::RpmReached);
    }
  }

  // Move indexer up and down.
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

  // Check for final conditions.
  if (rotationCount >= windingCount)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
    PlaySound(Tone::StoppedWindingSuccess);
    state = State::Stop;
  }
}

void StateController()
{
  static State previousState = State::Standby;

  if (state == State::Standby)
  {
  }
  else if (state == State::Winding)
  {
    StateWinding(previousState != State::Winding);
  }
  else if (state == State::Pause)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
  }
  else if (state == State::Stop)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
    state = State::Standby;
  }

  previousState = state;
}

// Call prior to enabling tick interrupt.
bool HomeIndexerStepper()
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

// Callback when hall effect sensor triggers event on pin interupt.
void CountRotation()
{
  static unsigned long prevMicros;

  if (state == State::Winding)
  {
    // Motor PID controller.
    unsigned long rotationDeltaTimeInstanious = micros() - prevMicros;
    rotationDeltaRunningMedian.add(rotationDeltaTimeInstanious);
    //int instaniousRpm = (1.0 / (rotationDeltaTimeInstanious * 2.0)) * 1000.0 * 1000.0 * 60.0;
    pidInputRPM = MotorRpm(rotationDeltaTimeInstanious);
    motorPID.Compute();
    analogWrite(PIN_MOTOR_PWM, pidOutputPWM);
    prevMicros = micros();
  }

   // Save rotation count (two pulses per rotation).
  static bool countToggle = true;
  countToggle = !countToggle;
  if (countToggle)
  {
    rotationCount++;
  }
}

// Timer interupt callback
void IndexerStepperCallback()
{
  stepper.run();
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

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

  pinMode(PIN_LED_START, OUTPUT);
  pinMode(PIN_LED_PAUSE, OUTPUT);
  pinMode(PIN_LED_STOP, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(8000);

  buttonMenuPrevious.begin();
  buttonMenuNext.begin();
  buttonOptionDecrement.begin();
  buttonOptionIncrement.begin();

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_SENSOR), CountRotation, FALLING);

  /*
  if (HomeIndexerStepper() == false)
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
  */

  motorPID.SetOutputLimits(motorPwmMin, motorPwmMax);
  motorPID.SetMode(AUTOMATIC);

  // Setup timer for stepper motor tick.
  Timer1.initialize(1000); // microseconds.
  Timer1.attachInterrupt(IndexerStepperCallback);

  state = State::Standby;
  menuSelect = int(MenuItem::Home);
}

void loop()
{
  StateController();

  CheckControlButtons();

  CheckMenuButtons();

  UpdateLCD(); // ~52ms processing time.
}
