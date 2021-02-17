/*
  Project: Coil Winder
  Author: Reuben Strangelove
  Date: 2021/15/2
  Description: Coil winder machine controller for winding electric musical intrument pickup coils.

  Hardware:
    Microcontroller: Arduino Nano (Atmel Atmega328P)
    Motor driver: TB6612FNG module.
    Stepper driver: TMC2208 module (non-UART version).
    

  Notes:
    

  To Do:
    Add no motor RPM detected timeout.


  HISTORY

  VERSION  AUTHOR  DATE  NOTES
  ===============================
  0.1.0 ReubenStr 2021/15/2 Development phase.

*/

#include <Arduino.h>           // Built-in.
#include <Wire.h>              // Built-in.
#include <EEPROM.h>            // Built-in.
#include <LiquidCrystal_I2C.h> // https://github.com/johnrickman/LiquidCrystal_I2C
#include <AccelStepper.h>      // https://github.com/waspinator/AccelStepper
#include <JC_Button.h>         // https://github.com/JChristensen/JC_Button
#include <TimerOne.h>          // https://playground.arduino.cc/Code/Timer1/
#include <PID_v1.h>            // https://github.com/br3ttb/Arduino-PID-Library
#include <RunningMedian.h>     // https://github.com/RobTillaart/RunningMedian

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

// Stepper.
// Stepper is 200 steps per revolutioin and driver set for 1/2 microstepping.
// Indexer lead screw is 2mm travel per turn (0.0787402 inches).
// 0.0787402" / (200 / (1/2)) = 0.0001968505" per step
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPER_DRIVER_STEP, PIN_STEPPER_DRIVER_DIR);
const float stepperStepsPerRevolution = 200; // Most steppers are 200 steps/rev.
const float stepperLeadScrewTravelPerRevolution = 0.0787402;
const float stepperMicrosteps = 0.5;                                                                                        // Other user selectable options: 1/2, 1/4, 1/8, 1/16. (but precision is not necessary).
const double stepperTravelPerSteps = stepperLeadScrewTravelPerRevolution / (stepperStepsPerRevolution / stepperMicrosteps); //0.0001968505 inches.
const double indexPositionUserIncrement = 0.025;                                                                            // Inches.
const float indexPositionUserIncrementHeld = 0.100;
const unsigned int indexRpmUserIncrement = 10;
const unsigned int indexRpmUserIncrementHeld = 10;
const double indexPositionMax = 2.300; // Inches. Determined by machine physical travel limit.
const double indexPositionMin = 0;     // Inches.
const unsigned int indexSpeedRpmMin = 10;
const unsigned int indexSpeedRpmMax = 200;

double indexPosition = 0.0; // Inches from homed position.

// DC Motor.
const unsigned int motorPwmMin = 64;
const unsigned int motorPwmMax = 255;
const unsigned int motorIncrementPwmDelay = 50; // milliseconds.
const unsigned int setPointRpmIncrement = 5;
const unsigned int skipReadingsNum = 2;
const unsigned int motorStartRpm = 300;
const unsigned int windingCountUserIncrement = 10;
const unsigned int windingCountUserIncrementHeld = 100;
const unsigned int windingRpmUserIncrement = 10;
const unsigned int windingRpmUserIncrementHeld = 10;
const unsigned int windingCountMin = 10;
const unsigned int windingCountMax = 60000;
const unsigned int windingSpeedRpmMin = 300;
const unsigned int windingSpeedRpmMax = 1000;
volatile unsigned int rotationCount;
volatile bool motorPIDStartupFlag;
RunningMedian rotationDeltaRunningMedian(20); // For cleaner RPM visualization.

// Motor PID controller.
double pidSetpoint, pidInputRPM, pidOutputPWM;
const double Kp = .055, Ki = .075, Kd = .01;
PID motorPID(&pidInputRPM, &pidOutputPWM, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Buttons.
Button buttonMenuPrevious(PIN_BUTTON_MENU_PREVIOUS);
Button buttonMenuNext(PIN_BUTTON_MENU_NEXT);
Button buttonOptionDecrement(PIN_BUTTON_OPTION_DOWN);
Button buttonOptionIncrement(PIN_BUTTON_OPTION_UP);
const unsigned int buttonToneLengthMs = 20;
const unsigned int holdToStartSec = 3;
const unsigned int holdToStopSec = 3;
const unsigned int buttonAnalogActiveThreshold = 10;
const unsigned int buttonHeldDelayForMultipleIncrementMs = 500; // milliseconds.

// Display.
LiquidCrystal_I2C lcd(0x27, 20, 2);
const unsigned int toggleDisplayDelay = 2000;
const unsigned int userMenuTimeoutDuringPaused = 2500; // milliseconds.

enum class DisplaySet
{
  UserVariables,
  Countdown,
  Winding,
  Paused,
  Stopped
};

// Main state.
enum class State
{
  Standby,
  Winding,
  Paused,
  Stopped
};
State state = State::Standby;
unsigned int changeStateCountdown;

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

// User params.
struct UserParams
{
  unsigned int windingCount;
  unsigned int windingSpeedRpm;
  unsigned int windingDirection;
  unsigned int indexSpeedRpm;
  double indexTop;
  double indexBottom;
  bool playSounds;
} userParams;

int menuSelect = int(MenuItem::WindCount);

// Tones.
enum class Tone
{
  StepperHomeSuccess,
  StepperHomeFailed,
  MenuPrevious,
  MenuNext,
  OptionDecrement,
  OptionIncrement,
  OptionDecrementHeld,
  OptionIncrementHeld,
  Countdown,
  Started,
  Paused,
  Stopped,
  RpmReached,
  StoppedWindingSuccess,
  StoppedWinding
};

// LEDs.
enum class LED
{
  Start,
  Pause,
  Stop
};

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void SetLED(LED led, bool state)
{
  if (led == LED::Start)
  {
    analogWrite(PIN_LED_START, state ? 127 : 0);
  }
  else if (led == LED::Pause)
  {
    analogWrite(PIN_LED_PAUSE, state ? 127 : 0);
  }
  else if (led == LED::Stop)
  {
    analogWrite(PIN_LED_STOP, state ? 127 : 0);
  }
}

void UpdateLEDs()
{
  if (state == State::Winding)
  {
    SetLED(LED::Start, HIGH);
  }
  else
  {
    SetLED(LED::Start, LOW);
  }

  if (state == State::Paused)
  {
    SetLED(LED::Pause, HIGH);
  }
  else
  {
    SetLED(LED::Pause, LOW);
  }
}

void PlaySound(Tone t)
{
  if (!userParams.playSounds)
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
  else if (t == Tone::OptionDecrementHeld)
  {
    tone(PIN_BUZZER, 880, 10);
  }
  else if (t == Tone::OptionIncrementHeld)
  {
    tone(PIN_BUZZER, 1100, 10);
  }
  else if (t == Tone::Countdown)
  {
    tone(PIN_BUZZER, 880, 100);
  }
  else if (t == Tone::Started)
  {
    for (int i = 0; i < 4; i++)
    {
      tone(PIN_BUZZER, 880 + i * 40, 100);
      delay(100);
    }
  }
  else if (t == Tone::Paused)
  {
    for (int i = 0; i < 4; i++)
    {
      tone(PIN_BUZZER, 880, 50);
      delay(100);
    }
  }
  else if (t == Tone::Stopped)
  {
    for (int i = 0; i < 4; i++)
    {
      tone(PIN_BUZZER, 880 - i * 40, 100);
      delay(100);
    }
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

unsigned int MotorRpm(unsigned long timeDeltaUs)
{
  // Calculate motor RPM from delta time between rotational sensors.
  // 1 / ((timeDelta in microseconds between sensor trigger) * (two sensors triggers per rotation)) * (1000 us/ms) *  (1000 ms/s) * (60 s/min)
  return (1.0 / (timeDeltaUs * 2.0)) * 1000.0 * 1000.0 * 60.0;
}

unsigned int CalcStepsPerSecondFromRpm(float rpm)
{
  return (rpm / 60.0) * (stepperStepsPerRevolution / stepperMicrosteps);
}

void SetIndexPosition(float pos)
{
  stepper.moveTo(pos / stepperTravelPerSteps);
}

void LoadUserParams()
{
  EEPROM.get(0, userParams);

  // Load default value if saved value is out of range (will happen for first time retrieval);
  if (userParams.windingCount < windingCountMin || userParams.windingCount > windingCountMax)
    userParams.windingCount = windingCountMin;

  if (userParams.windingSpeedRpm < windingSpeedRpmMin || userParams.windingSpeedRpm > windingSpeedRpmMax)
    userParams.windingSpeedRpm = windingSpeedRpmMin;

  if (userParams.windingDirection != 0 || userParams.windingDirection != 1)
    userParams.windingDirection = 0;

  if (userParams.indexSpeedRpm < indexSpeedRpmMin || userParams.indexSpeedRpm > indexSpeedRpmMax)
    userParams.indexSpeedRpm = indexSpeedRpmMin;

  if (userParams.indexTop < indexPositionMin || userParams.indexTop > indexPositionMax || isnan(userParams.indexTop))
    userParams.indexTop = indexPositionMin;

  if (userParams.indexBottom < indexPositionMin || userParams.indexBottom > indexPositionMax || isnan(userParams.indexBottom))
    userParams.indexBottom = indexPositionMin;

  if (userParams.playSounds != true || userParams.playSounds != false)
    userParams.playSounds = true;
}

void SaveUserParams()
{
  // Note: put() only writes data if the data has changed.
  EEPROM.put(0, userParams);
}

bool CheckControlButtons()
{
  static unsigned long startHeldCount;
  static unsigned long pauseHeldCount;
  static unsigned long stopHeldCount;
  static bool pauseReleasedFlag;
  static unsigned int previousChangeStateCountdownStart;
  static unsigned int previousChangeStateCountdownStop;

  bool buttonHeldFlag = false;

  // Start button.
  if (analogRead(PIN_BUTTON_START) < int(buttonAnalogActiveThreshold))
  {
    if (state == State::Standby)
    {
      if (millis() - startHeldCount >= holdToStartSec * 1000)
      {
        SaveUserParams();
        PlaySound(Tone::Started);
        state = State::Winding;
      }
      else if (millis() - startHeldCount > 100)
      {
        changeStateCountdown = holdToStartSec - (millis() - startHeldCount) / 1000;
        if (previousChangeStateCountdownStart != changeStateCountdown)
        {
          previousChangeStateCountdownStart = changeStateCountdown;
          PlaySound(Tone::Countdown);
        }
        buttonHeldFlag = true;
      }
    }
  }
  else
  {  
    startHeldCount = millis();
  }

  // Pause button.
  if (analogRead(PIN_BUTTON_PAUSE) < int(buttonAnalogActiveThreshold))
  {
    if (state == State::Winding || state == State::Paused)
    {
      if (millis() - pauseHeldCount > 100)
      {
        if (pauseReleasedFlag)
        {
          pauseReleasedFlag = false;
          if (state == State::Winding)
          {
            PlaySound(Tone::Paused);
            state = State::Paused;
          }
          else if (state == State::Paused)
          {
            PlaySound(Tone::Started);
            state = State::Winding;
          }
        }
      }
    }
  }
  else
  {
    pauseHeldCount = millis();
    pauseReleasedFlag = true;
  }

  // Stop button.
  if (analogRead(PIN_BUTTON_STOP) < int(buttonAnalogActiveThreshold))
  {
    if (state == State::Paused || state == State::Winding)
    {
      state = State::Paused;

      if (millis() - stopHeldCount >= holdToStopSec * 1000)
      {
        PlaySound(Tone::Stopped);
        state = State::Stopped;
      }
      else if (millis() - stopHeldCount > 100)
      {
        changeStateCountdown = holdToStopSec - (millis() - stopHeldCount) / 1000;
        if (previousChangeStateCountdownStop != changeStateCountdown)
        {
          previousChangeStateCountdownStop = changeStateCountdown;
          PlaySound(Tone::Countdown);
        }
        buttonHeldFlag = true;
      }
      SetLED(LED::Stop, HIGH);
    }
  }
  else
  {
    SetLED(LED::Stop, LOW);
    stopHeldCount = millis();
  }

  return buttonHeldFlag;
}

bool CheckMenuButtons()
{
  bool buttonPressedFlag = false;

  buttonMenuPrevious.read();
  buttonMenuNext.read();
  buttonOptionDecrement.read();
  buttonOptionIncrement.read();

  if (state == State::Standby || state == State::Paused)
  {

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
      buttonPressedFlag = true;
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
      buttonPressedFlag = true;
    }

    if (menuSelect == int(MenuItem::Home))
    {
      return buttonPressedFlag;
    }

    // Option decrement.
    if (buttonOptionDecrement.wasPressed())
    {
      if (menuSelect == int(MenuItem::WindCount))
      {
        if (userParams.windingCount >= windingCountMin + windingCountUserIncrement)
        {
          userParams.windingCount -= windingCountUserIncrement;
        }
      }
      else if (menuSelect == int(MenuItem::WindSpeed))
      {
        if (userParams.windingSpeedRpm >= windingSpeedRpmMin + windingRpmUserIncrement)
        {
          userParams.windingSpeedRpm -= windingRpmUserIncrement;
        }
      }
      else if (menuSelect == int(MenuItem::WindDirection))
      {
        if (userParams.windingDirection == 0)
          userParams.windingDirection = 1;
        else
          userParams.windingDirection = 0;
      }
      else if (menuSelect == int(MenuItem::IndexSpeed))
      {
        if (userParams.indexSpeedRpm >= indexSpeedRpmMin + indexRpmUserIncrement)
        {
          userParams.indexSpeedRpm -= indexRpmUserIncrement;
        }
      }
      else if (menuSelect == int(MenuItem::IndexTop))
      {
        userParams.indexTop -= indexPositionUserIncrement;
        if (userParams.indexTop < 0)
          userParams.indexTop = 0;
        SetIndexPosition(userParams.indexTop);
      }
      else if (menuSelect == int(MenuItem::IndexBottom))
      {
        userParams.indexBottom -= indexPositionUserIncrement;
        if (userParams.indexBottom < 0)
          userParams.indexBottom = 0;
        SetIndexPosition(userParams.indexBottom);
      }
      else if (menuSelect == int(MenuItem::PlaySounds))
      {
        if (userParams.playSounds)
          userParams.playSounds = false;
        else
          userParams.playSounds = true;
      }
      PlaySound(Tone::OptionDecrement);
      buttonPressedFlag = true;
    }

    // Option increment.
    if (buttonOptionIncrement.wasPressed())
    {
      if (menuSelect == int(MenuItem::WindCount))
      {
        if (userParams.windingCount <= windingCountMax - windingSpeedRpmMin)
        {
          userParams.windingCount += windingCountUserIncrement;
        }
      }
      else if (menuSelect == int(MenuItem::WindSpeed))
      {
        if (userParams.windingSpeedRpm <= windingSpeedRpmMax - windingRpmUserIncrement)
        {
          userParams.windingSpeedRpm += windingRpmUserIncrement;
        }
      }
      else if (menuSelect == int(MenuItem::WindDirection))
      {
        if (userParams.windingDirection == 0)
          userParams.windingDirection = 1;
        else
          userParams.windingDirection = 0;
      }
      else if (menuSelect == int(MenuItem::IndexSpeed))
      {
        if (userParams.indexSpeedRpm <= indexSpeedRpmMax - indexRpmUserIncrement)
        {
          userParams.indexSpeedRpm += indexRpmUserIncrement;
        }
      }
      else if (menuSelect == int(MenuItem::IndexTop))
      {
        userParams.indexTop += indexPositionUserIncrement;
        if (userParams.indexTop > indexPositionMax)
          userParams.indexTop = indexPositionMax;
        SetIndexPosition(userParams.indexTop);
      }
      else if (menuSelect == int(MenuItem::IndexBottom))
      {
        userParams.indexBottom += indexPositionUserIncrement;
        if (userParams.indexBottom > indexPositionMax)
          userParams.indexBottom = indexPositionMax;
        SetIndexPosition(userParams.indexBottom);
      }
      else if (menuSelect == int(MenuItem::PlaySounds))
      {
        if (userParams.playSounds)
          userParams.playSounds = false;
        else
          userParams.playSounds = true;
      }
      PlaySound(Tone::OptionIncrement);
      buttonPressedFlag = true;
    }

    // Option decrement held down.
    if (buttonOptionDecrement.pressedFor(buttonHeldDelayForMultipleIncrementMs))
    {
      if (menuSelect == int(MenuItem::WindCount))
      {
        if (userParams.windingCount >= windingCountMin + windingCountUserIncrementHeld)
        {
          userParams.windingCount -= windingCountUserIncrementHeld;
          PlaySound(Tone::OptionDecrementHeld);
        }
        else
        {
          userParams.windingCount = windingCountMin;
        }
      }
      else if (menuSelect == int(MenuItem::WindSpeed))
      {
        if (userParams.windingSpeedRpm >= windingSpeedRpmMin + windingRpmUserIncrementHeld)
        {
          userParams.windingSpeedRpm -= windingRpmUserIncrementHeld;
          PlaySound(Tone::OptionDecrementHeld);
        }
        else
        {
          userParams.windingSpeedRpm = windingSpeedRpmMin;
        }
      }
      else if (menuSelect == int(MenuItem::IndexSpeed))
      {
        if (userParams.indexSpeedRpm >= indexSpeedRpmMin + indexRpmUserIncrementHeld)
        {
          userParams.indexSpeedRpm -= indexRpmUserIncrementHeld;
          PlaySound(Tone::OptionDecrementHeld);
        }
        else
        {
          userParams.indexSpeedRpm = indexSpeedRpmMin;
        }
      }
      else if (menuSelect == int(MenuItem::IndexTop))
      {
        if (userParams.indexTop >= indexPositionMin + indexPositionUserIncrementHeld)
        {
          userParams.indexTop -= indexPositionUserIncrementHeld;
          SetIndexPosition(userParams.indexTop);
          PlaySound(Tone::OptionDecrementHeld);
        }
        else
        {
          userParams.indexTop = indexPositionMin;
        }
      }
      else if (menuSelect == int(MenuItem::IndexBottom))
      {
        if (userParams.indexBottom >= indexPositionMin + indexPositionUserIncrementHeld)
        {
          userParams.indexBottom -= indexPositionUserIncrementHeld;
          SetIndexPosition(userParams.indexBottom);
          PlaySound(Tone::OptionDecrementHeld);
        }
        else
        {
          userParams.indexBottom = indexPositionMin;
        }
      }
      buttonPressedFlag = true;
    }

    // Option increment held down.
    if (buttonOptionIncrement.pressedFor(buttonHeldDelayForMultipleIncrementMs))
    {
      if (menuSelect == int(MenuItem::WindCount))
      {
        if (userParams.windingCount <= windingCountMax - windingCountUserIncrementHeld)
        {
          userParams.windingCount += windingCountUserIncrementHeld;
          PlaySound(Tone::OptionIncrementHeld);
        }
        else
        {
          userParams.windingCount = windingCountMax;
        }
      }
      else if (menuSelect == int(MenuItem::WindSpeed))
      {
        if (userParams.windingSpeedRpm <= windingSpeedRpmMax - windingRpmUserIncrementHeld)
        {
          userParams.windingSpeedRpm += windingRpmUserIncrementHeld;
          PlaySound(Tone::OptionIncrementHeld);
        }
        else
        {
          userParams.windingSpeedRpm = windingSpeedRpmMax;
        }
      }
      else if (menuSelect == int(MenuItem::IndexSpeed))
      {
        if (userParams.indexSpeedRpm <= indexSpeedRpmMax - indexRpmUserIncrementHeld)
        {
          userParams.indexSpeedRpm += indexRpmUserIncrementHeld;
          PlaySound(Tone::OptionIncrementHeld);
        }
        else
        {
          userParams.indexSpeedRpm = indexSpeedRpmMax;
        }
      }
      else if (menuSelect == int(MenuItem::IndexTop))
      {
        if (userParams.indexTop <= indexPositionMax - indexPositionUserIncrementHeld)
        {
          userParams.indexTop += indexPositionUserIncrementHeld;
          SetIndexPosition(userParams.indexTop);
          PlaySound(Tone::OptionIncrementHeld);
        }
        else
        {
          userParams.indexTop = indexPositionMax;
        }
      }
      else if (menuSelect == int(MenuItem::IndexBottom))
      {
        if (userParams.indexBottom <= indexPositionMax - indexPositionUserIncrementHeld)
        {
          userParams.indexBottom += indexPositionUserIncrementHeld;
          SetIndexPosition(userParams.indexBottom);
          PlaySound(Tone::OptionIncrementHeld);
        }
        else
        {
          userParams.indexBottom = indexPositionMax;
        }
      }
      buttonPressedFlag = true;
    }
  }

  return buttonPressedFlag;
}

// ~52ms processing time.
void UpdateLCD(DisplaySet displaySet)
{
  char buf[20];

  if (displaySet == DisplaySet::UserVariables)
  {
    if (menuSelect == int(MenuItem::Home))
    {
      PrintLine(0, "Coil Winder");
      PrintLine(1, "");
    }
    else if (menuSelect == int(MenuItem::WindCount))
    {
      PrintLine(0, "Count:");
      sprintf(buf, "%u Rotations", userParams.windingCount);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
      PrintLine(0, "Speed:");
      sprintf(buf, "%u RPM", userParams.windingSpeedRpm);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::WindDirection))
    {
      PrintLine(0, "Direction:");
      if (userParams.windingDirection == 0)
        PrintLine(1, "CC");
      else if (userParams.windingDirection == 1)
        PrintLine(1, "CCW");
    }
    else if (menuSelect == int(MenuItem::IndexSpeed))
    {
      PrintLine(0, "Index Speed:");
      sprintf(buf, "%u RPM", userParams.indexSpeedRpm);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::IndexTop))
    {
      PrintLine(0, "Index Top:");
      sprintf(buf, dtostrf(userParams.indexTop, 5, 3, "%5.3f")); // Compiler warning is ignorable.
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::IndexBottom))
    {
      PrintLine(0, "Index Bottom:");
      sprintf(buf, dtostrf(userParams.indexBottom, 5, 3, "%5.3f")); // Compiler warning is ignorable.
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::PlaySounds))
    {
      PrintLine(0, "Play sounds:");
      if (userParams.playSounds)
        PrintLine(1, "Yes");
      else
        PrintLine(1, "No");
    }
  }
  else if (displaySet == DisplaySet::Winding)
  {
    static bool toggleDisplay;
    static unsigned long toggleDisplayMillis = millis();
    if (millis() - toggleDisplayMillis > toggleDisplayDelay)
    {
      toggleDisplayMillis = millis();
      toggleDisplay = !toggleDisplay;
    }
    PrintLine(0, "Winding...");
    if (toggleDisplay)
      sprintf(buf, "%u of %u", rotationCount, userParams.windingCount);
    else
    {
      sprintf(buf, "%u of %u RPM", MotorRpm(rotationDeltaRunningMedian.getAverage()), userParams.windingSpeedRpm);
    }

    PrintLine(1, buf);
  }
  else if (displaySet == DisplaySet::Paused)
  {
    PrintLine(0, "Winding paused.");
    sprintf(buf, "%u of %u", rotationCount, userParams.windingCount);
    PrintLine(1, buf);
  }
  else if (state == State::Stopped)
  {
    PrintLine(0, "Winding stoped.");
    sprintf(buf, "%u of %u", rotationCount, userParams.windingCount);
    PrintLine(1, buf);
  }
  else if (displaySet == DisplaySet::Countdown)
  {
    if (state == State::Standby)
    {
      PrintLine(0, "Hold button");
      sprintf(buf, "to start.    (%i)", changeStateCountdown);
      PrintLine(1, buf);
    }
    else if (state == State::Paused)
    {
      PrintLine(0, "Hold button");
      char buf[20];
      sprintf(buf, "to stop.     (%i)", changeStateCountdown);
      PrintLine(1, buf);
    }
  }
}

void StateWinding(bool firstRunFlag)
{
  static unsigned long motorIncrementPwmMillis = 0;
  static bool accelerationPhaseFlag = true;
  static bool playRpmReachedToneFlag;

  int posBottom = userParams.indexBottom / stepperTravelPerSteps;
  int posTop = userParams.indexTop / stepperTravelPerSteps;

  // Prepare variables and hardware for motor startup.
  if (firstRunFlag)
  {
    motorIncrementPwmMillis = millis();
    rotationDeltaRunningMedian.clear();
    accelerationPhaseFlag = true;
    playRpmReachedToneFlag = true;
    motorPIDStartupFlag = true;
    pidSetpoint = motorStartRpm;

    int indexerStepperSpeedStepsSec = CalcStepsPerSecondFromRpm(userParams.indexSpeedRpm);
    int indexerStepperAcclerationStepsSec = CalcStepsPerSecondFromRpm(userParams.indexSpeedRpm) * 3;

    stepper.setMaxSpeed(indexerStepperSpeedStepsSec);
    stepper.setAcceleration(indexerStepperAcclerationStepsSec);

    // Reset motor PID (hacky method to reset internal accumulator, but it works).
    motorPID.SetOutputLimits(0, 1);
    motorPID.SetOutputLimits(motorPwmMin, motorPwmMax);
    motorPID.SetMode(AUTOMATIC);

    if (userParams.windingDirection == 0)
    {
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
    }
    else if (userParams.windingDirection == 1)
    {
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
    }

    analogWrite(PIN_MOTOR_PWM, motorPwmMin);

    char buf[128];
    Serial.println("Starting winding...");
    sprintf(buf, "Indexer stepper RPM: %u | steps/sec: %u | accleration steps/sec^s %u", userParams.indexSpeedRpm, indexerStepperSpeedStepsSec, indexerStepperAcclerationStepsSec);
    Serial.println(buf);
  }

  // Output debug motor values.
  static unsigned long start = millis();
  if (millis() - start > 250)
  {
    start = millis();
    char buf[128];
    sprintf(buf, "Detected (avg.) RPM: %4i | Set Point RPM: %4i | Final RPM: %4i | Motor PWM: %3i || Stepper steps/sec: %i",
            MotorRpm(rotationDeltaRunningMedian.getAverage()), int(pidSetpoint), userParams.windingSpeedRpm, int(pidOutputPWM), int(stepper.speed()));
    Serial.println(buf);
  }

  // Accelerate motor until target RPM is reached.
  if (accelerationPhaseFlag)
  {
    if (millis() - motorIncrementPwmMillis > motorIncrementPwmDelay)
    {
      motorIncrementPwmMillis = millis();

      if (pidSetpoint <= userParams.windingSpeedRpm - setPointRpmIncrement)
      {
        pidSetpoint += setPointRpmIncrement;
      }
      else
      {
        accelerationPhaseFlag = false;
      }
    }
  }
  else
  {
    pidSetpoint = userParams.windingSpeedRpm;
  }

  // Play tone once when motor RPM reaches target RPM.
  if (playRpmReachedToneFlag)
  {
    if (MotorRpm(rotationDeltaRunningMedian.getAverage()) >= userParams.windingSpeedRpm)
    {
      playRpmReachedToneFlag = false;
      PlaySound(Tone::RpmReached);
    }
  }

  // Move indexer up and down.
  static bool indexDirectionToggle;
  if (!stepper.isRunning())
  {
    indexDirectionToggle = !indexDirectionToggle;

    if (indexDirectionToggle)
    {
      stepper.moveTo(posBottom);
    }
    else
    {
      stepper.moveTo(posTop);
    }
  }

  // Check for final conditions.
  if (rotationCount >= userParams.windingCount)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
    PlaySound(Tone::StoppedWindingSuccess);
    state = State::Stopped;
  }
}

void StateController()
{
  static bool resetWindingStateFlag = true;

  if (state == State::Standby)
  {
    rotationCount = 0;
    resetWindingStateFlag = true;
  }
  else if (state == State::Winding)
  {
    StateWinding(resetWindingStateFlag);
    resetWindingStateFlag = false;
  }
  else if (state == State::Paused)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
    stepper.stop();
    resetWindingStateFlag = true;
  }
  else if (state == State::Stopped)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
    stepper.stop();
    state = State::Standby;
  }
}

// Call prior to enabling stepper tick interrupt.
bool HomeIndexerStepper()
{
  PrintLine(0, "Homing indexer");

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(2000);

  // Move stepper down physical range (+10%) in order to hit the limit switch.
  stepper.setCurrentPosition(0);
  stepper.moveTo(-((indexPositionMax + (indexPositionMax * 0.10)) / stepperTravelPerSteps));

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

  if (indexSuccessFlag == false)
  {
    PrintLine(0, "Indexer failed");
    PrintLine(1, "to home!");
    PlaySound(Tone::StepperHomeFailed);
    while (true)
      ;
  }
  else
  {
    PlaySound(Tone::StepperHomeSuccess);
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
    if (motorPIDStartupFlag)
    {
      // Deny first reading to reset delta time calculation.
      motorPIDStartupFlag = false;
    }
    else
    {
      unsigned long rotationDeltaTimeInstanious = micros() - prevMicros;
      rotationDeltaRunningMedian.add(rotationDeltaTimeInstanious);
      pidInputRPM = MotorRpm(rotationDeltaTimeInstanious);
      motorPID.Compute();
      analogWrite(PIN_MOTOR_PWM, pidOutputPWM);
    }
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

  buttonMenuPrevious.begin();
  buttonMenuNext.begin();
  buttonOptionDecrement.begin();
  buttonOptionIncrement.begin();

  LoadUserParams();

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_SENSOR), CountRotation, FALLING);

  HomeIndexerStepper();

  // Setup timer for stepper motor tick.
  Timer1.initialize(250); // microseconds.
  Timer1.attachInterrupt(IndexerStepperCallback);

  state = State::Standby;
  menuSelect = int(MenuItem::Home);
}

void loop()
{
  StateController();

  UpdateLEDs();

  bool menuButtonActiveFlag = CheckMenuButtons();

  bool controlButtonActiveFlag = CheckControlButtons();

  if (controlButtonActiveFlag)
  {
    UpdateLCD(DisplaySet::Countdown);
  }
  else if (state == State::Standby)
  {
    UpdateLCD(DisplaySet::UserVariables);
  }
  else if (state == State::Winding)
  {
    UpdateLCD(DisplaySet::Winding);
  }
  else if (state == State::Paused)
  {
    static unsigned long startMenuSwap = millis();
    static bool menuSwapFlag = false;
    if (menuButtonActiveFlag)
    {
      menuSwapFlag = true;
      startMenuSwap = millis();
    }
    else
    {
      if (millis() - startMenuSwap > userMenuTimeoutDuringPaused)
      {
        menuSwapFlag = false;
      }
    }
    if (menuSwapFlag)
    {
      UpdateLCD(DisplaySet::UserVariables);
    }
    else
    {
      UpdateLCD(DisplaySet::Paused);
    }
  }
  else if (state == State::Stopped)
  {
    UpdateLCD(DisplaySet::Stopped);
  }
}
