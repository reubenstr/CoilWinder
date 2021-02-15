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
// Indexer lead screw is 2mm travel per turn (0.0787402 inches).
// 0.0787402 / 400 = 0.0001968505 per step (at 1/2 step microstepping)
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPER_DRIVER_STEP, PIN_STEPPER_DRIVER_DIR);
const double stepperTravelPerSteps = 0.0001968505; // Inches.
const double indexUserIncrement = 0.025;           // Inches.
const double indexPositionMax = 2.300;             // Inches. Determined by machine physical travel limit.
const double indexPositionMin = 0;                 // Inches.
double indexPosition = 0.0;                        // Inches from homed position.

// DC Motor.
const int motorPwmMin = 64;
const int motorPwmMax = 255;
const int motorIncrementPwmDelay = 50; // milliseconds.
const int setPointRpmIncrement = 5;
const int skipReadingsNum = 2;
volatile int rotationCount;
RunningMedian rotationDeltaRunningMedian(20); // For cleaner RPM visualization.

// Motor PID controller.
double pidSetpoint, pidInputRPM, pidOutputPWM;
double Kp = .05, Ki = .075, Kd = .01;
PID motorPID(&pidInputRPM, &pidOutputPWM, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Buttons.
Button buttonMenuPrevious(PIN_BUTTON_MENU_PREVIOUS);
Button buttonMenuNext(PIN_BUTTON_MENU_NEXT);
Button buttonOptionDecrement(PIN_BUTTON_OPTION_DOWN);
Button buttonOptionIncrement(PIN_BUTTON_OPTION_UP);
const int buttonToneLengthMs = 20;
const int holdToStartSec = 3;
const int holdToStopSec = 3;
const int buttonAnalogActiveThreshold = 10;

// Display.
LiquidCrystal_I2C lcd(0x27, 20, 2);
const int toggleDisplayDelay = 2000;

// Main state.
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

// User params.
struct UserParams
{
  unsigned int windingCount = 1000;
  int windingSpeedRpm = 1000;
  int windingDirection;
  int indexSpeed;
  double indexTop = 0.25;
  double indexBottom = 0.0;
  bool playSounds = true;
} userParams;

// User params min/max values.
const unsigned int windingCountMin = 10;
const unsigned int windingCountMax = 60000;
const int windingSpeedRpmMin = 300;
const int windingSpeedRpmMax = 1000;
const double indexTopMin = indexPositionMin;
const double indexTopMax = indexPositionMax;
const double indexBottomMin = indexPositionMin;
const double indexBottomMax = indexPositionMax;
const int indexSpeedRpmMin = 10;
const int indexSpeedRpmMax = 100;

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

int MotorRpm(unsigned long timeDeltaUs)
{
  // Calculate motor RPM from delta time between rotational sensors.
  // 1 / ((timeDelta in microseconds between sensor trigger) * (two sensors triggers per rotation)) * (1000 us/ms) *  (1000 ms/s) * (60 s/min)
  return (1.0 / (timeDeltaUs * 2.0)) * 1000.0 * 1000.0 * 60.0;
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

  if (userParams.indexSpeed < indexSpeedRpmMin || userParams.indexSpeed > indexSpeedRpmMax)
    userParams.indexSpeed = indexSpeedRpmMin;

  if (userParams.indexTop < indexTopMin || userParams.indexTop > indexTopMax || isnan(userParams.indexTop))
    userParams.indexTop = indexTopMin;

  if (userParams.indexBottom < indexBottomMin || userParams.indexBottom > indexBottomMax || isnan(userParams.indexBottom))
    userParams.indexBottom = indexBottomMin;

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
  static bool startReleasedFlag;
  static bool pauseReleasedFlag;
  static bool stopReleasedFlag;

  bool buttonHeldFlag = false;

  // Start button.
  if (analogRead(PIN_BUTTON_START) < buttonAnalogActiveThreshold)
  {
    if (state == State::Standby)
    {
      if (millis() - startHeldCount >= holdToStartSec * 1000)
      {
        state = State::Winding;
      }
      else if (millis() - startHeldCount > 100)
      {
        PrintLine(0, "Hold button");
        char buf[20];
        sprintf(buf, "to start.    (%u)", holdToStartSec - (millis() - startHeldCount) / 1000);
        PrintLine(1, buf);

        buttonHeldFlag = true;
      }
      SetLED(LED::Start, HIGH);
    }
  }
  else
  {
    SetLED(LED::Start, LOW);
    startHeldCount = millis();
  }

  // Pause button.
  if (analogRead(PIN_BUTTON_PAUSE) < buttonAnalogActiveThreshold)
  {
    if (state == State::Winding || state == State::Pause)
    {
      if (millis() - pauseHeldCount > 100)
      {
        if (pauseReleasedFlag)
        {
          pauseReleasedFlag = false;
          if (state == State::Winding)
          {
            state = State::Pause;
          }
          else if (state == State::Pause)
          {
            state = State::Winding;
          }
        }
      }
      SetLED(LED::Pause, HIGH);
    }
  }
  else
  {
    SetLED(LED::Pause, LOW);
    pauseHeldCount = millis();
    pauseReleasedFlag = true;
  }

  // Stop button.
  if (analogRead(PIN_BUTTON_STOP) < buttonAnalogActiveThreshold)
  {
    if (state == State::Pause || state == State::Winding)
    {
      state = State::Pause;

      if (millis() - stopHeldCount >= holdToStopSec * 1000)
      {
        state = State::Stop;
      }
      else if (millis() - stopHeldCount > 100)
      {
        PrintLine(0, "Hold button");
        char buf[20];
        sprintf(buf, "to stop.     (%u)", holdToStopSec - (millis() - stopHeldCount) / 1000);
        PrintLine(1, buf);

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
      if (userParams.windingCount >= windingCountMin + 10)
      {
        userParams.windingCount -= 10;
      }
    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
      if (userParams.windingSpeedRpm >= windingSpeedRpmMin + 10)
      {
        userParams.windingSpeedRpm -= 10;
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
      if (userParams.indexSpeed >= indexSpeedRpmMin + 10)
      {
        userParams.indexSpeed -= 10;
      }
    }
    else if (menuSelect == int(MenuItem::IndexTop))
    {
      userParams.indexTop -= indexUserIncrement;
      if (userParams.indexTop < 0)
        userParams.indexTop = 0;
      SetIndexPosition(userParams.indexTop);
    }
    else if (menuSelect == int(MenuItem::IndexBottom))
    {
      userParams.indexBottom -= indexUserIncrement;
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
  }

  // Option increment.
  if (buttonOptionIncrement.wasPressed())
  {
    if (menuSelect == int(MenuItem::WindCount))
    {
      if (userParams.windingCount <= windingCountMax - 10)
      {
        userParams.windingCount += 10;
      }
    }
    else if (menuSelect == int(MenuItem::WindSpeed))
    {
      if (userParams.windingSpeedRpm <= windingSpeedRpmMax - 10)
      {
        userParams.windingSpeedRpm += 10;
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
      if (userParams.indexSpeed <= indexSpeedRpmMax - 10)
      {
        userParams.indexSpeed += 10;
      }
    }
    else if (menuSelect == int(MenuItem::IndexTop))
    {
      userParams.indexTop += indexUserIncrement;
      if (userParams.indexTop > indexPositionMax)
        userParams.indexTop = indexPositionMax;
      SetIndexPosition(userParams.indexTop);
    }
    else if (menuSelect == int(MenuItem::IndexBottom))
    {
      userParams.indexBottom += indexUserIncrement;
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
  }

  // Option decrement held down.
  if (buttonOptionDecrement.pressedFor(500))
  {
    if (menuSelect == int(MenuItem::WindCount))
    {
      if (userParams.windingCount >= windingCountMin + 100)
      {
        userParams.windingCount -= 100;
      }
    }
  }

  // Option increment held down.
  if (buttonOptionIncrement.pressedFor(500))
  {
    if (menuSelect == int(MenuItem::WindCount))
    {
      if (userParams.windingCount <= windingCountMax - 100)
      {
        userParams.windingCount += 100;
      }
    }
  }
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
      sprintf(buf, "%u RPM", userParams.indexSpeed);
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::IndexTop))
    {
      PrintLine(0, "Index Top:");
      sprintf(buf, dtostrf(userParams.indexTop, 5, 3, "%5.3f"));
      PrintLine(1, buf);
    }
    else if (menuSelect == int(MenuItem::IndexBottom))
    {
      PrintLine(0, "Index Bottom:");
      sprintf(buf, dtostrf(userParams.indexBottom, 5, 3, "%5.3f"));
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
  else if (state == State::Winding)
  {
    PrintLine(0, "Winding...");
    if (toggleDisplay)
      sprintf(buf, "%u of %u", rotationCount, userParams.windingCount);
    else
    {
      sprintf(buf, "%u RPM", MotorRpm(rotationDeltaRunningMedian.getAverage()));
    }

    PrintLine(1, buf);
  }
  else if (state == State::Pause)
  {
    PrintLine(0, "Winding paused.");
    sprintf(buf, "%u of %u", rotationCount, userParams.windingCount);
    PrintLine(1, buf);
  }
  else if (state == State::Stop)
  {
    PrintLine(0, "Winding stoped.");
    sprintf(buf, "%u of %u", rotationCount, userParams.windingCount);
    PrintLine(1, buf);
  }
}

void StateWinding(bool firstRunFlag)
{
  static unsigned long motorIncrementPwmMillis = 0;
  static bool accelerationPhaseFlag = true;
  static bool playRpmReachedToneFlag;

  int posBottom = userParams.indexBottom / stepperTravelPerSteps;
  int posTop = userParams.indexTop / stepperTravelPerSteps;

  // Output debug motor values.
  static unsigned long start = millis();
  if (millis() - start > 250)
  {
    start = millis();
    char buf[128];
    sprintf(buf, "Detected (avg.) RPM: %4i | Set Point RPM: %4i | Final RPM: %4i | Motor PWM: %3i", MotorRpm(rotationDeltaRunningMedian.getAverage()), int(pidSetpoint), userParams.windingSpeedRpm, int(pidOutputPWM));
    Serial.println(buf);
  }

  // Prepare variables and hardware for motor startup.
  if (firstRunFlag)
  {
    motorIncrementPwmMillis = millis();
    accelerationPhaseFlag = true;
    rotationDeltaRunningMedian.clear();
    playRpmReachedToneFlag = true;

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
  if (rotationCount >= userParams.windingCount)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
    PlaySound(Tone::StoppedWindingSuccess);
    state = State::Stop;
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
  else if (state == State::Pause)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
    stepper.stop();
    resetWindingStateFlag = true;
  }
  else if (state == State::Stop)
  {
    analogWrite(PIN_MOTOR_PWM, 0);
    stepper.stop();
    state = State::Standby;
  }
}

// Call prior to enabling stepper tick interrupt.
bool HomeIndexerStepper()
{
  PrintLine(0, "Homing stepper");

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

  LoadUserParams();

  attachInterrupt(digitalPinToInterrupt(PIN_HALL_EFFECT_SENSOR), CountRotation, FALLING);

  HomeIndexerStepper();

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

  CheckMenuButtons();

  bool controlButtonHeldFlag = CheckControlButtons();

  if (!controlButtonHeldFlag)
    UpdateLCD(); // ~52ms processing time.

  SaveUserParams();
}
