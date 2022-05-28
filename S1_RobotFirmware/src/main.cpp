#include <Arduino.h>
#include "../lib/FastLED-3.4.0/src/FastLED.h"
#include "../lib/ESP32Servo-0.10.0/src/ESP32Servo.h"
#include "../lib/ArduinoJson-v6.19.1.h"
#include "../lib/PS3_Controller_Host/src/Ps3Controller.h"

#include <Wire.h>
#include "../lib/Adafruit_Unified_Sensor/Adafruit_Sensor.h"
#include "../lib/Adafruit_BNO055/Adafruit_BNO055.h"
#include "../lib/Adafruit_BNO055/utility/imumaths.h"

// Debug Values

#define LOG_CONTROLLER_INPUT false
#define LOG_CONTROLLER_ACCELEROMETER false
#define LOG_ORIENTATION true
#define LOG_VOLTAGE false
#define LOG_CONTROLLER_CONNECTED false
#define REBOOT_ON_TIMEOUT false

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// System values

// Pinout reference
// green 13 GN
// orange 2 OR
// yellow 4 YL
// blue 12 BL
// purple 14 PR
// silver 15 GY
//SDA brown
//SDL white

#define PIN_NUM_NEOPIXEL_OUTPUT 4
#define PIN_DRIVE_MOTOR_LEFT 12
#define PIN_DRIVE_MOTOR_RIGHT 2
#define PIN_WEAPON_MOTOR 14
#define PIN_WEAPON_MOTOR_COPY 13
#define PIN_VOLTAGE_CHECK 27
#define UPDATE_STEP_LENGTH_MILLIS 10

// Orientation Sensor
//VARIABLES I CANT RENAME!
bool _useIMU = true;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
int val;    // variable to read the value from the analog pin. can't be renamed because dumb.
sensors_event_t event; //I dont think this can be renamed either
int _analogPinValue = 0; // Don't want to rename or get rid of
float _currentXOrientation = 0;
bool _isUpsideDown = false;
float _currentXOrientationOffset = 0;
uint8_t overallSystem, gyro, accel, _magnetometerCalibrationLevel;

// Controller Button Readings
bool _controllerEmergencyShutdownButtonPressed;
bool _controllerTurboModeButtonPressed;
bool _controllerSlowModeButtonPressed;

bool _controllerDirectionUpPressed;
bool _controllerDirectionDownPressed;
bool _controllerDirectionLeftPressed;
bool _controllerDirectionRightPressed;

int _controllerLeftStickX;
int _controllerLeftStickY;
int _controllerRightStickX;
int _controllerRightStickY;
int _controllerL2;
int _controllerR2;

float _leftTriggerNormalizedValue = 0;
float _rightTriggerNormalizedValue = 0;

// Controller Accelerometer Readings
int _controllerCurrentAx = 0;
int _controllerCurrentAy = 0;
int _controllerCurrentAz = 0;
int _controllerCurrentGz = 0;
int _controllerLastAx = 0;
int _controllerLastAy = 0;
int _controllerLastAz = 0;
int _controllerLastGz = 0;

// LED
#define LED_TYPE 1 // 0: 8x8 neopixel, 1: 2x2, 2: single

#define TOTAL_LED 64

#define LED_WEAPON_R3 35
#define LED_WEAPON_R2 43
#define LED_WEAPON_R1 51
#define LED_WEAPON_R1 51
#define LED_WEAPON_R1 51
#define LED_WEAPON_C1 59
#define LED_WEAPON_C2 60
#define LED_WEAPON_L1 52
#define LED_WEAPON_L2 44
#define LED_WEAPON_L3 36

#define MAX_BRIGHTNESS 80

#define HUE_BLUE 171

#define SAT_FORWARD 0
#define HUE_FORWARD 96

#define HUE_NEUTRAL 213
#define SAT_NEUTRAL 255

#define HUE_REVERSE 0
#define SAT_REVERSE 255

#define HUE_DISCONNECTED 45

#define HUE_PURPLE 213
#define STARTING_BRIGHTNESS 40

CRGB leds[TOTAL_LED];
int _driveLeftLEDs[5] = {39, 31, 23, 15, 7};
int _driveRightLEDs[5] = {32, 24, 16, 8, 0};
bool _shouldShowTargetAngle = false;

// ESC Declarations and true values

#define DRIVE_POWER_MAX 100 //Range 0 to 100, 100 being full power
#define WEAPON_POWER_MAX 100 //Range 0 to 100, 100 being full power

#define CENTER_TRIM_DRIVE_ESC -3.0
#define CENTER_TRIM_WEAPON_ESC 0.0

bool _motorsAreAttached = false;

float _worldLeftMotorUnthrottledNormalizedSpeed = 0;
float _worldLeftMotorThrottledNormalizedSpeed = 0;

float _worldRightMotorUnthrottledNormalizedSpeed = 0;
float _worldRightMotorThrottledNormalizedSpeed = 0;

Servo _driveMotorLeft;
float _targetLocalLeftMotorNormalizedSpeed = 0;
float _currentLocalLeftMotorNormalizedSpeed = 0;


Servo _driveMotorRight;
float _targetLocalRightMotorNormalizedSpeed = 0;
float _currentLocalRightMotorNormalizedSpeed = 0;

Servo _weaponMotor;
Servo _weaponMotorCopy;
float _weaponMotorUnthrottledNormalizedSpeed = 0;
float _weaponMotorThrottledNormalizedSpeed = 0;
float _targetWeaponMotorNormalizedSpeed = 0;
float _currentWeaponMotorNormalizedSpeed = 0;

// Drive hypothetical or interpretation
#define DETHROTTLE_WEAPON_FACTOR 20.0 //last 10
#define DETHROTTLE_WEAPON_HARD_MINIMUM 0.25 // last 0.5
#define DETHROTTLE_WEAPON_ON_HARD_TURN true

#define SLOW_MULTIPLIER 0.33
#define TURBO_MULTIPLIER 1
#define DEFAULT_MULTIPLIER 0.66

#define DRIVE_NEUTRAL_CLAMP_TO_ZERO 0.1
#define ANALOG_CONTROLLER_CLAMP_TO_ZERO 2
#define DRIVE_SPEED_SMOOTHING_FACTOR 0.25 // smaller = slower, bigger = faster
#define WEAPON_SPEED_SMOOTHING_FACTOR 0.1 // smaller = slower, bigger = faster
#define DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR 0.8

bool _reverseAllDriveMotors = true; // RPB: This is the case of like if we have a gear
bool _perfectForwardEngaged = false;
float _perfectForwardStartAngle = 0;
bool _videoGameStyleDriveControls = true;
bool _currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank = false;
int _videoGameAngleDeltaDegrees = 0;
int _currentRightJoystickAngle = 0;


// Emergency Shutdown
#define TIME_TO_HOLD_SHUTDOWN_BUTTON_MILLIS 3000
ulong _forceShutdownAccumulatedMillis = 0;
ulong _forceShutdownStartedMillis = 0;
float _forceShutdownProgress = 0;
bool _forceShutdownTurnedOn = false;

// Controller Disconnection
#define CONTROLLER_TIMEOUT_THRESHOLD_MILLIS 1000
ulong _lastControllerAccelerometerChangedTime = 0;
ulong _controllerTimeoutDeclaredStartedMillis = 0;

// Battery Status
#define voltageCutoffDrive  3.2 //BS1
#define voltageCutoffWeapon  3.4 //BS2
#define voltageCutoffHalfway  3.7 //BS3
#define voltageCutoffFull  4.05 //BS4
#define voltageOverLimit  4.25 //BS5

#define BATTERY_STAGE_UNDEFINED -1
#define BATTERY_STAGE_DEAD 0
#define BATTERY_STAGE_CUTOFF_WEAPON 1
#define BATTERY_STAGE_UNDERHALF 2
#define BATTERY_STAGE_OVERHALF 3
#define BATTERY_STAGE_FULL 4
#define BATTERY_STAGE_OVERFULL 5

int _currentBatteryStage = BATTERY_STAGE_UNDEFINED;
ulong _accumulatedTimeSinceBatteryStageSwitched = 0;
int voltageResultRawValue = 0;
float _calculatedVoltageVolts;
float _calculatedVoltagePerCell = 0;

void Start();
void Update();
void setup();
void loop();
void AttachMotors(bool shouldAttachMotors);
void SetDriveMotorSpeed(float newSpeed, bool isLeftMotor, int motorLeds[]);
void SetWeaponMotorSpeed(float newSpeed);
void ReadVoltage();
void ReadOrientation();
void DisplayAnglePoint(float angleToDisplay, CRGB color);
void SetCenterLEDs(CRGB color);
void UpdateCenterLEDColors();
int GetBrightnessForSpeed(float speed);
void OnForceShutDown(ulong shutdownEventReasonStartTime);
void OnControllerDataTimedOut();
void UpdateShutdownTimer();
bool IsControllerDataTimedOut();
float AttenuateWeaponThrottleBasedOnAngleDelta(float inputTargetWeaponSpeed);
void InterpretControllerInput();
void UpdateTargetSignalValues();
void UpdateESCsToTargetValues();
void LogOrientation();
void LogVoltage();
void LogControllerInputReadings();
void LogControllerAccelerometerReadings();
void UpdateAllLEDs();
void ClearAllLEDs();
void InterpretOrientation();
void InterpretVoltage();
void InitializeSerialCommunication();
void InitializeLEDs();
void BootAnimation();
void UpdateDriveLEDs(float newSpeed, bool isLeftMotor, int motorLeds[]);
void UpdateWeaponLEDs(float newSpeed);
void InitializeController();
void CheckAndInitializeOrientationSensor();
void InterpretShutdown();
void InterpretAccelerometerChange();
bool IsAnyDPadButtonPressed();
void InterpretDPadInputToDrive();
void InterpretAnalogSticksAsVideoGameDrive();
void InterpretAnalogSticksAsTankDrive();
void InterpretTriggersForWeaponMotorControl();
void InitializePWMTimers();
void UpdateDriveTargetSignalValues();
void InitializeMotorSpeeds();
void UpdateWeaponTargetSignalValues();

#pragma region System Control and Safeguards

void InitializeSerialCommunication()
{
    Serial.begin(115200);
}

void OnForceShutDown(ulong shutdownEventReasonStartTime)
{
    AttachMotors(false);

    _targetLocalLeftMotorNormalizedSpeed = 0;
    _targetLocalRightMotorNormalizedSpeed = 0;
    _targetWeaponMotorNormalizedSpeed = 0;
    UpdateESCsToTargetValues();
}

bool IsControllerDataTimedOut()
{
    return millis() > (_lastControllerAccelerometerChangedTime + CONTROLLER_TIMEOUT_THRESHOLD_MILLIS);
}

void OnControllerDataTimedOut()
{
    if(LOG_CONTROLLER_CONNECTED)
    {
        Serial.println("We are timed out...");
        Serial.print(" Timeout started: ");
        Serial.print(_controllerTimeoutDeclaredStartedMillis);
    }

    _controllerTimeoutDeclaredStartedMillis =
            _lastControllerAccelerometerChangedTime + CONTROLLER_TIMEOUT_THRESHOLD_MILLIS;

    if (_controllerTimeoutDeclaredStartedMillis + 5000 < millis() && REBOOT_ON_TIMEOUT)
    {
        ESP.restart();
    }

    OnForceShutDown(_controllerTimeoutDeclaredStartedMillis);
}

void UpdateShutdownTimer()
{
    if (_forceShutdownAccumulatedMillis > TIME_TO_HOLD_SHUTDOWN_BUTTON_MILLIS && !_forceShutdownTurnedOn)
    {
        _forceShutdownStartedMillis = millis();
        _forceShutdownTurnedOn = true;
        Serial.println("Force shut down.");
    }
}

#pragma endregion

#pragma region LED Output

void InitializeLEDs()
{
    pinMode(PIN_NUM_NEOPIXEL_OUTPUT, OUTPUT);
    delay(10);
    FastLED.addLeds<NEOPIXEL, PIN_NUM_NEOPIXEL_OUTPUT>(leds, TOTAL_LED);
    FastLED.show();
}

void BootAnimation()
{
    for (int i = 0; i < HUE_PURPLE; ++i)
    {
        CRGB color = CHSV(i, 255, MAX_BRIGHTNESS);

        leds[0] = color;
        leds[1] = color;
        leds[2] = color;
        leds[3] = color;

        FastLED.show();
        delay(2);
    }
}

void UpdateDriveLEDs(float newSpeed, bool isLeftMotor, int motorLeds[])
{
    if (LED_TYPE == 0) // 8x8
    {
        if (!_motorsAreAttached)
        {
            leds[motorLeds[0]] = CRGB::Black;
            leds[motorLeds[1]] = CRGB::Black;
            leds[motorLeds[2]] = CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS);
            leds[motorLeds[3]] = CRGB::Black;
            leds[motorLeds[4]] = CRGB::Black;
        }
        else
        {
            if (!IsControllerDataTimedOut())
            {
                int brightness = GetBrightnessForSpeed(newSpeed);

                if (newSpeed > 0.666)
                {
                    leds[motorLeds[0]] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                    leds[motorLeds[1]] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                    leds[motorLeds[2]] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                    leds[motorLeds[3]] = CRGB::Black;
                    leds[motorLeds[4]] = CRGB::Black;
                }
                else if (newSpeed > 0.333 && newSpeed <= 0.666)
                {
                    leds[motorLeds[0]] = CRGB::Black;
                    leds[motorLeds[1]] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                    leds[motorLeds[2]] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                    leds[motorLeds[3]] = CRGB::Black;
                    leds[motorLeds[4]] = CRGB::Black;
                }
                else if (newSpeed > 0 && newSpeed <= 0.333)
                {
                    leds[motorLeds[0]] = CRGB::Black;
                    leds[motorLeds[1]] = CRGB::Black;
                    leds[motorLeds[2]] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                    leds[motorLeds[3]] = CRGB::Black;
                    leds[motorLeds[4]] = CRGB::Black;
                }
                else if (newSpeed == 0)
                {
                    leds[motorLeds[0]] = CRGB::Black;
                    leds[motorLeds[1]] = CRGB::Black;
                    leds[motorLeds[2]] = CHSV(HUE_NEUTRAL, SAT_NEUTRAL, STARTING_BRIGHTNESS);
                    leds[motorLeds[3]] = CRGB::Black;
                    leds[motorLeds[4]] = CRGB::Black;
                }
                else if (newSpeed >= -0.333 && newSpeed < 0)
                {
                    leds[motorLeds[0]] = CRGB::Black;
                    leds[motorLeds[1]] = CRGB::Black;
                    leds[motorLeds[2]] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                    leds[motorLeds[3]] = CRGB::Black;
                    leds[motorLeds[4]] = CRGB::Black;
                }
                else if (newSpeed >= -0.666 && newSpeed < -0.333)
                {
                    leds[motorLeds[0]] = CRGB::Black;
                    leds[motorLeds[1]] = CRGB::Black;
                    leds[motorLeds[2]] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                    leds[motorLeds[3]] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                    leds[motorLeds[4]] = CRGB::Black;
                }
                else if (newSpeed < -0.666)
                {
                    leds[motorLeds[0]] = CRGB::Black;
                    leds[motorLeds[1]] = CRGB::Black;
                    leds[motorLeds[2]] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                    leds[motorLeds[3]] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                    leds[motorLeds[4]] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                }
            }
        }
    }
    else if (LED_TYPE == 1) //2x2, 0 = main, 1 = right, 2 = left, 3= weapon
    {
        int ledNumber = 1;
        if (isLeftMotor)
        {
            ledNumber = 2;
        }

        if (!_motorsAreAttached)
        {
            leds[ledNumber] = CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS);
        }
        else
        {
            if (!IsControllerDataTimedOut())
            {
                int brightness = GetBrightnessForSpeed(newSpeed);

                if (newSpeed > 0.666)
                {
                    leds[ledNumber] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                }
                else if (newSpeed > 0.333 && newSpeed <= 0.666)
                {
                    leds[ledNumber] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                }
                else if (newSpeed > 0 && newSpeed <= 0.333)
                {
                    leds[ledNumber] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                }
                else if (newSpeed == 0)
                {
                    leds[ledNumber] = CHSV(HUE_NEUTRAL, SAT_NEUTRAL, STARTING_BRIGHTNESS);
                }
                else if (newSpeed >= -0.333 && newSpeed < 0)
                {
                    leds[ledNumber] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                }
                else if (newSpeed >= -0.666 && newSpeed < -0.333)
                {
                    leds[ledNumber] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                }
                else if (newSpeed < -0.666)
                {
                    leds[ledNumber] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                }
            }
        }
    }
    else if (LED_TYPE == 2) // 1x1 single
    {

    }
}

void UpdateWeaponLEDs(float newSpeed)
{
    if (LED_TYPE == 0) // 8x8
    {
        if (!_motorsAreAttached)
        {
            leds[LED_WEAPON_C1] = CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS);
            leds[LED_WEAPON_C2] = CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS);
            leds[LED_WEAPON_L1] = CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS);
            leds[LED_WEAPON_R1] = CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS);
            leds[LED_WEAPON_L2] = CRGB::Black;
            leds[LED_WEAPON_R2] = CRGB::Black;
            leds[LED_WEAPON_L3] = CRGB::Black;
            leds[LED_WEAPON_R3] = CRGB::Black;
        }
        else
        {

            int brightness = GetBrightnessForSpeed(newSpeed);
            if (newSpeed > 0.666)
            {
                leds[LED_WEAPON_C1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_C2] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_L1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_R1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_L2] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_R2] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_L3] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_R3] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
            }
            else if (newSpeed > 0.333 && newSpeed <= 0.666)
            {
                leds[LED_WEAPON_C1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_C2] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_L1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_R1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_L2] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_R2] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_L3] = CRGB::Black;
                leds[LED_WEAPON_R3] = CRGB::Black;
            }
            else if (newSpeed > 0 && newSpeed <= 0.333)
            {
                leds[LED_WEAPON_C1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_C2] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_L1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_R1] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
                leds[LED_WEAPON_L2] = CRGB::Black;
                leds[LED_WEAPON_R2] = CRGB::Black;
                leds[LED_WEAPON_L3] = CRGB::Black;
                leds[LED_WEAPON_R3] = CRGB::Black;
            }
            else if (newSpeed == 0)
            {
                leds[LED_WEAPON_C1] = CHSV(HUE_NEUTRAL, SAT_NEUTRAL, brightness);
                leds[LED_WEAPON_C2] = CHSV(HUE_NEUTRAL, SAT_NEUTRAL, brightness);
                leds[LED_WEAPON_L1] = CHSV(HUE_NEUTRAL, SAT_NEUTRAL, brightness);
                leds[LED_WEAPON_R1] = CHSV(HUE_NEUTRAL, SAT_NEUTRAL, brightness);
                leds[LED_WEAPON_L2] = CRGB::Black;
                leds[LED_WEAPON_R2] = CRGB::Black;
                leds[LED_WEAPON_L3] = CRGB::Black;
                leds[LED_WEAPON_R3] = CRGB::Black;
            }
            else if (newSpeed < 0 && newSpeed >= -0.333)
            {
                leds[LED_WEAPON_C1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_C2] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_L1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_R1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_L2] = CRGB::Black;
                leds[LED_WEAPON_R2] = CRGB::Black;
                leds[LED_WEAPON_L3] = CRGB::Black;
                leds[LED_WEAPON_R3] = CRGB::Black;
            }
            else if (newSpeed < -0.333 && newSpeed >= -0.666)
            {
                leds[LED_WEAPON_C1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_C2] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_L1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_R1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_L2] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_R2] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_L3] = CRGB::Black;
                leds[LED_WEAPON_R3] = CRGB::Black;
            }
            if (newSpeed < -0.666)
            {
                leds[LED_WEAPON_C1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_C2] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_L1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_R1] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_L2] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_R2] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_L3] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
                leds[LED_WEAPON_R3] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
            }
        }
    }
    else if (LED_TYPE == 1) //2x2, 0 = main, 1 = right, 2 = left, 3= weapon
    {
        if (!_motorsAreAttached)
        {
            leds[3] = CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS);
        }
        else
        {

            int brightness = GetBrightnessForSpeed(newSpeed);
            if (newSpeed > 0.666)
            {
                leds[3] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
            }
            else if (newSpeed > 0.333 && newSpeed <= 0.666)
            {
                leds[3] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
            }
            else if (newSpeed > 0 && newSpeed <= 0.333)
            {
                leds[3] = CHSV(HUE_FORWARD, SAT_FORWARD, brightness);
            }
            else if (newSpeed == 0)
            {
                leds[3] = CHSV(HUE_NEUTRAL, SAT_NEUTRAL, brightness);
            }
            else if (newSpeed < 0 && newSpeed >= -0.333)
            {
                leds[3] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
            }
            else if (newSpeed < -0.333 && newSpeed >= -0.666)
            {
                leds[3] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
            }
            if (newSpeed < -0.666)
            {
                leds[3] = CHSV(HUE_REVERSE, SAT_REVERSE, brightness);
            }
        }
    }
    else if (LED_TYPE == 2) // 1x1 single
    {

    }
}

void UpdateAllLEDs()
{
    ClearAllLEDs();
    UpdateDriveLEDs(_currentLocalLeftMotorNormalizedSpeed, true, _driveLeftLEDs);
    UpdateDriveLEDs(_currentLocalRightMotorNormalizedSpeed, false, _driveRightLEDs);
    UpdateWeaponLEDs(_currentWeaponMotorNormalizedSpeed);
    UpdateCenterLEDColors();
}

void ClearAllLEDs()
{
    for (int i = 0; i < 64; ++i)
    {
        leds[i] = CRGB::Black;
    }
}

void SetCenterLEDs(CRGB color)
{
    if (LED_TYPE == 0) // 8x8
    {
        leds[3] = color;
        leds[4] = color;
        leds[11] = color;
        leds[12] = color;
        leds[19] = color;
        leds[20] = color;
        leds[27] = color;
        leds[28] = color;
    }
    else if (LED_TYPE == 1) //2x2, 0 = main, 1 = right, 2 = left, 3= weapon
    {
        leds[0] = color;
    }
    else if (LED_TYPE == 2) // 1x1 single
    {
        leds[0] = color;
    }

    /*

    */
}

void UpdateCenterLEDColors()
{
    SetCenterLEDs(CRGB::Black);
    if (LED_TYPE == 0)
    {
        if (_useIMU)
        {

            if (!_isUpsideDown)
            {
                CRGB color = CHSV(HUE_PURPLE, 0, MAX_BRIGHTNESS);
                leds[27] = color;
                leds[28] = color;
            }
            else
            {
                CRGB color = CHSV(HUE_PURPLE, 255, MAX_BRIGHTNESS);
                leds[27] = color;
                leds[28] = color;
            }

        }
        else
        {
            CRGB color = CHSV(HUE_RED, 255, MAX_BRIGHTNESS);
            leds[27] = color;
            leds[28] = color;
        }
    }

    if (LED_TYPE == 0)
    {
        if (_forceShutdownProgress > 0.01f && _forceShutdownProgress < 0.99f && !_forceShutdownTurnedOn)
        {
            CRGB color = CHSV(HUE_DISCONNECTED, 255, (float) _forceShutdownProgress * (float) MAX_BRIGHTNESS + 25);
            leds[3] = color;
            leds[4] = color;

            if (_forceShutdownProgress > 0.25f)
            {
                leds[11] = color;
                leds[12] = color;
            }

            if (_forceShutdownProgress > 0.5f)
            {
                leds[19] = color;
                leds[20] = color;
            }

            if (_forceShutdownProgress > 0.75f)
            {
                leds[27] = color;
                leds[28] = color;
            }
        }
    }
    else if (LED_TYPE == 1)
    {
        if (_forceShutdownProgress > 0.01f && _forceShutdownProgress < 0.99f && !_forceShutdownTurnedOn)
        {
            CRGB color = CHSV(HUE_DISCONNECTED, 255, (float) _forceShutdownProgress * (float) MAX_BRIGHTNESS + 25);
            leds[0] = color;
        }
        else
        {
            if (_currentBatteryStage == BATTERY_STAGE_DEAD)
            {
                if (millis() % 666 < 333)
                {
                    CRGB color = CHSV(HUE_RED, 255, MAX_BRIGHTNESS);
                    leds[0] = color;
                }
                else
                {
                    CRGB color = CHSV(HUE_RED, 255, 0);
                    leds[0] = color;
                }
            }
            else if (_currentBatteryStage == BATTERY_STAGE_CUTOFF_WEAPON)
            {
                CRGB color = CHSV(HUE_RED, 255, MAX_BRIGHTNESS);
                leds[0] = color;
            }
            else if (_currentBatteryStage == BATTERY_STAGE_UNDERHALF)
            {
                CRGB color = CHSV(HUE_YELLOW, 255, MAX_BRIGHTNESS);
                leds[0] = color;
            }
            else if (_currentBatteryStage == BATTERY_STAGE_OVERHALF)
            {
                CRGB color = CHSV(HUE_GREEN, 255, MAX_BRIGHTNESS);
                leds[0] = color;
            }
            else if (_currentBatteryStage == BATTERY_STAGE_FULL)
            {
                if (millis() % 400 < 200)
                {
                    CRGB color = CHSV(HUE_GREEN, 255, MAX_BRIGHTNESS);
                    leds[0] = color;
                }
                else
                {
                    CRGB color = CHSV(HUE_GREEN, 120, 25);
                    leds[0] = color;
                }
            }
            else if (_currentBatteryStage == BATTERY_STAGE_OVERFULL)
            {
                if (millis() % 200 < 100)
                {
                    CRGB color = CHSV(HUE_RED, 255, MAX_BRIGHTNESS);
                    leds[0] = color;
                }
                else
                {
                    CRGB color = CHSV(HUE_RED, 255, 0);
                    leds[0] = color;
                }
            }
            else if (_currentBatteryStage == BATTERY_STAGE_UNDEFINED)
            {
                CRGB color = CHSV(HUE_RED, 255, 0);
                leds[0] = color;
            }
        }
    }

    if (IsControllerDataTimedOut() && (millis() - CONTROLLER_TIMEOUT_THRESHOLD_MILLIS) % 1000 < 500)
    {
        SetCenterLEDs(CHSV(HUE_BLUE, 255, MAX_BRIGHTNESS));
    }

    if (_forceShutdownTurnedOn && (millis() - _forceShutdownStartedMillis) % 200 < 100)
    {
        SetCenterLEDs(CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS));
    }

}

void DisplayAnglePoint(float angleToDisplay, CRGB color)
{
    if (LED_TYPE != 0)
    {
        return;
    }

    int angleDisplayIndex = -1;

    while (angleToDisplay > 360 || angleToDisplay < 0)
    {
        if (angleToDisplay < 0)
        {
            angleToDisplay += 360;
        }
        if (angleToDisplay > 360)
        {
            angleToDisplay += -360;
        }
    }


    if (angleToDisplay < 15.6)
    {
        angleDisplayIndex = 4;
    }
    else if (angleToDisplay < 29.35)
    {
        angleDisplayIndex = 5;
    }
    else if (angleToDisplay < 40.27)
    {
        angleDisplayIndex = 6;
    }
    else if (angleToDisplay < 49.73)
    {
        angleDisplayIndex = 7;
    }
    else if (angleToDisplay < 60.65)
    {
        angleDisplayIndex = 15;
    }
    else if (angleToDisplay < 74.4)
    {
        angleDisplayIndex = 23;
    }
    else if (angleToDisplay < 90)
    {
        angleDisplayIndex = 31;
    }
    else if (angleToDisplay < 105.6)
    {
        angleDisplayIndex = 39;
    }
    else if (angleToDisplay < 119.35)
    {
        angleDisplayIndex = 47;
    }
    else if (angleToDisplay < 130.27)
    {
        angleDisplayIndex = 55;
    }
    else if (angleToDisplay < 139.73)
    {
        angleDisplayIndex = 63;
    }
    else if (angleToDisplay < 150.65)
    {
        angleDisplayIndex = 62;
    }
    else if (angleToDisplay < 164.4)
    {
        angleDisplayIndex = 61;
    }
    else if (angleToDisplay < 180)
    {
        angleDisplayIndex = 60;
    }
    else if (angleToDisplay < 195.6)
    {
        angleDisplayIndex = 59;
    }
    else if (angleToDisplay < 209.35)
    {
        angleDisplayIndex = 58;
    }
    else if (angleToDisplay < 220.27)
    {
        angleDisplayIndex = 57;
    }
    else if (angleToDisplay < 229.73)
    {
        angleDisplayIndex = 56;
    }
    else if (angleToDisplay < 240.65)
    {
        angleDisplayIndex = 48;
    }
    else if (angleToDisplay < 254.4)
    {
        angleDisplayIndex = 40;
    }
    else if (angleToDisplay < 270)
    {
        angleDisplayIndex = 32;
    }
    else if (angleToDisplay < 285.6)
    {
        angleDisplayIndex = 24;
    }
    else if (angleToDisplay < 299.35)
    {
        angleDisplayIndex = 16;
    }
    else if (angleToDisplay < 310.29)
    {
        angleDisplayIndex = 8;
    }
    else if (angleToDisplay < 319.75)
    {
        angleDisplayIndex = 0;
    }
    else if (angleToDisplay < 330.67)
    {
        angleDisplayIndex = 1;
    }
    else if (angleToDisplay < 344.42)
    {
        angleDisplayIndex = 2;
    }
    else
    {
        angleDisplayIndex = 3;
    }


    for (int i = 0; i < 64; i++)
    {
        if (i == angleDisplayIndex)
        {
            leds[angleDisplayIndex] = color;
        }
    }
}

int GetBrightnessForSpeed(float speed)
{
    if (speed < 0)
    {
        speed = -speed;
    }

    int brightnessDelta = MAX_BRIGHTNESS - STARTING_BRIGHTNESS;
    return STARTING_BRIGHTNESS + (int) (speed * (float) brightnessDelta);
}

#pragma endregion

#pragma region Log Output

void LogControllerInputReadings()
{
    if (!LOG_CONTROLLER_INPUT)
    {
        return;
    }

    Serial.print("Ps3.data.analog.stick.ly: ");
    Serial.println(Ps3.data.analog.stick.ly, DEC);

    Serial.print("Ps3.data.analog.stick.ry: ");
    Serial.println(Ps3.data.analog.stick.ry, DEC);

    Serial.print("Ps3.data.analog.button.l2: ");
    Serial.println(Ps3.data.analog.button.l2, DEC);

    Serial.print("Ps3.data.analog.button.r2: ");
    Serial.println(Ps3.data.analog.button.r2, DEC);

    Serial.print("Ps3.data.button.l1: ");
    Serial.println(Ps3.data.button.l1, DEC);

    Serial.print("Ps3.data.button.r1: ");
    Serial.println(Ps3.data.button.r1, DEC);
}

void LogControllerAccelerometerReadings()
{
    if (!LOG_CONTROLLER_ACCELEROMETER)
    {
        return;
    }

    Serial.println("Accelerometer data:");

    Serial.print(" ax:  ");
    Serial.print(Ps3.data.sensor.accelerometer.x, DEC);

    Serial.print(" ay: ");
    Serial.print(Ps3.data.sensor.accelerometer.y, DEC);

    Serial.print(" az: ");
    Serial.print(Ps3.data.sensor.accelerometer.z, DEC);

    Serial.print(" gz: ");
    Serial.print(Ps3.data.sensor.gyroscope.z, DEC);
}

void LogOrientation()
{
    if (!LOG_ORIENTATION)
    {
        return;
    }

    /*
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
     */

    Serial.print("_magnetometerCalibrationLevel: ");
    Serial.print(_magnetometerCalibrationLevel, 4);
    
    Serial.print("\t   X: ");
    Serial.print(_currentXOrientation, 4);

    Serial.print("\t   XRAW: ");
    Serial.print(event.orientation.x, 4);
    Serial.println("");
    
}

void LogVoltage()
{
    if (!LOG_VOLTAGE)
    {
        return;
    }

    Serial.println();
    Serial.print(voltageResultRawValue);
    Serial.print("rawV, ");
    Serial.print(_calculatedVoltageVolts);
    Serial.print("v, (");
    Serial.print(_calculatedVoltagePerCell);
    Serial.print("v per cell)");
    Serial.println("v");
}

#pragma endregion

#pragma region Input Readings

void InitializeController()
{
    Ps3.begin("00:02:72:3F:5F:02");
    Ps3.getAddress();
}

void CheckAndInitializeOrientationSensor()
{
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        _useIMU = false;
    }

    delay(10);

    if (_useIMU)
    {
        bno.setExtCrystalUse(true);
    }
}

void ReadOrientation()
{
    if (!_useIMU)
    {
        return;
    }

    bno.getEvent(&event);
    bno.getCalibration(&overallSystem, &gyro, &accel, &_magnetometerCalibrationLevel);
    
    LogOrientation();
}

void ReadVoltage()
{
    voltageResultRawValue = analogRead(PIN_VOLTAGE_CHECK);
    _calculatedVoltageVolts = 0.00353 * (float) voltageResultRawValue + 3.88894;
    _calculatedVoltagePerCell = _calculatedVoltageVolts / (float) 4.0;

    LogVoltage();
}

void ReadControllerInput()
{
    _controllerCurrentAx = Ps3.data.sensor.accelerometer.x;
    _controllerCurrentAy = Ps3.data.sensor.accelerometer.y;
    _controllerCurrentAz = Ps3.data.sensor.accelerometer.z;
    _controllerCurrentGz = Ps3.data.sensor.gyroscope.z;

    _controllerEmergencyShutdownButtonPressed = Ps3.data.button.select;

    _controllerSlowModeButtonPressed = Ps3.data.button.l1;
    _controllerTurboModeButtonPressed = Ps3.data.button.r1;

    _controllerDirectionUpPressed = Ps3.data.button.up;
    _controllerDirectionDownPressed = Ps3.data.button.down;
    _controllerDirectionLeftPressed = Ps3.data.button.left;
    _controllerDirectionRightPressed = Ps3.data.button.right;

    _controllerLeftStickX = Ps3.data.analog.stick.lx;
    _controllerLeftStickY = Ps3.data.analog.stick.ly;
    _controllerRightStickX = Ps3.data.analog.stick.rx;
    _controllerRightStickY = Ps3.data.analog.stick.ry;
    _controllerL2 = Ps3.data.analog.button.l2;
    _controllerR2 = Ps3.data.analog.button.r2;

    LogControllerAccelerometerReadings();
    LogControllerInputReadings();
}

#pragma endregion

#pragma region Input Interpretation

void InterpretOrientation()
{
    _currentXOrientation = 360 - event.orientation.x;
    
    if (event.orientation.z > 90 || event.orientation.z < -90)
    {
        _isUpsideDown = true;
        _currentXOrientation += 180;
    }
    else
    {
        _isUpsideDown = false;
    }

    if(_currentXOrientation > 360)
    {
        _currentXOrientation -= 360;
    }
   
    if(_magnetometerCalibrationLevel == 0 || _magnetometerCalibrationLevel == 1)
    {
        _currentXOrientationOffset = 0;
    }
    else
    {
        _currentXOrientationOffset = 120;
    }
    
    _currentXOrientation += _currentXOrientationOffset;
    
    if(_currentXOrientation > 360)
    {
        _currentXOrientation -= 360;
    }

    if (_shouldShowTargetAngle)
    {
        DisplayAnglePoint(_currentRightJoystickAngle + _currentXOrientation, CRGB::Green);
    }

    DisplayAnglePoint(180 + _currentXOrientation, CRGB::White);
}

void InterpretVoltage()
{
    int possibleNewBatteryStage = 0;

    if (_calculatedVoltagePerCell < voltageCutoffDrive)
    {
        possibleNewBatteryStage = BATTERY_STAGE_DEAD;
    }
    else if (_calculatedVoltagePerCell >= voltageCutoffDrive && _calculatedVoltagePerCell < voltageCutoffWeapon)
    {
        possibleNewBatteryStage = BATTERY_STAGE_CUTOFF_WEAPON;
    }
    else if (_calculatedVoltagePerCell >= voltageCutoffWeapon && _calculatedVoltagePerCell < voltageCutoffHalfway)
    {
        possibleNewBatteryStage = BATTERY_STAGE_UNDERHALF;
    }
    else if (_calculatedVoltagePerCell >= voltageCutoffHalfway && _calculatedVoltagePerCell < voltageCutoffFull)
    {
        possibleNewBatteryStage = BATTERY_STAGE_OVERHALF;
    }
    else if (_calculatedVoltagePerCell >= voltageCutoffFull && _calculatedVoltagePerCell < voltageOverLimit)
    {
        possibleNewBatteryStage = BATTERY_STAGE_FULL;
    }
    else
    {
        possibleNewBatteryStage = BATTERY_STAGE_OVERFULL;
    }

    if (_currentBatteryStage == BATTERY_STAGE_UNDEFINED)
    {
        _currentBatteryStage = possibleNewBatteryStage;
    }

    if (possibleNewBatteryStage != _currentBatteryStage)
    {
        _accumulatedTimeSinceBatteryStageSwitched += UPDATE_STEP_LENGTH_MILLIS;
    }
    else
    {
        _accumulatedTimeSinceBatteryStageSwitched = 0;
    }

    if (_accumulatedTimeSinceBatteryStageSwitched > 1000)
    {
        _currentBatteryStage = possibleNewBatteryStage;
        _accumulatedTimeSinceBatteryStageSwitched = 0;
    }
}

void InterpretShutdown()
{
    if (_controllerEmergencyShutdownButtonPressed)
    {
        _forceShutdownAccumulatedMillis += UPDATE_STEP_LENGTH_MILLIS;

        _forceShutdownProgress = (float) _forceShutdownAccumulatedMillis / (float) TIME_TO_HOLD_SHUTDOWN_BUTTON_MILLIS;
    }
    else
    {
        _forceShutdownAccumulatedMillis = 0;
        _forceShutdownProgress = 0;
    }
}

void InterpretAccelerometerChange()
{
    if (
            _controllerLastAx == _controllerCurrentAx &&
            _controllerLastAy == _controllerCurrentAy &&
            _controllerLastAz == _controllerCurrentAz &&
            _controllerLastGz == _controllerCurrentGz
            )
    {
        if(LOG_CONTROLLER_CONNECTED)
        {
            Serial.println("Controller is totally still- physically impossible. is it offline?");

        }
    }
    else
    {
        _lastControllerAccelerometerChangedTime = millis();
    }

    _controllerLastAx = _controllerCurrentAx;
    _controllerLastAy = _controllerCurrentAy;
    _controllerLastAz = _controllerCurrentAz;
    _controllerLastGz = _controllerCurrentGz;
}

bool IsAnyDPadButtonPressed()
{
    return _controllerDirectionLeftPressed || _controllerDirectionRightPressed || _controllerDirectionUpPressed ||
           _controllerDirectionDownPressed;
}

void InterpretDPadInputToDrive()
{
    float leftMixedValue = 0;
    float rightMixedValue = 0;

    if (!_controllerDirectionUpPressed && !_controllerDirectionDownPressed)
    {
        _perfectForwardEngaged = false;
    }

    if (_controllerDirectionUpPressed)
    {


        if (!_perfectForwardEngaged)
        {
            _perfectForwardEngaged = true;
            _perfectForwardStartAngle = _currentXOrientation;
        }
        else
        {
            Serial.print("startAngle: ");
            Serial.print(_perfectForwardStartAngle);
            Serial.print(", currentAngle: ");
            Serial.print(_currentXOrientation);

            int angleDelta = (int) (_currentXOrientation - _perfectForwardStartAngle + 180 + 360) % 360 - 180;

            Serial.print("angle diff:");
            Serial.print(angleDelta);

            // RPB: angle range is always 0 to 360

            if (angleDelta > 3)
            {
                Serial.print("correcting towards right:");
                leftMixedValue += 1;
                rightMixedValue += 1 * DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR;
                // too much to the left, turn right
            }
            else if (angleDelta < -3)
            {
                Serial.print("correcting towards left");
                leftMixedValue += 1 * DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR;
                rightMixedValue += 1;
                //too much to the right, turn left
            }
            else
            {
                Serial.print("perfect forward");
                leftMixedValue += 1;
                rightMixedValue += 1;
                // perfect
            }

            Serial.println("");
        }
    }


    if (_controllerDirectionDownPressed)
    {

        if (!_perfectForwardEngaged)
        {
            _perfectForwardEngaged = true;
            _perfectForwardStartAngle = _currentXOrientation;
        }
        else
        {
            Serial.print("startAngle: ");
            Serial.print(_perfectForwardStartAngle);
            Serial.print(", currentAngle: ");
            Serial.print(_currentXOrientation);

            int angleDelta = (int) (_currentXOrientation - _perfectForwardStartAngle + 180 + 360) % 360 - 180;

            Serial.print("angle diff:");
            Serial.print(angleDelta);

            // RPB: angle range is always 0 to 360

            if (angleDelta > 3)
            {
                Serial.print("correcting towards right:");
                leftMixedValue -= 1 * DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR;
                rightMixedValue -= 1;
                // too much to the left, turn right
            }
            else if (angleDelta < -3)
            {
                Serial.print("correcting towards left");
                leftMixedValue -= 1;
                rightMixedValue -= 1 * DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR;
                //too much to the right, turn left
            }
            else
            {
                Serial.print("perfect forward");
                leftMixedValue -= 1;
                rightMixedValue -= 1;
                // perfect
            }

            Serial.println("");
        }
    }

    if (_controllerDirectionLeftPressed)
    {
        _perfectForwardEngaged = false;
        leftMixedValue -= 1;
        rightMixedValue += 1;
    }

    if (_controllerDirectionRightPressed)
    {
        _perfectForwardEngaged = false;
        leftMixedValue += 1;
        rightMixedValue -= 1;
    }

    if (leftMixedValue > 1)
    {
        leftMixedValue = 1;
    }

    if (leftMixedValue < -1)
    {
        leftMixedValue = -1;
    }

    if (rightMixedValue > 1)
    {
        rightMixedValue = 1;
    }

    if (rightMixedValue < -1)
    {
        rightMixedValue = -1;
    }

    _worldLeftMotorUnthrottledNormalizedSpeed = leftMixedValue;
    _worldRightMotorUnthrottledNormalizedSpeed = rightMixedValue;
}

void InterpretAnalogSticksAsVideoGameDrive()
{
    _currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank = true;
    _shouldShowTargetAngle = false;

    float leftMixedValue = 0;
    float rightMixedValue = 0;
    float rx = (float) _controllerRightStickX / (float) 128;
    float ry = -(float) _controllerRightStickY / (float) 128;
    float ly = -(float) _controllerLeftStickY / (float) 128;

    if (rx == 0)
    {
        rx = 0.001;
    }

    if (abs(rx) > 0.5 || abs(ry) > 0.5)
    {
        _shouldShowTargetAngle = true;

        float angleStick = atan2(ry, rx) * RAD_TO_DEG + 90;

        _currentRightJoystickAngle = angleStick;
        
        // RPB: attempted to make reversing and turning intuitive. it was weird instead.
        /*
        if(ly < 0)
        {
            angleStick += 180;
            if(angleStick > 360)
            {
                angleStick -= 360;
            }
        }
         */

        _videoGameAngleDeltaDegrees = (int) (_currentXOrientation - angleStick + 360) % 360 - 180;

        Serial.print("targetAngle: ");
        Serial.print(_currentRightJoystickAngle);
        Serial.print(", currentAngle: ");
        Serial.print(_currentXOrientation);

        Serial.print("angle diff:");
        Serial.print(_videoGameAngleDeltaDegrees);

        float angleDeltaSpeedMultiplier = ((float) abs(_videoGameAngleDeltaDegrees)) / 180;
        //rpb: angle diff can range from 0 to 180, so divide by 180

        if (_videoGameAngleDeltaDegrees > 3)
        {
            Serial.print("correcting towards right:");
            leftMixedValue += 0.1 + 0.5 * angleDeltaSpeedMultiplier;
            rightMixedValue += -0.1 + -0.5 * angleDeltaSpeedMultiplier;
            // too much to the left, turn right
        }
        else if (_videoGameAngleDeltaDegrees < -3)
        {
            Serial.print("correcting towards left");
            leftMixedValue += -0.1 + -0.5 * angleDeltaSpeedMultiplier;
            rightMixedValue += 0.1 + 0.5 * angleDeltaSpeedMultiplier;
            //too much to the right, turn left
        }
        else
        {
            Serial.print("perfect forward");
            leftMixedValue += 0;
            rightMixedValue += 0;
            // perfect
        }
    }
    else
    {
        _videoGameAngleDeltaDegrees = 0;
    }

    if (abs(ly) > (0.1))
    {
        leftMixedValue += ly * 0.8;
        rightMixedValue += ly * 0.8;
    }


    Serial.println();

    _worldLeftMotorUnthrottledNormalizedSpeed = leftMixedValue;
    _worldRightMotorUnthrottledNormalizedSpeed = rightMixedValue;
}

void InterpretAnalogSticksAsTankDrive()
{
    if (_controllerLeftStickY < ANALOG_CONTROLLER_CLAMP_TO_ZERO &&
        _controllerLeftStickY > -ANALOG_CONTROLLER_CLAMP_TO_ZERO)
    {
        _worldLeftMotorUnthrottledNormalizedSpeed = 0;
    }
    else
    {
        _worldLeftMotorUnthrottledNormalizedSpeed = -(float) _controllerLeftStickY / (float) 128;
    }

    if (_controllerRightStickY < ANALOG_CONTROLLER_CLAMP_TO_ZERO &&
        _controllerRightStickY > -ANALOG_CONTROLLER_CLAMP_TO_ZERO)
    {
        _worldRightMotorUnthrottledNormalizedSpeed = 0;
    }
    else
    {
        _worldRightMotorUnthrottledNormalizedSpeed = -(float) _controllerRightStickY / (float) 128;
    }
}

void InterpretTriggersForWeaponMotorControl()
{
    if (_controllerL2 < ANALOG_CONTROLLER_CLAMP_TO_ZERO)
    {
        _leftTriggerNormalizedValue = 0;
    }
    else
    {
        _leftTriggerNormalizedValue = (float) _controllerL2 / (float) 256;
    }

    if (_controllerR2 < ANALOG_CONTROLLER_CLAMP_TO_ZERO)
    {
        _rightTriggerNormalizedValue = 0;
    }
    else
    {
        _rightTriggerNormalizedValue = (float) _controllerR2 / (float) 256;
    }

    if (_leftTriggerNormalizedValue < 0.05 && _rightTriggerNormalizedValue < 0.05)
    {
        _weaponMotorUnthrottledNormalizedSpeed = 0;
    }
    else
    {
        if (_leftTriggerNormalizedValue > _rightTriggerNormalizedValue)
        {
            // Left trigger is dominant
            _weaponMotorUnthrottledNormalizedSpeed = (float) -1 * (float) (_leftTriggerNormalizedValue);
        }
        else
        {
            _weaponMotorUnthrottledNormalizedSpeed = (float) 1 * (_rightTriggerNormalizedValue);
            // Right trigger is dominant
        }
    }
}

void InterpretControllerInput()
{
    InterpretShutdown();
    InterpretAccelerometerChange();

    _currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank = false;

    if (IsAnyDPadButtonPressed())
    {
        InterpretDPadInputToDrive();
    }
    else
    {
        if (_videoGameStyleDriveControls)
        {
            InterpretAnalogSticksAsVideoGameDrive();
        }
        else
        {
            InterpretAnalogSticksAsTankDrive();
        }
    }

    InterpretTriggersForWeaponMotorControl();
}

#pragma endregion

#pragma region Motor Control

void InitializePWMTimers()
{
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    ESP32PWM::allocateTimer(4);
    ESP32PWM::allocateTimer(5);

    _driveMotorLeft.setPeriodHertz(50);
    _driveMotorRight.setPeriodHertz(50);
    _weaponMotor.setPeriodHertz(50);
    _weaponMotorCopy.setPeriodHertz(50);
}

void InitializeMotorSpeeds()
{
    SetWeaponMotorSpeed(0);
    SetDriveMotorSpeed(0, true, _driveLeftLEDs);
    SetDriveMotorSpeed(0, false, _driveRightLEDs);
}

void AttachMotors(bool shouldAttachMotors)
{
    if (shouldAttachMotors)
    {
        _driveMotorLeft.attach(PIN_DRIVE_MOTOR_LEFT, 1000, 2000);
        _driveMotorRight.attach(PIN_DRIVE_MOTOR_RIGHT, 1000, 2000);
        _weaponMotor.attach(PIN_WEAPON_MOTOR, 1000, 2000);
        _weaponMotorCopy.attach(PIN_WEAPON_MOTOR_COPY, 1000, 2000);
    }
    else
    {
        _driveMotorLeft.detach();
        _driveMotorRight.detach();
        _weaponMotor.detach();
        _weaponMotorCopy.detach();
    }

    _motorsAreAttached = shouldAttachMotors;
}

void UpdateDriveTargetSignalValues()
{
    float throttleMultiplier = 0;

    if (_controllerTurboModeButtonPressed && !_controllerSlowModeButtonPressed)
    {
        throttleMultiplier = TURBO_MULTIPLIER;
    }
    else if (!_controllerTurboModeButtonPressed && _controllerSlowModeButtonPressed)
    {
        throttleMultiplier = SLOW_MULTIPLIER;
    }
    else
    {
        throttleMultiplier = DEFAULT_MULTIPLIER;
    }

    _worldLeftMotorThrottledNormalizedSpeed = _worldLeftMotorUnthrottledNormalizedSpeed * throttleMultiplier;
    _worldRightMotorThrottledNormalizedSpeed = _worldRightMotorUnthrottledNormalizedSpeed * throttleMultiplier;

    if (_currentBatteryStage == BATTERY_STAGE_DEAD)
    {
        _worldLeftMotorThrottledNormalizedSpeed = 0;
        _worldRightMotorThrottledNormalizedSpeed = 0;
    }

    if (!_isUpsideDown)
    {
        _targetLocalLeftMotorNormalizedSpeed = _worldLeftMotorThrottledNormalizedSpeed;
        _targetLocalRightMotorNormalizedSpeed = _worldRightMotorThrottledNormalizedSpeed;
    }
    else
    {
        _targetLocalRightMotorNormalizedSpeed = -_worldLeftMotorThrottledNormalizedSpeed;
        _targetLocalLeftMotorNormalizedSpeed = -_worldRightMotorThrottledNormalizedSpeed;
    }
}

void UpdateWeaponTargetSignalValues()
{
    _weaponMotorThrottledNormalizedSpeed = AttenuateWeaponThrottleBasedOnAngleDelta(
            _weaponMotorUnthrottledNormalizedSpeed);

    if (_currentBatteryStage == BATTERY_STAGE_DEAD || _currentBatteryStage == BATTERY_STAGE_CUTOFF_WEAPON)
    {
        _weaponMotorThrottledNormalizedSpeed = 0;
    }
    else
    {
        _targetWeaponMotorNormalizedSpeed = _weaponMotorThrottledNormalizedSpeed;
    }
}

void UpdateTargetSignalValues()
{
    UpdateDriveTargetSignalValues();
    UpdateWeaponTargetSignalValues();
}

void UpdateESCsToTargetValues()
{
    // Smooths the signal to prevent violent changes to the drive signal (can destroy gearbox)

    if (_currentLocalLeftMotorNormalizedSpeed != _targetLocalLeftMotorNormalizedSpeed)
    {
        if (abs(_currentLocalLeftMotorNormalizedSpeed - _targetLocalLeftMotorNormalizedSpeed) <
            DRIVE_SPEED_SMOOTHING_FACTOR)
        {
            _currentLocalLeftMotorNormalizedSpeed = _targetLocalLeftMotorNormalizedSpeed;
        }
        else if (_currentLocalLeftMotorNormalizedSpeed < _targetLocalLeftMotorNormalizedSpeed)
        {
            _currentLocalLeftMotorNormalizedSpeed += DRIVE_SPEED_SMOOTHING_FACTOR;
        }
        else if (_currentLocalLeftMotorNormalizedSpeed > _targetLocalLeftMotorNormalizedSpeed)
        {
            _currentLocalLeftMotorNormalizedSpeed -= DRIVE_SPEED_SMOOTHING_FACTOR;
        }
    }

    if (_currentLocalRightMotorNormalizedSpeed != _targetLocalRightMotorNormalizedSpeed)
    {
        if (abs(_currentLocalRightMotorNormalizedSpeed - _targetLocalRightMotorNormalizedSpeed) <
            DRIVE_SPEED_SMOOTHING_FACTOR)
        {
            _currentLocalRightMotorNormalizedSpeed = _targetLocalRightMotorNormalizedSpeed;
        }
        else if (_currentLocalRightMotorNormalizedSpeed < _targetLocalRightMotorNormalizedSpeed)
        {
            _currentLocalRightMotorNormalizedSpeed += DRIVE_SPEED_SMOOTHING_FACTOR;
        }
        else if (_currentLocalRightMotorNormalizedSpeed > _targetLocalRightMotorNormalizedSpeed)
        {
            _currentLocalRightMotorNormalizedSpeed -= DRIVE_SPEED_SMOOTHING_FACTOR;
        }
    }

    if (_currentWeaponMotorNormalizedSpeed != _targetWeaponMotorNormalizedSpeed)
    {
        if (abs(_currentWeaponMotorNormalizedSpeed - _targetWeaponMotorNormalizedSpeed) < WEAPON_SPEED_SMOOTHING_FACTOR)
        {
            _currentWeaponMotorNormalizedSpeed = _targetWeaponMotorNormalizedSpeed;
        }
        else if (_currentWeaponMotorNormalizedSpeed < _targetWeaponMotorNormalizedSpeed)
        {
            _currentWeaponMotorNormalizedSpeed += WEAPON_SPEED_SMOOTHING_FACTOR;
        }
        else if (_currentWeaponMotorNormalizedSpeed > _targetWeaponMotorNormalizedSpeed)
        {
            _currentWeaponMotorNormalizedSpeed -= WEAPON_SPEED_SMOOTHING_FACTOR;
        }
    }

    _currentWeaponMotorNormalizedSpeed = _targetWeaponMotorNormalizedSpeed;

    SetDriveMotorSpeed(_currentLocalLeftMotorNormalizedSpeed, true, _driveLeftLEDs);
    SetDriveMotorSpeed(_currentLocalRightMotorNormalizedSpeed, false, _driveRightLEDs);
    SetWeaponMotorSpeed(_currentWeaponMotorNormalizedSpeed);
}

void SetDriveMotorSpeed(float newSpeed, bool isLeftMotor, int motorLeds[])
{
    if (newSpeed < DRIVE_NEUTRAL_CLAMP_TO_ZERO && newSpeed > -DRIVE_NEUTRAL_CLAMP_TO_ZERO)
    {
        newSpeed = 0;
    }

    if (_reverseAllDriveMotors)
    {
        newSpeed = -newSpeed;
    }

    if (isLeftMotor)
    {
        int newPowerToWrite = 90 + CENTER_TRIM_DRIVE_ESC - int(newSpeed * (float) (90 * DRIVE_POWER_MAX / 100));
        _driveMotorLeft.write(newPowerToWrite);
    }
    else
    {
        int newPowerToWrite = 90 + CENTER_TRIM_DRIVE_ESC - int(newSpeed * (float) (90 * DRIVE_POWER_MAX / 100));
        _driveMotorRight.write(newPowerToWrite);
    }
}

float AttenuateWeaponThrottleBasedOnAngleDelta(float inputTargetWeaponSpeed)
{

    if (!DETHROTTLE_WEAPON_ON_HARD_TURN)
    {
        return inputTargetWeaponSpeed;
    }
    float dethrottleMultiplier = 1;


    if (_currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank)
    {
        dethrottleMultiplier =
                1 - abs((float) _videoGameAngleDeltaDegrees / (float) 180 * (float) DETHROTTLE_WEAPON_FACTOR);

        if (dethrottleMultiplier > 1)
        {
            dethrottleMultiplier = 1;
        }

        if (dethrottleMultiplier < DETHROTTLE_WEAPON_HARD_MINIMUM)
        {
            dethrottleMultiplier = DETHROTTLE_WEAPON_HARD_MINIMUM;
        }
    }
    else
    {
        dethrottleMultiplier = 1;
    }

    return inputTargetWeaponSpeed * dethrottleMultiplier;
}

void SetWeaponMotorSpeed(float newSpeed)
{
    int newPowerToWrite = 90 + CENTER_TRIM_WEAPON_ESC + int(newSpeed * (float) (90 * WEAPON_POWER_MAX / 100));

    _weaponMotor.write(180 - newPowerToWrite);
    _weaponMotorCopy.write(newPowerToWrite);

}

#pragma endregion

#pragma region Lifecycle

void Start()
{
    InitializeSerialCommunication();
    InitializeLEDs();
    InitializeController();
    InitializePWMTimers();
    CheckAndInitializeOrientationSensor();
    InitializeMotorSpeeds();
    BootAnimation();
}

void Update()
{
    ReadControllerInput();
    ReadVoltage();
    ReadOrientation();

    InterpretOrientation();
    InterpretVoltage();
    InterpretControllerInput();

    UpdateTargetSignalValues();

    if (_forceShutdownTurnedOn)
    {
        OnForceShutDown(_forceShutdownStartedMillis);
    }
    else
    {
        if (!IsControllerDataTimedOut()) // Means we got fresh data
        {
            if (!_motorsAreAttached)
            {
                AttachMotors(true);
            }

            UpdateESCsToTargetValues();
        }
        else// we timed out...
        {
            OnControllerDataTimedOut();
        }
    }

    UpdateShutdownTimer();
    UpdateAllLEDs();
}// lol unity style

void setup()
{
    Start();
}

void loop()
{
    Update();
    FastLED.show();
    delay(UPDATE_STEP_LENGTH_MILLIS);
}

#pragma endregion


