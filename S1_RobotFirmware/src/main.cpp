#include <Arduino.h>
#include "../lib/FastLED-3.4.0/src/FastLED.h"
#include "../lib/ESP32Servo-0.10.0/src/ESP32Servo.h"
#include "../lib/ArduinoJson-v6.19.1.h"
#include "../lib/PS3_Controller_Host/src/Ps3Controller.h"

#include <Wire.h>
#include "../lib/Adafruit_Unified_Sensor/Adafruit_Sensor.h"
#include "../lib/Adafruit_BNO055/Adafruit_BNO055.h"
#include "../lib/Adafruit_BNO055/utility/imumaths.h"

//SDA brown
//SDL white

//green 13 GN
//orange 2 OR
//yellow 4 YL
//blue 12 BL
//purple 14 PR
//silver 15 GY

#define TOTAL_LED 64
#define PIN_NUM_NEOPIXEL_OUTPUT 4
#define PIN_DRIVE_MOTOR_LEFT 12
#define PIN_DRIVE_MOTOR_RIGHT 2
#define PIN_WEAPON_MOTOR 14
#define PIN_WEAPON_MOTOR_COPY 13
#define PIN_VOLTAGE_CHECK 27

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
#define DRIVE_POWER_MAX 100 //Range 0 to 100, 100 being full power
#define WEAPON_POWER_MAX 100 //Range 0 to 100, 100 being full power

#define BATTERY_STAGE_UNDEFINED -1
#define BATTERY_STAGE_DEAD 0
#define BATTERY_STAGE_CUTOFF_WEAPON 1
#define BATTERY_STAGE_UNDERHALF 2
#define BATTERY_STAGE_OVERHALF 3
#define BATTERY_STAGE_FULL 4
#define BATTERY_STAGE_OVERFULL 5

const float DETHROTTLE_FACTOR = 10;
const float DETHROTTLE_HARD_MINIMUM = 0.5;
const bool DETHROTTLE_WEAPON_ON_HARD_TURN = true;



#define TIME_TO_HOLD_SHUTDOWN_BUTTON_MILLIS 3000

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int ledType = 1; // 0: 8x8 neopixel, 1: 2x2, 2: single

int val;    // variable to read the value from the analog pin
int _analogPinValue = 0;

CRGB leds[TOTAL_LED];
float _neutralRange = 0.1;

int _driveLeftLEDs[5] = {39, 31, 23, 15, 7};
int _driveRightLEDs[5] = {32, 24, 16, 8, 0};

int _controlNeutralZone = 2;

float _currentLeftMotorSpeed = 0;
float _targetLeftMotorSpeed = 0;
float _currentRightMotorSpeed = 0;
float _targetRightMotorSpeed = 0;
float _currentWeaponMotorSpeed = 0;
float _targetWeaponMotorSpeed = 0;

float _rampupSpeed = 0.25;

bool _reverseAllDriveMotors = true; // RPB: This is the case of like if we have a gear 

Servo _driveMotorLeft;
Servo _driveMotorRight;
Servo _weaponMotor;
Servo _weaponMotorCopy;

sensors_event_t event;

int _delayLength = 10;

ulong _forceShutdownAccumulatedMillis = 0;
ulong _forceShutdownStartedMillis = 0;
ulong _lastUpdateMillis = 0;
ulong _accumulatedTimeSinceBatteryStageSwitched = 0;


ulong _timeoutThresholdMillis = 1000;
ulong _timeoutDeclaredStartedMillis = 0;
float _forceShutdownProgress = 0;
bool _forceShutdownTurnedOn = false;

bool _useIMU = true;
// from ControllerManager

float _leftVerticalValue = 0;
float _rightVerticalValue = 0;
float _leftTriggerValue = 0;
float _rightTriggerValue = 0;

int lastAx = 0;
int lastAy = 0;
int lastAz = 0;
int lastGz = 0;

int voltageResultRaw = 0;

float currentXOrientation = 0;

bool _turboOn = false;
bool _slowOn = false;

// from TranslationManager

const float SLOW_MULTIPLIER = 0.5;
const float TURBO_MULTIPLIER = 1;
const float DEFAULT_MULTIPLIER = 0.99;

float _leftMotorThrottle = 0;
float _rightMotorThrottle = 0;
float _weaponMotorThrottle = 0;

float voltagePerCell = 0;
const float voltageMinimum = 0; //BS 0
const float voltageCutoffDrive = 3.2; //BS1
const float voltageCutoffWeapon = 3.4; //BS2
const float voltageCutoffHalfway = 3.7; //BS3
const float voltageCutoffFull = 4.05; //BS4
const float voltageOverLimit = 4.25; //BS5

int currentBatteryStage = BATTERY_STAGE_UNDEFINED; 

const float _centerTrim = -3;
const float _centerTrimWeapon = 0;

bool _isUpsideDown = false;

bool _motorsAreAttached = false;

bool _perfectForwardEngaged = false;
float _perfectForwardStartAngle = 0;
float _perfectForwardCorrectionFactor = 0.8f;

bool _shouldShowTargetAngle = false;


bool _videoGameStyleDriveControls = true;
bool _currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank = false;
int _videoGameAngleDeltaDegrees = 0;

int currentJoystickAngle = 0;

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

void SetCenterLEDs(CRGB color)
{
    if (ledType == 0) // 8x8
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
    else if (ledType == 1) //2x2, 0 = main, 1 = right, 2 = left, 3= weapon
    {
        leds[0] = color;
    }
    else if (ledType == 2) // 1x1 single
    {
        leds[0] = color;
    }

    /*

    */
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

void SetWeaponMotorSpeed(float newSpeed)
{
    if (ledType == 0) // 8x8
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
    else if (ledType == 1) //2x2, 0 = main, 1 = right, 2 = left, 3= weapon
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
    else if (ledType == 2) // 1x1 single
    {

    }


    int newPowerToWrite = 90 + _centerTrimWeapon + int(newSpeed * (float) (90 * WEAPON_POWER_MAX / 100));

    _weaponMotor.write(180-newPowerToWrite);
    _weaponMotorCopy.write( newPowerToWrite);

}

bool IsTimedOut()
{
    return millis() > (_lastUpdateMillis + _timeoutThresholdMillis);
}


void DisplayAnglePoint(float angleToDisplay, CRGB color)
{
    if (ledType != 0)
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

void UpdateInputValues()
{

    /*
    Serial.println("Accelerometer data:");

    Serial.print(" ax:  ");
    Serial.print(Ps3.data.sensor.accelerometer.x, DEC);

    Serial.print(" ay: ");
    Serial.print(Ps3.data.sensor.accelerometer.y, DEC);

    Serial.print(" az: ");
    Serial.print(Ps3.data.sensor.accelerometer.z, DEC);

    Serial.print(" gz: ");
    Serial.print(Ps3.data.sensor.gyroscope.z, DEC);
*/
    _currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank = false;
    
    if (
            lastAx == Ps3.data.sensor.accelerometer.x &&
            lastAy == Ps3.data.sensor.accelerometer.y &&
            lastAz == Ps3.data.sensor.accelerometer.z &&
            lastGz == Ps3.data.sensor.gyroscope.z
            )
    {
        Serial.println("Controller is totally still- physically impossible. is it offline?");
    }
    else
    {
        _lastUpdateMillis = millis();
    }

    lastAx = Ps3.data.sensor.accelerometer.x;
    lastAy = Ps3.data.sensor.accelerometer.y;
    lastAz = Ps3.data.sensor.accelerometer.z;
    lastGz = Ps3.data.sensor.gyroscope.z;

    if (Ps3.data.button.select)
    {
        _forceShutdownAccumulatedMillis += _delayLength;

        _forceShutdownProgress = (float) _forceShutdownAccumulatedMillis / (float) TIME_TO_HOLD_SHUTDOWN_BUTTON_MILLIS;
    }
    else
    {
        _forceShutdownAccumulatedMillis = 0;
        _forceShutdownProgress = 0;
    }

    if (!Ps3.data.button.up && !Ps3.data.button.down)
    {
        _perfectForwardEngaged = false;
    }

    if (Ps3.data.button.left || Ps3.data.button.right || Ps3.data.button.up || Ps3.data.button.down)
    {
        float leftMixedValue = 0;
        float rightMixedValue = 0;


        if (Ps3.data.button.up)
        {


            if (!_perfectForwardEngaged)
            {
                _perfectForwardEngaged = true;
                _perfectForwardStartAngle = currentXOrientation;
            }
            else
            {
                Serial.print("startAngle: ");
                Serial.print(_perfectForwardStartAngle);
                Serial.print(", currentAngle: ");
                Serial.print(currentXOrientation);

                int angleDelta = (int) (currentXOrientation - _perfectForwardStartAngle + 180 + 360) % 360 - 180;

                Serial.print("angle diff:");
                Serial.print(angleDelta);

                // RPB: angle range is always 0 to 360

                if (angleDelta > 3)
                {
                    Serial.print("correcting towards right:");
                    leftMixedValue += 1;
                    rightMixedValue += 1 * _perfectForwardCorrectionFactor;
                    // too much to the left, turn right
                }
                else if (angleDelta < -3)
                {
                    Serial.print("correcting towards left");
                    leftMixedValue += 1 * _perfectForwardCorrectionFactor;
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


        if (Ps3.data.button.down)
        {

            if (!_perfectForwardEngaged)
            {
                _perfectForwardEngaged = true;
                _perfectForwardStartAngle = currentXOrientation;
            }
            else
            {
                Serial.print("startAngle: ");
                Serial.print(_perfectForwardStartAngle);
                Serial.print(", currentAngle: ");
                Serial.print(currentXOrientation);

                int angleDelta = (int) (currentXOrientation - _perfectForwardStartAngle + 180 + 360) % 360 - 180;

                Serial.print("angle diff:");
                Serial.print(angleDelta);

                // RPB: angle range is always 0 to 360

                if (angleDelta > 3)
                {
                    Serial.print("correcting towards right:");
                    leftMixedValue -= 1 * _perfectForwardCorrectionFactor;
                    rightMixedValue -= 1;
                    // too much to the left, turn right
                }
                else if (angleDelta < -3)
                {
                    Serial.print("correcting towards left");
                    leftMixedValue -= 1;
                    rightMixedValue -= 1 * _perfectForwardCorrectionFactor;
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

        if (Ps3.data.button.left)
        {
            _perfectForwardEngaged = false;
            leftMixedValue -= 1;
            rightMixedValue += 1;
        }

        if (Ps3.data.button.right)
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

        _leftVerticalValue = leftMixedValue;
        _rightVerticalValue = rightMixedValue;
    }
    else
    {
        if (_videoGameStyleDriveControls)
        {
            _currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank = true;
            _shouldShowTargetAngle = false;

            float leftMixedValue = 0;
            float rightMixedValue = 0;
            float rx = (float) Ps3.data.analog.stick.rx / (float) 128;
            float ry = -(float) Ps3.data.analog.stick.ry / (float) 128;
            float ly = -(float) Ps3.data.analog.stick.ly / (float) 128;

            if (rx == 0)
            {
                rx = 0.001;
            }

            if (abs(rx) > 0.5 || abs(ry) > 0.5)
            {
                _shouldShowTargetAngle = true;

                float angleStick = atan2(ry, rx) * RAD_TO_DEG + 90;

                currentJoystickAngle = angleStick;

                _videoGameAngleDeltaDegrees = (int) (currentXOrientation - angleStick + 360) % 360 - 180;

                Serial.print("targetAngle: ");
                Serial.print(currentJoystickAngle);
                Serial.print(", currentAngle: ");
                Serial.print(currentXOrientation);

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

            _leftVerticalValue = leftMixedValue;
            _rightVerticalValue = rightMixedValue;
        }
        else
        {
            // RPB: Do oldschool tank drive

            if (Ps3.data.analog.stick.ly < _controlNeutralZone && Ps3.data.analog.stick.ly > -_controlNeutralZone)
            {
                _leftVerticalValue = 0;
            }
            else
            {
                _leftVerticalValue = -(float) Ps3.data.analog.stick.ly / (float) 128;
            }

            if (Ps3.data.analog.stick.ry < _controlNeutralZone && Ps3.data.analog.stick.ry > -_controlNeutralZone)
            {
                _rightVerticalValue = 0;
            }
            else
            {
                _rightVerticalValue = -(float) Ps3.data.analog.stick.ry / (float) 128;
            }
        }


    }


    if (Ps3.data.analog.button.l2 < _controlNeutralZone)
    {
        _leftTriggerValue = 0;
    }
    else
    {
        _leftTriggerValue = (float) Ps3.data.analog.button.l2 / (float) 256;
    }

    if (Ps3.data.analog.button.r2 < _controlNeutralZone)
    {
        _rightTriggerValue = 0;
    }
    else
    {
        _rightTriggerValue = (float) Ps3.data.analog.button.r2 / (float) 256;
    }

    _slowOn = Ps3.data.button.l1;
    _turboOn = Ps3.data.button.r1;

    /*
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
    */

    if (!_isUpsideDown)
    {
        _targetLeftMotorSpeed = _leftMotorThrottle;
        _targetRightMotorSpeed = _rightMotorThrottle;
    }
    else
    {
        _targetRightMotorSpeed = -_leftMotorThrottle;
        _targetLeftMotorSpeed = -_rightMotorThrottle;
    }

    _targetWeaponMotorSpeed = _weaponMotorThrottle;
}

void UpdateThrottleValues()
{
    float throttleMultiplier = 0;

    if (_turboOn && !_slowOn)
    {
        throttleMultiplier = TURBO_MULTIPLIER;
    }
    else if (!_turboOn && _slowOn)
    {
        throttleMultiplier = SLOW_MULTIPLIER;
    }
    else
    {
        throttleMultiplier = DEFAULT_MULTIPLIER;
    }

    _leftMotorThrottle = _leftVerticalValue * throttleMultiplier;
    _rightMotorThrottle = _rightVerticalValue * throttleMultiplier;

    if (_leftTriggerValue < 0.05 && _rightTriggerValue < 0.05)
    {
        _weaponMotorThrottle = 0;
    }
    else
    {
        if (_leftTriggerValue > _rightTriggerValue)
        {
            // Left trigger is dominant
            _weaponMotorThrottle = (float) -1 * (float) (_leftTriggerValue);
        }
        else
        {
            _weaponMotorThrottle = (float) 1 * (_rightTriggerValue);
            // Right trigger is dominant
        }
    }


    if(currentBatteryStage == BATTERY_STAGE_DEAD)
    {
        _leftMotorThrottle = 0;
        _rightMotorThrottle = 0;
    }
    
    if(currentBatteryStage == BATTERY_STAGE_DEAD || currentBatteryStage == BATTERY_STAGE_CUTOFF_WEAPON)
    {
        _weaponMotorThrottle = 0;
    }
}

void SetDriveMotorSpeed(float newSpeed, bool isLeftMotor, int motorLeds[])
{

    if (newSpeed < _neutralRange && newSpeed > -_neutralRange)
    {
        newSpeed = 0;
    }

    if (ledType == 0) // 8x8
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
            if (!IsTimedOut())
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
    else if (ledType == 1) //2x2, 0 = main, 1 = right, 2 = left, 3= weapon
    {
        int ledNumber = 1;
        if(isLeftMotor)
        {
            ledNumber = 2;
        }
        
        if (!_motorsAreAttached)
        {
            leds[ledNumber] = CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS);
        }
        else
        {
            if (!IsTimedOut())
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
    else if (ledType == 2) // 1x1 single
    {

    }


    if (_reverseAllDriveMotors)
    {
        newSpeed = -newSpeed;
    }

    if (isLeftMotor)
    {
        int newPowerToWrite = 90 + _centerTrim - int(newSpeed * (float) (90 * DRIVE_POWER_MAX / 100));
        _driveMotorLeft.write(newPowerToWrite);
    }
    else
    {
        int newPowerToWrite = 90 + _centerTrim - int(newSpeed * (float) (90 * DRIVE_POWER_MAX / 100));
        _driveMotorRight.write(newPowerToWrite);
    }
}


void Start()
{
    pinMode(PIN_NUM_NEOPIXEL_OUTPUT, OUTPUT);

    Ps3.begin("00:02:72:3F:5F:02");
    Ps3.getAddress();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    ESP32PWM::allocateTimer(4);
    ESP32PWM::allocateTimer(5);
    Serial.begin(115200);

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        _useIMU = false;
    }

    FastLED.addLeds<NEOPIXEL, PIN_NUM_NEOPIXEL_OUTPUT>(leds, TOTAL_LED);

    FastLED.show();
    
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

    if (_useIMU)
    {
        bno.setExtCrystalUse(true);
    }

    _driveMotorLeft.setPeriodHertz(50);
    _driveMotorRight.setPeriodHertz(50);
    _weaponMotor.setPeriodHertz(50);
    _weaponMotorCopy.setPeriodHertz(50);

    delay(50);
    
    SetWeaponMotorSpeed(0);
    SetDriveMotorSpeed(0, true, _driveLeftLEDs);
    SetDriveMotorSpeed(0, false, _driveRightLEDs);
    FastLED.show();

}

float AttenuateWeaponThrottleBasedOnAngleDelta(float inputTargetWeaponSpeed)
{

    if (!DETHROTTLE_WEAPON_ON_HARD_TURN)
    {
        return inputTargetWeaponSpeed;
    }
    float dethrottleMultiplier = 1;
    
    
    if(_currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank)
    {
        dethrottleMultiplier = 1 - abs((float)_videoGameAngleDeltaDegrees / (float)180 * (float)DETHROTTLE_FACTOR);
        
        if(dethrottleMultiplier > 1)
        {
            dethrottleMultiplier = 1;
        }
        
        if(dethrottleMultiplier < DETHROTTLE_HARD_MINIMUM)
        {
            dethrottleMultiplier = DETHROTTLE_HARD_MINIMUM;
        }
    }
    else
    {
        dethrottleMultiplier = 1;
    }
    
    return inputTargetWeaponSpeed * dethrottleMultiplier;
}

void UpdateESCsToTargetValues()
{
    if (_currentLeftMotorSpeed != _targetLeftMotorSpeed)
    {
        if (abs(_currentLeftMotorSpeed - _targetLeftMotorSpeed) < _rampupSpeed)
        {
            _currentLeftMotorSpeed = _targetLeftMotorSpeed;
        }
        else if (_currentLeftMotorSpeed < _targetLeftMotorSpeed)
        {
            _currentLeftMotorSpeed += _rampupSpeed;
        }
        else if (_currentLeftMotorSpeed > _targetLeftMotorSpeed)
        {
            _currentLeftMotorSpeed -= _rampupSpeed;
        }
    }

    /*
    if(!_isUpsideDown)
    {
        SetDriveMotorSpeed(_currentLeftMotorSpeed, true, _driveLeftLEDs);
    }
    else
    {
        SetDriveMotorSpeed(-_currentLeftMotorSpeed, false, _driveLeftLEDs);
    } */

    SetDriveMotorSpeed(_currentLeftMotorSpeed, true, _driveLeftLEDs);

    if (_currentRightMotorSpeed != _targetRightMotorSpeed)
    {
        if (abs(_currentRightMotorSpeed - _targetRightMotorSpeed) < _rampupSpeed)
        {
            _currentRightMotorSpeed = _targetRightMotorSpeed;
        }
        else if (_currentRightMotorSpeed < _targetRightMotorSpeed)
        {
            _currentRightMotorSpeed += _rampupSpeed;
        }
        else if (_currentRightMotorSpeed > _targetRightMotorSpeed)
        {
            _currentRightMotorSpeed -= _rampupSpeed;
        }
    }

    /*
    if(!_isUpsideDown)
    {
        SetDriveMotorSpeed(_currentRightMotorSpeed, false, _driveRightLEDs);
    }
    else
    {
        SetDriveMotorSpeed(-_currentRightMotorSpeed, true, _driveRightLEDs);
    } */

    SetDriveMotorSpeed(_currentRightMotorSpeed, false, _driveRightLEDs);


    _currentWeaponMotorSpeed = AttenuateWeaponThrottleBasedOnAngleDelta(_targetWeaponMotorSpeed);
    SetWeaponMotorSpeed(_currentWeaponMotorSpeed);
}


void GetAndPrintOrientation()
{
    if (!_useIMU)
    {
        return;
    }


    bno.getEvent(&event);

    /* Display the floating point data */

    /*
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
     */

    if (event.orientation.z > 90 || event.orientation.z < -90)
    {
        _isUpsideDown = true;
    }
    else
    {
        _isUpsideDown = false;
    }

    if (event.orientation.z > 90 || event.orientation.z < -90)
    {
        currentXOrientation = event.orientation.x;
    }

    currentXOrientation = 360 - event.orientation.x;

    if (_shouldShowTargetAngle)
    {
        DisplayAnglePoint(currentJoystickAngle + currentXOrientation, CRGB::Green);
    }

    DisplayAnglePoint(180 + currentXOrientation, CRGB::White);

}


void OnForceShutDown(ulong shutdownEventReasonStartTime)
{
    AttachMotors(false);

    _targetLeftMotorSpeed = 0;
    _targetRightMotorSpeed = 0;
    _targetWeaponMotorSpeed = 0;
    UpdateESCsToTargetValues();
}

void OnTimeout()
{
    Serial.println("We are timed out...");
    Serial.print(" Timeout started: ");
    Serial.print(_timeoutDeclaredStartedMillis);

    _timeoutDeclaredStartedMillis = _lastUpdateMillis + _timeoutThresholdMillis;

    if (_timeoutDeclaredStartedMillis + 5000 < millis())
    {
        ESP.restart();
    }

    OnForceShutDown(_timeoutDeclaredStartedMillis);
}

void UpdateCenterLEDColors()
{
    SetCenterLEDs(CRGB::Black);
    if (ledType == 0)
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

    if (ledType == 0)
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
    else if (ledType == 1)
    {
        if (_forceShutdownProgress > 0.01f && _forceShutdownProgress < 0.99f && !_forceShutdownTurnedOn)
        {
            CRGB color = CHSV(HUE_DISCONNECTED, 255, (float) _forceShutdownProgress * (float) MAX_BRIGHTNESS + 25);
            leds[0] = color;
        }
        else
        {
            if(currentBatteryStage == BATTERY_STAGE_DEAD)
            {
                if(millis() % 666 < 333)
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
            else if(currentBatteryStage == BATTERY_STAGE_CUTOFF_WEAPON)
            {
                CRGB color = CHSV(HUE_RED, 255, MAX_BRIGHTNESS);
                leds[0] = color;
            }
            else if(currentBatteryStage == BATTERY_STAGE_UNDERHALF)
            {
                CRGB color = CHSV(HUE_YELLOW, 255, MAX_BRIGHTNESS);
                leds[0] = color;
            }
            else if(currentBatteryStage == BATTERY_STAGE_OVERHALF)
            {
                CRGB color = CHSV(HUE_GREEN, 255, MAX_BRIGHTNESS);
                leds[0] = color;
            }
            else if(currentBatteryStage == BATTERY_STAGE_FULL)
            {
                if(millis() % 400 < 200)
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
            else if (currentBatteryStage == BATTERY_STAGE_OVERFULL)
            {
                if(millis() % 200 < 100)
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
            else if (currentBatteryStage == BATTERY_STAGE_UNDEFINED)
            {
                CRGB color = CHSV(HUE_RED, 255, 0);
                leds[0] = color;
            }
        }


        //const float voltageCutoffDrive = 3.2;
        //const float voltageCutoffWeapon = 3.4;
        //const float voltageCutoffHalfway = 3.8;
        //const float voltageOverLimit = 4.25;
        

    }

    if (IsTimedOut() && (millis() - _timeoutThresholdMillis) % 1000 < 500)
    {
        SetCenterLEDs(CHSV(HUE_BLUE, 255, MAX_BRIGHTNESS));
    }

    if (_forceShutdownTurnedOn && (millis() - _forceShutdownStartedMillis) % 200 < 100)
    {
        SetCenterLEDs(CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS));
    }

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

void ReadVoltage()
{
    voltageResultRaw = analogRead(PIN_VOLTAGE_CHECK);
    Serial.println();
    
    Serial.print(voltageResultRaw);
    Serial.print("rawV, ");
    
    float voltageReal = 0.00353 * (float)voltageResultRaw + 3.88894;
    Serial.print(voltageReal);
    Serial.print("v, (");
    voltagePerCell = voltageReal/(float)4.0;
    Serial.print(voltagePerCell);
    Serial.print("v per cell)");
    Serial.println("v");

    int possibleNewBatteryStage = 0;
    
    if(voltagePerCell < voltageCutoffDrive)
    {
        possibleNewBatteryStage = BATTERY_STAGE_DEAD;
    }
    else if(voltagePerCell >= voltageCutoffDrive && voltagePerCell < voltageCutoffWeapon)
    {
        possibleNewBatteryStage = BATTERY_STAGE_CUTOFF_WEAPON;
    }
    else if(voltagePerCell >= voltageCutoffWeapon && voltagePerCell < voltageCutoffHalfway)
    {
        possibleNewBatteryStage = BATTERY_STAGE_UNDERHALF;
    }
    else if(voltagePerCell >= voltageCutoffHalfway && voltagePerCell < voltageCutoffFull)
    {
        possibleNewBatteryStage = BATTERY_STAGE_OVERHALF;
    }
    else if(voltagePerCell >= voltageCutoffFull && voltagePerCell < voltageOverLimit)
    {
        possibleNewBatteryStage = BATTERY_STAGE_FULL;
    }
    else
    {
        possibleNewBatteryStage = BATTERY_STAGE_OVERFULL;
    }

    if(currentBatteryStage == BATTERY_STAGE_UNDEFINED)
    {
        currentBatteryStage = possibleNewBatteryStage;
    }
    
    if(possibleNewBatteryStage != currentBatteryStage)
    {
        _accumulatedTimeSinceBatteryStageSwitched += _delayLength;
    }
    else
    {
        _accumulatedTimeSinceBatteryStageSwitched = 0;
    }
    
    if(_accumulatedTimeSinceBatteryStageSwitched > 1000)
    {
        currentBatteryStage = possibleNewBatteryStage;
        _accumulatedTimeSinceBatteryStageSwitched = 0;
    }
}

void Update()
{
    ReadVoltage();
    
    
    for (int i = 0; i < 64; ++i)
    {
        leds[i] = CRGB::Black;
    }

    SetCenterLEDs(CRGB::Black);


    UpdateInputValues();
    UpdateThrottleValues();

    UpdateShutdownTimer();

    if (_forceShutdownTurnedOn)
    {
        OnForceShutDown(_forceShutdownStartedMillis);
    }
    else
    {
        if (!IsTimedOut()) // Means we got fresh data
        {
            if (!_motorsAreAttached)
            {
                AttachMotors(true);
            }

            UpdateESCsToTargetValues();
        }
        else// we timed out...
        {
            OnTimeout();
        }
    }

    UpdateCenterLEDColors();

    GetAndPrintOrientation();
}// lol unity style

void setup()
{
    Start();
}

void loop()
{
    Update();
    FastLED.show();
    delay(_delayLength);
}
