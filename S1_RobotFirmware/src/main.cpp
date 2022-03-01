#include <Arduino.h>
#include "../lib/FastLED-3.4.0/src/FastLED.h"
#include "../lib/ESP32Servo-0.10.0/src/ESP32Servo.h"
#include "../lib/ArduinoJson-v6.19.1.h"
#include "../lib/PS3_Controller_Host/src/Ps3Controller.h"

#include <Wire.h>
#include "../lib/Adafruit_Unified_Sensor/Adafruit_Sensor.h"
#include "../lib/Adafruit_BNO055/Adafruit_BNO055.h"
#include "../lib/Adafruit_BNO055/utility/imumaths.h"

//orange 15- neopixel
//yellow 2 - left
//green 0 
//blue 4 - right
//purple 12
//grey 14 

#define TOTAL_LED 64
#define PIN_NUM_NEOPIXEL_OUTPUT 15
#define PIN_DRIVE_MOTOR_LEFT 2
#define PIN_DRIVE_MOTOR_RIGHT 4
#define PIN_WEAPON_MOTOR 12

#define LED_WEAPON_R3 35
#define LED_WEAPON_R2 43
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
#define DRIVE_POWER_MAX 66 //Range 0 to 100, 100 being full power
#define WEAPON_POWER_MAX 100 //Range 0 to 100, 100 being full power

#define TIME_TO_HOLD_SHUTDOWN_BUTTON_MILLIS 3000

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

Adafruit_BNO055 bno = Adafruit_BNO055(55);
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

int _delayLength = 10;

ulong _forceShutdownAccumulatedMillis = 0;
ulong _forceShutdownStartedMillis = 0;
ulong _lastUpdateMillis = 0;

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

bool _turboOn = false;
bool _slowOn = false;

// from TranslationManager

const float SLOW_MULTIPLIER = 0.333;
const float TURBO_MULTIPLIER = 1;
const float DEFAULT_MULTIPLIER = 0.666;

float _leftMotorThrottle = 0;
float _rightMotorThrottle = 0;
float _weaponMotorThrottle = 0;

const float _centerTrim = -3;

bool _isUpsideDown = false;

bool _motorsAreAttached = false;

void AttachMotors(bool shouldAttachMotors)
{
    if (shouldAttachMotors)
    {
        _driveMotorLeft.attach(PIN_DRIVE_MOTOR_LEFT, 1000, 2000);
        _driveMotorRight.attach(PIN_DRIVE_MOTOR_RIGHT, 1000, 2000);
        _weaponMotor.attach(PIN_WEAPON_MOTOR, 1000, 2000);
    }
    else
    {
        _driveMotorLeft.detach();
        _driveMotorRight.detach();
        _weaponMotor.detach();
    }

    _motorsAreAttached = shouldAttachMotors;
}

void SetCenterLEDs(CRGB color)
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
    int newPowerToWrite = 90 + _centerTrim + int(newSpeed * (float) (90 * WEAPON_POWER_MAX / 100));

    _weaponMotor.write(newPowerToWrite);

}

bool IsTimedOut()
{
    return millis() > (_lastUpdateMillis + _timeoutThresholdMillis);
}

void UpdateInputValues()
{

    Serial.println("Accelerometer data:");

    Serial.print(" ax:  ");
    Serial.print(Ps3.data.sensor.accelerometer.x, DEC);

    Serial.print(" ay: ");
    Serial.print(Ps3.data.sensor.accelerometer.y, DEC);

    Serial.print(" az: ");
    Serial.print(Ps3.data.sensor.accelerometer.z, DEC);

    Serial.print(" gz: ");
    Serial.print(Ps3.data.sensor.gyroscope.z, DEC);

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

    if (Ps3.data.button.triangle)
    {
        _forceShutdownAccumulatedMillis += _delayLength;

        _forceShutdownProgress = (float) _forceShutdownAccumulatedMillis / (float) TIME_TO_HOLD_SHUTDOWN_BUTTON_MILLIS;
    }
    else
    {
        _forceShutdownAccumulatedMillis = 0;
        _forceShutdownProgress = 0;
    }

    if (Ps3.data.button.left || Ps3.data.button.right || Ps3.data.button.up || Ps3.data.button.down)
    {
        float leftMixedValue = 0;
        float rightMixedValue = 0;


        if (Ps3.data.button.up)
        {
            leftMixedValue += 1;
            rightMixedValue += 1;
        }

        if (Ps3.data.button.down)
        {
            leftMixedValue -= 1;
            rightMixedValue -= 1;
        }

        if (Ps3.data.button.left)
        {
            leftMixedValue -= 1;
            rightMixedValue += 1;
        }

        if (Ps3.data.button.right)
        {
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
}

void SetDriveMotorSpeed(float newSpeed, bool isLeftMotor, int motorLeds[])
{

    if (newSpeed < _neutralRange && newSpeed > -_neutralRange)
    {
        newSpeed = 0;
    }

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
}

void Start()
{
    pinMode(PIN_NUM_NEOPIXEL_OUTPUT, OUTPUT);

    Ps3.begin("00:02:72:3F:5F:02");

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    Serial.begin(115200);

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        _useIMU = false;
    }

    FastLED.addLeds<NEOPIXEL, PIN_NUM_NEOPIXEL_OUTPUT>(leds, TOTAL_LED);

    SetCenterLEDs(CHSV(HUE_PURPLE, 255, MAX_BRIGHTNESS));

    FastLED.show();
    delay(500);

    if(_useIMU)
    {
        bno.setExtCrystalUse(true);
    }

    _driveMotorLeft.setPeriodHertz(50);
    _driveMotorRight.setPeriodHertz(50);
    _weaponMotor.setPeriodHertz(50);


    for (int i = 0; i < 3; ++i)
    {
        SetCenterLEDs(CHSV(HUE_NEUTRAL, 0, MAX_BRIGHTNESS));
        FastLED.show();
        delay(166);

        SetCenterLEDs(CRGB::Black);
        FastLED.show();
        delay(166);
    }

    SetWeaponMotorSpeed(0);
    SetDriveMotorSpeed(0, true, _driveLeftLEDs);
    SetDriveMotorSpeed(0, false, _driveRightLEDs);
    FastLED.show();

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


    _currentWeaponMotorSpeed = _targetWeaponMotorSpeed;
    SetWeaponMotorSpeed(_currentWeaponMotorSpeed);
}


void GetAndPrintOrientation()
{
    if(!_useIMU)
    {
        return;
    }
    
    sensors_event_t event;
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

    if (_timeoutDeclaredStartedMillis + 7500 < millis())
    {
        ESP.restart();
    }

    OnForceShutDown(_timeoutDeclaredStartedMillis);
}

void UpdateCenterLEDColors()
{
    SetCenterLEDs(CRGB::Black);

    if(_useIMU)
    {
        if(!_isUpsideDown)
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

void Update()
{
    SetCenterLEDs(CRGB::Black);

    GetAndPrintOrientation();

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
