#include <Arduino.h>
#include "../lib/FastLED-3.4.0/src/FastLED.h"
#include "../lib/ESP32Servo-0.10.0/src/ESP32Servo.h"
#include "../lib/ArduinoJson-v6.19.1.h"
#include "../lib/PS3_Controller_Host/src/Ps3Controller.h"

#define TOTAL_LED 17
#define PIN_NUM_NEOPIXEL_OUTPUT 0
#define PIN_DRIVE_MOTOR_LEFT 4
#define PIN_DRIVE_MOTOR_RIGHT 2
#define PIN_WEAPON_MOTOR 15

#define LED_WEAPON_R3 5
#define LED_WEAPON_R2 6
#define LED_WEAPON_R1 7
#define LED_WEAPON_C 8
#define LED_WEAPON_L1 9
#define LED_WEAPON_L2 10
#define LED_WEAPON_L3 11

#define HUE_FORWARD 96
#define HUE_REVERSE 0
#define HUE_NEUTRAL 40
#define STARTING_BRIGHTNESS 20
#define DRIVE_POWER_MAX 100 //Range 0 to 100, 100 being full power
#define WEAPON_POWER_MAX 100 //Range 0 to 100, 100 being full power

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

CRGB leds[TOTAL_LED];
int val;    // variable to read the value from the analog pin
float neutralRange = 0.1;

int DriveLeftLEDs[5] = {4, 3, 2, 1, 0};
int DriveRightLEDs[5] = {12, 13, 14, 15, 16};

int controlNeutralZone = 5;

float currentLeftMotorSpeed = 0;
float targetLeftMotorSpeed = 0;
float currentRightMotorSpeed = 0;
float targetRightMotorSpeed = 0;
float currentWeaponMotorSpeed = 0;
float targetWeaponMotorSpeed = 0;

float rampupSpeed = 1;

Servo DriveMotorLeft;
Servo DriveMotorRight;
Servo WeaponMotor;

ulong lastUpdateMillis = 0;
ulong updateTimeoutMillis = 5000;

// from ControllerManager

float _leftVerticalValue = 0;
float _rightVerticalValue = 0;
float _leftTriggerValue = 0;
float _rightTriggerValue = 0;

bool _turboOn = false;
bool _slowOn = false;

// from TranslationManager

const float SLOW_MULTIPLIER = 0.2;
const float TURBO_MULTIPLIER = 1;
const float DEFAULT_MULTIPLIER = 0.5;

float _leftMotorThrottle = 0;
float _rightMotorThrottle = 0;
float _weaponMotorThrottle = 0;


void SetAllLEDs(CRGB color)
{
    for (int i = 0; i < TOTAL_LED; ++i)
    {
        leds[i] = color;
    }
}

int GetBrightnessForSpeed(float speed)
{
    if (speed < 0)
    {
        speed = -speed;
    }

    int brightnessDelta = 255 - STARTING_BRIGHTNESS;
    return STARTING_BRIGHTNESS + (int) (speed * (float) brightnessDelta);
}

void SetWeaponMotorSpeed(float newSpeed)
{
    int brightness = GetBrightnessForSpeed(newSpeed);
    if (newSpeed > 0.666)
    {
        leds[LED_WEAPON_C] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_L1] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_R1] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_L2] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_R2] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_L3] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_R3] = CHSV(HUE_FORWARD, 255, brightness);
    }
    else if (newSpeed > 0.333 && newSpeed <= 0.666)
    {
        leds[LED_WEAPON_C] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_L1] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_R1] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_L2] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_R2] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_L3] = CRGB::Black;
        leds[LED_WEAPON_R3] = CRGB::Black;
    }
    else if (newSpeed > 0 && newSpeed <= 0.333)
    {
        leds[LED_WEAPON_C] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_L1] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_R1] = CHSV(HUE_FORWARD, 255, brightness);
        leds[LED_WEAPON_L2] = CRGB::Black;
        leds[LED_WEAPON_R2] = CRGB::Black;
        leds[LED_WEAPON_L3] = CRGB::Black;
        leds[LED_WEAPON_R3] = CRGB::Black;
    }
    else if (newSpeed == 0)
    {
        leds[LED_WEAPON_C] = CHSV(HUE_NEUTRAL, 255, 255);
        leds[LED_WEAPON_L1] = CHSV(HUE_NEUTRAL, 255, 255);
        leds[LED_WEAPON_R1] = CHSV(HUE_NEUTRAL, 255, 255);
        leds[LED_WEAPON_L2] = CRGB::Black;
        leds[LED_WEAPON_R2] = CRGB::Black;
        leds[LED_WEAPON_L3] = CRGB::Black;
        leds[LED_WEAPON_R3] = CRGB::Black;
    }
    else if (newSpeed < 0 && newSpeed >= -0.333)
    {
        leds[LED_WEAPON_C] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_L1] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_R1] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_L2] = CRGB::Black;
        leds[LED_WEAPON_R2] = CRGB::Black;
        leds[LED_WEAPON_L3] = CRGB::Black;
        leds[LED_WEAPON_R3] = CRGB::Black;
    }
    else if (newSpeed < -0.333 && newSpeed >= -0.666)
    {
        leds[LED_WEAPON_C] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_L1] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_R1] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_L2] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_R2] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_L3] = CRGB::Black;
        leds[LED_WEAPON_R3] = CRGB::Black;
    }
    if (newSpeed < -0.666)
    {
        leds[LED_WEAPON_C] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_L1] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_R1] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_L2] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_R2] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_L3] = CHSV(HUE_REVERSE, 255, brightness);
        leds[LED_WEAPON_R3] = CHSV(HUE_REVERSE, 255, brightness);
    }

    int newPowerToWrite = 90 + int(newSpeed * (float) (90 * WEAPON_POWER_MAX / 100));

    WeaponMotor.write(newPowerToWrite);

}

bool IsTimedOut()
{
    return millis() > (lastUpdateMillis + updateTimeoutMillis);
}

void UpdateInputValues()
{
    if(Ps3.data.analog.stick.ly < controlNeutralZone && Ps3.data.analog.stick.ly > -controlNeutralZone)
    {
        _leftVerticalValue = 0;
    }
    else
    {
        _leftVerticalValue = -(float)Ps3.data.analog.stick.ly/(float)128;
    }
    
    if(Ps3.data.analog.stick.ry < controlNeutralZone && Ps3.data.analog.stick.ry > -controlNeutralZone)
    {
        _rightVerticalValue = 0;
    }
    else
    {
        _rightVerticalValue = -(float)Ps3.data.analog.stick.ry/(float)128;
    }
    
    if(Ps3.data.analog.button.l2 < controlNeutralZone)
    {
        _leftTriggerValue = 0;
    }
    else
    {
        _leftTriggerValue = (float)Ps3.data.analog.button.l2/(float)256;
    }

    if(Ps3.data.analog.button.r2 < controlNeutralZone)
    {
        _rightTriggerValue = 0;
    }
    else
    {
        _rightTriggerValue = (float)Ps3.data.analog.button.r2/(float)256;
    }
    
    _slowOn = Ps3.data.button.l1;
    _turboOn = Ps3.data.button.r1;

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

    targetLeftMotorSpeed = _leftMotorThrottle;
    targetRightMotorSpeed = _rightMotorThrottle;
    targetWeaponMotorSpeed = _weaponMotorThrottle;
}

void UpdateThrottleValues()
{
    float throttleMultiplier = 0;

    if (_turboOn && !_slowOn)
    {
        throttleMultiplier = TURBO_MULTIPLIER;
    }
    else if(!_turboOn && _slowOn)
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
            _weaponMotorThrottle = (float)-1*(float)(_leftTriggerValue);
        }
        else
        {
            _weaponMotorThrottle = (float)1*(_rightTriggerValue);
            // Right trigger is dominant
        }
    }
}

void SetDriveMotorSpeed(float newSpeed, bool isLeftMotor, int motorLeds[])
{
    if (newSpeed < neutralRange && newSpeed > -neutralRange)
    {
        newSpeed = 0;
    }

    if (!IsTimedOut())
    {
        int brightness = GetBrightnessForSpeed(newSpeed);

        if (newSpeed > 0.666)
        {
            leds[motorLeds[0]] = CHSV(HUE_FORWARD, 255, brightness);
            leds[motorLeds[1]] = CHSV(HUE_FORWARD, 255, brightness);
            leds[motorLeds[2]] = CHSV(HUE_FORWARD, 255, brightness);
            leds[motorLeds[3]] = CRGB::Black;
            leds[motorLeds[4]] = CRGB::Black;
        }
        else if (newSpeed > 0.333 && newSpeed <= 0.666)
        {
            leds[motorLeds[0]] = CRGB::Black;
            leds[motorLeds[1]] = CHSV(HUE_FORWARD, 255, brightness);
            leds[motorLeds[2]] = CHSV(HUE_FORWARD, 255, brightness);
            leds[motorLeds[3]] = CRGB::Black;
            leds[motorLeds[4]] = CRGB::Black;
        }
        else if (newSpeed > 0 && newSpeed <= 0.333)
        {
            leds[motorLeds[0]] = CRGB::Black;
            leds[motorLeds[1]] = CRGB::Black;
            leds[motorLeds[2]] = CHSV(HUE_FORWARD, 255, brightness);
            leds[motorLeds[3]] = CRGB::Black;
            leds[motorLeds[4]] = CRGB::Black;
        }
        else if (newSpeed == 0)
        {
            leds[motorLeds[0]] = CRGB::Black;
            leds[motorLeds[1]] = CRGB::Black;
            leds[motorLeds[2]] = CHSV(HUE_NEUTRAL, 255, 255);
            leds[motorLeds[3]] = CRGB::Black;
            leds[motorLeds[4]] = CRGB::Black;
        }
        else if (newSpeed >= -0.333 && newSpeed < 0)
        {
            leds[motorLeds[0]] = CRGB::Black;
            leds[motorLeds[1]] = CRGB::Black;
            leds[motorLeds[2]] = CHSV(HUE_REVERSE, 255, brightness);
            leds[motorLeds[3]] = CRGB::Black;
            leds[motorLeds[4]] = CRGB::Black;
        }
        else if (newSpeed >= -0.666 && newSpeed < -0.333)
        {
            leds[motorLeds[0]] = CRGB::Black;
            leds[motorLeds[1]] = CRGB::Black;
            leds[motorLeds[2]] = CHSV(HUE_REVERSE, 255, brightness);
            leds[motorLeds[3]] = CHSV(HUE_REVERSE, 255, brightness);
            leds[motorLeds[4]] = CRGB::Black;
        }
        else if (newSpeed < -0.666)
        {
            leds[motorLeds[0]] = CRGB::Black;
            leds[motorLeds[1]] = CRGB::Black;
            leds[motorLeds[2]] = CHSV(HUE_REVERSE, 255, brightness);
            leds[motorLeds[3]] = CHSV(HUE_REVERSE, 255, brightness);
            leds[motorLeds[4]] = CHSV(HUE_REVERSE, 255, brightness);
        }
    }

    if (isLeftMotor)
    {
        int newPowerToWrite = 90 - int(newSpeed * (float) (90 * DRIVE_POWER_MAX / 100));
        DriveMotorLeft.write(newPowerToWrite);
    }
    else
    {
        int newPowerToWrite = 90 + int(newSpeed * (float) (90 * DRIVE_POWER_MAX / 100));
        DriveMotorRight.write(newPowerToWrite);
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

    FastLED.addLeds<NEOPIXEL, PIN_NUM_NEOPIXEL_OUTPUT>(leds, TOTAL_LED);

    SetAllLEDs(CRGB::Purple);

    FastLED.show();
    delay(500);
    DriveMotorLeft.setPeriodHertz(50);
    DriveMotorRight.setPeriodHertz(50);
    WeaponMotor.setPeriodHertz(50);
    DriveMotorLeft.attach(PIN_DRIVE_MOTOR_LEFT, 1000, 2000);
    DriveMotorRight.attach(PIN_DRIVE_MOTOR_RIGHT, 1000, 2000);
    WeaponMotor.attach(PIN_WEAPON_MOTOR, 1000, 2000);

    for (int i = 0; i < 3; ++i)
    {
        SetAllLEDs(CRGB::White);
        FastLED.show();
        delay(166);

        SetAllLEDs(CRGB::Black);
        FastLED.show();
        delay(166);
    }

    SetWeaponMotorSpeed(0);
    SetDriveMotorSpeed(0, true, DriveLeftLEDs);
    SetDriveMotorSpeed(0, false, DriveRightLEDs);
    FastLED.show();

}

void UpdateESCsToTargetValues()
{
    if (currentLeftMotorSpeed != targetLeftMotorSpeed)
    {
        if (abs(currentLeftMotorSpeed - targetLeftMotorSpeed) < rampupSpeed)
        {
            currentLeftMotorSpeed = targetLeftMotorSpeed;
        }
        else if (currentLeftMotorSpeed < targetLeftMotorSpeed)
        {
            currentLeftMotorSpeed += rampupSpeed;
        }
        else if (currentLeftMotorSpeed > targetLeftMotorSpeed)
        {
            currentLeftMotorSpeed -= rampupSpeed;
        }
    }

    SetDriveMotorSpeed(currentLeftMotorSpeed, true, DriveLeftLEDs);

    if (currentRightMotorSpeed != targetRightMotorSpeed)
    {
        if (abs(currentRightMotorSpeed - targetRightMotorSpeed) < rampupSpeed)
        {
            currentRightMotorSpeed = targetRightMotorSpeed;
        }
        else if (currentRightMotorSpeed < targetRightMotorSpeed)
        {
            currentRightMotorSpeed += rampupSpeed;
        }
        else if (currentRightMotorSpeed > targetRightMotorSpeed)
        {
            currentRightMotorSpeed -= rampupSpeed;
        }
    }

    SetDriveMotorSpeed(currentRightMotorSpeed, false, DriveRightLEDs);

    currentWeaponMotorSpeed = targetWeaponMotorSpeed;
    SetWeaponMotorSpeed(currentWeaponMotorSpeed);
}

void OnTimeout()
{
    Serial.print("Timed out. Resetting all motors.");
    targetLeftMotorSpeed = 0;
    targetRightMotorSpeed = 0;
    targetWeaponMotorSpeed = 0;
    SetDriveMotorSpeed(0, true, DriveLeftLEDs);
    SetDriveMotorSpeed(0, false, DriveRightLEDs);
    SetWeaponMotorSpeed(0);

    if ((millis() - lastUpdateMillis) % 2000 < 1000)
    {
        SetAllLEDs(CRGB::Blue);
    }
    else
    {
        SetAllLEDs(CRGB::Black);
    }

}

void Update()
{
    SetAllLEDs(CRGB::Black);


    if (Ps3.isConnected())
    {
        UpdateInputValues();
        UpdateThrottleValues();
        
        lastUpdateMillis = millis();
    }
    
    if (!IsTimedOut()) // Means we got fresh data
    {
        UpdateESCsToTargetValues();
    }
    else // we timed out...
    {
        OnTimeout();
    }

}// lol unity style

void setup()
{
    Start();
}

void loop()
{
    Update();
    FastLED.show();
    delay(10);
}
