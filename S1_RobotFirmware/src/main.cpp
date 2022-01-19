#include <Arduino.h>
#include "../lib/FastLED-3.4.0/src/FastLED.h"
#include "../lib/ESP32Servo-0.10.0/src/ESP32Servo.h"
#include "BluetoothSerial.h"
#include "../lib/ArduinoJson-v6.19.1.h"

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
#define POWER 100 //Range 0 to 100, 100 being full power


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

CRGB leds[TOTAL_LED];
int val;    // variable to read the value from the analog pin
float neutralRange = 0.1;
String inputString = "";
BluetoothSerial SerialBT;

int DriveLeftLEDs[5] = {4, 3, 2, 1, 0};
int DriveRightLEDs[5] = {12, 13, 14, 15, 16};

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

char receivedChar;// received value will be stored as CHAR in this variable

ulong  lastUpdateMillis = 0;
ulong updateTimeoutMillis = 5000;


char JSONMessage[] = "{\"l\": 0.1, \"r\": -0.3, \"w\": 0.5}";
StaticJsonDocument<300> jsonDocument;


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

    int weaponPower = 180 * newSpeed;

    SerialBT.print("Setting Weapon Motor to :");// write on BT app
    SerialBT.println(weaponPower);// write on BT app      
    WeaponMotor.write(weaponPower);

}

bool IsTimedOut()
{
    return millis() > (lastUpdateMillis + updateTimeoutMillis);
}

void SetDriveMotorSpeed(float newSpeed, bool isLeftMotor, int motorLeds[])
{
    if (newSpeed < neutralRange && newSpeed > -neutralRange)
    {
        newSpeed = 0;
    }
 
    if(!IsTimedOut())
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
        int newPowerToWrite = 90 - int(newSpeed * (float) (90 * POWER / 100));
        SerialBT.print("Setting Left Drive to :");// write on BT app
        SerialBT.println(newPowerToWrite);// write on BT app      
        DriveMotorLeft.write(newPowerToWrite);
    }
    else
    {
        int newPowerToWrite = 90 + int(newSpeed * (float) (90 * POWER / 100));
        SerialBT.print("Setting Right Drive to :");// write on BT app
        SerialBT.println(newPowerToWrite);// write on BT app      
        DriveMotorRight.write(newPowerToWrite);
    }
}

void ReadJsonData(String jsonData)
{
    auto error = deserializeJson(jsonDocument, jsonData);
    if (error)
    {   //Check for errors in parsing

        Serial.println("Parsing failed");
        delay(5000);
        return;

    }

    float value = 0;

    value = jsonDocument["l"];
    Serial.print("l:");
    Serial.print(value);
    targetLeftMotorSpeed = value;

    value = jsonDocument["r"];
    Serial.print(", r:");
    Serial.print(value);
    targetRightMotorSpeed = value;

    value = jsonDocument["w"];
    Serial.print(", w:");
    Serial.println(value);
    targetWeaponMotorSpeed = value;
}

void Start()
{
    pinMode(PIN_NUM_NEOPIXEL_OUTPUT, OUTPUT);
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    //Serial.begin(115200);

    SerialBT.begin();
    Serial.println("Bluetooth Started! Ready to pair...");

    FastLED.addLeds<NEOPIXEL, PIN_NUM_NEOPIXEL_OUTPUT>(leds, TOTAL_LED);

    SetAllLEDs(CRGB::Purple);

    FastLED.show();
    delay(500);
    DriveMotorLeft.setPeriodHertz(50);
    DriveMotorRight.setPeriodHertz(50);
    WeaponMotor.setPeriodHertz(50);
    DriveMotorLeft.attach(PIN_DRIVE_MOTOR_LEFT, 1000, 2000);
    DriveMotorRight.attach(PIN_DRIVE_MOTOR_RIGHT, 1000, 2000);
    WeaponMotor.attach(PIN_WEAPON_MOTOR, 750, 2000);

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

    if((millis()-lastUpdateMillis)%2000 < 1000)
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
    
    while (SerialBT.available() > 0)
    {
        char recieved = SerialBT.read();
        inputString += recieved;

        // Process message when new line character is recieved
        if (recieved == '\n')
        {
            Serial.print("BT Received: ");
            SerialBT.print("BT Received: ");
            Serial.print(inputString);
            ReadJsonData(inputString);
            inputString = ""; // Clear recieved buffer

            lastUpdateMillis = millis();
        }
    }

    if(!IsTimedOut()) // Means we got fresh data
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
