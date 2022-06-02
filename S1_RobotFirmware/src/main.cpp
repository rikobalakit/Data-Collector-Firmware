/// Hiiiiii, this is Data Collector's firmware!
/// I am trying to clean it up and reorganize the project
/// But it is pretty messy
/// As I work on it more, I'll try to flesh out the self-documentation of it.
/// Also, I'm not too familiar with Platformio's methods of including files for build
/// and header files weird me out because I'm so used to C#
/// I'm a bit anxious this code makes me look like an amateur :sweat-smile-emoji:
/// but anyway, here it is
/// - Pearl Grey, benevolent dictator of Grey Skies Automation

#include "main.h"

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
    if (LOG_CONTROLLER_CONNECTED)
    {
        Serial.println("We are timed out...");
        Serial.print(" Timeout started: ");
        Serial.print(_controllerTimeoutDeclaredStartedMillis);
    }

    _controllerTimeoutDeclaredStartedMillis =
            _lastControllerAccelerometerChangedTime + CONTROLLER_TIMEOUT_THRESHOLD_MILLIS;

    if (_controllerTimeoutDeclaredStartedMillis + 5000 < millis() && EVENT_MODE)
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

    // TODO: add properties for each LED instead of just the index being vaguely referenced
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

void UpdateWeaponLEDs(float newSpeed)
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

    leds[0] = color;
}

void UpdateCenterLEDColors()
{
    SetCenterLEDs(CRGB::Black);
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


    if (IsControllerDataTimedOut() && (millis() - CONTROLLER_TIMEOUT_THRESHOLD_MILLIS) % 1000 < 500)
    {
        SetCenterLEDs(CHSV(HUE_BLUE, 255, MAX_BRIGHTNESS));
    }

    if (_forceShutdownTurnedOn && (millis() - _forceShutdownStartedMillis) % 200 < 100)
    {
        SetCenterLEDs(CHSV(HUE_DISCONNECTED, 255, MAX_BRIGHTNESS));
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

    Serial.print("isFullyCalibrated: ");
    Serial.print(bno.isFullyCalibrated());

    //event.gyro.heading;
    //    event.orientation.x;

    Serial.print("\t   X: ");
    Serial.print(_currentXOrientation, 4);

    Serial.print("\t   event.gyro.roll: ");
    Serial.print((float) event.gyro.roll, 4);

    Serial.print("\t   event.gyro.pitch: ");
    Serial.print((float) event.gyro.pitch, 4);

    Serial.print("\t   event.gyro.heading: ");
    Serial.print((float) event.gyro.heading, 4);

    Serial.print("\t   event.orientation.x: ");
    Serial.print((float) event.orientation.x, 4);

    /*
    Serial.print("\t   XOFF: ");
    Serial.print(_currentXOrientationOffset, 4);

    Serial.print("\t   XDELTA: ");
    Serial.print((float)_eventOrientationXDelta, 4);

    Serial.print("\t   UPSIDEDOWN?: ");
    Serial.print(_isUpsideDown);
     */

    Serial.print("\t   XRAW: ");
    Serial.print(event.orientation.x, 4);
    Serial.println("");

}

void printMac(const unsigned char *mac) {
    printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

void SetFormulaId()
{
    unsigned char thisEspMacAddress[6] = {0};
    esp_efuse_mac_get_default(thisEspMacAddress);

    // during R&D, this is the "free" one
    unsigned char formula1MacAddress[] = {0x34, 0x94, 0x54, 0x25, 0x04, 0xBC};

    if (thisEspMacAddress[0] == formula1MacAddress[0]
        && thisEspMacAddress[1] == formula1MacAddress[1]
        && thisEspMacAddress[2] == formula1MacAddress[2]
        && thisEspMacAddress[3] == formula1MacAddress[3]
        && thisEspMacAddress[4] == formula1MacAddress[4]
        && thisEspMacAddress[5] == formula1MacAddress[5])
    {
        _esp32Id = 1;
        return;
    }
    // during R&D, this is the "installed" one
    unsigned char formula2MacAddress[] = {0x30, 0xC6, 0xF7, 0x23, 0x97, 0x3C};

    if (thisEspMacAddress[0] == formula2MacAddress[0]
        && thisEspMacAddress[1] == formula2MacAddress[1]
        && thisEspMacAddress[2] == formula2MacAddress[2]
        && thisEspMacAddress[3] == formula2MacAddress[3]
        && thisEspMacAddress[4] == formula2MacAddress[4]
        && thisEspMacAddress[5] == formula2MacAddress[5])
    {
        _esp32Id = 2;
        return;
    }
    
    
    _esp32Id = -2; // error
}

float CalculateVoltageForUniqueAddress(int rawReading)
{
    if(_esp32Id == -1)
    {
        SetFormulaId();
    }

    // use Formula 1
    
    if(_esp32Id == 1)
    {
        return -4.51879523 + 0.00882277 * (float) rawReading -
               0.00000081 * (float) (rawReading * rawReading);
    }
    
    if(_esp32Id == 2)
    {
        return -4.36430528 + 0.00924324 * (float) rawReading -
               0.00000092 * (float) (rawReading * rawReading);
    }
    
    
}

void LogVoltage()
{
    if (!LOG_VOLTAGE)
    {
        return;
    }

    _rawReadings[_readingsIndex] = _voltageResultRawValue;
    _readingsIndex++;
    if (_readingsIndex >= 100)
    {
        _readingsIndex = 0;
    }

    int readingsTotal = 0;
    for (int i = 0; i < 100; i++)
    {
        readingsTotal += _rawReadings[i];
    }

    float averageReading = (float) readingsTotal / (float) 100;

    Serial.println();
    Serial.print(averageReading, 4);
    Serial.print(" rawV avg, ");
    Serial.print(_voltageResultRawValue);
    Serial.print(" rawV, ");
    Serial.print(_calculatedVoltageVolts);
    Serial.print("v, (");
    Serial.print(_calculatedVoltagePerCell);
    Serial.print("v per cell)   ");
    Serial.println("");


}

#pragma endregion

#pragma region Input Readings

void InitializeController()
{
    // if you figure out how to cheese this to interfere with my controls, you are both smart and an asshole.
    Ps3.begin("00:02:72:3F:5F:02");
    Ps3.getAddress();
}

void CheckAndInitializeOrientationSensor()
{

    if (!bno.begin(bno.OPERATION_MODE_IMUPLUS))
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
    _voltageResultRawValue = analogRead(PIN_VOLTAGE_CHECK);
    _calculatedVoltageVolts = CalculateVoltageForUniqueAddress(_voltageResultRawValue);
    //i hope this stays accurate between different copies/spares
    //only accurate between 12.0v and 16.8v, gets super wacky below that.
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

void AttemptToRecalibrateIfSuddenChangeDetected()
{
    if (_previousEventOrientationX == -1)
    {
        _previousEventOrientationX = _currentXOrientation;
    }


    _eventOrientationXDelta = ((int) (_currentXOrientation - _previousEventOrientationX + 180 + 360)) % 360 - 180;

    if (abs(_eventOrientationXDelta) > 45 && _currentXOrientationOffset == 0 && _magnetometerCalibrationLevel > 2)
    {
        _currentXOrientationOffset = -_eventOrientationXDelta;
    }

    _previousEventOrientationX = _currentXOrientation;
}

void InterpretRollForRumble()
{
    // heading can be between -180 to 180
    //stable heading should be between -180 to -170 and -10 to 10 and 170 to 180.

    float rollReadingAbsolute = abs(event.gyro.heading);
    float differenceFrom90 = abs(90 - rollReadingAbsolute);

    Serial.print("\t   rollDelta: ");
    Serial.print(differenceFrom90, 4);

    if (differenceFrom90 < (90 - GYRO_LIFT_ANGLE_TOLERANCE))
    {
        float normalizedStrengthOfRollover =
                ((90 - GYRO_LIFT_ANGLE_TOLERANCE) - differenceFrom90) / (90 - GYRO_LIFT_ANGLE_TOLERANCE);
        Serial.print("currently rolling over: ");
        Serial.print(normalizedStrengthOfRollover, 4);
        if (ps3IsConnected())
        {
            Ps3.setRumble(normalizedStrengthOfRollover * 50 + 50, 500);
        }


    }
    else
    {
        // not rolling over
        if (ps3IsConnected())
        {
            Ps3.setRumble(0, 500);
        }
    }
}

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

    if (_currentXOrientation > 360)
    {
        _currentXOrientation -= 360;
    }

    _currentXOrientation += _currentXOrientationOffset;

    if (_currentXOrientation > 360)
    {
        _currentXOrientation -= 360;
    }

    InterpretRollForRumble();

}

void InterpretVoltage()
{
    int possibleNewBatteryStage = 0;

    if (_calculatedVoltagePerCell < VOLTAGE_CUTOFF_EVERYTHING)
    {
        possibleNewBatteryStage = BATTERY_STAGE_DEAD;
    }
    else if (_calculatedVoltagePerCell >= VOLTAGE_CUTOFF_EVERYTHING &&
             _calculatedVoltagePerCell < VOLTAGE_CUTOFF_WEAPON)
    {
        possibleNewBatteryStage = BATTERY_STAGE_CUTOFF_WEAPON;
    }
    else if (_calculatedVoltagePerCell >= VOLTAGE_CUTOFF_WEAPON && _calculatedVoltagePerCell < VOLTAGE_CUTOFF_HALFWAY)
    {
        possibleNewBatteryStage = BATTERY_STAGE_UNDERHALF;
    }
    else if (_calculatedVoltagePerCell >= VOLTAGE_CUTOFF_HALFWAY && _calculatedVoltagePerCell < VOLTAGE_CUTOFF_FULL)
    {
        possibleNewBatteryStage = BATTERY_STAGE_OVERHALF;
    }
    else if (_calculatedVoltagePerCell >= VOLTAGE_CUTOFF_FULL && _calculatedVoltagePerCell < VOLTAGE_CUTOFF_OVERLIMIT)
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
        if (LOG_CONTROLLER_CONNECTED)
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
        _linearDirectionStabilizationEnabled = false;
    }

    if (_controllerDirectionUpPressed)
    {


        if (!_linearDirectionStabilizationEnabled)
        {
            _linearDirectionStabilizationEnabled = true;
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
                rightMixedValue += 1 * (1 - DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR);
                // too much to the left, turn right
            }
            else if (angleDelta < -3)
            {
                Serial.print("correcting towards left");
                leftMixedValue += 1 * (1 - DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR);
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

        if (!_linearDirectionStabilizationEnabled)
        {
            _linearDirectionStabilizationEnabled = true;
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
                leftMixedValue -= 1 * (1 - DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR);
                rightMixedValue -= 1;
                // too much to the left, turn right
            }
            else if (angleDelta < -3)
            {
                Serial.print("correcting towards left");
                leftMixedValue -= 1;
                rightMixedValue -= 1 * (1 - DRIVE_PERFECT_FORWARD_CORRECTION_FACTOR);
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
        // TODO: add weapon throttle attentuation
        _linearDirectionStabilizationEnabled = false;
        leftMixedValue -= 1;
        rightMixedValue += 1;
    }

    if (_controllerDirectionRightPressed)
    {
        // TODO: add weapon throttle attentuation
        _linearDirectionStabilizationEnabled = false;
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

        if (_videoGameAngleDeltaDegrees > VIDEO_GAME_DRIVE_ANGLE_TOLERANCE)
        {
            Serial.print("correcting towards right:");
            leftMixedValue += 0.1 + VIDEO_GAME_DRIVE_ANGLE_CORRECT_FACTOR * angleDeltaSpeedMultiplier;
            rightMixedValue += -0.1 + -VIDEO_GAME_DRIVE_ANGLE_CORRECT_FACTOR * angleDeltaSpeedMultiplier;
            // too much to the left, turn right
        }
        else if (_videoGameAngleDeltaDegrees < -VIDEO_GAME_DRIVE_ANGLE_TOLERANCE)
        {
            Serial.print("correcting towards left");
            leftMixedValue += -0.1 + -VIDEO_GAME_DRIVE_ANGLE_CORRECT_FACTOR * angleDeltaSpeedMultiplier;
            rightMixedValue += 0.1 + VIDEO_GAME_DRIVE_ANGLE_CORRECT_FACTOR * angleDeltaSpeedMultiplier;
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
    // TODO-RPB: Inversion-sensitive weapon controls requires an inversion smoothing function
    // and careful prep of all motors and ESCs to be in the same direction

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
    // TODO: I think im doing this wrong...

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

    if (_currentBatteryStage == BATTERY_STAGE_DEAD && !EVENT_MODE)
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
    // TODO: Need a way to detect a vert weapon is on-board. No need to do this for horizontal spinners.
    if (!DETHROTTLE_WEAPON_ON_HARD_TURN)
    {
        return inputTargetWeaponSpeed;
    }
    float dethrottleMultiplier = 1;


    if (_currentlyDoingVideoGameStyleControlInsteadOfDPadOrTank)
    {
        //TODO: this is where to also add the dethrottle based off the roll angle

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



