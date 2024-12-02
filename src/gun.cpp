/**
 * @file gun.cpp
 * @brief Implementation of the Gun class for controlling a gun system.
 * @author Jason Davis
 */

#include <Arduino.h>
#include "driver/ledc.h"
#include <ESP32Servo.h>
#include "camera.h"
#include "gun.h"

const uint8_t THETA_MIN = 31;
const uint8_t THETA_MAX = 70;
const uint8_t SERVO_PWM_FREQ = 50;

// Roller Motors
// const uint8_t ROLLER_1 = 14;
// const uint8_t ROLLER_2 = 12;
const uint16_t ROLLER_PWM_FREQ = 10000;
const uint8_t ROLLER_1_CHANNEL = 15;
const uint8_t ROLLER_2_CHANNEL = 14;
const uint8_t ROLLER_PWM_RESOLUTION = 8;
const uint8_t MAX_ROLLER_DUTY_CYCLE = 70;

const uint8_t MIN_ROLLER_DUTY_CYCLE = 25;
const uint8_t MAX_ROLLER_DUTY_CYCLE = 70; // pulse width percent
const uint8_t MIN_PWM = (float(MIN_ROLLER_DUTY_CYCLE) / 100) * 255;
const uint8_t MAX_PWM = (float(MAX_ROLLER_DUTY_CYCLE) / 100) * 255;
const uint8_t START_PWM = 25;
const uint8_t SERVO_DELAY = 200;

uint8_t maxPWM = (float(MAX_ROLLER_DUTY_CYCLE) / 100) * 255;
uint8_t startPWM = 25;

uint8_t laserControlPin;
Servo fireControlServo;
uint8_t servoControlPin;
uint8_t rollerPin_1;
uint8_t rollerPin_2;
Camera camera;

uint8_t theta;
uint16_t k_ss;

int16_t yawInput;
int16_t pitchInput;

bool safetyOn;
bool laserOn;
bool rollersOn;
bool servoLinkageReset;

bool demoMode;

/**
 * @brief Constructor for the Gun class.
 * @param laser_control_pin Pin controlling the laser.
 * @param servo_control_pin Pin controlling the servo motor.
 * @param roller_pin_1 Pin controlling roller motor 1.
 * @param roller_pin_2 Pin controlling roller motor 2.
 * @param hl_camera HUSKYLENS camera object.
 */
Gun::Gun(uint8_t laser_control_pin, uint8_t servo_control_pin, uint8_t roller_pin_1, uint8_t roller_pin_2, HUSKYLENS hl_camera)
{
    laserControlPin = laser_control_pin;
    servoControlPin = servo_control_pin;
    rollerPin_1 = roller_pin_1;
    rollerPin_2 = roller_pin_2;
    camera = hl_camera;

    theta = THETA_MIN;
    safetyOn = false;
    laserOn = false;
    rollersOn = false;

    yawInput = 0;
    pitchInput = 0;

    // Initializing the servo motor
    fireControlServo.attach(servoControlPin);
    fireControlServo.write(THETA_MIN);
    servoLinkageReset = true;

    // GPIO Pin Initialization using standard Arduino stuff
    pinMode(laserControlPin, OUTPUT);
    pinMode(servoControlPin, OUTPUT);

    // Roller Setup
    ledcSetup(ROLLER_1_CHANNEL, ROLLER_PWM_FREQ, ROLLER_PWM_RESOLUTION);
    ledcAttachPin(rollerPin_1, ROLLER_1_CHANNEL);

    ledcSetup(ROLLER_2_CHANNEL, ROLLER_PWM_FREQ, ROLLER_PWM_RESOLUTION);
    ledcAttachPin(rollerPin_2, ROLLER_2_CHANNEL);
}

/**
 * @brief Getter for the Camera object.
 * @return The Camera object.
 */
Camera Gun::getCamera()
{
    return camera;
}

/**
 * @brief Toggle the safety state.
 */
void Gun::toggleSafety()
{
    if (safetyOn)
    {
        safetyOn = false;
    }
    else
    {
        safetyOn = true;
    }
}

/**
 * @brief Toggle the laser state.
 */
void Gun::toggleLaser()
{
    if (digitalRead(laserControlPin) == HIGH)
    {
        digitalWrite(laserControlPin, LOW);
    }
    else if (digitalRead(laserControlPin) == LOW)
    {
        digitalWrite(laserControlPin, HIGH);
    }
}

/**
 * @brief Start the rollers.
 */
void Gun::startRollers()
{
    // 8 bit unsigned defines a PWM with analogWrite()
    // 0 is off, 255 is full on
    for (uint8_t i = startPWM; i < maxPWM; i++)
    {
        ledcWrite(ROLLER_1_CHANNEL, i);
        ledcWrite(ROLLER_2_CHANNEL, i);
    }

    Serial.println("done.");
    rollersOn = true;
}

/**
 * @brief Stop the rollers.
 */
void Gun::stopRollers()
{
    for (uint8_t i = maxPWM; i > 0; i--)
    {
        ledcWrite(ROLLER_1_CHANNEL, i);
        ledcWrite(ROLLER_2_CHANNEL, i);
    }

    rollersOn = false;
}

/**
 * @brief Fire the gun.
 */
void Gun::fire()
{
    if ((rollersOn || demoMode) && (!safetyOn) || (masterControlMode == 1 && targetLocked))
    {
        while (theta < THETA_MAX)
        {
            theta++;
            fireControlServo.write(theta);
        }
        delay(SERVO_DELAY);

        // now it must be reset
        servoLinkageReset = false;
    }

    if (masterControlMode == 1)
    {
        resetServoArmPosition();
    }
}

/**
 * @brief Reset the servo arm position.
 */
void Gun::resetServoArmPosition()
{
    while (theta > THETA_MIN)
    {
        theta--;
        fireControlServo.write(theta);
    }
    servoLinkageReset = true;
}

/**
 * @brief Update the shooting solution based on the HUSKYLENS result.
 * @param result The HUSKYLENS result object.
 */
void Gun::updateShootingSolution(HUSKYLENSResult result)
{
    int16_t x = result.xCenter;
    int16_t y = result.yCenter;

    int16_t errorX = ((int16_t)x) - camera.getCX();
    int16_t errorY = ((int16_t)y) - camera.getCY();

    if (!rollersOn)
    {
        startRollers();
        delay(100);
    }

    if (abs(errorX) < 40 && abs(errorY) < 30)
    {
        fire();
        delay(50);
    }
    else
    {
        yawInput = (int8_t)((float)(255 * errorX / camera.getCX()));
        pitchInput = (int8_t)((float)(255 * errorY / camera.getCY()));
    }
}

/**
 * @brief Get the state of the laser.
 * @return The state of the laser.
 */
bool Gun::laser()
{
    return laserOn;
}

/**
 * @brief Get the state of the safety.
 * @return The state of the safety.
 */
bool Gun::safety()
{
    return safetyOn;
}

/**
 * @brief Get the state of the rollers.
 * @return The state of the rollers.
 */
bool Gun::rollers()
{
    return rollersOn;
}

/**
 * @brief Get the yaw input.
 * @return The yaw input.
 */
int8_t Gun::getYawInput()
{
    return yawInput;
}

/**
 * @brief Get the pitch input.
 * @return The pitch input.
 */
int8_t Gun::getPitchInput()
{
    return pitchInput;
}
