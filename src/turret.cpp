/**
 * @file turret.cpp
 * @brief Implementation file for the Turret class controlling the turret movement.
 */

#include <Arduino.h>
#include "turret.h"

const uint8_t maxYawDutyCycle = 85; ///< Maximum duty cycle for yaw movement.
const uint8_t maxYawPWM = (((float) maxYawDutyCycle)/100) * 255; ///< Maximum PWM value for yaw movement.
uint8_t yaw_pwm = 0; ///< PWM value for yaw movement.

const uint16_t yawPWMFreq = 20000; ///< PWM frequency for yaw movement.
const uint8_t yawPWMChannel_FWD = 10; ///< PWM channel for forward yaw movement.
const uint8_t yawPWMChannel_REV = 11; ///< PWM channel for reverse yaw movement.
const uint8_t yawPWMResolution = 8; ///< PWM resolution for yaw movement.

const uint8_t maxPitchDutyCycle = 85; ///< Maximum duty cycle for pitch movement.
const uint8_t maxPitchPWM = (((float) maxPitchDutyCycle)/100) * 255; ///< Maximum PWM value for pitch movement.
uint8_t pitch_pwm = 0; ///< PWM value for pitch movement.

const uint16_t pitchPWMFreq = 20000; ///< PWM frequency for pitch movement.
const uint8_t pitchPWMChannel_FWD = 9; ///< PWM channel for forward pitch movement.
const uint8_t pitchPWMChannel_REV = 8; ///< PWM channel for reverse pitch movement.
const uint8_t pitchPWMResolution = 8; ///< PWM resolution for pitch movement.

uint8_t yawPWM_FWD; ///< Pin for forward yaw movement.
uint8_t yawPWM_REV; ///< Pin for reverse yaw movement.

uint8_t pitchPWM_FWD; ///< Pin for forward pitch movement.
uint8_t pitchPWM_REV; ///< Pin for reverse pitch movement.

ESP32Encoder pitchEncoder; ///< Encoder for pitch movement.
uint8_t ENC_A; ///< Encoder pin A.
uint8_t ENC_B; ///< Encoder pin B.
int8_t current_pos; ///< Current position of the pitch.
int8_t target_pos; ///< Target position for the pitch.
bool positionSet; ///< Flag indicating if the target position is set.
bool stickMoved; ///< Flag indicating if the stick is moved.

/**
 * @brief Constructor for the Turret class.
 * @param yaw_pin_1 Pin for forward yaw movement.
 * @param yaw_pin_2 Pin for reverse yaw movement.
 * @param pitch_pin_1 Pin for forward pitch movement.
 * @param pitch_pin_2 Pin for reverse pitch movement.
 * @param encoder Encoder for pitch movement.
 * @param enc_pin_1 Encoder pin A.
 * @param enc_pin_2 Encoder pin B.
 */
Turret::Turret(uint8_t yaw_pin_1, uint8_t yaw_pin_2, uint8_t pitch_pin_1, uint8_t pitch_pin_2, ESP32Encoder encoder, uint8_t enc_pin_1, uint8_t enc_pin_2)
{
    yawPWM_FWD = yaw_pin_1;
    yawPWM_REV = yaw_pin_2;
    pitchPWM_FWD = pitch_pin_1;
    pitchPWM_REV = pitch_pin_2;

    ESP32Encoder pitchEncoder = encoder;
    uint8_t ENC_A = enc_pin_1;
    uint8_t ENC_B = enc_pin_2;

    configureEncoder();
    
    // Yaw Control Setup
    ledcSetup(yawPWMChannel_FWD, yawPWMFreq, yawPWMResolution);
    ledcSetup(yawPWMChannel_REV, yawPWMFreq, yawPWMResolution);
    ledcAttachPin(yawPWM_FWD, yawPWMChannel_FWD);
    ledcAttachPin(yawPWM_REV, yawPWMChannel_REV);
    ledcWrite(yawPWMChannel_FWD, 0);
    ledcWrite(yawPWMChannel_REV, 0);
    
    // Pitch Control Setup
    ledcSetup(pitchPWMChannel_FWD, pitchPWMFreq, pitchPWMResolution);
    ledcSetup(pitchPWMChannel_REV, pitchPWMFreq, pitchPWMResolution);
    ledcAttachPin(pitchPWM_FWD, pitchPWMChannel_FWD);
    ledcAttachPin(pitchPWM_REV, pitchPWMChannel_REV);
    ledcWrite(pitchPWMChannel_FWD, 0);
    ledcWrite(pitchPWMChannel_REV, 0);
}

/**
 * @brief Configure the encoder for pitch movement.
 */
void Turret::configureEncoder()
{
    // Encoder Setup
    pitchEncoder.attachHalfQuad(5, 18);
    ESP32Encoder::useInternalWeakPullResistors = UP;
    pitchEncoder.clearCount();
    pitchEncoder.setCount(1000);
    current_pos = pitchEncoder.getCount();
    target_pos = current_pos;
    positionSet = false;
    stickMoved = false;
}

/**
 * @brief Move the turret in the yaw direction.
 * @param input Yaw input value.
 */
void Turret::moveTurret_yaw(int8_t input)
{  
    yaw_pwm = float (abs(input))/128 * 255;

    if (yaw_pwm > maxYawPWM)
    {
        yaw_pwm = maxYawPWM;
    }

    if (abs(input) <= 5)
    {
        ledcWrite(yawPWMChannel_FWD, 0);
        ledcWrite(yawPWMChannel_REV, 0);
    }

    // Yaw CW
    else if (input > 5)
    {
        ledcWrite(yawPWMChannel_REV, 0);
        ledcWrite(yawPWMChannel_FWD, yaw_pwm);
    }

    // Yaw CCW
    else if (input < -5)
    {
        ledcWrite(yawPWMChannel_FWD, 0);
        ledcWrite(yawPWMChannel_REV, yaw_pwm);
    }
}

/**
 * @brief Move the turret in the pitch direction.
 * @param input Pitch input value.
 * @param inputSource Source of the input (stick, controller, or camera).
 */
void Turret::moveTurret_pitch(int8_t input, String inputSource)
{   
    if (inputSource == "stick")
    {
        pitch_pwm = (uint8_t) ((float) (abs(input))/128 * 255);
    }
    else if (inputSource == "controller")
    {
        pitch_pwm = (uint8_t) ((float) (abs(input))/100 * 255);
    }
    else if (inputSource == "camera")
    {
        pitch_pwm = (uint8_t) ((float) (abs(input))/100 * 255);
    }

    if (pitch_pwm > maxPitchPWM)
    {
        pitch_pwm = maxPitchPWM;
    }

    // Pitch up
    if (input > 3)
    {
        ledcWrite(pitchPWMChannel_FWD, 0);
        ledcWrite(pitchPWMChannel_REV, pitch_pwm);
    }

    // Pitch down
    else if (input < -3)
    {
        ledcWrite(pitchPWMChannel_REV, 0);
        ledcWrite(pitchPWMChannel_FWD, pitch_pwm);
    }

    else
    {
        ledcWrite(pitchPWMChannel_FWD, 0);
        ledcWrite(pitchPWMChannel_REV, 0);
    }
}

/**
 * @brief Update the current pitch position.
 */
void Turret::updatePitchPosition()
{
    current_pos = pitchEncoder.getCount();
}

/**
 * @brief Set the target position for pitch movement.
 * @param target Target position for pitch movement.
 */
void Turret::setTargetPosition(uint8_t target)
{
    target_pos = target;
}

/**
 * @brief Find targets using the turret.
 */
void Turret::findTargets()
{
    moveTurret_yaw(25);
}
