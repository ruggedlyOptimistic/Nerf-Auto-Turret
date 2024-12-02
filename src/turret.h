#include <Arduino.h>
#include <ESP32Encoder.h>

class Turret
{
private:
    // These two pins are for yaw motor speed control
    // const uint8_t yawPWM_FWD = 19;
    // const uint8_t yawPWM_REV = 18;
    const uint8_t maxYawDutyCycle = 85;
    const uint8_t maxYawPWM = (((float) maxYawDutyCycle)/100) * 255;
    uint8_t yaw_pwm = 0;

    const uint16_t yawPWMFreq = 20000;
    const uint8_t yawPWMChannel_FWD = 10;
    const uint8_t yawPWMChannel_REV = 11;
    const uint8_t yawPWMResolution = 8;

    // These two pins are for pitch motor speed control
    // const uint8_t pitchPWM_FWD = 26;
    // const uint8_t pitchPWM_REV = 27;
    const uint8_t maxPitchDutyCycle = 85;
    const uint8_t maxPitchPWM = (((float) maxPitchDutyCycle)/100) * 255;
    uint8_t pitch_pwm = 0;

    const uint16_t pitchPWMFreq = 20000;
    const uint8_t pitchPWMChannel_FWD = 9;
    const uint8_t pitchPWMChannel_REV = 8;
    const uint8_t pitchPWMResolution = 8;

    uint8_t yawPWM_FWD;
    uint8_t yawPWM_REV;

    uint8_t pitchPWM_FWD;
    uint8_t pitchPWM_REV;

    ESP32Encoder pitchEncoder;
    uint8_t ENC_A = 0;
    uint8_t ENC_B = 4;
    int8_t current_pos; // must be signed!!

public:
    Turret(uint8_t yaw_pin_1, uint8_t yaw_pin_2, uint8_t pitch_pin_1, uint8_t pitch_pin_2,
    ESP32Encoder encoder, uint8_t enc_pin_1, uint8_t enc_pin_2);

    void configureEncoder();
    void moveTurret_yaw(int8_t input);
    void moveTurret_pitch(int8_t input, String inputSource);
    void updatePitchPosition();
    void setTargetPosition(uint8_t target);
    void findTargets();
};