#ifndef GUN_H
#define GUN_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "camera.h"
#include "driver/ledc.h"

class Gun
{
    private:
        uint8_t laserControlPin;
        Servo fireControlServo;
        uint8_t servoControlPin;
        uint8_t rollerPin_1;
        uint8_t rollerPin_2;
        Camera camera;

        uint8_t theta;
        uint16_t k_ss;

        bool safetyOn;
        bool laserOn;
        bool rollersOn;
        bool servoLinkageReset;

    public:
        // Constructor
        Gun(uint8_t laser_control_pin, uint8_t servo_control_pin, uint8_t roller_pin_1, uint8_t roller_pin_2, HUSKYLENS hl_camera);

        // Member functions
        Camera getCamera();
        void toggleSafety();
        void toggleLaser();
        void startRollers();
        void stopRollers();
        void toggleRollers();
        void updateShootingSolution(HUSKYLENSResult result);
        void fire();
        void resetServoArmPosition();
        bool laser();
        bool safety();
        bool rollers();
        int8_t getYawInput();
        int8_t getPitchInput();
};

#endif // GUN_H
