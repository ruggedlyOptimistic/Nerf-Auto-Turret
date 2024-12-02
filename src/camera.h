#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <HuskyLens/HUSKYLENS.h>

class Camera
{
protected:
    HUSKYLENS camera;
    const int16_t cX = 160;
    const int16_t cY = 120;

    uint8_t cameraXGain;
    uint8_t cameraYGain;
    int16_t x;
    int16_t y;
    int16_t errorX;
    int16_t errorY;

    bool targetLocked;
    bool isAvailable;

public:
    Camera(HUSKYLENS hl_camera);

    HUSKYLENS getCamera();
    int16_t getTargetX(HUSKYLENSResult result);
    int16_t getTargetY(HUSKYLENSResult result);
    uint8_t getCameraXGain() const;
    void setCameraXGain(uint8_t newGain);
    uint8_t getCameraYGain() const;
    void setCameraYGain(uint8_t newGain);
    bool getTargetStatus();
    bool getCameraAvailability();
    uint16_t getCX() const;
    uint16_t getCY() const;
};

#endif // CAMERA_H
