/**
 * @file camera.cpp
 * @author Jason Davis
 * @brief Implementation of the Camera class for handling HUSKYLENS camera.
 */

#include <Arduino.h>
#include "HuskyLens/HUSKYLENS.h"
#include "camera.h"

HUSKYLENS camera; ///< HUSKYLENS object for camera communication.
const int16_t cX = 160; ///< Default X-coordinate of the center of the camera image.
const int16_t cY = 120; ///< Default Y-coordinate of the center of the camera image.

uint8_t cameraXGain; ///< Gain for X-axis in camera.
uint8_t cameraYGain; ///< Gain for Y-axis in camera.
int16_t x; ///< X-coordinate of the detected target.
int16_t y; ///< Y-coordinate of the detected target.
int16_t errorX; ///< Error in X-coordinate between the target and the center of the camera image.
int16_t errorY; ///< Error in Y-coordinate between the target and the center of the camera image.

bool targetLocked; ///< Flag indicating whether a target is locked or not.
bool isAvailable; ///< Flag indicating whether the camera is available or not.

/**
 * @brief Constructor for the Camera class.
 * @param hl_camera The HUSKYLENS camera object.
 */
Camera::Camera(HUSKYLENS hl_camera)
{
    camera = hl_camera;
    
    cameraXGain = 100;
    cameraYGain = 100;
    
    x = -1;
    y = -1;

    errorX = 999;
    errorY = 999;
    
    targetLocked = false;
    isAvailable = false;
}

/**
 * @brief Getter for the HUSKYLENS camera object.
 * @return The HUSKYLENS camera object.
 */
HUSKYLENS Camera::getCamera()
{
    return camera;
}

/**
 * @brief Get the X-coordinate of the detected target from the HUSKYLENSResult.
 * @param result The HUSKYLENSResult object containing information about the detected target.
 * @return The X-coordinate of the detected target, or -1 if no target is detected.
 */
int16_t Camera::getTargetX(HUSKYLENSResult result)
{
    x = -1;

    if (result.command == COMMAND_RETURN_BLOCK)
    {
        x = result.xCenter;
    }

    return x;
}

/**
 * @brief Get the Y-coordinate of the detected target from the HUSKYLENSResult.
 * @param result The HUSKYLENSResult object containing information about the detected target.
 * @return The Y-coordinate of the detected target, or -1 if no target is detected.
 */
int16_t Camera::getTargetY(HUSKYLENSResult result)
{
    y = -1;

    if (result.command == COMMAND_RETURN_BLOCK)
    {
        y = result.yCenter;
    }

    return y;
}

/**
 * @brief Get the camera X-axis gain.
 * @return The camera X-axis gain.
 */
uint8_t Camera::getCameraXGain() const 
{
    return cameraXGain;
}

/**
 * @brief Set the camera X-axis gain.
 * @param newGain The new gain value for the X-axis.
 */
void Camera::setCameraXGain(uint8_t newGain)
{
    cameraXGain = newGain;
}

/**
 * @brief Get the camera Y-axis gain.
 * @return The camera Y-axis gain.
 */
uint8_t Camera::getCameraYGain() const 
{
    return cameraYGain;
}

/**
 * @brief Set the camera Y-axis gain.
 * @param newGain The new gain value for the Y-axis.
 */
void Camera::setCameraYGain(uint8_t newGain) 
{
    cameraYGain = newGain;
} 

/**
 * @brief Get the default X-coordinate of the center of the camera image.
 * @return The default X-coordinate of the center of the camera image.
 */
uint16_t Camera::getCX() const
{
    return cX;
}

/**
 * @brief Get the default Y-coordinate of the center of the camera image.
 * @return The default Y-coordinate of the center of the camera image.
 */
uint16_t Camera::getCY() const
{
    return cY;
}
