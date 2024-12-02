/**
 * @file main.cpp
 * @brief Robot control system using PS4 controller, gun, camera, and turret.
 * @author Your Name
 */

#include <Arduino.h>
#include "PS4_CONTROLLER.h"
#include "driver/ledc.h"
#include <ESP32Encoder.h>
#include <HuskyLens/HUSKYLENS.h>
#include "gun.h"
#include "camera.h"
#include "turret.h"

const char* mac = "9c:9c:1f:ca:cf:d2"; ///< Bluetooth MAC address of the PS4 controller.
const uint16_t GUN_DELAY = 50/portTICK_PERIOD_MS; ///< Delay for the gun task.
const uint16_t TURRET_DELAY = 50/portTICK_PERIOD_MS; ///< Delay for the turret task.
const uint16_t CONTROLLER_DELAY = 5/portTICK_PERIOD_MS; ///< Delay for the controller task.

bool autoTargetMode; ///< Flag indicating whether auto-targeting mode is enabled.
bool demoMode = false; ///< Flag indicating whether demo mode is enabled.

bool laser; ///< Flag indicating the state of the laser.
bool safety; ///< Flag indicating the state of the safety mechanism.
bool rollers; ///< Flag indicating the state of the rollers.
bool r1; ///< Flag indicating the state of the R1 button.

uint8_t yawInput; ///< Yaw input for the turret.
uint8_t pitchInput; ///< Pitch input for the turret.
String pitchInputDevice; ///< Device providing pitch input.

PS4_CONTROLLER controller; ///< PS4 controller instance.
ESP32Encoder encoder; ///< Encoder instance for the turret.
HUSKYLENS camera; ///< Camera instance.
Gun gun; ///< Gun instance.
Turret turret; ///< Turret instance.

/**
 * @brief Toggles demo mode.
 */
void toggleDemoMode()
{
  demoMode = !demoMode;
}

/**
 * @brief Toggles auto-targeting mode.
 */
void toggleAutoTargetMode()
{
  autoTargetMode = !autoTargetMode;
}

/**
 * @brief Toggles the laser state.
 */
void toggleLaser()
{
  laser = !laser;
}

/**
 * @brief Toggles the rollers state.
 */
void toggleRollers()
{
  rollers = !rollers;
}

/**
 * @brief Toggles the safety state.
 */
void toggleSafety()
{
  safety = !safety;
}

/**
 * @brief Task for handling PS4 controller input.
 * @param p_params Task parameters (unused).
 */
void task_controller(void* p_params)
{
  for (;;)
  {
    if (controller.squareDown())
    {
      toggleAutoTargetMode();
    }

    if (controller.triangleDown())
    {
      toggleLaser();
    }

    if (controller.circleDown())
    {
      toggleSafety();
    }

    if (controller.crossDown())
    {
      toggleRollers();
    }

    if (controller.optionsDown())
    {
      toggleDemoMode();
    }

    if (controller.r1Down())
    {
      r1 = true;
    }
    else
    {
      r1 = false;
    }

    if (!autoTargetMode)
    {
        yawInput = controller.rxValue();
        pitchInput = controller.lyValue();
        pitchInputDevice = "stick";
    }

    vTaskDelay(CONTROLLER_DELAY);
  }
}

/**
 * @brief Task for handling gun operations.
 * @param p_params Task parameters (unused).
 */
void task_gun(void* p_params)
{
  for(;;)
  {
    if (autoTargetMode)
    {
      HUSKYLENSResult result = gun.getCamera().read();
      gun.updateShootingSolution(result);

      yawInput = gun.getYawInput();
      pitchInput = gun.getPitchInput();
      pitchInputDevice = "camera";
    }

    if ((laser && !gun.laser()) || (!laser && gun.laser()))
    {
      gun.toggleLaser();
    }

    if ((safety && !gun.safety()) || (!safety && gun.safety()))
    {
      gun.toggleSafety();
    }

    if ((rollers && !gun.rollers()) || (!rollers && gun.rollers()))
    {
      gun.toggleRollers();
    }

    if (r1)
    {
      gun.fire();
    }
    else
    {
      gun.resetServoArmPosition();
    }

    vTaskDelay(GUN_DELAY);
  }
}

/**
 * @brief Task for handling turret movements.
 * @param p_params Task parameters (unused).
 */
void task_turret(void* p_params)
{
  for(;;)
  {
    turret.updatePitchPosition();
    turret.moveTurret_yaw(yawInput);
    turret.moveTurret_pitch(pitchInput, pitchInputDevice);

    vTaskDelay(TURRET_DELAY);
  }
}

/**
 * @brief Arduino setup function.
 */
void setup() 
{
    Serial.begin(115200);
    delay(20);
    Wire.begin();
    delay(20);

    Serial.println("Initializing robot system peripherals. Please wait...");
    Serial.println("Controller ready to pair!");

    PS4_CONTROLLER controller(mac);
    Gun gun(32, 33, 14, 12, camera);

    while (!gun.getCamera().begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
        Serial.println(F("2.Please recheck the connection."));
        digitalWrite(2, HIGH);
        delay(100);
        digitalWrite(2, LOW);
    }
  
  Turret turret(19,18,26,27,encoder,0,4);
  Serial.println("System Peripherals Initialized. Robot ready for commands");

  xTaskCreate(task_controller, "task_controller", 2048, NULL, 5, NULL);
  xTaskCreate(task_gun, "task_gun", 1024, NULL, 3, NULL);
  xTaskCreate(task_turret, "task_turret", 1024, NULL, 3, NULL);
}

/**
 * @brief Arduino loop function (not used in FreeRTOS).
 */
void loop() 
{}
