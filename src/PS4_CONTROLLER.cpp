/**
 * @file PS4_CONTROLLER.cpp
 * @brief Implementation file for the PS4_CONTROLLER class to control a PS4 controller.
 * @author Jason Davis
 */

#include <PS4Controller.h>
#include "PS4_CONTROLLER.h"

const char* mac; ///< Bluetooth MAC address of the PS4 controller.
bool paired; ///< Flag indicating whether the controller is paired.

int8_t rx; ///< X-axis value of the right analog stick.
int8_t rxOffset; ///< Offset for the X-axis value of the right analog stick.
int8_t ly; ///< Y-axis value of the left analog stick.
int8_t lyOffset; ///< Offset for the Y-axis value of the left analog stick.
int8_t rt; ///< Right trigger value.

bool square = false; ///< State of the square button.
bool triangle = false; ///< State of the triangle button.
bool circle = false; ///< State of the circle button.
bool cross = false; ///< State of the cross (X) button.
bool r1 = false; ///< State of the R1 button.
bool options = false; ///< State of the Options button.

bool autoTargetMode; ///< Flag indicating whether the auto-targeting mode is enabled.
bool controllerPaired; ///< Flag indicating whether the controller is paired.

/**
 * @brief Constructor for the PS4_CONTROLLER class.
 * @param mac_address The Bluetooth MAC address of the PS4 controller.
 */
PS4_CONTROLLER::PS4_CONTROLLER(const char* mac_address)
{
    mac = mac_address;

    controllerPaired = false;

    PS4.begin(mac);
    PS4.attach(notify);
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisconnect);

    controllerPaired = true;

    autoTargetMode = false;
    rxOffset = PS4.data.analog.stick.rx;
    lyOffset = PS4.data.analog.stick.ly;
}

/**
 * @brief Toggle the auto-targeting mode.
 */
void PS4_CONTROLLER::toggleAutoTargetMode()
{
    autoTargetMode = !autoTargetMode;
}

/**
 * @brief Notification handler for the PS4 controller events.
 */
void PS4_CONTROLLER::notify()
{

    if( PS4.event.button_down.square )
    {
        square = true;
        toggleAutoTargetMode();
    }
    else
    {
        square = false;
    }

    if( PS4.event.button_down.circle )
    {
        circle = true;
    }
    else
    {
        circle = false;
    }

if (!autoTargetMode)
    {

        if( PS4.event.button_down.cross )
        {
            square = true;
        }
        else
        {
            square = false;
        }

        if( PS4.event.button_down.triangle )
        {
            triangle = true;
        }
        else
        {
            triangle = false;
        }

        if( PS4.event.button_down.r1)
        {
            r1 = true;
        }
        else
        {
            r1 = false;
        }
        
        if( PS4.event.button_down.options )
        {
            options = true;
        }
        else
        {
            options = false;
        }

        if (abs(PS4.event.analog_move.stick.rx) && abs(PS4.data.analog.stick.rx - rxOffset) > 5)
        {
            rx = PS4.data.analog.stick.rx - rxOffset;
        }

        if (!abs(PS4.event.analog_move.stick.rx))
        {        
            rx = 0;
        }

        if (abs(PS4.event.analog_move.stick.ly) )
        {
            ly = PS4.data.analog.stick.ly - lyOffset;
        }

        if (!abs(PS4.event.analog_move.stick.ly))
        {        
            ly = 0;
        }

    }

}

/**
 * @brief Handler for the controller connection event.
 */
void PS4_CONTROLLER::onConnect()
{
    uint8_t counter = 0;
    Serial.print("Controller connected: ");
    Serial.println(PS4.isConnected());
    Serial.print("Testing connection stability...");

    while (PS4.isConnected() && counter < 255)
    {
        counter++;
    }

    if (PS4.isConnected())
    {
        Serial.println("Connection stable!");
    }
    else
    {
        Serial.println("Connection unstable!");
    }

    controllerPaired = true;

    Serial.println("Controller initialization complete.");
    Serial.println("Turret ready for input");
}

/**
 * @brief Handler for the controller disconnection event.
 */
void PS4_CONTROLLER::onDisconnect()
{
    controllerPaired = false;
    //Serial.println("Controller disconnected!");
}

/**
 * @brief Check if the square button is pressed.
 * @return True if the square button is pressed, false otherwise.
 */
bool PS4_CONTROLLER::squareDown()
{
    return square;
}

/**
 * @brief Check if the triangle button is pressed.
 * @return True if the triangle button is pressed, false otherwise.
 */
bool PS4_CONTROLLER::triangleDown()
{
    return triangle;
}

/**
 * @brief Check if the circle button is pressed.
 * @return True if the circle button is pressed, false otherwise.
 */
bool PS4_CONTROLLER::circleDown()
{
    return circle;
}

/**
 * @brief Check if the cross (X) button is pressed.
 * @return True if the cross button is pressed, false otherwise.
 */
bool PS4_CONTROLLER::crossDown()
{
    return cross;
}

/**
 * @brief Check if the R1 button is pressed.
 * @return True if the R1 button is pressed, false otherwise.
 */
bool PS4_CONTROLLER::r1Down()
{
    return r1;
}

/**
 * @brief Check if the Options button is pressed.
 * @return True if the Options button is pressed, false otherwise.
 */
bool PS4_CONTROLLER::optionsDown()
{
    return options;
}

/**
 * @brief Get the X-axis value of the right analog stick.
 * @return The X-axis value of the right analog stick.
 */
int8_t PS4_CONTROLLER::rxValue()
{
    return rx;
}

/**
 * @brief Get the Y-axis value of the left analog stick.
 * @return The Y-axis value of the left analog stick.
 */
int8_t PS4_CONTROLLER::lyValue()
{
    return ly;
}
