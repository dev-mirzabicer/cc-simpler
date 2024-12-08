#ifndef BUSCONFIG_H
#define BUSCONFIG_H

#include <cstdint>

/**
 * @brief Configuration parameters for I2C bus.
 */
struct I2CBusConfig
{
    int sdaPin;         // GPIO pin for SDA
    int sclPin;         // GPIO pin for SCL
    uint32_t frequency; // I2C frequency in Hz
    // Add any additional parameters required by your hardware
};

/**
 * @brief Configuration parameters for Sonar sensors.
 *
 * Includes trigger and echo pins required for sonar operation.
 */
struct SonarConfig
{
    int triggerPin; // GPIO pin for trigger
    int echoPin;    // GPIO pin for echo
    // Additional parameters (timing, etc.) can be added as required.
};

#endif // BUSCONFIG_H
