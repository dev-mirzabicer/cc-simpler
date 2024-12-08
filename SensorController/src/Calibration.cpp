#include "Calibration.h"
#include <Arduino.h> // For Serial logging
#include <cmath>

/**
 * @brief Constructor for Calibration.
 * Initializes calibration parameters to default values.
 */
Calibration::Calibration()
    : imuCalibrated(false), magCalibrated(false), pressCalibrated(false),
      imuAxBias(0.0f), imuAyBias(0.0f), imuAzBias(0.0f), imuGyroZBias(0.0f),
      magOffsetX(0.0f), magOffsetY(0.0f), magScaleX(1.0f), magScaleY(1.0f),
      pressureRef(101325.0f), fluidDensity(1025.0f) // Typical seawater density ~1025 kg/m³
{
}

/**
 * @brief Perform initial calibration retrieval from the sensors.
 *
 * Retrieves calibration parameters from IMU, Magnetometer, and PressureSensor.
 */
SensorStatus Calibration::initCalibration(IMU *imu, Magnetometer *mag, PressureSensor *press)
{
    // IMU biases
    {
        float bx, by, bz;
        SensorStatus s = imu->getAccelerometerBias(&bx, &by, &bz);
        if (s == SensorStatus::SUCCESS)
        {
            imuAxBias = bx;
            imuAyBias = by;
            imuAzBias = bz;
        }
        else
        {
            Serial.println("Calibration: Failed to get IMU accelerometer biases.");
            return s;
        }

        float bgz;
        s = imu->getGyroZBias(&bgz);
        if (s == SensorStatus::SUCCESS)
        {
            imuGyroZBias = bgz;
            imuCalibrated = true;
        }
        else
        {
            Serial.println("Calibration: Failed to get IMU gyroscope Z bias.");
            return s;
        }
    }

    // Magnetometer calibration parameters
    {
        float ox, oy, sx, sy;
        SensorStatus s = mag->getCalibrationParameters(&ox, &oy, &sx, &sy);
        if (s == SensorStatus::SUCCESS)
        {
            magOffsetX = ox;
            magOffsetY = oy;
            magScaleX = sx;
            magScaleY = sy;
            magCalibrated = true;
        }
        else
        {
            Serial.println("Calibration: Failed to get Magnetometer calibration parameters.");
            return s;
        }
    }

    // Pressure sensor calibration parameters
    {
        float refP, dens;
        SensorStatus s = press->getCalibrationParameters(&refP, &dens);
        if (s == SensorStatus::SUCCESS)
        {
            pressureRef = refP;
            fluidDensity = dens;
            pressCalibrated = true;
        }
        else
        {
            Serial.println("Calibration: Failed to get Pressure Sensor calibration parameters.");
            return s;
        }
    }

    return SensorStatus::SUCCESS;
}

/**
 * @brief Apply IMU calibration to raw sensor data.
 */
void Calibration::applyIMUCalibration(float &ax, float &ay, float &az, float &gyroZ)
{
    if (!imuCalibrated)
    {
        // If not calibrated, do not modify the data
        return;
    }

    ax -= imuAxBias;
    ay -= imuAyBias;
    az -= imuAzBias;
    gyroZ -= imuGyroZBias;
}

/**
 * @brief Apply Magnetometer calibration to yaw.
 *
 * Currently, since the Magnetometer returns calibrated yaw, this function does not modify yaw.
 * If raw magnetometer data were available, this function would apply offset and scaling.
 */
void Calibration::applyMagCalibration(float &yaw)
{
    if (!magCalibrated || std::isnan(yaw))
    {
        return;
    }

    // Since Magnetometer wrapper provides calibrated yaw, no further calibration is needed here.
    // This function is retained for potential future calibration steps.
}

/**
 * @brief Apply Pressure calibration if needed.
 *
 * Currently, since PressureSensor returns calibrated depth, this function does not modify depth.
 * If raw pressure data were available, this function would convert pressure to depth.
 */
void Calibration::applyPressureCalibration(float &depth)
{
    if (!pressCalibrated || std::isnan(depth))
    {
        return;
    }

    // Since PressureSensor wrapper provides calibrated depth, no further calibration is needed here.
    // This function is retained for potential future calibration steps.
}

/**
 * @brief Retrieve the current IMU gyroscope Z bias.
 */
float Calibration::getIMUGyroZBias() const
{
    return imuGyroZBias;
}
