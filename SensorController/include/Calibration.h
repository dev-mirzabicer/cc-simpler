#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "SensorStatus.h"
#include "../libraries/IMU/IMU.h"
#include "../libraries/Magnetometer/Magnetometer.h"
#include "../libraries/PressureSensor/PressureSensor.h"

/**
 * @brief The Calibration class handles retrieving and storing calibration parameters from sensors.
 */
class Calibration
{
public:
    /**
     * @brief Constructor for Calibration.
     * Initializes calibration parameters to default values.
     */
    Calibration();

    /**
     * @brief Perform initial calibration retrieval from the sensors.
     *
     * @param imu Pointer to IMU object.
     * @param mag Pointer to Magnetometer object.
     * @param press Pointer to PressureSensor object.
     * @return SensorStatus SUCCESS if all calibration parameters are successfully retrieved, otherwise appropriate error code.
     */
    SensorStatus initCalibration(IMU *imu, Magnetometer *mag, PressureSensor *press);

    /**
     * @brief Apply IMU calibration to raw sensor data.
     *
     * @param ax Reference to acceleration X (m/s²).
     * @param ay Reference to acceleration Y (m/s²).
     * @param az Reference to acceleration Z (m/s²).
     * @param gyroZ Reference to angular velocity around Z-axis (rad/s).
     */
    void applyIMUCalibration(float &ax, float &ay, float &az, float &gyroZ);

    /**
     * @brief Apply Magnetometer calibration to yaw.
     *
     * @param yaw Reference to yaw angle (rad).
     */
    void applyMagCalibration(float &yaw);

    /**
     * @brief Apply Pressure calibration if needed.
     *
     * Adjusts depth based on calibration parameters if required.
     *
     * @param depth Reference to depth (m).
     */
    void applyPressureCalibration(float &depth);

    /**
     * @brief Retrieve the current IMU gyroscope Z bias.
     *
     * @return float Gyroscope Z bias (rad/s).
     */
    float getIMUGyroZBias() const;

private:
    bool imuCalibrated;
    bool magCalibrated;
    bool pressCalibrated;

    // IMU calibration parameters
    float imuAxBias;
    float imuAyBias;
    float imuAzBias;
    float imuGyroZBias;

    // Magnetometer calibration parameters
    float magOffsetX;
    float magOffsetY;
    float magScaleX;
    float magScaleY;

    // Pressure sensor calibration parameters
    float pressureRef;
    float fluidDensity;
};

#endif // CALIBRATION_H
