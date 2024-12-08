#ifndef FILTER_H
#define FILTER_H

#include <cstdint>

/**
 * @brief The Filter class provides static methods for data filtering.
 *
 * Implementations include:
 * - Complementary filter for yaw using yaw from magnetometer and gyroZ integration.
 * - Exponential moving average for linear accelerations, depth, and sonar distances.
 */
class Filter
{
public:
    /**
     * @brief Apply a complementary filter to estimate yaw.
     *
     * @param prevYawFiltered The previous filtered yaw value (rad).
     * @param gyroZ The gyroscope Z-axis angular velocity (rad/s).
     * @param magYaw The magnetometer yaw angle (rad).
     * @param dt The time step (s).
     * @param alpha The complementary filter parameter (0 < alpha < 1), high alpha leans toward gyro.
     * @return float The updated filtered yaw in rad.
     */
    static float complementaryYawFilter(float prevYawFiltered, float gyroZ, float magYaw, float dt, float alpha);

    /**
     * @brief Apply an exponential moving average filter for a given input.
     *
     * @param prevValue The previously filtered value.
     * @param rawValue The new raw input value.
     * @param lambda The smoothing factor (0 < lambda < 1). High lambda = more smoothing.
     * @return float The updated filtered value.
     */
    static float exponentialMovingAverage(float prevValue, float rawValue, float lambda);
};

#endif // FILTER_H
