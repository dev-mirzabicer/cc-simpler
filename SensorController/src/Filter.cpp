#include "Filter.h"
#include <cmath>

/**
 * @brief Apply a complementary filter to estimate yaw.
 *
 * Combines gyroscope integration and magnetometer reading to provide a stable yaw estimate.
 */
float Filter::complementaryYawFilter(float prevYawFiltered, float gyroZ, float magYaw, float dt, float alpha)
{
    // Integrate gyroscope data
    float gyroYawEstimate = prevYawFiltered + gyroZ * dt;

    // Normalize angles to [-pi, pi]
    auto normalizeAngle = [](float angle)
    {
        while (angle > M_PI)
            angle -= 2.0f * M_PI;
        while (angle < -M_PI)
            angle += 2.0f * M_PI;
        return angle;
    };

    gyroYawEstimate = normalizeAngle(gyroYawEstimate);
    magYaw = normalizeAngle(magYaw);

    // Complementary filter
    float filtered = alpha * gyroYawEstimate + (1.0f - alpha) * magYaw;
    filtered = normalizeAngle(filtered);
    return filtered;
}

/**
 * @brief Apply an exponential moving average filter for a given input.
 *
 * Smooths the input data by blending the previous filtered value with the new raw value.
 */
float Filter::exponentialMovingAverage(float prevValue, float rawValue, float lambda)
{
    if (std::isnan(rawValue))
    {
        // If rawValue is NAN, retain the previous filtered value
        return prevValue;
    }
    return lambda * prevValue + (1.0f - lambda) * rawValue;
}
