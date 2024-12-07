#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class PIDController
{
public:
    /**
     * @brief Constructor for PIDController.
     *
     * @param kp_ Proportional gain.
     * @param ki_ Integral gain.
     * @param kd_ Derivative gain.
     * @param integralLimit_ Maximum absolute value for integral term to prevent windup.
     */
    PIDController(float kp_, float ki_, float kd_, float integralLimit_ = 100.0f);

    /**
     * @brief Initialize the PID Controller by resetting its state.
     */
    void init();

    /**
     * @brief Compute the PID output based on setpoint and measured value.
     *
     * @param setpoint Desired setpoint.
     * @param measuredValue Current measured value.
     * @param dt Time delta in seconds.
     * @return float PID output.
     */
    float compute(float setpoint, float measuredValue, float dt);

    /**
     * @brief Reset the PID Controller's state.
     */
    void reset();

    /**
     * @brief Update PID parameters dynamically.
     *
     * @param kp_ New proportional gain.
     * @param ki_ New integral gain.
     * @param kd_ New derivative gain.
     */
    void updateParameters(float kp_, float ki_, float kd_);

private:
    float kp;
    float ki;
    float kd;
    float previousError;
    float integral;
    float integralLimit;
    SemaphoreHandle_t pidMutex; // Mutex for protecting PID computations

    /**
     * @brief Clamp a value between min and max.
     *
     * @param value Value to clamp.
     * @param minVal Minimum allowable value.
     * @param maxVal Maximum allowable value.
     * @return float Clamped value.
     */
    float clamp(float value, float minVal, float maxVal);
};

#endif // PIDCONTROLLER_H
