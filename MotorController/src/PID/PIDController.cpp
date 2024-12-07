#include "PIDController.h"

// Constructor
PIDController::PIDController(float kp_, float ki_, float kd_, float integralLimit_)
    : kp(kp_), ki(ki_), kd(kd_), previousError(0.0f), integral(0.0f), integralLimit(integralLimit_)
{
    // Initialize mutex
    pidMutex = xSemaphoreCreateMutex();
    if (pidMutex == NULL)
    {
        Serial.println("PIDController: Failed to create mutex.");
    }
}

// Initialize PID Controller
void PIDController::init()
{
    reset();
}

// Compute PID Output
float PIDController::compute(float setpoint, float measuredValue, float dt)
{
    if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE)
    {
        float error = setpoint - measuredValue;
        integral += error * dt;

        // Anti-windup via clamping
        integral = clamp(integral, -integralLimit, integralLimit);

        float derivative = (error - previousError) / dt;
        previousError = error;

        float output = (kp * error) + (ki * integral) + (kd * derivative);

        xSemaphoreGive(pidMutex);
        return output;
    }
    else
    {
        // If mutex not acquired, return zero to prevent unintended actuator behavior
        return 0.0f;
    }
}

// Reset PID Controller
void PIDController::reset()
{
    if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE)
    {
        previousError = 0.0f;
        integral = 0.0f;
        xSemaphoreGive(pidMutex);
    }
}

// Update PID Parameters
void PIDController::updateParameters(float kp_, float ki_, float kd_)
{
    if (xSemaphoreTake(pidMutex, portMAX_DELAY) == pdTRUE)
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;
        xSemaphoreGive(pidMutex);
    }
}

// Helper function to clamp a value between min and max
float PIDController::clamp(float value, float minVal, float maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}
