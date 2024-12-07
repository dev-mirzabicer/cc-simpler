#include "PIDController.h"

// Constructor
PIDController::PIDController(float kp_val, float ki_val, float kd_val, float integralLimit_val)
    : kp(kp_val), ki(ki_val), kd(kd_val), previousError(0.0f), integral(0.0f), integralLimit(integralLimit_val)
{
    // Initialize mutex
    pidMutex = xSemaphoreCreateMutex();
    if (pidMutex == NULL)
    {
        Serial.println("Failed to create PIDController mutex.");
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

        // Anti-windup
        if (integral > integralLimit)
            integral = integralLimit;
        else if (integral < -integralLimit)
            integral = -integralLimit;

        float derivative = (error - previousError) / dt;
        previousError = error;

        float output = (kp * error) + (ki * integral) + (kd * derivative);

        xSemaphoreGive(pidMutex);
        return output;
    }
    else
    {
        // If unable to take semaphore, return zero or previous output
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
