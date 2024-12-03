#include "PIDController.h"

// Constructor
PIDController::PIDController(float kp_val, float ki_val, float kd_val, float integralLimit_val)
    : kp(kp_val), ki(ki_val), kd(kd_val), previousError(0.0f), integral(0.0f), integralLimit(integralLimit_val)
{
    // Initialize variables
}

// Initialize PID Controller
void PIDController::init()
{
    reset();
}

// Compute PID Output
float PIDController::compute(float setpoint, float measuredValue, float dt)
{
    std::lock_guard<std::mutex> lock(pidMutex);
    float error = setpoint - measuredValue;
    integral += error * dt;

    // Anti-windup
    if (integral > integralLimit)
        integral = integralLimit;
    else if (integral < -integralLimit)
        integral = -integralLimit;

    float derivative = (error - previousError) / dt;
    previousError = error;

    return (kp * error) + (ki * integral) + (kd * derivative);
}

// Reset PID Controller
void PIDController::reset()
{
    std::lock_guard<std::mutex> lock(pidMutex);
    previousError = 0.0f;
    integral = 0.0f;
}
