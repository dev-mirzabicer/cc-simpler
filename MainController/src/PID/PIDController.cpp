#include "PIDController.h"

// Constructor with optional integral limit
PIDController::PIDController(float kp_val, float ki_val, float kd_val, float integralLim)
    : kp(kp_val), ki(ki_val), kd(kd_val), previousError(0.0f), integral(0.0f), integralLimit(integralLim)
{
    // Initialize variables
}

// Initialize PID controller
void PIDController::init()
{
    reset();
}

// Compute PID output with anti-windup and variable time step
float PIDController::compute(float setpoint, float measuredValue, float dt)
{
    std::lock_guard<std::mutex> lock(pidMutex); // Ensure thread safety

    float error = setpoint - measuredValue;
    integral += error * dt;

    // Anti-windup: Clamp the integral term
    if (integral > integralLimit)
        integral = integralLimit;
    else if (integral < -integralLimit)
        integral = -integralLimit;

    float derivative = (error - previousError) / dt;
    previousError = error;

    return (kp * error) + (ki * integral) + (kd * derivative);
}

// Reset PID controller
void PIDController::reset()
{
    std::lock_guard<std::mutex> lock(pidMutex); // Ensure thread safety
    previousError = 0.0f;
    integral = 0.0f;
}
