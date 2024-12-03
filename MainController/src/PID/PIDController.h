#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>
#include <mutex> // Include mutex for thread safety

class PIDController
{
public:
    PIDController(float kp, float ki, float kd, float integralLimit = 100.0f);
    void init();
    float compute(float setpoint, float measuredValue, float dt);
    void reset();
    // Additional methods as needed

private:
    float kp;
    float ki;
    float kd;
    float previousError;
    float integral;
    float integralLimit; // Maximum absolute value for integral term
    std::mutex pidMutex; // Mutex for protecting PID calculations
    // Private members for PID calculation
};

#endif // PIDCONTROLLER_H
