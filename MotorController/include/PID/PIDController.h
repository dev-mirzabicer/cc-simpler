#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>
#include <mutex>

class PIDController
{
public:
    PIDController(float kp, float ki, float kd, float integralLimit = 100.0f);
    void init();
    float compute(float setpoint, float measuredValue, float dt);
    void reset();

private:
    float kp;
    float ki;
    float kd;
    float previousError;
    float integral;
    float integralLimit;
    std::mutex pidMutex;
};

#endif // PIDCONTROLLER_H
