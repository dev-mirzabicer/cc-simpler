#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Ensure inclusion of FreeRTOS headers for SemaphoreHandle_t

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
    SemaphoreHandle_t pidMutex;
};

#endif // PIDCONTROLLER_H
