#ifndef PUMPCONTROLLER_H
#define PUMPCONTROLLER_H

#include <Arduino.h>
#include "../Utils/Utilities.h"

class PumpController
{
public:
    PumpController(uint8_t pwmPin, uint8_t sensorPin);
    void init();
    void setControl(float control);     // Control in m/s (-5 to +5 m/s)
    float getCurrentPumpStatus() const; // Current pump status in m/s
    void updateStatus();                // Update pump status based on sensor

private:
    uint8_t pwmPin;
    uint8_t sensorPin;
    float pumpStatus; // Current pump status in m/s

    // Calibration parameters
    float maxControl; // Maximum control in m/s corresponding to PWM=255
    float minControl; // Minimum control in m/s corresponding to PWM=0

    // Sensor reading
    float readSensor() const; // Read pump sensor (e.g., from ADC)
};

#endif // PUMPCONTROLLER_H
