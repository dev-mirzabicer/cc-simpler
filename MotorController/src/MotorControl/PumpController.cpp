#include "PumpController.h"

// Constructor
PumpController::PumpController(uint8_t pwmPin, uint8_t sensorPin)
    : pwmPin(pwmPin), sensorPin(sensorPin),
      pumpStatus(0.0f),
      maxControl(PUMP_MAX_CONTROL), minControl(PUMP_MIN_CONTROL)
{
    // Initialize variables
}

// Initialize PumpController
void PumpController::init()
{
    pinMode(pwmPin, OUTPUT);
    analogWrite(pwmPin, 0);    // Initialize PWM to 0
    pinMode(sensorPin, INPUT); // Initialize sensor pin as input

    pumpStatus = 0.0f; // Initialize pump status to neutral
}

// Set control for pump (negative for suction, positive for pumping out)
void PumpController::setControl(float control)
{
    // Clamp control to allowed range
    control = clamp(control, minControl, maxControl);

    // Determine direction based on control sign
    if (control >= 0.0f)
    {
        // Forward direction (pumping out)
        // If direction control is required, implement accordingly
    }
    else
    {
        // Reverse direction (suction)
        control = -control; // Make control positive for PWM mapping
        // If direction control is required, implement accordingly
    }

    // Map control to PWM
    float pwmValue = mapFloat(control, 0.0f, maxControl, 0.0f, 255.0f);
    pwmValue = clamp(pwmValue, 0.0f, 255.0f);
    analogWrite(pwmPin, (uint8_t)pwmValue);

    // Update pumpStatus based on control
    pumpStatus = (control > 0.0f) ? control : -control;
}

// Get current pump status
float PumpController::getCurrentPumpStatus() const
{
    return pumpStatus;
}

// Read pump sensor (e.g., pressure sensor via ADC)
float PumpController::readSensor() const
{
    // Example implementation: read analog value and map to pump status
    int sensorValue = analogRead(sensorPin);
    // sensorValue ranges from 0-4095 (12-bit ADC)
    float voltage = sensorValue * (3.3f / 4095.0f); // Convert to voltage
    // Map voltage to pump status (m/s) based on calibration
    float status = mapFloat(clamp(voltage, 0.0f, 3.3f), 0.0f, 3.3f, minControl, maxControl);
    return status;
}

// Update pump status based on sensor readings
void PumpController::updateStatus()
{
    pumpStatus = readSensor();
}
