#include "PumpController.h"
#include "../config.h"

// Constructor
PumpController::PumpController(uint8_t pwmPin_, uint8_t sensorPin_)
    : pwmPin(pwmPin_), sensorPin(sensorPin_), pumpStatus(0.0f),
      maxControl(PUMP_MAX_CONTROL), minControl(PUMP_MIN_CONTROL)
{
    // Initialize mutex
    pumpMutex = xSemaphoreCreateMutex();
    if (pumpMutex == NULL)
    {
        Serial.println("PumpController: Failed to create pumpMutex.");
    }
}

// Initialize PumpController
void PumpController::init()
{
    pinMode(pwmPin, OUTPUT);
    analogWrite(pwmPin, 0);    // Initialize PWM to 0
    pinMode(sensorPin, INPUT); // Initialize sensor pin as input

    // Initialize pump status to neutral
    {
        if (xSemaphoreTake(pumpMutex, portMAX_DELAY) == pdTRUE)
        {
            pumpStatus = 0.0f;
            xSemaphoreGive(pumpMutex);
        }
    }

    Serial.println("PumpController initialized.");
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
        digitalWrite(pwmPin, HIGH); // Set direction to forward
    }
    else
    {
        // Reverse direction (suction)
        digitalWrite(pwmPin, LOW); // Set direction to reverse
        control = -control;        // Make control positive for PWM mapping
    }

    // Map control to PWM value
    float pwmValue = mapFloat(control, 0.0f, maxControl, 0.0f, 255.0f);
    pwmValue = clamp(pwmValue, 0.0f, 255.0f);
    analogWrite(pwmPin, (uint8_t)pwmValue);

    // Update pumpStatus based on control
    {
        if (xSemaphoreTake(pumpMutex, portMAX_DELAY) == pdTRUE)
        {
            pumpStatus = (control > 0.0f) ? control : -control;
            xSemaphoreGive(pumpMutex);
        }
    }

    Serial.print("Pump control set to: ");
    Serial.println(pumpStatus);
}

// Get current pump status
float PumpController::getCurrentPumpStatus() const
{
    float status = 0.0f;
    if (xSemaphoreTake(pumpMutex, portMAX_DELAY) == pdTRUE)
    {
        status = pumpStatus;
        xSemaphoreGive(pumpMutex);
    }
    return status;
}

// Read pump sensor (e.g., pressure sensor via ADC)
float PumpController::readSensor() const
{
    // Example implementation: read analog value and map to pump status
    int sensorValue = analogRead(sensorPin);
    // Assuming sensorValue ranges from 0-4095 (12-bit ADC)
    float voltage = sensorValue * (3.3f / 4095.0f); // Convert to voltage
    // Map voltage to pump status (m/s) based on calibration
    float status = mapFloat(clamp(voltage, 0.0f, 3.3f), 0.0f, 3.3f, minControl, maxControl);
    return status;
}

// Update pump status based on sensor readings
void PumpController::updateStatus()
{
    float status = readSensor();
    if (xSemaphoreTake(pumpMutex, portMAX_DELAY) == pdTRUE)
    {
        pumpStatus = status;
        xSemaphoreGive(pumpMutex);
    }

    Serial.print("Pump status updated to: ");
    Serial.println(pumpStatus);
}

// Helper function to clamp a value between min and max
float PumpController::clamp(float value, float minVal, float maxVal) const
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

// Helper function to map a float from one range to another
float PumpController::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
