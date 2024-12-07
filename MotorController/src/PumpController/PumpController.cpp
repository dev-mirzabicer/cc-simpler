#include "PumpController.h"
#include "../config.h"

// Constructor
PumpController::PumpController(uint8_t pwmPin_, uint8_t sensorPin_, uint8_t channel_)
    : pwmPin(pwmPin_), sensorPin(sensorPin_), pumpStatus(0.0f),
      channel(channel_),
      maxControl(1.0f), // 0.0 to 1.0
      minControl(0.0f)
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
    // Setup LEDC for PWM
    ledcSetup(channel, PWM_FREQUENCY_PUMPS, PWM_RESOLUTION_PUMPS);
    ledcAttachPin(pwmPin, channel);

    // Initialize PWM to 0 (pump off)
    ledcWrite(channel, 0);

    pinMode(pwmPin, OUTPUT);
    pinMode(sensorPin, INPUT);

    // Initialize pump status to 0
    {
        if (xSemaphoreTake(pumpMutex, portMAX_DELAY) == pdTRUE)
        {
            pumpStatus = 0.0f;
            xSemaphoreGive(pumpMutex);
        }
    }

    Serial.println("PumpController initialized.");
}

// Set control for pump (0.0 to +1.0)
void PumpController::setControl(float control)
{
    // Clamp control to allowed range
    control = clamp(control, minControl, maxControl);

    // Map control to PWM value (0 to 255 for 8-bit resolution)
    uint32_t pwmValue = mapFloat(control, 0.0f, maxControl, 0, 255);
    pwmValue = (uint32_t)clamp((float)pwmValue, 0.0f, 255.0f);

    // Write to PWM channel
    ledcWrite(channel, pwmValue);

    // Update pumpStatus based on control
    {
        if (xSemaphoreTake(pumpMutex, portMAX_DELAY) == pdTRUE)
        {
            pumpStatus = control;
            xSemaphoreGive(pumpMutex);
        }
    }

    Serial.print("Pump control set to: ");
    Serial.print(pumpStatus);
    Serial.println(" (abstract units)");
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
    // Map voltage to pump status (abstract units)
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
