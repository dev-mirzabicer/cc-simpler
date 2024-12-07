#ifndef PUMPCONTROLLER_H
#define PUMPCONTROLLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class PumpController
{
public:
    /**
     * @brief Constructor for PumpController.
     *
     * @param pwmPin GPIO pin for PWM control of the pump.
     * @param sensorPin GPIO pin for reading pump sensor feedback (ADC).
     * @param channel_ LEDC channel number for this pump.
     */
    PumpController(uint8_t pwmPin, uint8_t sensorPin, uint8_t channel_);

    /**
     * @brief Initialize the PumpController hardware and state.
     */
    void init();

    /**
     * @brief Set the control value for the pump.
     *
     * @param control Desired pump control value (0.0 to 1.0).
     */
    void setControl(float control);

    /**
     * @brief Get the current pump status based on sensor feedback.
     *
     * @return float Current pump status (abstract units).
     */
    float getCurrentPumpStatus() const;

    /**
     * @brief Update pump status based on sensor readings.
     */
    void updateStatus();

private:
    uint8_t pwmPin;
    uint8_t sensorPin;
    float pumpStatus;
    uint8_t channel; // LEDC channel number

    // Control limits
    float maxControl;
    float minControl;

    // Mutex for thread-safe access to pumpStatus
    mutable SemaphoreHandle_t pumpMutex;

    /**
     * @brief Clamp a value between min and max.
     *
     * @param value Value to clamp.
     * @param minVal Minimum allowable value.
     * @param maxVal Maximum allowable value.
     * @return float Clamped value.
     */
    float clamp(float value, float minVal, float maxVal) const;

    /**
     * @brief Map a float from one range to another.
     *
     * @param x Input value.
     * @param in_min Input range minimum.
     * @param in_max Input range maximum.
     * @param out_min Output range minimum.
     * @param out_max Output range maximum.
     * @return float Mapped value.
     */
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const;
};

#endif // PUMPCONTROLLER_H
