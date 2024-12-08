// PumpController.h
#ifndef PUMP_CONTROLLER_H
#define PUMP_CONTROLLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @brief The PumpController class controls a single pump via PWM and direction control, and reads sensor status.
 */
class PumpController
{
public:
    /**
     * @brief Constructor for PumpController.
     * @param pwmPin_ GPIO pin for PWM signal.
     * @param dirPin_ GPIO pin for direction control.
     * @param sensorPin_ GPIO pin for sensor input.
     * @param channel_ LEDC channel for PWM.
     */
    PumpController(uint8_t pwmPin_, uint8_t dirPin_, uint8_t sensorPin_, uint8_t channel_);

    /**
     * @brief Initialize the pump controller hardware.
     */
    void init();

    /**
     * @brief Set the pump control.
     * @param control Control value in [0.0, 1.0]. Higher values increase pump speed.
     */
    void setControl(float control);

    /**
     * @brief Get the current pump status.
     * @return Current status as an abstract unit.
     */
    float getCurrentPumpStatus() const;

    /**
     * @brief Update pump status based on sensor readings.
     */
    void updateStatus();

private:
    uint8_t pwmPin;
    uint8_t dirPin;
    uint8_t sensorPin;
    uint8_t channel;
    SemaphoreHandle_t pumpMutex;
    float pumpStatus;

    /**
     * @brief Clamp a value between min and max.
     * @param value Value to clamp.
     * @param minVal Minimum allowable value.
     * @param maxVal Maximum allowable value.
     * @return Clamped value.
     */
    float clamp(float value, float minVal, float maxVal) const;

    /**
     * @brief Map control value to PWM counts.
     * @param control Control value in [0.0, 1.0].
     * @return PWM count value.
     */
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const;
};

#endif // PUMP_CONTROLLER_H
