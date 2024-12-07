#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include <Encoder.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class MotorDriver
{
public:
    /**
     * @brief Constructor for MotorDriver.
     *
     * @param pwmPin GPIO pin for PWM control of the motor.
     * @param encoderPinA GPIO pin A for motor encoder.
     * @param encoderPinB GPIO pin B for motor encoder.
     * @param channel_ LEDC channel number for this motor.
     */
    MotorDriver(uint8_t pwmPin, uint8_t encoderPinA, uint8_t encoderPinB, uint8_t channel_);

    /**
     * @brief Initialize the MotorDriver hardware and state.
     */
    void init();

    /**
     * @brief Set the desired speed for the motor.
     *
     * @param speed Desired speed in m/s (clamped within min and max speed).
     */
    void setSpeed(float speed);

    /**
     * @brief Get the current speed of the motor based on encoder feedback.
     *
     * @return float Current speed in m/s.
     */
    float getCurrentSpeed() const;

    /**
     * @brief Update the current speed based on encoder counts.
     *
     * @param dt Time delta in seconds since the last update.
     */
    void updateSpeed(float dt);

private:
    uint8_t pwmPin;
    Encoder encoder;
    long lastEncoderCount;
    float currentSpeed;
    uint8_t channel; // LEDC channel number

    // Control limits
    float maxSpeed;
    float minSpeed;

    // Physical constants
    float wheelCircumference;
    int encoderCountsPerRev;

    // Mutex for thread-safe access to currentSpeed
    mutable SemaphoreHandle_t speedMutex;

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

#endif // MOTORDRIVER_H
