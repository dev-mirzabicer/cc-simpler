// MotorDriver.h
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <Encoder.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @brief The MotorDriver class controls a single motor via PWM and direction control, and reads encoder counts.
 */
class MotorDriver
{
public:
    /**
     * @brief Constructor for MotorDriver.
     * @param pwmPin_ GPIO pin for PWM signal.
     * @param dirPin_ GPIO pin for direction control.
     * @param encoderPinA_ GPIO pin A for encoder.
     * @param encoderPinB_ GPIO pin B for encoder.
     * @param channel_ LEDC channel for PWM.
     */
    MotorDriver(uint8_t pwmPin_, uint8_t dirPin_, uint8_t encoderPinA_, uint8_t encoderPinB_, uint8_t channel_);

    /**
     * @brief Initialize the motor driver hardware.
     */
    void init();

    /**
     * @brief Set the motor speed.
     * @param speed Speed in m/s (-1.0 to +1.0). Negative for reverse.
     */
    void setSpeed(float speed);

    /**
     * @brief Get the current motor speed.
     * @return Current speed in m/s.
     */
    float getCurrentSpeed() const;

    /**
     * @brief Update the motor speed based on encoder counts.
     * @param dt Time delta in seconds.
     */
    void updateSpeed(float dt);

private:
    uint8_t pwmPin;
    uint8_t dirPin;
    Encoder encoder;
    uint8_t channel;
    SemaphoreHandle_t speedMutex;
    long lastEncoderCount;
    float currentSpeed;
    float wheelCircumference;
    int encoderCountsPerRev;

    /**
     * @brief Clamp a value between min and max.
     * @param value Value to clamp.
     * @param minVal Minimum allowable value.
     * @param maxVal Maximum allowable value.
     * @return Clamped value.
     */
    float clamp(float value, float minVal, float maxVal) const;

    /**
     * @brief Map speed to PWM counts.
     * @param speed Speed in m/s (0.0 to 1.0).
     * @return PWM count value.
     */
    uint32_t mapSpeedToPWM(float speed) const;
};

#endif // MOTOR_DRIVER_H
