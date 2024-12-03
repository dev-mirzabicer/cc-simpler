#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include "../Utils/Utilities.h"
#include <Encoder.h> // Include Encoder library

class MotorDriver
{
public:
    MotorDriver(uint8_t pwmPin, uint8_t dirPin, uint8_t encoderPinA, uint8_t encoderPinB);
    void init();
    void setSpeed(float speed);    // Speed in m/s (-maxSpeed to +maxSpeed)
    float getCurrentSpeed() const; // Current speed in m/s
    void updateSpeed(float dt);    // Update speed based on encoder counts

private:
    uint8_t pwmPin;
    uint8_t dirPin;
    Encoder encoder; // Encoder instance
    long lastEncoderCount;
    float currentSpeed; // Current speed in m/s

    // Calibration parameters
    float maxSpeed; // Maximum speed in m/s corresponding to PWM=255
    float minSpeed; // Minimum speed in m/s corresponding to PWM=0

    // Wheel and encoder parameters
    float wheelCircumference; // In meters
    uint16_t encoderCountsPerRev;
};

#endif // MOTORDRIVER_H
