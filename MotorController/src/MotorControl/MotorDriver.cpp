#include "MotorDriver.h"

// Constructor
MotorDriver::MotorDriver(uint8_t pwmPin, uint8_t dirPin, uint8_t encoderPinA, uint8_t encoderPinB)
    : pwmPin(pwmPin), dirPin(dirPin),
      encoder(encoderPinA, encoderPinB),
      lastEncoderCount(0), currentSpeed(0.0f),
      maxSpeed(LEFT_MOTOR_MAX_SPEED), minSpeed(LEFT_MOTOR_MIN_SPEED),
      wheelCircumference(WHEEL_CIRCUMFERENCE), encoderCountsPerRev(ENCODER_COUNTS_PER_REV)
{
    // Initialize variables
}

// Initialize MotorDriver
void MotorDriver::init()
{
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    analogWrite(pwmPin, 0);    // Initialize PWM to 0
    digitalWrite(dirPin, LOW); // Initialize direction to LOW

    encoder.write(0); // Reset encoder count
    lastEncoderCount = 0;
    currentSpeed = 0.0f;
}

// Set speed for motor (negative for reverse, positive for forward)
void MotorDriver::setSpeed(float speed)
{
    // Clamp speed to allowed range
    speed = clamp(speed, minSpeed, maxSpeed);

    // Determine direction
    if (speed >= 0.0f)
    {
        digitalWrite(dirPin, HIGH); // Forward
    }
    else
    {
        digitalWrite(dirPin, LOW); // Reverse
        speed = -speed;            // Make speed positive for PWM mapping
    }

    // Map speed to PWM
    float pwmValue = mapFloat(speed, 0.0f, maxSpeed, 0.0f, 255.0f);
    pwmValue = clamp(pwmValue, 0.0f, 255.0f);
    analogWrite(pwmPin, (uint8_t)pwmValue);

    // Update currentSpeed based on direction
    currentSpeed = (digitalRead(dirPin) == HIGH) ? speed : -speed;
}

// Get current motor speed
float MotorDriver::getCurrentSpeed() const
{
    return currentSpeed;
}

// Update speed based on encoder counts
void MotorDriver::updateSpeed(float dt)
{
    long currentCount = encoder.read();
    long deltaCount = currentCount - lastEncoderCount;
    lastEncoderCount = currentCount;

    // Calculate rotations per second
    float rotations = (float)deltaCount / encoderCountsPerRev;
    float speed = (rotations * wheelCircumference) / dt; // m/s

    // Update currentSpeed
    currentSpeed = speed;
}
