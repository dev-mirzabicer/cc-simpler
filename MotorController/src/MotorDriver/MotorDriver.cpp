#include "MotorDriver.h"
#include "../config.h"

// Constructor
MotorDriver::MotorDriver(uint8_t pwmPin_, uint8_t dirPin_, uint8_t encoderPinA_, uint8_t encoderPinB_)
    : pwmPin(pwmPin_), dirPin(dirPin_), encoder(encoderPinA_, encoderPinB_),
      lastEncoderCount(0), currentSpeed(0.0f),
      maxSpeed(LEFT_MOTOR_MAX_SPEED), minSpeed(LEFT_MOTOR_MIN_SPEED),
      wheelCircumference(WHEEL_CIRCUMFERENCE), encoderCountsPerRev(ENCODER_COUNTS_PER_REV)
{
    // Initialize mutex
    speedMutex = xSemaphoreCreateMutex();
    if (speedMutex == NULL)
    {
        Serial.println("MotorDriver: Failed to create speedMutex.");
    }
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

    Serial.println("MotorDriver initialized.");
}

// Set speed for motor (negative for reverse, positive for forward)
void MotorDriver::setSpeed(float speed)
{
    // Clamp speed to allowed range
    speed = clamp(speed, minSpeed, maxSpeed);

    // Determine direction based on speed sign
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
    {
        if (xSemaphoreTake(speedMutex, portMAX_DELAY) == pdTRUE)
        {
            currentSpeed = (digitalRead(dirPin) == HIGH) ? speed : -speed;
            xSemaphoreGive(speedMutex);
        }
    }

    Serial.print("Motor speed set to: ");
    Serial.print(currentSpeed);
    Serial.println(" m/s");
}

// Get current motor speed
float MotorDriver::getCurrentSpeed() const
{
    float speed = 0.0f;
    if (xSemaphoreTake(speedMutex, portMAX_DELAY) == pdTRUE)
    {
        speed = currentSpeed;
        xSemaphoreGive(speedMutex);
    }
    return speed;
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
    {
        if (xSemaphoreTake(speedMutex, portMAX_DELAY) == pdTRUE)
        {
            currentSpeed = speed;
            xSemaphoreGive(speedMutex);
        }
    }

    Serial.print("Motor speed updated to: ");
    Serial.print(speed);
    Serial.println(" m/s");
}

// Helper function to clamp a value between min and max
float MotorDriver::clamp(float value, float minVal, float maxVal) const
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

// Helper function to map a float from one range to another
float MotorDriver::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
