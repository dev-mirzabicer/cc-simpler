#include "MotorDriver.h"
#include "../config.h"

// Constructor
MotorDriver::MotorDriver(uint8_t pwmPin_, uint8_t encoderPinA_, uint8_t encoderPinB_, uint8_t channel_)
    : pwmPin(pwmPin_), encoder(encoderPinA_, encoderPinB_),
      lastEncoderCount(0), currentSpeed(0.0f),
      wheelCircumference(WHEEL_CIRCUMFERENCE), encoderCountsPerRev(ENCODER_COUNTS_PER_REV),
      channel(channel_)
{
    // Initialize mutex
    speedMutex = xSemaphoreCreateMutex();
    if (speedMutex == NULL)
    {
        Serial.println("MotorDriver: Failed to create speedMutex.");
    }

    // Set min and max speed
    maxSpeed = 1.0f;  // m/s
    minSpeed = -1.0f; // m/s
}

// Initialize MotorDriver
void MotorDriver::init()
{
    // Setup LEDC for PWM
    ledcSetup(channel, PWM_FREQUENCY_MOTORS, PWM_RESOLUTION_MOTORS);
    ledcAttachPin(pwmPin, channel);

    // Initialize PWM to neutral (1.5 ms pulse)
    // Map 0 m/s to 1.5 ms pulse -> 3277 counts
    ledcWrite(channel, 3277);

    pinMode(pwmPin, OUTPUT);
    encoder.write(0); // Reset encoder count
    lastEncoderCount = 0;
    currentSpeed = 0.0f;

    Serial.println("MotorDriver initialized.");
}

// Set speed for motor (-1.0 to +1.0 m/s)
void MotorDriver::setSpeed(float speed)
{
    // Clamp speed to allowed range
    speed = clamp(speed, -1.0f, 1.0f);

    // Map speed to PWM value
    uint32_t pwmValue = mapFloat(speed, -1.0f, 1.0f, 1638, 4915); // 1.0m/s -> 2.0ms, -1.0m/s -> 1.0ms
    pwmValue = (uint32_t)clamp((float)pwmValue, 1638.0f, 4915.0f);

    // Write to PWM channel
    ledcWrite(channel, pwmValue);

    // Update currentSpeed based on speed command
    {
        if (xSemaphoreTake(speedMutex, portMAX_DELAY) == pdTRUE)
        {
            currentSpeed = speed;
            xSemaphoreGive(speedMutex);
        }
    }

    Serial.print("Motor speed set to: ");
    Serial.print(speed);
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
    float rotations = ((float)deltaCount / encoderCountsPerRev) / dt; // rev/s

    float speed = (rotations * wheelCircumference); // m/s

    // Update currentSpeed based on encoder
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
