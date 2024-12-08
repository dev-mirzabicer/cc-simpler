#include "MotorDriver.h"
#include "../config.h"

// Constructor
MotorDriver::MotorDriver(uint8_t pwmPin_, uint8_t dirPin_, uint8_t encoderPinA_, uint8_t encoderPinB_, uint8_t channel_)
    : pwmPin(pwmPin_), dirPin(dirPin_), encoder(encoderPinA_, encoderPinB_),
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
}

// Initialize MotorDriver
void MotorDriver::init()
{
    // Setup LEDC for PWM
    ledcSetup(channel, PWM_FREQUENCY_MOTORS, PWM_RESOLUTION_MOTORS);
    ledcAttachPin(pwmPin, channel);

    // Initialize PWM to neutral (1.5 ms pulse)
    ledcWrite(channel, 4915); // 1.5ms corresponds to ~4915 counts

    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    // Initialize direction to forward
    digitalWrite(dirPin, HIGH);

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

    // Map speed to PWM value
    uint32_t pwmValue = mapSpeedToPWM(speed);                      // 0 to 65535 for 16-bit
    pwmValue = (uint32_t)clamp((float)pwmValue, 3277.0f, 6553.0f); // 1.0ms to 2.0ms

    // Write to PWM channel
    ledcWrite(channel, pwmValue);

    // Update currentSpeed based on speed command
    {
        if (xSemaphoreTake(speedMutex, portMAX_DELAY) == pdTRUE)
        {
            currentSpeed = (speed * (speed >= 0.0f ? 1.0f : -1.0f));
            xSemaphoreGive(speedMutex);
        }
    }

    Serial.print("Motor speed set to: ");
    Serial.print(speed);
    Serial.println(" m/s");
}

// Helper function to map speed to PWM
uint32_t MotorDriver::mapSpeedToPWM(float speed) const
{
    // Map speed from 0.0f to 1.0f m/s to PWM counts from 3277 (1.0ms) to 6553 (2.0ms)
    // 1ms pulse: 3277 counts
    // 2ms pulse: 6553 counts
    // Linear mapping

    // Ensure speed is between 0 and 1.0f
    speed = clamp(speed, 0.0f, 1.0f);

    // Map speed to PWM counts
    uint32_t pwmCount = (uint32_t)(3277 + (speed * (6553 - 3277)));
    return pwmCount;
}

// Get current speed
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

    float speed_m_s = (rotations * wheelCircumference); // m/s

    // Update currentSpeed based on encoder
    {
        if (xSemaphoreTake(speedMutex, portMAX_DELAY) == pdTRUE)
        {
            currentSpeed = speed_m_s;
            xSemaphoreGive(speedMutex);
        }
    }

    Serial.print("Motor speed updated to: ");
    Serial.print(speed_m_s);
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
