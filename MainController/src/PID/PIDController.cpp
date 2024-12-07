#include "PathFollower.h"
#include "../config.h"

// Constructor
PathFollower::PathFollower()
    : interESPComm(nullptr), localPathAdjuster(nullptr),
      kp_linear(LEFT_MOTOR_KP), ki_linear(LEFT_MOTOR_KI), kd_linear(LEFT_MOTOR_KD),
      kp_angular(RIGHT_MOTOR_KP), ki_angular(RIGHT_MOTOR_KI), kd_angular(RIGHT_MOTOR_KD),
      currentWaypointIndex(0),
      pidLinearX(kp_linear, ki_linear, kd_linear, 100.0f),
      pidLinearY(kp_linear, ki_linear, kd_linear, 100.0f),
      pidLinearZ(kp_linear, ki_linear, kd_linear, 50.0f),
      pidAngularX(kp_angular, ki_angular, kd_angular, 50.0f),
      pidAngularY(kp_angular, ki_angular, kd_angular, 50.0f),
      pidAngularZ(kp_angular, ki_angular, kd_angular, 50.0f),
      prevLinearX(0.0f),
      prevLinearY(0.0f),
      prevLinearZ(0.0f),
      prevAngularX(0.0f),
      prevAngularY(0.0f),
      prevAngularZ(0.0f),
      maxLinearAccel(5.0f),  // m/s²
      maxAngularAccel(45.0f) // degrees/s²
{
    // Initialize PID controllers with parameters from config
    velocityCmdMutex = xSemaphoreCreateMutex();
    if (velocityCmdMutex == NULL)
    {
        Serial.println("Failed to create PathFollower mutex.");
    }
}

void PathFollower::init(InterESPCommunication *comm, LocalPathAdjuster *adjuster)
{
    interESPComm = comm;
    localPathAdjuster = adjuster;

    // Initialize PID controllers
    pidLinearX.init();
    pidLinearY.init();
    pidLinearZ.init();
    pidAngularX.init();
    pidAngularY.init();
    pidAngularZ.init();
}

VelocityCommand PathFollower::followPath(const State &currentState, const Path &globalPath, const OccupancyGrid &occupancyGrid)
{
    // TODO Path Following Logic

    // Placeholder for computed commands
    VelocityCommand cmd = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f}; // Example values

    // Update lastVelocityCmdStored with mutex
    if (xSemaphoreTake(velocityCmdMutex, portMAX_DELAY) == pdTRUE)
    {
        lastVelocityCmdStored = cmd;
        xSemaphoreGive(velocityCmdMutex);
    }

    return cmd;
}

VelocityCommand PathFollower::getLastVelocityCommand() const
{
    VelocityCommand cmd = {0, 0, 0, 0, 0, 0};
    if (xSemaphoreTake(velocityCmdMutex, portMAX_DELAY) == pdTRUE)
    {
        cmd = lastVelocityCmdStored;
        xSemaphoreGive(velocityCmdMutex);
    }
    return cmd;
}

// Helper function to clamp a value between min and max
float PathFollower::clamp(float value, float minVal, float maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

// Helper function to map a float from one range to another
float PathFollower::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
