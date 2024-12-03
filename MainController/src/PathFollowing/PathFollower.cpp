#include "PathFollower.h"

// Constructor
PathFollower::PathFollower()
    : interESPComm(nullptr), localPathAdjuster(nullptr),
      kp_linear(1.0f), ki_linear(0.0f), kd_linear(0.1f),
      kp_angular(1.0f), ki_angular(0.0f), kd_angular(0.1f),
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
    // Initialize PID controllers with default parameters
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
    std::lock_guard<std::mutex> lock(motorMutex); // Ensure thread safety

    VelocityCommand cmd = {0, 0, 0, 0, 0, 0};

    if (globalPath.waypoints.empty())
    {
        // No waypoints to follow
        return cmd;
    }

    // Determine the target waypoint (closest ahead within a threshold)
    Waypoint target;
    float minDist = INFINITY;
    size_t targetIndex = currentWaypointIndex;

    for (size_t i = currentWaypointIndex; i < globalPath.waypoints.size(); ++i)
    {
        float dist = sqrt(pow(globalPath.waypoints[i].x - currentState.x, 2) +
                          pow(globalPath.waypoints[i].y - currentState.y, 2) +
                          pow(globalPath.waypoints[i].z - currentState.z, 2));
        if (dist < minDist)
        {
            minDist = dist;
            target = globalPath.waypoints[i];
            targetIndex = i;
        }

        if (dist < 1.0f) // Threshold to move to next waypoint
        {
            currentWaypointIndex = i + 1;
        }
    }

    // If all waypoints are reached
    if (currentWaypointIndex >= globalPath.waypoints.size())
    {
        // Set all commands to zero
        return cmd;
    }

    // Calculate errors
    float errorX = target.x - currentState.x;
    float errorY = target.y - currentState.y;
    float errorZ = target.z - currentState.z;
    float errorYaw = target.yaw - currentState.yaw;

    // Normalize yaw error to [-180, 180]
    if (errorYaw > 180.0f)
        errorYaw -= 360.0f;
    if (errorYaw < -180.0f)
        errorYaw += 360.0f;

    // Compute time step (dt)
    float dt = 0.1f; // 10 Hz loop rate

    // PID control for linear velocities
    float controlX = pidLinearX.compute(errorX, currentState.x, dt);
    float controlY = pidLinearY.compute(errorY, currentState.y, dt);
    float controlZ = pidLinearZ.compute(errorZ, currentState.z, dt);

    // PID control for angular velocities using angular rates
    float controlRoll = pidAngularX.compute(0.0f, currentState.roll, dt);   // target roll is 0
    float controlPitch = pidAngularY.compute(0.0f, currentState.pitch, dt); // target pitch is 0
    float controlYaw = pidAngularZ.compute(errorYaw, currentState.yaw, dt);

    // Assign controls to command
    cmd.linearX = controlX;
    cmd.linearY = controlY;
    cmd.linearZ = controlZ;
    cmd.angularX = controlRoll;
    cmd.angularY = controlPitch;
    cmd.angularZ = controlYaw;

    // Adjust commands based on local path adjustments
    if (localPathAdjuster)
    {
        VelocityCommand adjustedCmd = localPathAdjuster->adjustPath(currentState, globalPath, occupancyGrid);
        cmd.linearX += adjustedCmd.linearX;
        cmd.linearY += adjustedCmd.linearY;
        cmd.linearZ += adjustedCmd.linearZ;
        cmd.angularX += adjustedCmd.angularX;
        cmd.angularY += adjustedCmd.angularY;
        cmd.angularZ += adjustedCmd.angularZ;
    }

    // Clamp commands to maximum limits
    cmd.linearX = clamp(cmd.linearX, -10.0f, 10.0f);   // Example limits (m/s)
    cmd.linearY = clamp(cmd.linearY, -10.0f, 10.0f);   // Example limits (m/s)
    cmd.linearZ = clamp(cmd.linearZ, -5.0f, 5.0f);     // Pump control limits (m/s)
    cmd.angularX = clamp(cmd.angularX, -45.0f, 45.0f); // Degrees/s
    cmd.angularY = clamp(cmd.angularY, -45.0f, 45.0f); // Degrees/s
    cmd.angularZ = clamp(cmd.angularZ, -90.0f, 90.0f); // Degrees/s

    // Apply acceleration limits
    cmd.linearX = clamp(cmd.linearX, prevLinearX - maxLinearAccel * dt, prevLinearX + maxLinearAccel * dt);
    cmd.linearY = clamp(cmd.linearY, prevLinearY - maxLinearAccel * dt, prevLinearY + maxLinearAccel * dt);
    cmd.linearZ = clamp(cmd.linearZ, prevLinearZ - maxLinearAccel * dt, prevLinearZ + maxLinearAccel * dt);
    cmd.angularX = clamp(cmd.angularX, prevAngularX - maxAngularAccel * dt, prevAngularX + maxAngularAccel * dt);
    cmd.angularY = clamp(cmd.angularY, prevAngularY - maxAngularAccel * dt, prevAngularY + maxAngularAccel * dt);
    cmd.angularZ = clamp(cmd.angularZ, prevAngularZ - maxAngularAccel * dt, prevAngularZ + maxAngularAccel * dt);

    // Update previous commands
    prevLinearX = cmd.linearX;
    prevLinearY = cmd.linearY;
    prevLinearZ = cmd.linearZ;
    prevAngularX = cmd.angularX;
    prevAngularY = cmd.angularY;
    prevAngularZ = cmd.angularZ;

    // Store the last velocity command
    {
        std::lock_guard<std::mutex> lock(velocityCmdMutex);
        lastVelocityCmdStored = cmd;
    }

    return cmd;
}

float PathFollower::clamp(float value, float minVal, float maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

float PathFollower::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

VelocityCommand PathFollower::getLastVelocityCommand() const
{
    std::lock_guard<std::mutex> lock(velocityCmdMutex); // Ensure thread safety
    return lastVelocityCmdStored;
}
