#include "LocalPathAdjuster.h"

// Constructor
LocalPathAdjuster::LocalPathAdjuster()
    : maxLinearSpeed(5.0f), minLinearSpeed(-5.0f),
      maxLinearAccel(2.0f), maxLinearDecel(2.0f),
      maxAngularSpeed(90.0f), minAngularSpeed(-90.0f),
      maxAngularAccel(45.0f), maxAngularDecel(45.0f),
      timeStep(0.1f), goalThreshold(1.0f), obstacleThreshold(1.0f),
      headingWeight(0.5f), clearanceWeight(0.3f), smoothnessWeight(0.1f),
      energyWeight(0.1f), prevCmd{0, 0, 0, 0, 0, 0}
{
    // Initialize parameters
}

void LocalPathAdjuster::init()
{
    // Any additional initialization
}

VelocityCommand LocalPathAdjuster::adjustPath(const State &currentState, const Path &globalPath, const OccupancyGrid &occupancyGrid)
{
    std::lock_guard<std::mutex> lock(adjusterMutex); // Ensure thread safety

    VelocityCommand bestCmd = {0, 0, 0, 0, 0, 0};
    float bestCost = INFINITY;

    // Define the range of possible velocities with dynamic window
    // Calculate dynamic window based on current velocity and acceleration constraints

    // Example for linearX
    float currentLinearX = currentState.vx; // Current forward velocity
    float linearXMin = clamp(currentLinearX - maxLinearDecel * timeStep, minLinearSpeed, maxLinearSpeed);
    float linearXMax = clamp(currentLinearX + maxLinearAccel * timeStep, minLinearSpeed, maxLinearSpeed);

    // Similarly for angularZ
    float currentAngularZ = currentState.angularVz; // Current yaw rate
    float angularZMin = clamp(currentAngularZ - maxAngularDecel * timeStep, minAngularSpeed, maxAngularSpeed);
    float angularZMax = clamp(currentAngularZ + maxAngularAccel * timeStep, minAngularSpeed, maxAngularSpeed);

    // Iterate over possible velocity commands within dynamic window
    for (float linearX = linearXMin; linearX <= linearXMax; linearX += 0.5f)
    {
        for (float angularZ = angularZMin; angularZ <= angularZMax; angularZ += 10.0f)
        {
            // Create a velocity command
            VelocityCommand cmd = {linearX, 0, 0, 0, 0, angularZ};

            // Simulate the state after applying the velocity command
            State simulatedState = simulateState(currentState, cmd, timeStep);

            // Check for collision
            if (isCollision(simulatedState, occupancyGrid))
            {
                continue; // Skip commands that result in collision
            }

            // Calculate cost
            float cost = calculateCost(simulatedState, globalPath);

            // Choose the command with the lowest cost
            if (cost < bestCost)
            {
                bestCost = cost;
                bestCmd = cmd;
            }
        }
    }

    // Update previous command for smoothness calculation
    prevCmd = bestCmd;

    return bestCmd;
}

State LocalPathAdjuster::simulateState(const State &currentState, const VelocityCommand &cmd, float dt) const
{
    State simulated = currentState;

    // Update position based on velocity and acceleration
    simulated.x += cmd.linearX * dt;
    simulated.y += cmd.linearY * dt; // If lateral movement is applicable
    simulated.z += cmd.linearZ * dt; // Pump control

    // Update orientation based on angular velocities
    simulated.roll += cmd.angularX * dt;
    simulated.pitch += cmd.angularY * dt;
    simulated.yaw += cmd.angularZ * dt;

    // Normalize yaw to [0, 360)
    if (simulated.yaw >= 360.0f)
        simulated.yaw -= 360.0f;
    if (simulated.yaw < 0.0f)
        simulated.yaw += 360.0f;

    // Update velocity
    simulated.vx = cmd.linearX;
    simulated.vy = cmd.linearY;
    simulated.vz = cmd.linearZ;
    simulated.angularVx = cmd.angularX;
    simulated.angularVy = cmd.angularY;
    simulated.angularVz = cmd.angularZ;
    simulated.velocity = sqrt(simulated.vx * simulated.vx + simulated.vy * simulated.vy + simulated.vz * simulated.vz);

    return simulated;
}

bool LocalPathAdjuster::isCollision(const State &simulatedState, const OccupancyGrid &occupancyGrid) const
{
    // Convert simulated position to grid coordinates
    int gridX = round(simulatedState.x / occupancyGrid.resolution);
    int gridY = round(simulatedState.y / occupancyGrid.resolution);
    int gridZ = round(simulatedState.z / occupancyGrid.resolution);

    // Check bounds
    if (gridX < 0 || gridX >= occupancyGrid.sizeX ||
        gridY < 0 || gridY >= occupancyGrid.sizeY ||
        gridZ < 0 || gridZ >= occupancyGrid.sizeZ)
    {
        return true; // Out of bounds treated as collision
    }

    // Check occupancy
    size_t index = gridZ * occupancyGrid.sizeY * occupancyGrid.sizeX + gridY * occupancyGrid.sizeX + gridX;
    if (index >= occupancyGrid.gridData.size())
    {
        return true; // Invalid index treated as collision
    }

    return (occupancyGrid.gridData[index] == 1);
}

float LocalPathAdjuster::calculateCost(const State &simulatedState, const Path &globalPath) const
{
    if (globalPath.waypoints.empty())
    {
        return INFINITY;
    }

    // Calculate distance to the closest waypoint
    float minDist = INFINITY;
    Waypoint closestWaypoint;
    for (auto &wp : globalPath.waypoints)
    {
        float dist = sqrt(pow(wp.x - simulatedState.x, 2) +
                          pow(wp.y - simulatedState.y, 2) +
                          pow(wp.z - simulatedState.z, 2));
        if (dist < minDist)
        {
            minDist = dist;
            closestWaypoint = wp;
        }
    }

    // Calculate heading alignment
    float desiredYaw = closestWaypoint.yaw;
    float yawError = abs(simulatedState.yaw - desiredYaw);
    if (yawError > 180.0f)
        yawError = 360.0f - yawError;

    float headingCost = yawError / 180.0f; // Normalize to [0,1]

    // Calculate clearance based on distance to obstacles
    // Use the inverse of minDist
    float clearanceCost = 1.0f / (minDist + 1e-5f); // Avoid division by zero

    // Calculate smoothness (change in velocity)
    float smoothnessCost = calculateSmoothness(simulatedState);

    // Calculate energy consumption (penalize excessive speeds)
    float energyCost = simulatedState.velocity / maxLinearSpeed; // Normalize to [0,1]

    // Combine costs with weights
    float totalCost = headingWeight * headingCost +
                      clearanceWeight * clearanceCost +
                      smoothnessWeight * smoothnessCost +
                      energyWeight * energyCost;

    return totalCost;
}

float LocalPathAdjuster::calculateSmoothness(const State &simulatedState) const
{
    // Calculate the difference between current and previous velocity commands
    float deltaLinearX = abs(simulatedState.vx - prevCmd.linearX);
    float deltaAngularZ = abs(simulatedState.angularVz - prevCmd.angularZ);

    // Normalize differences based on maximum possible changes
    float normalizedDeltaLinearX = deltaLinearX / (maxLinearAccel * timeStep);
    float normalizedDeltaAngularZ = deltaAngularZ / (maxAngularAccel * timeStep);

    // Combine normalized differences
    float smoothnessCost = normalizedDeltaLinearX + normalizedDeltaAngularZ;

    return smoothnessCost;
}

float LocalPathAdjuster::clamp(float value, float minVal, float maxVal) const
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

float LocalPathAdjuster::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
