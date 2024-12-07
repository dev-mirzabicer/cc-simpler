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
    // Initialize semaphore
    adjusterSemaphore = xSemaphoreCreateMutex();
    if (adjusterSemaphore == NULL)
    {
        Serial.println("LocalPathAdjuster: Failed to create adjusterSemaphore.");
    }
}

void LocalPathAdjuster::init()
{
    // Any additional initialization steps if necessary
    Serial.println("LocalPathAdjuster initialized.");
}

VelocityCommand LocalPathAdjuster::adjustPath(const State &currentState, const Path &globalPath, const OccupancyGrid &occupancyGrid)
{
    // Attempt to take the semaphore for thread safety
    if (xSemaphoreTake(adjusterSemaphore, portMAX_DELAY) == pdTRUE)
    {
        VelocityCommand bestCmd = {0, 0, 0, 0, 0, 0};
        float bestCost = INFINITY;

        // Define the range of possible velocities with dynamic window
        // Calculate dynamic window based on current velocity and acceleration constraints

        // Current velocities
        float currentLinearX = currentState.vx;         // Current forward velocity
        float currentAngularZ = currentState.angularVz; // Current yaw rate

        // Dynamic window for linearX
        float linearXMin = clamp(currentLinearX - maxLinearDecel * timeStep, minLinearSpeed, maxLinearSpeed);
        float linearXMax = clamp(currentLinearX + maxLinearAccel * timeStep, minLinearSpeed, maxLinearSpeed);

        // Dynamic window for angularZ
        float angularZMin = clamp(currentAngularZ - maxAngularDecel * timeStep, minAngularSpeed, maxAngularSpeed);
        float angularZMax = clamp(currentAngularZ + maxAngularAccel * timeStep, minAngularSpeed, maxAngularSpeed);

        // Sampling steps
        float linearStep = 0.5f;   // m/s
        float angularStep = 10.0f; // degrees/s

        // Iterate over possible velocity commands within dynamic window
        for (float linearX = linearXMin; linearX <= linearXMax; linearX += linearStep)
        {
            for (float angularZ = angularZMin; angularZ <= angularZMax; angularZ += angularStep)
            {
                // Create a velocity command
                VelocityCommand cmd = {linearX, 0.0f, 0.0f, 0.0f, 0.0f, angularZ};

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

        // Release the semaphore
        xSemaphoreGive(adjusterSemaphore);

        // Return the best velocity command found
        return bestCmd;
    }
    else
    {
        // If semaphore not acquired, return previous command to maintain smoothness
        return prevCmd;
    }
}

State LocalPathAdjuster::simulateState(const State &currentState, const VelocityCommand &cmd, float dt) const
{
    State simulated = currentState;

    // Update position based on velocity and time delta
    simulated.x += cmd.linearX * dt;
    simulated.y += cmd.linearY * dt; // Currently zero
    simulated.z += cmd.linearZ * dt; // Currently zero (pump control)

    // Update orientation based on angular velocities and time delta
    simulated.roll += cmd.angularX * dt;
    simulated.pitch += cmd.angularY * dt;
    simulated.yaw += cmd.angularZ * dt;

    // Normalize yaw to [0, 360)
    if (simulated.yaw >= 360.0f)
        simulated.yaw -= 360.0f;
    if (simulated.yaw < 0.0f)
        simulated.yaw += 360.0f;

    // Update velocities
    simulated.vx = cmd.linearX;
    simulated.vy = cmd.linearY;
    simulated.vz = cmd.linearZ;
    simulated.angularVx = cmd.angularX;
    simulated.angularVy = cmd.angularY;
    simulated.angularVz = cmd.angularZ;

    return simulated;
}

bool LocalPathAdjuster::isCollision(const State &simulatedState, const OccupancyGrid &occupancyGrid) const
{
    // Convert simulated position to grid indices
    int gridX = round(simulatedState.x / occupancyGrid.resolution);
    int gridY = round(simulatedState.y / occupancyGrid.resolution);
    int gridZ = round(simulatedState.z / occupancyGrid.resolution);

    // Check bounds
    if (gridX < 0 || gridX >= static_cast<int>(occupancyGrid.sizeX) ||
        gridY < 0 || gridY >= static_cast<int>(occupancyGrid.sizeY) ||
        gridZ < 0 || gridZ >= static_cast<int>(occupancyGrid.sizeZ))
    {
        return true; // Out of bounds treated as collision
    }

    // Calculate linear index for 3D grid
    size_t index = gridZ * occupancyGrid.sizeY * occupancyGrid.sizeX + gridY * occupancyGrid.sizeX + gridX;

    // Check if index is within gridData bounds
    if (index >= occupancyGrid.gridData.size())
    {
        return true; // Invalid index treated as collision
    }

    // Return collision status
    return (occupancyGrid.gridData[index] == 1);
}

float LocalPathAdjuster::calculateCost(const State &simulatedState, const Path &globalPath) const
{
    if (globalPath.waypoints.empty())
    {
        return INFINITY; // No path to follow
    }

    // Calculate distance to the closest waypoint
    float minDist = INFINITY;
    Waypoint closestWaypoint;
    for (const auto &wp : globalPath.waypoints)
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
    float smoothnessCost = calculateSmoothness(prevCmd);

    // Calculate energy consumption (penalize excessive speeds)
    float energyCost = (fabs(simulatedState.vx) + fabs(simulatedState.angularVz)) / (maxLinearSpeed + maxAngularSpeed / 10.0f); // Normalize to [0,1]

    // Combine costs with weights
    float totalCost = (headingWeight * headingCost) +
                      (clearanceWeight * clearanceCost) +
                      (smoothnessWeight * smoothnessCost) +
                      (energyWeight * energyCost);

    return totalCost;
}

float LocalPathAdjuster::calculateSmoothness(const VelocityCommand &cmd) const
{
    // Calculate the difference between current and previous velocity commands
    float deltaLinearX = abs(cmd.linearX - prevCmd.linearX);
    float deltaAngularZ = abs(cmd.angularZ - prevCmd.angularZ);

    // Normalize differences based on maximum possible changes
    float normalizedDeltaLinearX = deltaLinearX / (maxLinearAccel * timeStep);
    float normalizedDeltaAngularZ = deltaAngularZ / (maxAngularAccel * timeStep);

    // Combine normalized differences
    float smoothnessCost = normalizedDeltaLinearX + normalizedDeltaAngularZ;

    // Ensure smoothnessCost is within [0,2]
    smoothnessCost = clamp(smoothnessCost, 0.0f, 2.0f);

    return smoothnessCost;
}

State LocalPathAdjuster::simulateState(const State &currentState, const VelocityCommand &cmd, float dt) const
{
    State simulated = currentState;

    // Update position based on velocity and time delta
    simulated.x += cmd.linearX * dt;
    simulated.y += cmd.linearY * dt; // Currently zero
    simulated.z += cmd.linearZ * dt; // Currently zero (pump control)

    // Update orientation based on angular velocities and time delta
    simulated.roll += cmd.angularX * dt;
    simulated.pitch += cmd.angularY * dt;
    simulated.yaw += cmd.angularZ * dt;

    // Normalize yaw to [0, 360)
    if (simulated.yaw >= 360.0f)
        simulated.yaw -= 360.0f;
    if (simulated.yaw < 0.0f)
        simulated.yaw += 360.0f;

    // Update velocities
    simulated.vx = cmd.linearX;
    simulated.vy = cmd.linearY;
    simulated.vz = cmd.linearZ;
    simulated.angularVx = cmd.angularX;
    simulated.angularVy = cmd.angularY;
    simulated.angularVz = cmd.angularZ;

    return simulated;
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
    // Prevent division by zero
    if (in_max - in_min == 0)
        return out_min;

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
