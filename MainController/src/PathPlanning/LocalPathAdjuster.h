#ifndef LOCALPATHADJUSTER_H
#define LOCALPATHADJUSTER_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include "../StateEstimation/StateEstimator.h"
#include "../PathPlanning/GlobalPathPlanner.h"
#include <vector>
#include <cmath>
#include <mutex> // Include mutex for thread safety

// Structure to hold velocity commands
struct VelocityCommand
{
    float linearX;  // Forward/backward movement (m/s)
    float linearY;  // Lateral movement (m/s) - Optional
    float linearZ;  // Vertical movement (pump control) (m/s)
    float angularX; // Roll control (degrees/s)
    float angularY; // Pitch control (degrees/s)
    float angularZ; // Yaw control (degrees/s)
};

// Define maximum PWM values for commands if necessary

class LocalPathAdjuster
{
public:
    LocalPathAdjuster();
    void init();
    VelocityCommand adjustPath(const State &currentState, const Path &globalPath, const OccupancyGrid &occupancyGrid);
    // Additional methods as needed

private:
    // DWA parameters
    float maxLinearSpeed;
    float minLinearSpeed;
    float maxLinearAccel;
    float maxLinearDecel;
    float maxAngularSpeed;
    float minAngularSpeed;
    float maxAngularAccel;
    float maxAngularDecel;
    float timeStep;
    float goalThreshold;
    float obstacleThreshold;
    float headingWeight;
    float clearanceWeight;
    float smoothnessWeight;
    float energyWeight;

    // Previous velocity commands for smoothness calculation
    VelocityCommand prevCmd;

    // DWA related functions
    bool isCollision(const State &simulatedState, const OccupancyGrid &occupancyGrid) const;
    float calculateCost(const State &simulatedState, const Path &globalPath) const;
    State simulateState(const State &currentState, const VelocityCommand &cmd, float dt) const;
    float distanceToClosestWaypoint(const State &simulatedState, const Path &globalPath) const;
    float calculateSmoothness(const State &simulatedState) const;

    // Mutex for thread safety
    std::mutex adjusterMutex;

    // Helper functions
    float clamp(float value, float minVal, float maxVal) const;
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const;
};

#endif // LOCALPATHADJUSTER_H
