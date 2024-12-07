#ifndef LOCALPATHADJUSTER_H
#define LOCALPATHADJUSTER_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include "../StateEstimation/StateEstimator.h"
#include "../PathPlanning/GlobalPathPlanner.h"
#include <vector>
#include <cmath>
#include <freertos/semphr.h> // Include FreeRTOS semaphore for thread safety

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

// Structure for Waypoint
struct Waypoint
{
    float x;   // Position X (m)
    float y;   // Position Y (m)
    float z;   // Position Z (m)
    float yaw; // Orientation Yaw (degrees)
};

// Structure for Path
struct Path
{
    std::vector<Waypoint> waypoints; // List of waypoints
};

// Structure for OccupancyGrid
struct OccupancyGrid
{
    size_t sizeX;                  // Grid size in X dimension
    size_t sizeY;                  // Grid size in Y dimension
    size_t sizeZ;                  // Grid size in Z dimension
    float resolution;              // Grid resolution (meters per cell)
    std::vector<uint8_t> gridData; // 1D array representing 3D occupancy (0: free, 1: occupied)
};

class LocalPathAdjuster
{
public:
    /**
     * @brief Constructor for LocalPathAdjuster.
     */
    LocalPathAdjuster();

    /**
     * @brief Initialize the LocalPathAdjuster.
     */
    void init();

    /**
     * @brief Generate adjusted velocity commands based on current state, global path, and occupancy grid.
     *
     * @param currentState Current state of the AUV.
     * @param globalPath Global path plan.
     * @param occupancyGrid Environmental occupancy data.
     * @return VelocityCommand Adjusted velocity command.
     */
    VelocityCommand adjustPath(const State &currentState, const Path &globalPath, const OccupancyGrid &occupancyGrid);

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

    // Semaphore for thread safety
    SemaphoreHandle_t adjusterSemaphore;

    // DWA related functions
    bool isCollision(const State &simulatedState, const OccupancyGrid &occupancyGrid) const;
    float calculateCost(const State &simulatedState, const Path &globalPath) const;
    State simulateState(const State &currentState, const VelocityCommand &cmd, float dt) const;
    float distanceToClosestWaypoint(const State &simulatedState, const Path &globalPath) const;
    float calculateSmoothness(const VelocityCommand &cmd) const;

    // Helper functions
    float clamp(float value, float minVal, float maxVal) const;
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) const;
};

#endif // LOCALPATHADJUSTER_H
