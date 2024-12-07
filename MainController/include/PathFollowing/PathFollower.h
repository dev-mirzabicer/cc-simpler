#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include "../PID/PIDController.h"
#include "../Communication/InterESPCommunication.h"
#include "../PathPlanning/LocalPathAdjuster.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Structure to hold velocity commands
struct VelocityCommand
{
    float linearX;  // Forward/backward movement (m/s)
    float linearY;  // Lateral movement (m/s) - Optional
    float linearZ;  // Vertical movement (pump control) (m/s)
    float angularX; // Roll control (degrees/s)
    float angularY; // Pitch control (degrees/s)
    float angularZ; // Yaw control (degrees/s)
} __attribute__((packed));

class PathFollower
{
public:
    PathFollower();
    void init(InterESPCommunication *comm, LocalPathAdjuster *adjuster);
    VelocityCommand followPath(const State &currentState, const Path &globalPath, const OccupancyGrid &occupancyGrid);
    VelocityCommand getLastVelocityCommand() const;

private:
    InterESPCommunication *interESPComm;
    LocalPathAdjuster *localPathAdjuster;
    PIDController pidLinearX;
    PIDController pidLinearY;
    PIDController pidLinearZ;
    PIDController pidAngularX;
    PIDController pidAngularY;
    PIDController pidAngularZ;

    // Define PID parameters (to be tuned)
    float kp_linear;
    float ki_linear;
    float kd_linear;

    float kp_angular;
    float ki_angular;
    float kd_angular;

    // Waypoint tracking
    size_t currentWaypointIndex;

    // Previous velocity commands for acceleration limiting
    float prevLinearX;
    float prevLinearY;
    float prevLinearZ;
    float prevAngularX;
    float prevAngularY;
    float prevAngularZ;

    // Acceleration limits (units per command cycle)
    float maxLinearAccel;
    float maxAngularAccel;

    // Last known velocity command
    mutable SemaphoreHandle_t velocityCmdMutex;
    VelocityCommand lastVelocityCmdStored;

    // Helper functions
    float clamp(float value, float minVal, float maxVal);
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

    // Mutex for protecting velocity commands
    // (Removed std::mutex and replaced with FreeRTOS semaphore)
};

#endif // PATHFOLLOWER_H
