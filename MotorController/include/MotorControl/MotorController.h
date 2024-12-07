#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include "../PumpController/PumpController.h"
#include "../MotorDriver/MotorDriver.h"
#include "../PID/PIDController.h"
#include "../LocalPathPlanner/LocalPathPlanner.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct MotorCommand
{
    float leftMotorSpeed;  // Desired speed for left motor (-10 to 10 m/s)
    float rightMotorSpeed; // Desired speed for right motor (-10 to 10 m/s)
    float pumpControl;     // Desired pump control (-5 to 5 m/s)
} __attribute__((packed));

class MotorControllerModule
{
public:
    MotorControllerModule();
    void init();
    void update(const VelocityCommand &commands);
    Status getStatus() const;

private:
    MotorDriver leftMotor;
    MotorDriver rightMotor;
    PumpController pump;

    PIDController pidLeftMotor;
    PIDController pidRightMotor;
    PIDController pidPump;

    Status currentStatus;
    SemaphoreHandle_t statusMutex; // Mutex for protecting currentStatus

    // Helper functions
    MotorCommand mapVelocityToMotorCommand(const VelocityCommand &velCmd);
    void applyMotorCommands(const MotorCommand &motorCmd, float dt);
    float clamp(float value, float minVal, float maxVal);
    MotorCommand mapVelocityCommand(const VelocityCommand &velCmd); // Clarify naming
};

#endif // MOTORCONTROLLER_H
