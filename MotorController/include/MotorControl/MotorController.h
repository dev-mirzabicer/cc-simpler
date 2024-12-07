#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include "../PumpController/PumpController.h"
#include "../MotorDriver/MotorDriver.h"
#include "../PID/PIDController.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct MotorCommand
{
    float leftMotorSpeed;     // Desired speed for left motor (-1.0 to 1.0 m/s)
    float rightMotorSpeed;    // Desired speed for right motor (-1.0 to 1.0 m/s)
    float pumpIntakeControl;  // Desired pump intake control (0.0 to 1.0)
    float pumpOutflowControl; // Desired pump outflow control (0.0 to 1.0)
} __attribute__((packed));

class MotorControllerModule
{
public:
    MotorControllerModule();
    void init();
    void updateVelocityCommand(const VelocityCommand &commands);
    Status getStatus() const;
    void performControlStep();

private:
    MotorDriver leftMotor;
    MotorDriver rightMotor;
    PumpController pumpIntake;
    PumpController pumpOutflow;

    PIDController pidLeftMotor;
    PIDController pidRightMotor;
    PIDController pidPumpIntake;
    PIDController pidPumpOutflow;

    Status currentStatus;
    SemaphoreHandle_t statusMutex; // Mutex for protecting currentStatus

    // Latest VelocityCommand
    VelocityCommand latestCommands;
    SemaphoreHandle_t commandMutex; // Mutex for protecting latestCommands

    // Helper functions
    MotorCommand mapVelocityToMotorCommand(const VelocityCommand &velCmd);
    void applyMotorCommands(const MotorCommand &motorCmd, float dt);
    float clamp(float value, float minVal, float maxVal);
};

#endif // MOTORCONTROLLER_H
