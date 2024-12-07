#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include "../Utils/Utilities.h"
#include "../PID/PIDController.h"
#include "MotorDriver.h"
#include "PumpController.h"
#include <mutex>

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

    mutable std::mutex statusMutex;
    Status currentStatus;

    // Helper functions
    MotorCommand mapVelocityToMotorCommand(const VelocityCommand &velCmd);
    void applyMotorCommands(const MotorCommand &motorCmd, float dt);
};

#endif // MOTORCONTROLLER_H
