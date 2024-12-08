#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "MotorDriver.h"
#include "PumpController.h"
#include "PIDController.h"
#include "../Common/CommonMessageDefinitions.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @brief Structure representing motor and pump commands.
 */
struct MotorCommand
{
    float leftMotorSpeed;     // m/s
    float rightMotorSpeed;    // m/s
    float pumpIntakeControl;  // 0.0 to 1.0
    float pumpOutflowControl; // 0.0 to 1.0
} __attribute__((packed));

/**
 * @brief Structure holding status information for logging and communication.
 */
struct Status
{
    bool isOperational;
    float currentLeftMotorSpeed;    // m/s
    float currentRightMotorSpeed;   // m/s
    float currentPumpIntakeStatus;  // Abstract units
    float currentPumpOutflowStatus; // Abstract units
} __attribute__((packed));

/**
 * @brief The MotorControllerModule class manages motors and pumps, including PID control.
 */
class MotorControllerModule
{
public:
    /**
     * @brief Constructor for MotorControllerModule.
     */
    MotorControllerModule();

    /**
     * @brief Initialize the motor controller module.
     */
    void init();

    /**
     * @brief Update VelocityCommand from CommunicationTask.
     * @param commands VelocityCommand structure.
     */
    void updateVelocityCommand(const VelocityCommand &commands);

    /**
     * @brief Perform a control step: map commands, compute PID, and apply commands.
     */
    void performControlStep();

    /**
     * @brief Retrieve current status for logging and communication.
     * @return Status structure.
     */
    Status getStatus() const;

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
    SemaphoreHandle_t statusMutex;
    SemaphoreHandle_t commandMutex;

    MotorCommand latestCommands;

    /**
     * @brief Clamp a value between min and max.
     * @param value Value to clamp.
     * @param minVal Minimum allowable value.
     * @param maxVal Maximum allowable value.
     * @return Clamped value.
     */
    float clamp(float value, float minVal, float maxVal);

    /**
     * @brief Map VelocityCommand to MotorCommand.
     * @param velCmd VelocityCommand structure.
     * @return MotorCommand structure.
     */
    MotorCommand mapVelocityToMotorCommand(const VelocityCommand &velCmd);
};

#endif // MOTOR_CONTROLLER_H
