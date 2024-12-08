#include "MotorController.h"
#include "../config.h"

// Constructor
MotorControllerModule::MotorControllerModule()
    : leftMotor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B, 0),
      rightMotor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B, 1),
      pumpIntake(PUMP_INTAKE_PWM_PIN, PUMP_INTAKE_DIR_PIN, PUMP_INTAKE_SENSOR_PIN, 2),
      pumpOutflow(PUMP_OUTFLOW_PWM_PIN, PUMP_OUTFLOW_DIR_PIN, PUMP_OUTFLOW_SENSOR_PIN, 3),
      pidLeftMotor(LEFT_MOTOR_KP, LEFT_MOTOR_KI, LEFT_MOTOR_KD, 100.0f),
      pidRightMotor(RIGHT_MOTOR_KP, RIGHT_MOTOR_KI, RIGHT_MOTOR_KD, 100.0f),
      pidPumpIntake(PUMP_INTAKE_KP, PUMP_INTAKE_KI, PUMP_INTAKE_KD, 50.0f),
      pidPumpOutflow(PUMP_OUTFLOW_KP, PUMP_OUTFLOW_KI, PUMP_OUTFLOW_KD, 50.0f),
      currentStatus{false, 0.0f, 0.0f, 0.0f, 0.0f}
{
    // Initialize mutexes
    statusMutex = xSemaphoreCreateMutex();
    if (statusMutex == NULL)
    {
        Serial.println("MotorControllerModule: Failed to create statusMutex.");
    }

    commandMutex = xSemaphoreCreateMutex();
    if (commandMutex == NULL)
    {
        Serial.println("MotorControllerModule: Failed to create commandMutex.");
    }
}

// Initialize Motor Control Module
void MotorControllerModule::init()
{
    // Initialize actuators
    leftMotor.init();
    rightMotor.init();
    pumpIntake.init();
    pumpOutflow.init();

    // Initialize PID controllers
    pidLeftMotor.init();
    pidRightMotor.init();
    pidPumpIntake.init();
    pidPumpOutflow.init();

    // Initialize status as operational
    {
        if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE)
        {
            currentStatus.isOperational = true;
            currentStatus.currentLeftMotorSpeed = 0.0f;
            currentStatus.currentRightMotorSpeed = 0.0f;
            currentStatus.currentPumpIntakeStatus = 0.0f;
            currentStatus.currentPumpOutflowStatus = 0.0f;
            xSemaphoreGive(statusMutex);
        }
    }

    // Initialize latestCommands
    {
        if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE)
        {
            latestCommands = VelocityCommand{0.0f, 0.0f};
            xSemaphoreGive(commandMutex);
        }
    }

    Serial.println("MotorControllerModule initialized.");
}

// Update VelocityCommand from CommunicationTask
void MotorControllerModule::updateVelocityCommand(const VelocityCommand &commands)
{
    if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE)
    {
        latestCommands = commands;
        xSemaphoreGive(commandMutex);
    }

    Serial.println("VelocityCommand updated.");
}

// Perform control step: map commands, compute PID, apply commands
void MotorControllerModule::performControlStep()
{
    // Measure dt
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();

    float dt;
    if (lastTime == 0)
    {
        dt = 0.1f; // Initialize with default
    }
    else
    {
        dt = (currentTime - lastTime) / 1000.0f; // in seconds
        if (dt <= 0.0f)
        {
            dt = 0.1f; // default to 0.1s if invalid
        }
    }
    lastTime = currentTime;

    // Retrieve latest commands
    VelocityCommand currentCommands;
    if (xSemaphoreTake(commandMutex, portMAX_DELAY) == pdTRUE)
    {
        currentCommands = latestCommands;
        xSemaphoreGive(commandMutex);
    }

    // Map VelocityCommand to MotorCommand
    MotorCommand motorCmd = mapVelocityToMotorCommand(currentCommands);

    // Apply MotorCommands with PID control
    // For motors
    float pidOutputLeft = pidLeftMotor.compute(motorCmd.leftMotorSpeed, leftMotor.getCurrentSpeed(), dt);
    float pidOutputRight = pidRightMotor.compute(motorCmd.rightMotorSpeed, rightMotor.getCurrentSpeed(), dt);

    // Clamp PID outputs to actuator ranges
    pidOutputLeft = clamp(pidOutputLeft, LEFT_MOTOR_MIN_SPEED, LEFT_MOTOR_MAX_SPEED);
    pidOutputRight = clamp(pidOutputRight, RIGHT_MOTOR_MIN_SPEED, RIGHT_MOTOR_MAX_SPEED);

    // Apply PID outputs to motors
    leftMotor.setSpeed(pidOutputLeft);
    rightMotor.setSpeed(pidOutputRight);

    // For pumps
    // Determine which pump to control based on linearZ
    if (motorCmd.pumpIntakeControl > 0.0f)
    {
        float pidOutputPumpIntake = pidPumpIntake.compute(motorCmd.pumpIntakeControl, pumpIntake.getCurrentPumpStatus(), dt);
        pidOutputPumpIntake = clamp(pidOutputPumpIntake, 0.0f, PUMP_INTAKE_MAX_CONTROL);
        pumpIntake.setControl(pidOutputPumpIntake);
        pumpOutflow.setControl(0.0f); // Ensure outflow is off
    }
    else if (motorCmd.pumpOutflowControl > 0.0f)
    {
        float pidOutputPumpOutflow = pidPumpOutflow.compute(motorCmd.pumpOutflowControl, pumpOutflow.getCurrentPumpStatus(), dt);
        pidOutputPumpOutflow = clamp(pidOutputPumpOutflow, 0.0f, PUMP_OUTFLOW_MAX_CONTROL);
        pumpOutflow.setControl(pidOutputPumpOutflow);
        pumpIntake.setControl(0.0f); // Ensure intake is off
    }
    else
    {
        // No pump control
        pumpIntake.setControl(0.0f);
        pumpOutflow.setControl(0.0f);
    }

    // Update pump status based on sensor readings
    pumpIntake.updateStatus();
    pumpOutflow.updateStatus();

    // Update current status
    {
        if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE)
        {
            currentStatus.currentLeftMotorSpeed = leftMotor.getCurrentSpeed();
            currentStatus.currentRightMotorSpeed = rightMotor.getCurrentSpeed();
            currentStatus.currentPumpIntakeStatus = pumpIntake.getCurrentPumpStatus();
            currentStatus.currentPumpOutflowStatus = pumpOutflow.getCurrentPumpStatus();
            currentStatus.isOperational = true; // This should be updated based on actual conditions

            // Implement additional error checks if necessary
            // For example:
            // if (fabs(currentStatus.currentLeftMotorSpeed) > LEFT_MOTOR_MAX_SPEED * 0.9f) {
            //     currentStatus.isOperational = false;
            //     Serial.println("Left Motor approaching maximum speed!");
            // }

            xSemaphoreGive(statusMutex);
        }
    }

    Serial.println("MotorCommands applied with PID control.");
}

// Map VelocityCommand to MotorCommand
MotorCommand MotorControllerModule::mapVelocityToMotorCommand(const VelocityCommand &velCmd)
{
    MotorCommand motorCmd;
    // Differential drive mapping
    float v_left = velCmd.linearX - (velCmd.angularZ * TRACK_WIDTH / 2.0f);
    float v_right = velCmd.linearX + (velCmd.angularZ * TRACK_WIDTH / 2.0f);

    motorCmd.leftMotorSpeed = clamp(v_left, LEFT_MOTOR_MIN_SPEED, LEFT_MOTOR_MAX_SPEED);
    motorCmd.rightMotorSpeed = clamp(v_right, RIGHT_MOTOR_MIN_SPEED, RIGHT_MOTOR_MAX_SPEED);

    // Pump control from linearZ
    // Separate intake and outflow
    if (velCmd.linearZ > 0.0f)
    {
        motorCmd.pumpIntakeControl = clamp(velCmd.linearZ, 0.0f, PUMP_INTAKE_MAX_CONTROL);
        motorCmd.pumpOutflowControl = 0.0f;
    }
    else if (velCmd.linearZ < 0.0f)
    {
        motorCmd.pumpOutflowControl = clamp(-velCmd.linearZ, 0.0f, PUMP_OUTFLOW_MAX_CONTROL);
        motorCmd.pumpIntakeControl = 0.0f;
    }
    else
    {
        motorCmd.pumpIntakeControl = 0.0f;
        motorCmd.pumpOutflowControl = 0.0f;
    }

    return motorCmd;
}

// Get current status
Status MotorControllerModule::getStatus() const
{
    Status status;
    if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE)
    {
        status = currentStatus;
        xSemaphoreGive(statusMutex);
    }
    return status;
}

// Helper function to clamp a value between min and max
float MotorControllerModule::clamp(float value, float minVal, float maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}
