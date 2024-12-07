#include "MotorController.h"
#include "../config.h"

// Constructor
MotorControllerModule::MotorControllerModule()
    : leftMotor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, ENCODER_LEFT_PIN_A, ENCODER_LEFT_PIN_B),
      rightMotor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, ENCODER_RIGHT_PIN_A, ENCODER_RIGHT_PIN_B),
      pump(PUMP_PWM_PIN, PUMP_SENSOR_PIN),
      pidLeftMotor(LEFT_MOTOR_KP, LEFT_MOTOR_KI, LEFT_MOTOR_KD, 100.0f),
      pidRightMotor(RIGHT_MOTOR_KP, RIGHT_MOTOR_KI, RIGHT_MOTOR_KD, 100.0f),
      pidPump(PUMP_KP, PUMP_KI, PUMP_KD, 50.0f),
      currentStatus{false, 0.0f, 0.0f, 0.0f}
{
    // Initialize mutex
    statusMutex = xSemaphoreCreateMutex();
    if (statusMutex == NULL)
    {
        Serial.println("MotorControllerModule: Failed to create statusMutex.");
    }
}

// Initialize Motor Control Module
void MotorControllerModule::init()
{
    // Initialize actuators
    leftMotor.init();
    rightMotor.init();
    pump.init();

    // Initialize PID controllers
    pidLeftMotor.init();
    pidRightMotor.init();
    pidPump.init();

    // Initialize status as operational
    {
        if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE)
        {
            currentStatus.isOperational = true;
            xSemaphoreGive(statusMutex);
        }
    }

    Serial.println("MotorControllerModule initialized.");
}

// Map VelocityCommand to MotorCommand
MotorCommand MotorControllerModule::mapVelocityToMotorCommand(const VelocityCommand &velCmd)
{
    MotorCommand motorCmd;
    // Differential drive mapping
    // Assuming angularZ controls yaw by adjusting left and right motor speeds inversely
    motorCmd.leftMotorSpeed = clamp(velCmd.linearX - velCmd.angularZ, LEFT_MOTOR_MIN_SPEED, LEFT_MOTOR_MAX_SPEED);
    motorCmd.rightMotorSpeed = clamp(velCmd.linearX + velCmd.angularZ, RIGHT_MOTOR_MIN_SPEED, RIGHT_MOTOR_MAX_SPEED);

    // Pump control directly from linearZ
    motorCmd.pumpControl = clamp(velCmd.linearZ, PUMP_MIN_CONTROL, PUMP_MAX_CONTROL);

    return motorCmd;
}

// Apply MotorCommand to actuators using PID control
void MotorControllerModule::applyMotorCommands(const MotorCommand &motorCmd, float dt)
{
    // Compute PID outputs
    float pidOutputLeft = pidLeftMotor.compute(motorCmd.leftMotorSpeed, leftMotor.getCurrentSpeed(), dt);
    float pidOutputRight = pidRightMotor.compute(motorCmd.rightMotorSpeed, rightMotor.getCurrentSpeed(), dt);
    float pidOutputPump = pidPump.compute(motorCmd.pumpControl, pump.getCurrentPumpStatus(), dt);

    // Clamp PID outputs to actuator ranges
    pidOutputLeft = clamp(pidOutputLeft, LEFT_MOTOR_MIN_SPEED, LEFT_MOTOR_MAX_SPEED);
    pidOutputRight = clamp(pidOutputRight, RIGHT_MOTOR_MIN_SPEED, RIGHT_MOTOR_MAX_SPEED);
    pidOutputPump = clamp(pidOutputPump, PUMP_MIN_CONTROL, PUMP_MAX_CONTROL);

    // Apply PID outputs to actuators
    leftMotor.setSpeed(pidOutputLeft);
    rightMotor.setSpeed(pidOutputRight);
    pump.setControl(pidOutputPump);

    // Update pump status based on sensor readings
    pump.updateStatus();

    // Update current status
    {
        if (xSemaphoreTake(statusMutex, portMAX_DELAY) == pdTRUE)
        {
            currentStatus.currentLeftMotorSpeed = leftMotor.getCurrentSpeed();
            currentStatus.currentRightMotorSpeed = rightMotor.getCurrentSpeed();
            currentStatus.currentPumpStatus = pump.getCurrentPumpStatus();
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

// Update motor and pump based on received VelocityCommand
void MotorControllerModule::update(const VelocityCommand &commands)
{
    // Map VelocityCommand to MotorCommand
    MotorCommand motorCmd = mapVelocityToMotorCommand(commands);

    // Define time delta (dt) for PID computation
    float dt = 0.1f; // Assuming update is called at 10 Hz

    // Apply Motor Commands with PID control
    applyMotorCommands(motorCmd, dt);
}

// Get current status for logging or feedback
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
