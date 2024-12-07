#include <Arduino.h>
#include "config.h"
#include "Communication/InterESPCommunication.h"
#include "MotorControl/MotorController.h"
#include "Utils/Message.h"

// Instantiate modules
InterESPCommunication interESPComm;
MotorControllerModule motorController;

// FreeRTOS Task Handles
TaskHandle_t CommunicationTaskHandle = NULL;
TaskHandle_t MotorControlTaskHandle = NULL;
TaskHandle_t LoggingTaskHandle = NULL;

// Function to log current status
void logCurrentStatus(const Status &status)
{
    Serial.println("=== MotorController Status ===");
    Serial.print("Operational: ");
    Serial.println(status.isOperational ? "Yes" : "No");
    Serial.print("Left Motor Speed: ");
    Serial.print(status.currentLeftMotorSpeed);
    Serial.println(" m/s");
    Serial.print("Right Motor Speed: ");
    Serial.print(status.currentRightMotorSpeed);
    Serial.println(" m/s");
    Serial.print("Pump Intake Status: ");
    Serial.print(status.currentPumpIntakeStatus);
    Serial.println(" (abstract units)");
    Serial.print("Pump Outflow Status: ");
    Serial.print(status.currentPumpOutflowStatus);
    Serial.println(" (abstract units)");
    Serial.println("==============================");
}

// Task function prototypes
void CommunicationTask(void *pvParameters);
void MotorControlTask(void *pvParameters);
void LoggingTask(void *pvParameters);

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Wait for serial port to connect
    }
    Serial.println("MotorController Initializing...");

    // Initialize modules
    interESPComm.init();
    motorController.init();

    // Create tasks
    xTaskCreate(
        CommunicationTask,
        "CommunicationTask",
        TASK_STACK_SIZE_COMMUNICATION,
        NULL,
        TASK_PRIORITY_COMMUNICATION,
        &CommunicationTaskHandle);

    xTaskCreate(
        MotorControlTask,
        "MotorControlTask",
        TASK_STACK_SIZE_MOTOR_CONTROL,
        NULL,
        TASK_PRIORITY_MOTOR_CONTROL,
        &MotorControlTaskHandle);

    xTaskCreate(
        LoggingTask,
        "LoggingTask",
        TASK_STACK_SIZE_LOGGING,
        NULL,
        TASK_PRIORITY_LOGGING,
        &LoggingTaskHandle);

    Serial.println("MotorController Initialized.");
}

void loop()
{
    // The loop remains empty as tasks are managed by FreeRTOS
    vTaskDelay(portMAX_DELAY);
}

// Communication Task: Handles receiving VelocityCommands
void CommunicationTask(void *pvParameters)
{
    VelocityCommand receivedVelocity;

    while (1)
    {
        // Attempt to receive VelocityCommands
        if (interESPComm.receiveVelocityCommands(receivedVelocity))
        {
            // Update MotorController with received commands
            motorController.updateVelocityCommand(receivedVelocity);
            Serial.println("VelocityCommand processed.");
        }

        // Allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield for better multitasking
    }
}

// Motor Control Task: Performs control steps and sends Status updates
void MotorControlTask(void *pvParameters)
{
    Status status;

    while (1)
    {
        // Perform control step with dynamic dt
        motorController.performControlStep();

        // Retrieve current status from MotorController
        status = motorController.getStatus();

        // Enqueue Status for sending via Communication Module
        interESPComm.sendStatus(status);
        Serial.println("Status enqueued for sending by MotorControlTask.");

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

// Logging Task: Logs status information for diagnostics
void LoggingTask(void *pvParameters)
{
    Status status;

    while (1)
    {
        // Retrieve current status
        status = motorController.getStatus();

        // Log status
        logCurrentStatus(status);

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
    }
}
