// Communication/MainController/src/main.cpp

#include <Arduino.h>
#include "config.h"
#include "Communication/InterESPCommunication.h"
#include "Communication/RemoteCommunication.h"
#include "DataAggregation/DataAggregator.h"
#include "StateEstimation/StateEstimator.h"
#include "PathPlanning/LocalPathAdjuster.h"
#include "PathFollowing/PathFollower.h"
#include "../lib/CommonMessageDefinitions/Message.h"

// Instantiate modules
InterESPCommunication interESPComm;
RemoteCommunication remoteComm(17, 16); // TX_PIN = 17, RX_PIN = 16
DataAggregator dataAggregator;
StateEstimator stateEstimator;
LocalPathAdjuster localPathAdjuster;
PathFollower pathFollower;

// FreeRTOS Task Handles
TaskHandle_t SensorTaskHandle = NULL;
TaskHandle_t StateEstimationTaskHandle = NULL;
TaskHandle_t CommunicationTaskHandle = NULL;
TaskHandle_t PathFollowingTaskHandle = NULL;
TaskHandle_t MotorControlTaskHandle = NULL;
TaskHandle_t LoggingTaskHandle = NULL;

// FreeRTOS Queues
QueueHandle_t sensorDataQueue;
QueueHandle_t stateQueue;
QueueHandle_t velocityCmdQueue;
QueueHandle_t pathQueue;
QueueHandle_t occupancyGridQueue;
QueueHandle_t statusQueue; // Queue for Status updates

// Shared VelocityCommand and its mutex
VelocityCommand currentVelocityCmd = {0, 0};
SemaphoreHandle_t velocityCmdMutexHandle;

// Function to log current state
void logCurrentState(const float *state)
{
    Serial.println("=== StateEstimator State ===");
    Serial.print("Position -> X: ");
    Serial.print(state[0]);
    Serial.print(", Y: ");
    Serial.print(state[1]);
    Serial.print(", Z: ");
    Serial.println(state[2]);

    Serial.print("Velocity -> Vx: ");
    Serial.print(state[3]);
    Serial.print(", Vy: ");
    Serial.print(state[4]);
    Serial.print(", Vz: ");
    Serial.println(state[5]);

    Serial.print("Orientation -> Yaw: ");
    Serial.print(state[6]);
    Serial.println(" radians");

    Serial.print("Yaw Rate: ");
    Serial.print(state[7]);
    Serial.println(" rad/s");
    Serial.println("==============================");
}

// Function to log Status updates
void logStatus(const Status &status)
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
void SensorTask(void *pvParameters);
void StateEstimationTask(void *pvParameters);
void CommunicationTask(void *pvParameters);
void PathFollowingTask(void *pvParameters);
void MotorControlTask(void *pvParameters);
void LoggingTask(void *pvParameters);

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Wait for serial port to connect. Needed for native USB
    }
    Serial.println("Main Controller Initializing...");

    // Initialize all modules
    interESPComm.init();
    remoteComm.init(CommunicationMethod::Acoustic);
    dataAggregator.init(&interESPComm);
    stateEstimator.init();
    localPathAdjuster.init();
    pathFollower.init(&interESPComm, &localPathAdjuster);

    // Initialize FreeRTOS Queues
    sensorDataQueue = xQueueCreate(10, sizeof(SensorData));
    stateQueue = xQueueCreate(10, 8 * sizeof(float)); // State is an array of 8 floats
    velocityCmdQueue = xQueueCreate(10, sizeof(VelocityCommand));
    pathQueue = xQueueCreate(10, sizeof(Path));
    occupancyGridQueue = xQueueCreate(10, sizeof(OccupancyGrid));
    statusQueue = xQueueCreate(10, sizeof(Status)); // Initialize Status Queue

    if (sensorDataQueue == NULL || stateQueue == NULL || velocityCmdQueue == NULL || pathQueue == NULL || occupancyGridQueue == NULL || statusQueue == NULL)
    {
        Serial.println("Failed to create FreeRTOS queues.");
        while (1)
            ; // Halt execution
    }

    // Create a mutex for currentVelocityCmd
    velocityCmdMutexHandle = xSemaphoreCreateMutex();
    if (velocityCmdMutexHandle == NULL)
    {
        Serial.println("Failed to create velocityCmdMutexHandle.");
        while (1)
            ; // Halt execution
    }

    // Create tasks
    xTaskCreate(
        SensorTask,
        "SensorTask",
        TASK_STACK_SIZE_SENSOR_DATA,
        NULL,
        TASK_PRIORITY_SENSOR_DATA,
        &SensorTaskHandle);

    xTaskCreate(
        StateEstimationTask,
        "StateEstimationTask",
        TASK_STACK_SIZE_STATE_ESTIMATION,
        NULL,
        TASK_PRIORITY_STATE_ESTIMATION,
        &StateEstimationTaskHandle);

    xTaskCreate(
        CommunicationTask,
        "CommunicationTask",
        TASK_STACK_SIZE_COMMUNICATION,
        NULL,
        TASK_PRIORITY_COMMUNICATION,
        &CommunicationTaskHandle);

    xTaskCreate(
        PathFollowingTask,
        "PathFollowingTask",
        TASK_STACK_SIZE_PATH_PLANNING, // Reusing stack size for path following
        NULL,
        TASK_PRIORITY_PATH_PLANNING,
        &PathFollowingTaskHandle);

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

    Serial.println("Main Controller Initialized.");
}

void loop()
{
    remoteComm.processIncomingData();
    vTaskDelay(portMAX_DELAY);
}

// Sensor Data Collection Task
void SensorTask(void *pvParameters)
{
    while (1)
    {
        // Collect data from Sensor Aggregator
        SensorData aggregatedData = dataAggregator.collectData();

        // Send data to State Estimation Task via sensorDataQueue
        if (xQueueSend(sensorDataQueue, &aggregatedData, portMAX_DELAY) != pdPASS)
        {
            Serial.println("Failed to send sensor data to State Estimation Task.");
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

// State Estimation Task
void StateEstimationTask(void *pvParameters)
{
    SensorData receivedData;
    float estimatedState[8] = {0}; // [x, y, z, vx, vy, vz, yaw, yaw_rate]

    while (1)
    {
        // Receive sensor data from Sensor Task
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdPASS)
        {
            // Perform prediction with IMU data
            stateEstimator.predict(receivedData.ax, receivedData.ay, receivedData.az, receivedData.gyroZ, 0.1f); // Assuming dt = 0.1s

            // Perform update with measurements
            stateEstimator.update(receivedData.depth, receivedData.yaw);

            // Retrieve the estimated state
            stateEstimator.getState(estimatedState);

            // Send estimated state to Logging Task via stateQueue
            if (xQueueSend(stateQueue, &estimatedState, portMAX_DELAY) != pdPASS)
            {
                Serial.println("Failed to send state data to Logging Task.");
            }

            Serial.println("StateEstimator: Prediction and Update performed.");
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Minimal delay to yield CPU
    }
}

// Communication Task
void CommunicationTask(void *pvParameters)
{
    while (1)
    {
        // Process any received commands from Remote Communication
        remoteComm.processReceivedCommands();

        // Check if a new path is available and send it to Path Following Task
        Path receivedPath;
        if (remoteComm.receivePath(receivedPath))
        {
            if (xQueueSend(pathQueue, &receivedPath, portMAX_DELAY) != pdPASS)
            {
                Serial.println("Failed to send path to Path Following Task.");
            }
            else
            {
                Serial.println("Path received and sent to Path Following Task.");
            }
        }

        // Check if a new occupancy grid is available and send it to Path Following Task
        OccupancyGrid receivedGrid;
        if (remoteComm.receiveOccupancyGrid(receivedGrid))
        {
            if (xQueueSend(occupancyGridQueue, &receivedGrid, portMAX_DELAY) != pdPASS)
            {
                Serial.println("Failed to send Occupancy Grid to Path Following Task.");
            }
            else
            {
                Serial.println("Occupancy Grid received and sent to Path Following Task.");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust as needed
    }
}

// Path Following Task
void PathFollowingTask(void *pvParameters)
{
    Path receivedPath;
    OccupancyGrid receivedGrid;
    float currentState[8] = {0}; // [x, y, z, vx, vy, vz, yaw, yaw_rate]
    Path currentPath;
    OccupancyGrid currentOccupancyGrid;

    while (1)
    {
        // Check for new path
        if (xQueueReceive(pathQueue, &receivedPath, pdMS_TO_TICKS(10)) == pdPASS)
        {
            currentPath = receivedPath;
            Serial.println("Path received and updated for following.");
        }

        // Check for new occupancy grid
        if (xQueueReceive(occupancyGridQueue, &receivedGrid, pdMS_TO_TICKS(10)) == pdPASS)
        {
            currentOccupancyGrid = receivedGrid;
            Serial.println("Occupancy Grid received and updated for path following.");
        }

        // Check for new state estimation
        if (xQueueReceive(stateQueue, &currentState, pdMS_TO_TICKS(10)) == pdPASS)
        {
            // Follow the path using PathFollower
            VelocityCommand cmd = pathFollower.followPath(currentState, currentPath, currentOccupancyGrid);

            // Send velocity command to Motor Control Task via velocityCmdQueue
            if (xQueueSend(velocityCmdQueue, &cmd, portMAX_DELAY) != pdPASS)
            {
                Serial.println("Failed to send velocity command to Motor Control Task.");
            }
            else
            {
                Serial.println("VelocityCommand sent to Motor Control Task.");
            }

            // Update the shared currentVelocityCmd
            if (xSemaphoreTake(velocityCmdMutexHandle, portMAX_DELAY) == pdTRUE)
            {
                currentVelocityCmd.linearX = cmd.linearX;
                currentVelocityCmd.angularZ = cmd.angularZ;
                xSemaphoreGive(velocityCmdMutexHandle);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust as needed
    }
}

// Motor Control Task
void MotorControlTask(void *pvParameters)
{
    VelocityCommand receivedCmd;
    Status receivedStatus;
    unsigned long lastStatusRequest = 0;

    while (1)
    {
        // Receive velocity commands from Path Following Task
        if (xQueueReceive(velocityCmdQueue, &receivedCmd, portMAX_DELAY) == pdPASS)
        {
            // Send VelocityCommand to Motor Controller via InterESPCommunication
            if (interESPComm.sendVelocityCommand(receivedCmd))
            {
                // Update the shared currentVelocityCmd
                if (xSemaphoreTake(velocityCmdMutexHandle, portMAX_DELAY) == pdTRUE)
                {
                    currentVelocityCmd.linearX = receivedCmd.linearX;
                    currentVelocityCmd.angularZ = receivedCmd.angularZ;
                    xSemaphoreGive(velocityCmdMutexHandle);
                }

                Serial.println("VelocityCommand sent to Motor Controller.");
            }
            else
            {
                Serial.println("Failed to send VelocityCommand to Motor Controller.");
            }
        }

        // Periodically request Status from Motor Controller
        unsigned long currentMillis = millis();
        if (currentMillis - lastStatusRequest >= 1000) // Every 1 second
        {
            if (interESPComm.receiveStatus(receivedStatus))
            {
                // Send Status to Logging Task via statusQueue
                if (xQueueSend(statusQueue, &receivedStatus, portMAX_DELAY) != pdPASS)
                {
                    Serial.println("Failed to send Status to Logging Task.");
                }
                else
                {
                    Serial.println("Status received from Motor Controller and sent to Logging Task.");
                }
            }
            else
            {
                Serial.println("Failed to receive Status from Motor Controller.");
            }

            lastStatusRequest = currentMillis;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Minimal delay to yield CPU
    }
}

// Logging Task
void LoggingTask(void *pvParameters)
{
    float state[8] = {0}; // [x, y, z, vx, vy, vz, yaw, yaw_rate]
    Status currentStatus;

    while (1)
    {
        // Receive state data from State Estimation Task
        if (xQueueReceive(stateQueue, &state, pdMS_TO_TICKS(10)) == pdPASS)
        {
            // Log current state for debugging
            logCurrentState(state);
        }

        // Receive Status from Motor Control Task
        if (xQueueReceive(statusQueue, &currentStatus, pdMS_TO_TICKS(10)) == pdPASS)
        {
            // Log Status updates
            logStatus(currentStatus);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
    }
}
