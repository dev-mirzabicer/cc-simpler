// main.cpp
#include <Arduino.h>
#include "config.h"
#include "Communication/InterESPCommunication.h"
#include "MotorControl/MotorController.h"
#include "SensorController/SensorController.h"
#include "Utils/Message.h"

// Instantiate modules
InterESPCommunication interESPComm;
MotorControllerModule motorController;
SensorController sensorController;

// FreeRTOS Task Handles
TaskHandle_t CommunicationTaskHandle = NULL;
TaskHandle_t MotorControlTaskHandle = NULL;
TaskHandle_t LoggingTaskHandle = NULL;
TaskHandle_t DiagnosticsTaskHandle = NULL;

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
void DiagnosticsTask(void *pvParameters);

void setup()
{
    // Initialize serial communication for debugging
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Wait for serial port to connect
    }
    Serial.println("MainController Initializing...");

    // Initialize InterESPCommunication
    interESPComm.init();

    // Initialize SensorController with appropriate configurations
    I2CBusConfig imuI2CConfig = {21, 22, 400000};      // Example SDA, SCL pins and frequency
    I2CBusConfig magI2CConfig = {21, 22, 400000};      // Same I2C bus
    I2CBusConfig pressureI2CConfig = {21, 22, 400000}; // Same I2C bus

    SonarConfig forwardSonarConfig = {23, 19};     // Example trigger and echo pins
    SonarConfig oscillatingSonarConfig = {25, 17}; // Example trigger and echo pins

    SensorStatus sensorInitStatus = sensorController.initAllSensors(
        imuI2CConfig,
        magI2CConfig,
        pressureI2CConfig,
        Sonar::SonarType::FORWARD,
        Sonar::SonarType::OSCILLATING,
        forwardSonarConfig,
        oscillatingSonarConfig);

    if (sensorInitStatus != SensorStatus::SUCCESS)
    {
        Serial.println("MainController: Sensor initialization failed. Proceeding with degraded mode.");
    }

    // Start sensor-related tasks
    SensorStatus taskStartStatus = sensorController.startTasks(
        TASK_PRIORITY_SENSOR_READ,    // Define appropriately in config.h
        TASK_STACK_SIZE_SENSOR_READ); // Define appropriately in config.h

    if (taskStartStatus != SensorStatus::SUCCESS)
    {
        Serial.println("MainController: Failed to start SensorController tasks.");
        // Handle critical failure if necessary
    }

    // Initialize MotorController
    motorController.init();

    // Create CommunicationTask
    if (xTaskCreate(CommunicationTask,
                    "CommunicationTask",
                    TASK_STACK_SIZE_COMMUNICATION,
                    &sensorController,
                    TASK_PRIORITY_COMMUNICATION,
                    &CommunicationTaskHandle) != pdPASS)
    {
        Serial.println("MainController: Failed to create CommunicationTask.");
    }
    else
    {
        Serial.println("MainController: CommunicationTask started.");
    }

    // Create MotorControlTask
    if (xTaskCreate(MotorControlTask,
                    "MotorControlTask",
                    TASK_STACK_SIZE_MOTOR_CONTROL,
                    NULL,
                    TASK_PRIORITY_MOTOR_CONTROL,
                    &MotorControlTaskHandle) != pdPASS)
    {
        Serial.println("MainController: Failed to create MotorControlTask.");
    }
    else
    {
        Serial.println("MainController: MotorControlTask started.");
    }

    // Create DiagnosticsTask
    if (xTaskCreate(DiagnosticsTask,
                    "DiagnosticsTask",
                    TASK_STACK_SIZE_DIAGNOSTICS,
                    &sensorController,
                    TASK_PRIORITY_DIAGNOSTICS,
                    &DiagnosticsTaskHandle) != pdPASS)
    {
        Serial.println("MainController: Failed to create DiagnosticsTask.");
    }
    else
    {
        Serial.println("MainController: DiagnosticsTask started.");
    }

    // Create LoggingTask
    if (xTaskCreate(LoggingTask,
                    "LoggingTask",
                    TASK_STACK_SIZE_LOGGING,
                    &sensorController,
                    TASK_PRIORITY_LOGGING,
                    &LoggingTaskHandle) != pdPASS)
    {
        Serial.println("MainController: Failed to create LoggingTask.");
    }
    else
    {
        Serial.println("MainController: LoggingTask started.");
    }

    Serial.println("MainController Initialized.");
}

void loop()
{
    // The loop remains empty as tasks are managed by FreeRTOS
    vTaskDelay(portMAX_DELAY);
}

// Communication Task: Handles sending SENSOR_DATA and OCCUPANCY_GRID_UPDATE to MainController
void CommunicationTask(void *pvParameters)
{
    SensorController *controller = static_cast<SensorController *>(pvParameters);
    QueueHandle_t commDataQ = controller->getCommSensorDataQueue();
    QueueHandle_t changesQ = controller->getOccupancyChangesQueue();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

    while (1)
    {
        // Check if we have a processed SensorData to send
        SensorData procData;
        if (xQueueReceive(commDataQ, &procData, 0) == pdPASS)
        {
            // Send SENSOR_DATA message
            sendSensorData(interESPComm, procData);
        }

        // Check for changed cells
        std::vector<ChangedCell> changedCells;
        ChangedCell c;
        // Drain all changed cells available this cycle
        while (xQueueReceive(changesQ, &c, 0) == pdPASS)
        {
            changedCells.push_back(c);
        }

        if (!changedCells.empty())
        {
            // Send OCCUPANCY_GRID_UPDATE messages
            sendOccupancyUpdates(interESPComm, changedCells);
        }

        vTaskDelayUntil(&xLastWakeTime, period);
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
        Serial.println("MotorControlTask: Status enqueued for sending.");

        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

// Logging Task: Logs status information for diagnostics
void LoggingTask(void *pvParameters)
{
    SensorController *controller = static_cast<SensorController *>(pvParameters);
    OccupancyGrid *grid = controller->getOccupancyGrid();

    while (1)
    {
        // Retrieve current status
        Status status = motorController.getStatus();

        // Log status
        logCurrentStatus(status);

        // Optionally, log occupancy grid statistics
        // For example, count occupied cells
        int occupiedCount = 0;
        {
            SemaphoreHandle_t gridMutex = controller->getOccupancyGridMutex();
            if (xSemaphoreTake(gridMutex, portMAX_DELAY) == pdTRUE)
            {
                for (int x = 0; x < OccupancyGrid::GRID_SIZE_X; x++)
                {
                    for (int y = 0; y < OccupancyGrid::GRID_SIZE_Y; y++)
                    {
                        for (int z = 0; z < OccupancyGrid::GRID_SIZE_Z; z++)
                        {
                            if (grid->getCell(x, y, z))
                                occupiedCount++;
                        }
                    }
                }
                xSemaphoreGive(gridMutex);
            }
            else
            {
                Serial.println("LoggingTask: Failed to acquire occupancyGridMutex.");
            }
        }

        Serial.print("LoggingTask: Total Occupied Cells: ");
        Serial.println(occupiedCount);

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
    }
}

// Diagnostics Task: Monitors sensor health and sends DIAGNOSTIC_UPDATE messages
void DiagnosticsTask(void *pvParameters)
{
    SensorController *controller = static_cast<SensorController *>(pvParameters);

    IMU *imu = controller->getIMU();
    Magnetometer *mag = controller->getMagnetometer();
    PressureSensor *press = controller->getPressureSensor();
    Sonar *fSonar = controller->getForwardSonar();
    Sonar *oSonar = controller->getOscillatingSonar();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000); // 1 Hz

    while (1)
    {
        bool imuHealthy = (imu && imu->isHealthy());
        bool magHealthy = (mag && mag->isHealthy());
        bool pressHealthy = (press && press->isHealthy());
        bool fSonarHealthy = (fSonar && fSonar->isHealthy());
        bool oSonarHealthy = (oSonar && oSonar->isHealthy());

        // Pack bits: bit0=IMU, bit1=Mag, bit2=Press, bit3=FSonar, bit4=OSonar
        uint8_t healthBits = 0;
        if (imuHealthy)
            healthBits |= (1 << 0);
        if (magHealthy)
            healthBits |= (1 << 1);
        if (pressHealthy)
            healthBits |= (1 << 2);
        if (fSonarHealthy)
            healthBits |= (1 << 3);
        if (oSonarHealthy)
            healthBits |= (1 << 4);

        // Send diagnostic every cycle
        sendDiagnostic(interESPComm, healthBits);

        vTaskDelayUntil(&xLastWakeTime, period);
    }
}
