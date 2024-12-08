// SensorController.cpp
#include "SensorController.h"
#include "SensorReadTask.h"
#include "PreProcessingTask.h"
#include "OccupancyAdjustTask.h"
#include <Arduino.h> // For Serial logging

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

// Placeholder factory functions to create sensor implementations.
// These should be implemented based on the actual sensor libraries being used.
IMU *createIMUImplementation();
Magnetometer *createMagnetometerImplementation();
PressureSensor *createPressureSensorImplementation();
Sonar *createSonarImplementation();

// Constructor
SensorController::SensorController()
    : initialized(false),
      imu(nullptr),
      magnetometer(nullptr),
      pressureSensor(nullptr),
      forwardSonar(nullptr),
      oscillatingSonar(nullptr),
      submarineX(0.0f),
      submarineY(0.0f),
      submarineZ(0.0f),
      submarineYaw(0.0f)
{
    // Create mutex for submarine position
    positionMutex = xSemaphoreCreateMutex();
    if (positionMutex == NULL)
    {
        Serial.println("SensorController: Failed to create position mutex.");
        // Critical failure, halt execution
        while (1)
            ;
    }

    // Create queues
    sensorTxQueue = xQueueCreate(20, sizeof(SensorData));
    if (sensorTxQueue == NULL)
    {
        Serial.println("SensorController: Failed to create sensorTxQueue.");
        while (1)
            ;
    }

    preProcessingTxQueue = xQueueCreate(20, sizeof(SensorData));
    if (preProcessingTxQueue == NULL)
    {
        Serial.println("SensorController: Failed to create preProcessingTxQueue.");
        while (1)
            ;
    }

    occupancyAdjQueue = xQueueCreate(20, sizeof(SensorData));
    if (occupancyAdjQueue == NULL)
    {
        Serial.println("SensorController: Failed to create occupancyAdjQueue.");
        while (1)
            ;
    }

    occupancyChangesQueue = xQueueCreate(200, sizeof(ChangedCell));
    if (occupancyChangesQueue == NULL)
    {
        Serial.println("SensorController: Failed to create occupancyChangesQueue.");
        while (1)
            ;
    }

    commSensorDataQueue = xQueueCreate(20, sizeof(SensorData));
    if (commSensorDataQueue == NULL)
    {
        Serial.println("SensorController: Failed to create commSensorDataQueue.");
        while (1)
            ;
    }

    // Initialize OccupancyGrid
    occupancyGridMutex = xSemaphoreCreateMutex();
    if (occupancyGridMutex == NULL)
    {
        Serial.println("SensorController: Failed to create occupancyGridMutex.");
        while (1)
            ;
    }
    occupancyGrid.setOrigin(0.0f, 0.0f, 0.0f); // Set grid origin to (0,0,0)
}

// Initialize all sensors
SensorStatus SensorController::initAllSensors(const I2CBusConfig &i2cBusConfigIMU,
                                              const I2CBusConfig &i2cBusConfigMag,
                                              const I2CBusConfig &i2cBusConfigPressure,
                                              const Sonar::SonarType forwardSonarType,
                                              const Sonar::SonarType oscillatingSonarType,
                                              const SonarConfig &forwardSonarConfig,
                                              const SonarConfig &oscillatingSonarConfig)
{
    // Create sensor objects
    imu = createIMUImplementation();
    magnetometer = createMagnetometerImplementation();
    pressureSensor = createPressureSensorImplementation();
    forwardSonar = createSonarImplementation();
    oscillatingSonar = createSonarImplementation();

    if (!imu || !magnetometer || !pressureSensor || !forwardSonar || !oscillatingSonar)
    {
        Serial.println("SensorController: Failed to create sensor objects.");
        return SensorStatus::HARDWARE_ERROR;
    }

    SensorStatus status = SensorStatus::SUCCESS;

    // Initialize IMU
    {
        SensorStatus s = imu->init(0x68, (void *)&i2cBusConfigIMU); // Example I2C address for IMU
        if (s != SensorStatus::SUCCESS)
        {
            Serial.println("SensorController: IMU initialization failed.");
            status = s;
        }
        else
        {
            s = imu->calibrate();
            if (s != SensorStatus::SUCCESS)
            {
                Serial.println("SensorController: IMU calibration failed.");
                status = s;
            }
        }
    }

    // Initialize Magnetometer
    {
        SensorStatus s = magnetometer->init(0x1E, (void *)&i2cBusConfigMag); // Example I2C address for Magnetometer
        if (s != SensorStatus::SUCCESS)
        {
            Serial.println("SensorController: Magnetometer initialization failed.");
            status = s;
        }
        else
        {
            s = magnetometer->calibrate();
            if (s != SensorStatus::SUCCESS)
            {
                Serial.println("SensorController: Magnetometer calibration failed.");
                status = s;
            }
        }
    }

    // Initialize Pressure Sensor
    {
        SensorStatus s = pressureSensor->init(0x76, (void *)&i2cBusConfigPressure); // Example I2C address for Pressure Sensor
        if (s != SensorStatus::SUCCESS)
        {
            Serial.println("SensorController: Pressure Sensor initialization failed.");
            status = s;
        }
        else
        {
            s = pressureSensor->calibrate();
            if (s != SensorStatus::SUCCESS)
            {
                Serial.println("SensorController: Pressure Sensor calibration failed.");
                status = s;
            }
        }
    }

    // Initialize Forward Sonar
    {
        SensorStatus s = forwardSonar->init(forwardSonarType, (void *)&forwardSonarConfig);
        if (s != SensorStatus::SUCCESS)
        {
            Serial.println("SensorController: Forward Sonar initialization failed.");
            status = s;
        }
        else
        {
            s = forwardSonar->calibrate();
            if (s != SensorStatus::SUCCESS)
            {
                Serial.println("SensorController: Forward Sonar calibration failed.");
                status = s;
            }
        }
    }

    // Initialize Oscillating Sonar
    {
        SensorStatus s = oscillatingSonar->init(oscillatingSonarType, (void *)&oscillatingSonarConfig);
        if (s != SensorStatus::SUCCESS)
        {
            Serial.println("SensorController: Oscillating Sonar initialization failed.");
            status = s;
        }
        else
        {
            s = oscillatingSonar->calibrate();
            if (s != SensorStatus::SUCCESS)
            {
                Serial.println("SensorController: Oscillating Sonar calibration failed.");
                status = s;
            }
        }
    }

    // Determine if fully initialized
    initialized = (status == SensorStatus::SUCCESS);
    if (!initialized)
    {
        Serial.println("SensorController: Some sensors failed to initialize or calibrate. Running in degraded mode.");
    }
    else
    {
        Serial.println("SensorController: All sensors initialized and calibrated successfully.");
    }

    return status;
}

// Start sensor-related tasks
SensorStatus SensorController::startTasks(UBaseType_t sensorReadPriority, size_t sensorReadStack)
{
    // Create SensorReadTask
    if (xTaskCreate(vSensorReadTask, "SensorReadTask", sensorReadStack, this, sensorReadPriority, NULL) != pdPASS)
    {
        Serial.println("SensorController: Failed to create SensorReadTask.");
        return SensorStatus::HARDWARE_ERROR;
    }

    Serial.println("SensorController: SensorReadTask started.");

    // Create PreProcessingTask
    if (xTaskCreate(vPreProcessingTask, "PreProcessingTask", sensorReadStack, this, sensorReadPriority, NULL) != pdPASS)
    {
        Serial.println("SensorController: Failed to create PreProcessingTask.");
        return SensorStatus::HARDWARE_ERROR;
    }

    Serial.println("SensorController: PreProcessingTask started.");

    // Create OccupancyAdjustTask
    if (xTaskCreate(vOccupancyAdjustTask, "OccupancyAdjustTask", sensorReadStack, this, sensorReadPriority, NULL) != pdPASS)
    {
        Serial.println("SensorController: Failed to create OccupancyAdjustTask.");
        return SensorStatus::HARDWARE_ERROR;
    }

    Serial.println("SensorController: OccupancyAdjustTask started.");

    // Note: CommunicationTask and DiagnosticsTask are part of Group 3 and will be started later.

    return SensorStatus::SUCCESS;
}

// Check initialization status
bool SensorController::isInitialized() const
{
    return initialized;
}

// Queue accessors
QueueHandle_t SensorController::getSensorTxQueue() const
{
    return sensorTxQueue;
}

QueueHandle_t SensorController::getPreProcessingTxQueue() const
{
    return preProcessingTxQueue;
}

QueueHandle_t SensorController::getOccupancyAdjQueue() const
{
    return occupancyAdjQueue;
}

QueueHandle_t SensorController::getOccupancyChangesQueue() const
{
    return occupancyChangesQueue;
}

QueueHandle_t SensorController::getCommSensorDataQueue() const
{
    return commSensorDataQueue;
}

// Sensor accessors
IMU *SensorController::getIMU()
{
    return imu;
}

Magnetometer *SensorController::getMagnetometer()
{
    return magnetometer;
}

PressureSensor *SensorController::getPressureSensor()
{
    return pressureSensor;
}

Sonar *SensorController::getForwardSonar()
{
    return forwardSonar;
}

Sonar *SensorController::getOscillatingSonar()
{
    return oscillatingSonar;
}

// Position setters and getters
void SensorController::setSubmarinePosition(float x, float y, float z, float yaw)
{
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE)
    {
        submarineX = x;
        submarineY = y;
        submarineZ = z;
        submarineYaw = yaw;
        xSemaphoreGive(positionMutex);
    }
    else
    {
        Serial.println("SensorController: Failed to set submarine position due to mutex error.");
    }
}

void SensorController::getSubmarinePosition(float *x, float *y, float *z, float *yaw) const
{
    if (xSemaphoreTake((SemaphoreHandle_t)positionMutex, portMAX_DELAY) == pdTRUE)
    {
        *x = submarineX;
        *y = submarineY;
        *z = submarineZ;
        *yaw = submarineYaw;
        xSemaphoreGive((SemaphoreHandle_t)positionMutex);
    }
    else
    {
        // If mutex fails, return current stored values
        Serial.println("SensorController: Failed to get submarine position due to mutex error.");
        *x = submarineX;
        *y = submarineY;
        *z = submarineZ;
        *yaw = submarineYaw;
    }
}

// OccupancyGrid accessors
OccupancyGrid *SensorController::getOccupancyGrid()
{
    return &occupancyGrid;
}

SemaphoreHandle_t SensorController::getOccupancyGridMutex() const
{
    return occupancyGridMutex;
}
