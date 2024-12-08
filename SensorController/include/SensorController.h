// SensorController.h
#ifndef SENSORCONTROLLER_H
#define SENSORCONTROLLER_H

#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include "SensorStatus.h"
#include "../Common/CommonMessageDefinitions.h"
#include "../BusConfig.h"
#include "../libraries/IMU/IMU.h"
#include "../libraries/Magnetometer/Magnetometer.h"
#include "../libraries/PressureSensor/PressureSensor.h"
#include "../libraries/Sonar/Sonar.h"
#include "OccupancyGrid.h"

/**
 * @brief The SensorController manages all sensors (IMU, Magnetometer, PressureSensor, two Sonars),
 * initializes them, calibrates them, and creates tasks for data acquisition, pre-processing,
 * occupancy grid adjustments, and prepares for communication and diagnostics.
 */
class SensorController
{
public:
    /**
     * @brief Constructor for SensorController.
     * Initializes member variables and creates necessary synchronization primitives.
     */
    SensorController();

    /**
     * @brief Initialize all sensors with their respective configurations.
     *
     * @param i2cBusConfigIMU I2C configuration for IMU.
     * @param i2cBusConfigMag I2C configuration for Magnetometer.
     * @param i2cBusConfigPressure I2C configuration for Pressure Sensor.
     * @param forwardSonarType Type of the forward-looking sonar.
     * @param oscillatingSonarType Type of the oscillating sonar.
     * @param forwardSonarConfig Configuration for the forward-looking sonar.
     * @param oscillatingSonarConfig Configuration for the oscillating sonar.
     * @return SensorStatus SUCCESS if all sensors initialized and calibrated successfully, otherwise appropriate error code.
     */
    SensorStatus initAllSensors(const I2CBusConfig &i2cBusConfigIMU,
                                const I2CBusConfig &i2cBusConfigMag,
                                const I2CBusConfig &i2cBusConfigPressure,
                                const Sonar::SonarType forwardSonarType,
                                const Sonar::SonarType oscillatingSonarType,
                                const SonarConfig &forwardSonarConfig,
                                const SonarConfig &oscillatingSonarConfig);

    /**
     * @brief Start all FreeRTOS tasks related to sensor operations.
     *
     * @param sensorReadPriority Priority for the SensorReadTask.
     * @param sensorReadStack Stack size for the SensorReadTask.
     * @return SensorStatus SUCCESS if tasks started successfully, otherwise appropriate error code.
     */
    SensorStatus startTasks(UBaseType_t sensorReadPriority, size_t sensorReadStack);

    /**
     * @brief Check if SensorController is fully initialized and functional.
     * @return bool true if all required sensors initialized successfully, false if any critical sensor failed.
     */
    bool isInitialized() const;

    /**
     * @brief Get the handle to the sensorTxQueue where raw SensorData is pushed from SensorReadTask.
     * @return QueueHandle_t The queue handle.
     */
    QueueHandle_t getSensorTxQueue() const;

    /**
     * @brief Get the handle to the preProcessingTxQueue where processed SensorData is pushed from PreProcessingTask.
     * @return QueueHandle_t The queue handle.
     */
    QueueHandle_t getPreProcessingTxQueue() const;

    /**
     * @brief Get the handle to the occupancyAdjQueue where SensorData is pushed for occupancy grid adjustments.
     * @return QueueHandle_t The queue handle.
     */
    QueueHandle_t getOccupancyAdjQueue() const;

    /**
     * @brief Get the handle to the occupancyChangesQueue where changed cells are pushed for communication.
     * @return QueueHandle_t The queue handle.
     */
    QueueHandle_t getOccupancyChangesQueue() const;

    /**
     * @brief Get the handle to the commSensorDataQueue for communication tasks.
     * @return QueueHandle_t The queue handle.
     */
    QueueHandle_t getCommSensorDataQueue() const;

    /**
     * @brief Accessor for the IMU sensor.
     * @return IMU* Pointer to the IMU sensor.
     */
    IMU *getIMU();

    /**
     * @brief Accessor for the Magnetometer sensor.
     * @return Magnetometer* Pointer to the Magnetometer sensor.
     */
    Magnetometer *getMagnetometer();

    /**
     * @brief Accessor for the PressureSensor.
     * @return PressureSensor* Pointer to the PressureSensor.
     */
    PressureSensor *getPressureSensor();

    /**
     * @brief Accessor for the forward-looking Sonar.
     * @return Sonar* Pointer to the forward-looking Sonar.
     */
    Sonar *getForwardSonar();

    /**
     * @brief Accessor for the oscillating Sonar.
     * @return Sonar* Pointer to the oscillating Sonar.
     */
    Sonar *getOscillatingSonar();

    /**
     * @brief Set the current submarine position and orientation.
     * This should be called by MainController to update the submarine's estimated state.
     *
     * @param x Submarine's X position in meters.
     * @param y Submarine's Y position in meters.
     * @param z Submarine's Z position in meters.
     * @param yaw Submarine's yaw angle in radians.
     */
    void setSubmarinePosition(float x, float y, float z, float yaw);

    /**
     * @brief Retrieve the current submarine position and orientation.
     *
     * @param x Pointer to store submarine's X position in meters.
     * @param y Pointer to store submarine's Y position in meters.
     * @param z Pointer to store submarine's Z position in meters.
     * @param yaw Pointer to store submarine's yaw angle in radians.
     */
    void getSubmarinePosition(float *x, float *y, float *z, float *yaw) const;

    /**
     * @brief Accessor for the OccupancyGrid.
     * @return OccupancyGrid* Pointer to the OccupancyGrid.
     */
    OccupancyGrid *getOccupancyGrid();

    /**
     * @brief Accessor for the OccupancyGrid mutex.
     * @return SemaphoreHandle_t The mutex handle.
     */
    SemaphoreHandle_t getOccupancyGridMutex() const;

private:
    bool initialized;
    IMU *imu;
    Magnetometer *magnetometer;
    PressureSensor *pressureSensor;
    Sonar *forwardSonar;
    Sonar *oscillatingSonar;

    // Mutex for submarine position
    SemaphoreHandle_t positionMutex;

    // Submarine's current position and orientation
    float submarineX;
    float submarineY;
    float submarineZ;
    float submarineYaw;

    // Queues for data flow between tasks
    QueueHandle_t sensorTxQueue;         // From SensorReadTask to PreProcessingTask
    QueueHandle_t preProcessingTxQueue;  // From PreProcessingTask to OccupancyAdjustTask
    QueueHandle_t occupancyAdjQueue;     // From PreProcessingTask to OccupancyAdjustTask
    QueueHandle_t occupancyChangesQueue; // From OccupancyAdjustTask to CommunicationTask
    QueueHandle_t commSensorDataQueue;   // From OccupancyAdjustTask to CommunicationTask

    // Occupancy Grid and its mutex
    OccupancyGrid occupancyGrid;
    SemaphoreHandle_t occupancyGridMutex;

    // Friend tasks for access to private members if necessary
    friend void vSensorReadTask(void *pvParameters);
    friend void vPreProcessingTask(void *pvParameters);
    friend void vOccupancyAdjustTask(void *pvParameters);
};

#endif // SENSORCONTROLLER_H
