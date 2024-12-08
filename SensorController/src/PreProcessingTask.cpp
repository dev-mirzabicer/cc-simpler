#include "PreProcessingTask.h"
#include "SensorController.h"
#include "Filter.h"
#include "Calibration.h"
#include <Arduino.h>
#include <cmath>
#include <cfloat>
#include <vector>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

/**
 * @brief Constants for filtering algorithms.
 */
static const float YAW_ALPHA = 0.98f; // Complementary filter parameter for yaw
static const float EMA_LAMBDA = 0.9f; // Exponential moving average lambda for other data

// Previous filtered values
static bool firstRun = true;
static float prevYawFiltered = 0.0f;
static float prevAxFiltered = 0.0f;
static float prevAyFiltered = 0.0f;
static float prevAzFiltered = 0.0f;
static float prevDepthFiltered = 0.0f;
static float prevForwardSonarDistFiltered = 0.0f;
static float prevOscSonarDistFiltered = 0.0f;

// Previous timestamp for yaw dt calculation
static uint32_t lastTimestamp = 0;

// Calibration instance
static Calibration calibrationInstance;
static bool calibrationDone = false;

/**
 * @brief FreeRTOS task function for pre-processing sensor data.
 *
 * @param pvParameters Pointer to SensorController instance.
 */
void vPreProcessingTask(void *pvParameters)
{
    SensorController *controller = static_cast<SensorController *>(pvParameters);

    QueueHandle_t rawQ = controller->getSensorTxQueue();
    QueueHandle_t adjQ = controller->getOccupancyAdjQueue();

    // Initialize calibration
    IMU *imu = controller->getIMU();
    Magnetometer *mag = controller->getMagnetometer();
    PressureSensor *press = controller->getPressureSensor();
    SensorStatus cstatus = calibrationInstance.initCalibration(imu, mag, press);
    if (cstatus == SensorStatus::SUCCESS)
    {
        calibrationDone = true;
        Serial.println("PreProcessingTask: Calibration parameters acquired.");
    }
    else
    {
        Serial.println("PreProcessingTask: Calibration failed. Running without calibration.");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

    while (1)
    {
        SensorData rawData;
        if (xQueueReceive(rawQ, &rawData, portMAX_DELAY) == pdPASS)
        {
            // Apply calibration
            if (calibrationDone)
            {
                calibrationInstance.applyIMUCalibration(rawData.ax, rawData.ay, rawData.az, rawData.gyroZ);
                calibrationInstance.applyMagCalibration(rawData.yaw);
                calibrationInstance.applyPressureCalibration(rawData.depth);
            }

            // Compute dt for yaw filtering
            float dt = 0.02f; // default 20ms
            if (!firstRun)
            {
                uint32_t currTs = rawData.timestamp;
                if (currTs > lastTimestamp)
                {
                    float dtSec = (currTs - lastTimestamp) / 1000.0f;
                    if (dtSec > 0.0f && dtSec < 1.0f)
                        dt = dtSec;
                }
            }

            lastTimestamp = rawData.timestamp;

            // Filter Yaw using complementary filter
            if (firstRun || std::isnan(prevYawFiltered))
            {
                // On first run, initialize yaw
                if (std::isnan(rawData.yaw))
                    prevYawFiltered = 0.0f; // Fallback
                else
                    prevYawFiltered = rawData.yaw;
            }
            // Apply complementary filter
            {
                float gyroZ = (std::isnan(rawData.gyroZ)) ? 0.0f : rawData.gyroZ;
                float magYaw = (std::isnan(rawData.yaw)) ? prevYawFiltered : rawData.yaw;
                prevYawFiltered = Filter::complementaryYawFilter(prevYawFiltered, gyroZ, magYaw, dt, YAW_ALPHA);
            }

            // Filter accelerations using exponential moving average
            if (firstRun)
            {
                prevAxFiltered = std::isnan(rawData.ax) ? 0.0f : rawData.ax;
                prevAyFiltered = std::isnan(rawData.ay) ? 0.0f : rawData.ay;
                prevAzFiltered = std::isnan(rawData.az) ? 0.0f : rawData.az;
            }
            prevAxFiltered = Filter::exponentialMovingAverage(prevAxFiltered, rawData.ax, EMA_LAMBDA);
            prevAyFiltered = Filter::exponentialMovingAverage(prevAyFiltered, rawData.ay, EMA_LAMBDA);
            prevAzFiltered = Filter::exponentialMovingAverage(prevAzFiltered, rawData.az, EMA_LAMBDA);

            // Filter depth using exponential moving average
            if (firstRun)
                prevDepthFiltered = std::isnan(rawData.depth) ? 0.0f : rawData.depth;
            prevDepthFiltered = Filter::exponentialMovingAverage(prevDepthFiltered, rawData.depth, EMA_LAMBDA);

            // Filter forward sonar distance
            if (firstRun)
                prevForwardSonarDistFiltered = std::isnan(rawData.forwardSonarDist) ? 0.0f : rawData.forwardSonarDist;
            prevForwardSonarDistFiltered = Filter::exponentialMovingAverage(prevForwardSonarDistFiltered, rawData.forwardSonarDist, EMA_LAMBDA);

            // Filter oscillating sonar distance
            if (firstRun)
                prevOscSonarDistFiltered = std::isnan(rawData.oscillatingSonarDist) ? 0.0f : rawData.oscillatingSonarDist;
            prevOscSonarDistFiltered = Filter::exponentialMovingAverage(prevOscSonarDistFiltered, rawData.oscillatingSonarDist, EMA_LAMBDA);

            // Construct processed data
            SensorData processed = rawData;
            processed.ax = prevAxFiltered;
            processed.ay = prevAyFiltered;
            processed.az = prevAzFiltered;
            processed.gyroZ = rawData.gyroZ; // Already calibrated in Calibration
            processed.yaw = prevYawFiltered;
            processed.depth = prevDepthFiltered;
            processed.forwardSonarDist = prevForwardSonarDistFiltered;
            processed.oscillatingSonarDist = prevOscSonarDistFiltered;
            processed.oscillatingSonarAngle = rawData.oscillatingSonarAngle; // Angle controlled internally

            // Enqueue processed data to occupancyAdjQueue
            if (xQueueSend(adjQ, &processed, 0) != pdPASS)
            {
                Serial.println("PreProcessingTask: Failed to enqueue processed SensorData to occupancyAdjQueue, queue full.");
            }

            firstRun = false;
        }

        // Delay until next cycle
        vTaskDelayUntil(&xLastWakeTime, period);
    }
}
