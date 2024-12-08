#include "SensorReadTask.h"
#include "SensorController.h"
#include "Filter.h"
#include "Calibration.h"
#include <Arduino.h>
#include <cmath>
#include <cfloat> // For NAN

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

// Define sensor reading rates
static const TickType_t SENSOR_PERIOD_TICKS = pdMS_TO_TICKS(10); // 100 Hz
static const uint8_t MAG_PRESS_SONAR_PERIOD = 5;                 // 20 Hz

// Sonar angle sweep parameters for oscillating sonar
static const float SONAR_ANGLE_MIN = -M_PI / 3.0f; // -60 degrees
static const float SONAR_ANGLE_MAX = M_PI / 3.0f;  // +60 degrees
static const float SONAR_ANGLE_INCREMENT = 0.05f;  // Radians per cycle
static float sonarCurrentAngle = SONAR_ANGLE_MIN;
static bool sonarAngleIncreasing = true;

/**
 * @brief FreeRTOS task function for reading all sensors.
 *
 * Reads IMU at 100 Hz, and Magnetometer, PressureSensor, Forward Sonar, Oscillating Sonar at 20 Hz.
 * Updates the oscillating sonar angle between -60° and +60°.
 * Enqueues raw SensorData to sensorTxQueue.
 *
 * @param pvParameters Pointer to SensorController instance.
 */
void vSensorReadTask(void *pvParameters)
{
    SensorController *controller = static_cast<SensorController *>(pvParameters);
    if (!controller->isInitialized())
    {
        Serial.println("SensorReadTask: Warning - SensorController not fully initialized. Some sensors might not work.");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t cycleCount = 0;

    // Initialize last known values for non-always-updated fields
    static float lastYaw = NAN;
    static float lastDepth = NAN;
    static float lastForwardSonarDist = NAN;
    static float lastOscSonarDist = NAN;
    static float lastOscSonarAngle = NAN;

    while (1)
    {
        cycleCount++;
        if (cycleCount > MAG_PRESS_SONAR_PERIOD)
            cycleCount = 1;

        SensorData rawData;
        rawData.timestamp = (uint32_t)millis();

        // Read IMU every cycle (100 Hz)
        {
            IMU *imu = controller->getIMU();
            if (imu && imu->isHealthy())
            {
                float ax, ay, az, gz;
                SensorStatus s = imu->readData(&ax, &ay, &az, &gz);
                if (s == SensorStatus::SUCCESS)
                {
                    rawData.ax = ax;
                    rawData.ay = ay;
                    rawData.az = az;
                    rawData.gyroZ = gz;
                }
                else
                {
                    Serial.println("SensorReadTask: IMU read failed.");
                    rawData.ax = NAN;
                    rawData.ay = NAN;
                    rawData.az = NAN;
                    rawData.gyroZ = NAN;
                }
            }
            else
            {
                rawData.ax = NAN;
                rawData.ay = NAN;
                rawData.az = NAN;
                rawData.gyroZ = NAN;
                Serial.println("SensorReadTask: IMU not available or unhealthy.");
            }
        }

        // Read Magnetometer, PressureSensor, and Sonars every 5 cycles (20 Hz)
        if (cycleCount == 1)
        {
            // Magnetometer
            {
                Magnetometer *mag = controller->getMagnetometer();
                if (mag && mag->isHealthy())
                {
                    float y;
                    SensorStatus s = mag->readYaw(&y);
                    if (s == SensorStatus::SUCCESS)
                    {
                        rawData.yaw = y;
                        lastYaw = y;
                    }
                    else
                    {
                        Serial.println("SensorReadTask: Magnetometer read failed.");
                        rawData.yaw = NAN;
                    }
                }
                else
                {
                    rawData.yaw = NAN;
                    Serial.println("SensorReadTask: Magnetometer not available or unhealthy.");
                }
            }

            // Pressure Sensor
            {
                PressureSensor *p = controller->getPressureSensor();
                if (p && p->isHealthy())
                {
                    float d;
                    SensorStatus s = p->readDepth(&d);
                    if (s == SensorStatus::SUCCESS)
                    {
                        rawData.depth = d;
                        lastDepth = d;
                    }
                    else
                    {
                        Serial.println("SensorReadTask: Pressure sensor read failed.");
                        rawData.depth = NAN;
                    }
                }
                else
                {
                    rawData.depth = NAN;
                    Serial.println("SensorReadTask: Pressure sensor not available or unhealthy.");
                }
            }

            // Forward Sonar
            {
                Sonar *fs = controller->getForwardSonar();
                if (fs && fs->isHealthy())
                {
                    float dist;
                    SensorStatus s = fs->readDistance(&dist);
                    if (s == SensorStatus::SUCCESS)
                    {
                        rawData.forwardSonarDist = dist;
                        lastForwardSonarDist = dist;
                    }
                    else
                    {
                        Serial.println("SensorReadTask: Forward sonar read failed.");
                        rawData.forwardSonarDist = NAN;
                    }
                }
                else
                {
                    rawData.forwardSonarDist = NAN;
                    Serial.println("SensorReadTask: Forward sonar not available or unhealthy.");
                }
            }

            // Oscillating Sonar: set angle and read distance
            {
                Sonar *os = controller->getOscillatingSonar();
                if (os && os->isHealthy())
                {
                    // Update angle
                    if (sonarAngleIncreasing)
                    {
                        sonarCurrentAngle += SONAR_ANGLE_INCREMENT;
                        if (sonarCurrentAngle > SONAR_ANGLE_MAX)
                        {
                            sonarCurrentAngle = SONAR_ANGLE_MAX;
                            sonarAngleIncreasing = false;
                        }
                    }
                    else
                    {
                        sonarCurrentAngle -= SONAR_ANGLE_INCREMENT;
                        if (sonarCurrentAngle < SONAR_ANGLE_MIN)
                        {
                            sonarCurrentAngle = SONAR_ANGLE_MIN;
                            sonarAngleIncreasing = true;
                        }
                    }

                    // Set angle
                    SensorStatus s = os->setAngle(sonarCurrentAngle);
                    if (s != SensorStatus::SUCCESS)
                    {
                        Serial.println("SensorReadTask: Oscillating sonar setAngle failed.");
                        rawData.oscillatingSonarDist = NAN;
                        rawData.oscillatingSonarAngle = NAN;
                    }
                    else
                    {
                        float angleCheck;
                        s = os->getAngle(&angleCheck);
                        if (s == SensorStatus::SUCCESS)
                        {
                            // Now read distance
                            float dist;
                            s = os->readDistance(&dist);
                            if (s == SensorStatus::SUCCESS)
                            {
                                rawData.oscillatingSonarDist = dist;
                                rawData.oscillatingSonarAngle = angleCheck;
                                lastOscSonarDist = dist;
                                lastOscSonarAngle = angleCheck;
                            }
                            else
                            {
                                Serial.println("SensorReadTask: Oscillating sonar readDistance failed.");
                                rawData.oscillatingSonarDist = NAN;
                                rawData.oscillatingSonarAngle = angleCheck; // angle is known even if distance failed
                            }
                        }
                        else
                        {
                            Serial.println("SensorReadTask: Oscillating sonar getAngle failed.");
                            rawData.oscillatingSonarDist = NAN;
                            rawData.oscillatingSonarAngle = NAN;
                        }
                    }
                }
                else
                {
                    rawData.oscillatingSonarDist = NAN;
                    rawData.oscillatingSonarAngle = NAN;
                    Serial.println("SensorReadTask: Oscillating sonar not available or unhealthy.");
                }
            }
        }
        else
        {
            // For cycles that are not reading Magnetometer, PressureSensor, and Sonars,
            // reuse last known values to maintain consistent SensorData structure.
            rawData.yaw = lastYaw;
            rawData.depth = lastDepth;
            rawData.forwardSonarDist = lastForwardSonarDist;
            rawData.oscillatingSonarDist = lastOscSonarDist;
            rawData.oscillatingSonarAngle = lastOscSonarAngle;
        }

        // Enqueue rawData into sensorTxQueue for PreProcessingTask
        QueueHandle_t q = controller->getSensorTxQueue();
        if (xQueueSend(q, &rawData, 0) != pdPASS)
        {
            Serial.println("SensorReadTask: Failed to enqueue SensorData, sensorTxQueue full.");
        }

        // Delay until next cycle
        vTaskDelayUntil(&xLastWakeTime, SENSOR_PERIOD_TICKS);
    }
}
