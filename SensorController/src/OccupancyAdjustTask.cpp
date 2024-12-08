#include "OccupancyAdjustTask.h"
#include "SensorController.h"
#include "OccupancyGrid.h"
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
 * @brief FreeRTOS task function for adjusting the occupancy grid.
 *
 * @param pvParameters Pointer to SensorController instance.
 */
void vOccupancyAdjustTask(void *pvParameters)
{
    SensorController *controller = static_cast<SensorController *>(pvParameters);

    QueueHandle_t adjQ = controller->getOccupancyAdjQueue();
    QueueHandle_t changesQ = controller->getOccupancyChangesQueue();

    OccupancyGrid *grid = controller->getOccupancyGrid();
    SemaphoreHandle_t gridMutex = controller->getOccupancyGridMutex();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

    static const float OBSTACLE_RADIUS = 1.0f; // meters

    // For cell decay, define decay parameters
    const int DECAY_RATE = 1;             // decrement occupancy count by 1 per decay cycle
    const int DECAY_CYCLE_INTERVAL = 100; // every 100 cycles, i.e., every 2 seconds

    static int decayCycleCounter = 0;

    while (1)
    {
        SensorData procData;
        if (xQueueReceive(adjQ, &procData, portMAX_DELAY) == pdPASS)
        {
            // Retrieve submarine position
            float subX, subY, subZ, subYaw;
            controller->getSubmarinePosition(&subX, &subY, &subZ, &subYaw);

            std::vector<ChangedCell> changes;

            // Lambda function to mark obstacle and surrounding cells
            auto markObstacle = [&](float dist, float angleOffset)
            {
                if (std::isnan(dist))
                    return; // No obstacle information

                // Calculate obstacle position based on submarine position and sonar angle
                float totalYaw = subYaw + angleOffset;
                float obstacleX = subX + dist * cos(totalYaw);
                float obstacleY = subY + dist * sin(totalYaw);
                float obstacleZ = subZ; // Assuming sonar is horizontal; adjust if vertical sonars are used

                int gx, gy, gz;
                if (grid->worldToGrid(obstacleX, obstacleY, obstacleZ, &gx, &gy, &gz))
                {
                    // Mark cells within OBSTACLE_RADIUS in all dimensions
                    int radiusCells = static_cast<int>(OBSTACLE_RADIUS / OccupancyGrid::CELL_SIZE);
                    for (int dx = -radiusCells; dx <= radiusCells; dx++)
                    {
                        for (int dy = -radiusCells; dy <= radiusCells; dy++)
                        {
                            for (int dz = -radiusCells; dz <= radiusCells; dz++) // Handle z-axis
                            {
                                int nx = gx + dx;
                                int ny = gy + dy;
                                int nz = gz + dz;

                                // Calculate world coordinates of the cell
                                float cellX = 0.0f, cellY = 0.0f, cellZ = 0.0f;
                                grid->getOrigin(&cellX, &cellY, &cellZ);
                                cellX += nx * OccupancyGrid::CELL_SIZE;
                                cellY += ny * OccupancyGrid::CELL_SIZE;
                                cellZ += nz * OccupancyGrid::CELL_SIZE;

                                // Calculate distance from obstacle center
                                float distance = sqrt(pow(cellX - obstacleX, 2) + pow(cellY - obstacleY, 2) + pow(cellZ - obstacleZ, 2));

                                if (distance <= OBSTACLE_RADIUS)
                                {
                                    // Mark cell as occupied
                                    grid->markCellOccupied(nx, ny, nz, changes);
                                }
                            }
                        }
                    }
                }
            };

            // Acquire occupancy grid mutex
            if (xSemaphoreTake(gridMutex, portMAX_DELAY) == pdTRUE)
            {
                // Mark obstacle from forward sonar
                markObstacle(procData.forwardSonarDist, 0.0f);

                // Mark obstacle from oscillating sonar
                markObstacle(procData.oscillatingSonarDist, procData.oscillatingSonarAngle);

                xSemaphoreGive(gridMutex);
            }
            else
            {
                Serial.println("OccupancyAdjustTask: Failed to acquire occupancyGridMutex.");
            }

            // Enqueue changed cells to occupancyChangesQueue
            for (const auto &cell : changes)
            {
                if (xQueueSend(changesQ, &cell, 0) != pdPASS)
                {
                    Serial.println("OccupancyAdjustTask: occupancyChangesQueue full, some changes lost.");
                }
            }

            // Also send processed data to commSensorDataQueue for CommunicationTask
            QueueHandle_t commQ = controller->getCommSensorDataQueue();
            if (xQueueSend(commQ, &procData, 0) != pdPASS)
            {
                Serial.println("OccupancyAdjustTask: commSensorDataQueue full, processed data lost.");
            }

            // Handle cell decay
            decayCycleCounter++;
            if (decayCycleCounter >= DECAY_CYCLE_INTERVAL)
            {
                std::vector<ChangedCell> decayChanges;
                if (xSemaphoreTake(gridMutex, portMAX_DELAY) == pdTRUE)
                {
                    grid->decayOccupancy(DECAY_RATE, decayChanges);
                    xSemaphoreGive(gridMutex);
                }
                else
                {
                    Serial.println("OccupancyAdjustTask: Failed to acquire occupancyGridMutex for decay.");
                }

                // Enqueue decay changes to occupancyChangesQueue
                for (const auto &cell : decayChanges)
                {
                    if (xQueueSend(changesQ, &cell, 0) != pdPASS)
                    {
                        Serial.println("OccupancyAdjustTask: occupancyChangesQueue full, some decay changes lost.");
                    }
                }

                decayCycleCounter = 0;
            }
        }

        // Delay until next cycle
        vTaskDelayUntil(&xLastWakeTime, period);
    }
}
