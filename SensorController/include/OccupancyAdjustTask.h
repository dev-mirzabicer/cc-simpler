#ifndef OCCUPANCYADJUSTTASK_H
#define OCCUPANCYADJUSTTASK_H

#include "SensorController.h"

/**
 * @brief FreeRTOS task function for adjusting the occupancy grid.
 *
 * This task:
 * - Reads processed SensorData from occupancyAdjQueue.
 * - Uses submarine's current position to map sonar detections to grid coordinates.
 * - Marks detected obstacle cells and surrounding cells as occupied.
 * - Records changed cells and enqueues them to occupancyChangesQueue for communication.
 *
 * @param pvParameters Pointer to SensorController instance.
 */
void vOccupancyAdjustTask(void *pvParameters);

#endif // OCCUPANCYADJUSTTASK_H
