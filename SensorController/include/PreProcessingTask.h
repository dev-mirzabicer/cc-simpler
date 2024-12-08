#ifndef PREPROCESSINGTASK_H
#define PREPROCESSINGTASK_H

#include "SensorController.h"

/**
 * @brief FreeRTOS task function for pre-processing sensor data.
 *
 * This task:
 * - Reads raw SensorData from sensorTxQueue.
 * - Applies calibration and filtering.
 * - Enqueues processed SensorData to occupancyAdjQueue for occupancy grid adjustments.
 *
 * @param pvParameters Pointer to SensorController instance.
 */
void vPreProcessingTask(void *pvParameters);

#endif // PREPROCESSINGTASK_H
