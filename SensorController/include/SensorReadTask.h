#ifndef SENSORREADTASK_H
#define SENSORREADTASK_H

#include "SensorController.h"

/**
 * @brief FreeRTOS task function for reading all sensors.
 *
 * @param pvParameters Pointer to SensorController instance.
 */
void vSensorReadTask(void *pvParameters);

#endif // SENSORREADTASK_H
