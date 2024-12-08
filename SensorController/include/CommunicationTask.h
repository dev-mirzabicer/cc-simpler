#ifndef COMMUNICATIONTASK_H
#define COMMUNICATIONTASK_H

class SensorController;

/**
 * @brief CommunicationTask handles sending SENSOR_DATA and OCCUPANCY_GRID_UPDATE to MainController.
 *
 * It:
 * - Reads processed SensorData from commSensorDataQueue.
 * - Reads changed cells from occupancyChangesQueue.
 * - Batches changed cells (up to 15 per message) and sends them.
 * - Uses InterESPCommunication or AcousticCommunication to send messages.
 */
void vCommunicationTask(void *pvParameters);

#endif // COMMUNICATIONTASK_H
