// CommunicationTask.cpp
#include "CommunicationTask.h"
#include "SensorController.h"
#include "../Common/CommonMessageDefinitions.h"
#include <Arduino.h>
#include <cmath>
#include <cfloat>
#include <vector>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

extern InterESPCommunication interESPComm;

static const int MAX_CELLS_PER_MESSAGE = 15;
static const MessageType SENSOR_DATA_TYPE = MessageType::SENSOR_DATA;
static const MessageType OCC_UPDATE_TYPE = MessageType::OCCUPANCY_GRID_UPDATE;

/**
 * @brief Send Occupancy Grid Updates in batches.
 *
 * @param comm Communication interface.
 * @param cells Vector of changed cells.
 */
static void sendOccupancyUpdates(InterESPCommunication &comm, std::vector<ChangedCell> &cells)
{
    size_t total = cells.size();
    size_t index = 0;
    while (index < total)
    {
        size_t batchCount = (total - index) > MAX_CELLS_PER_MESSAGE ? MAX_CELLS_PER_MESSAGE : (total - index);

        // Construct message
        Message msg;
        msg.startByte = 0xAA;
        msg.type = OCC_UPDATE_TYPE;
        // payload:
        // payload[0] = count
        // then each cell: x,y,z as int16_t
        size_t payloadSize = 1 + batchCount * 6;
        msg.length = (uint16_t)payloadSize;
        if (payloadSize > MAX_PAYLOAD_SIZE)
        {
            Serial.println("CommunicationTask: Payload too large, should not happen.");
            // Handle error by sending fewer cells next time
        }

        uint8_t *p = msg.payload;
        p[0] = (uint8_t)batchCount;
        p += 1;

        for (size_t i = 0; i < batchCount; i++)
        {
            ChangedCell &c = cells[index + i];
            int16_t cx = (int16_t)c.x;
            int16_t cy = (int16_t)c.y;
            int16_t cz = (int16_t)c.z;

            memcpy(p, &cx, 2);
            p += 2;
            memcpy(p, &cy, 2);
            p += 2;
            memcpy(p, &cz, 2);
            p += 2;
        }

        msg.checksum = calculateCRC16(msg.payload, msg.length);

        if (!comm.sendMessage(msg))
        {
            Serial.println("CommunicationTask: Failed to send OCCUPANCY_GRID_UPDATE message.");
        }

        index += batchCount;
    }
}

/**
 * @brief Send Sensor Data.
 *
 * @param comm Communication interface.
 * @param data SensorData structure.
 */
static void sendSensorData(InterESPCommunication &comm, const SensorData &data)
{
    Message msg;
    msg.startByte = 0xAA;
    msg.type = SENSOR_DATA_TYPE;
    msg.length = sizeof(SensorData);
    memcpy(msg.payload, &data, sizeof(SensorData));
    msg.checksum = calculateCRC16(msg.payload, msg.length);
    if (!comm.sendMessage(msg))
    {
        Serial.println("CommunicationTask: Failed to send SENSOR_DATA message.");
    }
}

/**
 * @brief FreeRTOS task function for communication.
 *
 * @param pvParameters Pointer to SensorController instance.
 */
void vCommunicationTask(void *pvParameters)
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
