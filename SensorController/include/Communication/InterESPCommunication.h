// InterESPCommunication.h
#ifndef INTERESP_COMMUNICATION_H
#define INTERESP_COMMUNICATION_H

#include <Arduino.h>
#include <Wire.h>
#include "../Common/CommonMessageDefinitions.h"
#include "../Utils/Utilities.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <queue>

/**
 * @brief The InterESPCommunication class manages I2C communication between ESP modules.
 *
 * It handles receiving VelocityCommands and SensorData, as well as sending Status updates.
 */
class InterESPCommunication
{
public:
    InterESPCommunication();
    void init();
    bool receiveVelocityCommands(VelocityCommand &commands);
    bool receiveSensorData(SensorData &data);
    void sendStatus(const Status &status);

    /**
     * @brief Send a message over I2C.
     *
     * @param msg The message to send.
     * @return bool true if message was successfully enqueued for sending, false otherwise.
     */
    bool sendMessage(const Message &msg);

private:
    SemaphoreHandle_t i2cMutex;          // Mutex for protecting I2C operations
    SemaphoreHandle_t statusBufferMutex; // Mutex for protecting status buffer

    // Queues for storing received messages
    std::queue<VelocityCommand> velocityCommandQueue;
    std::queue<SensorData> sensorDataQueue;
    std::queue<Status> statusQueue;

    // Buffer to hold the latest serialized status message
    uint8_t serializedStatusBuffer[MAX_PAYLOAD_SIZE];
    size_t serializedStatusLength;

    // Helper functions
    void handleRequest();
    void handleReceive(int byteCount);
    void enqueueVelocityCommand(const VelocityCommand &cmd);
    void enqueueSensorData(const SensorData &data);
    bool dequeueVelocityCommand(VelocityCommand &cmd);
    bool dequeueSensorData(SensorData &data);
    void enqueueStatus(const Status &status);
    bool dequeueStatus(Status &status);
    void serializeStatus(const Status &status, Message &msg);
    bool deserializeMessage(const uint8_t *buffer, size_t length, Message &msg);

    // Callback functions need to be static or use a singleton pattern
    static InterESPCommunication *instance;

    // Static callback wrappers
    static void onRequestWrapper();
    static void onReceiveWrapper(int byteCount);
};

#endif // INTERESP_COMMUNICATION_H
