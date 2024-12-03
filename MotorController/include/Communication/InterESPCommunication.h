#ifndef INTERESP_COMMUNICATION_H
#define INTERESP_COMMUNICATION_H

#include <Arduino.h>
#include <Wire.h>
#include "../Utils/Message.h"
#include "../Utils/Utilities.h"
#include <mutex>
#include <queue>

class InterESPCommunication
{
public:
    InterESPCommunication();
    void init();
    bool receiveVelocityCommands(VelocityCommand &commands);
    void sendStatus(const Status &status);

private:
    std::mutex i2cMutex; // Mutex for protecting I2C operations

    // Queues for storing received VelocityCommands and to be sent Status
    std::queue<VelocityCommand> velocityCommandQueue;
    std::queue<Status> statusQueue;

    // Helper functions
    void handleRequest();
    void handleReceive(int byteCount);
    void enqueueVelocityCommand(const VelocityCommand &cmd);
    bool dequeueVelocityCommand(VelocityCommand &cmd);
    void enqueueStatus(const Status &status);
    bool dequeueStatus(Status &status);
    void serializeStatus(const Status &status, Message &msg);
    bool deserializeVelocityCommand(const uint8_t *buffer, size_t length, VelocityCommand &commands);

    // Callback functions need to be static or use a singleton pattern
    static InterESPCommunication *instance;

    // Static callback wrappers
    static void onRequestWrapper();
    static void onReceiveWrapper(int byteCount);
};

#endif // INTERESP_COMMUNICATION_H
