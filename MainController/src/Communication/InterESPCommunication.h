#ifndef INTERESP_COMMUNICATION_H
#define INTERESP_COMMUNICATION_H

#include <Arduino.h>
#include <Wire.h>
#include "../CommonMessageDefinitions/Message.h"
#include "../Utils/Utilities.h"
#include <mutex>
#include <queue>

class InterESPCommunication
{
public:
    InterESPCommunication();
    void init();

    // Methods to send VelocityCommand and receive Status
    bool sendVelocityCommand(const VelocityCommand &commands);
    bool receiveStatus(Status &status);

private:
    std::mutex i2cMutex; // Mutex for protecting I2C operations

    // Helper functions
    void serializeVelocityCommand(const VelocityCommand &commands, Message &msg);
    bool deserializeStatus(const uint8_t *buffer, size_t length, Status &status);

    // I2C communication helper
    bool sendMessageToSlave(const Message &msg, uint8_t slaveAddress);
    bool requestMessageFromSlave(Message &msg, uint8_t slaveAddress);
};

#endif // INTERESP_COMMUNICATION_H
