#include "InterESPCommunication.h"

// Initialize static instance pointer
InterESPCommunication *InterESPCommunication::instance = nullptr;

// Constructor
InterESPCommunication::InterESPCommunication()
{
    if (instance == nullptr)
    {
        instance = this;
    }
}

// Initialize I2C as Slave and set up callbacks
void InterESPCommunication::init()
{
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onRequest([]()
                   { InterESPCommunication::onRequestWrapper(); });
    Wire.onReceive([](int byteCount)
                   { InterESPCommunication::onReceiveWrapper(byteCount); });
    Serial.println("InterESPCommunication initialized as I2C Slave.");
}

// Static wrapper for onRequest
void InterESPCommunication::onRequestWrapper()
{
    if (instance)
    {
        instance->handleRequest();
    }
}

// Static wrapper for onReceive
void InterESPCommunication::onReceiveWrapper(int byteCount)
{
    if (instance)
    {
        instance->handleReceive(byteCount);
    }
}

// Handle I2C Requests (send Status updates)
void InterESPCommunication::handleRequest()
{
    std::lock_guard<std::mutex> lock(i2cMutex);
    if (!statusQueue.empty())
    {
        Status status = statusQueue.front();
        statusQueue.pop();

        Message msg;
        serializeStatus(status, msg);
        Wire.write((uint8_t *)&msg, sizeof(Message));
        Serial.println("Status message sent to MainController.");
    }
    else
    {
        // If no status to send, send a default status
        Status defaultStatus = {true, 0.0f, 0.0f, 0.0f};
        Message msg;
        serializeStatus(defaultStatus, msg);
        Wire.write((uint8_t *)&msg, sizeof(Message));
        Serial.println("Default status message sent to MainController.");
    }
}

// Handle I2C Receives (receive VelocityCommand)
void InterESPCommunication::handleReceive(int byteCount)
{
    std::lock_guard<std::mutex> lock(i2cMutex);
    if (byteCount < sizeof(Message))
    {
        Serial.println("Received incomplete VelocityCommand message.");
        return;
    }

    uint8_t buffer[MAX_PAYLOAD_SIZE];
    size_t len = Wire.readBytes(buffer, sizeof(Message));

    VelocityCommand commands;
    bool success = deserializeVelocityCommand(buffer, len, commands);
    if (success)
    {
        enqueueVelocityCommand(commands);
        Serial.println("VelocityCommand received and enqueued.");
    }
    else
    {
        Serial.println("Failed to deserialize VelocityCommand.");
    }
}

// Enqueue received VelocityCommand
void InterESPCommunication::enqueueVelocityCommand(const VelocityCommand &cmd)
{
    velocityCommandQueue.push(cmd);
}

// Dequeue VelocityCommand for processing
bool InterESPCommunication::dequeueVelocityCommand(VelocityCommand &cmd)
{
    if (!velocityCommandQueue.empty())
    {
        cmd = velocityCommandQueue.front();
        velocityCommandQueue.pop();
        return true;
    }
    return false;
}

// Enqueue Status for sending
void InterESPCommunication::enqueueStatus(const Status &status)
{
    statusQueue.push(status);
}

// Dequeue Status for sending
bool InterESPCommunication::dequeueStatus(Status &status)
{
    if (!statusQueue.empty())
    {
        status = statusQueue.front();
        statusQueue.pop();
        return true;
    }
    return false;
}

// Serialize Status into Message
void InterESPCommunication::serializeStatus(const Status &status, Message &msg)
{
    msg.startByte = 0xAA; // Example start byte
    msg.type = MessageType::STATUS_UPDATE;
    msg.length = sizeof(Status);
    memset(msg.payload, 0, sizeof(msg.payload));
    memcpy(msg.payload, &status, sizeof(Status));
    msg.checksum = calculateCRC16(msg.payload, msg.length);
}

// Deserialize VelocityCommand from buffer
bool InterESPCommunication::deserializeVelocityCommand(const uint8_t *buffer, size_t length, VelocityCommand &commands)
{
    if (length < sizeof(Message))
    {
        Serial.println("VelocityCommand message size mismatch.");
        return false;
    }

    Message msg;
    memcpy(&msg, buffer, sizeof(Message));

    // Verify checksum
    uint16_t computedChecksum = calculateCRC16(msg.payload, msg.length);
    if (computedChecksum != msg.checksum)
    {
        Serial.println("VelocityCommand checksum mismatch.");
        return false;
    }

    if (msg.type != MessageType::VELOCITY_COMMAND)
    {
        Serial.println("Incorrect message type for VelocityCommand.");
        return false;
    }

    // Deserialize VelocityCommand
    if (msg.length > sizeof(VelocityCommand))
    {
        Serial.println("VelocityCommand payload size mismatch.");
        return false;
    }

    memcpy(&commands, msg.payload, sizeof(VelocityCommand));
    return true;
}

// Receive VelocityCommand from queue
bool InterESPCommunication::receiveVelocityCommands(VelocityCommand &commands)
{
    std::lock_guard<std::mutex> lock(i2cMutex);
    return dequeueVelocityCommand(commands);
}

// Send Status by enqueuing it to the statusQueue
void InterESPCommunication::sendStatus(const Status &status)
{
    std::lock_guard<std::mutex> lock(i2cMutex);
    enqueueStatus(status);
}
