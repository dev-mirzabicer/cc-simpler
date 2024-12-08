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

    // Create the mutexes
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL)
    {
        Serial.println("Failed to create I2C mutex.");
    }

    statusBufferMutex = xSemaphoreCreateMutex();
    if (statusBufferMutex == NULL)
    {
        Serial.println("Failed to create statusBufferMutex.");
    }

    // Initialize serializedStatusBuffer with default status
    Status defaultStatus = {true, 0.0f, 0.0f, 0.0f, 0.0f};
    Message defaultMsg;
    serializeStatus(defaultStatus, defaultMsg);
    memcpy(serializedStatusBuffer, &defaultMsg, sizeof(Message));
    serializedStatusLength = sizeof(Message);
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
    // Minimal work in ISR: copy the serializedStatusBuffer to Wire buffer
    if (xSemaphoreTake(statusBufferMutex, portMAX_DELAY) == pdTRUE)
    {
        Wire.write(serializedStatusBuffer, serializedStatusLength);
        xSemaphoreGive(statusBufferMutex);
    }

    Serial.println("Status message sent to MainController.");
}

// Handle I2C Receives (receive VelocityCommand or SensorData)
void InterESPCommunication::handleReceive(int byteCount)
{
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
        if (byteCount < sizeof(Message))
        {
            Serial.println("Received incomplete message.");
            xSemaphoreGive(i2cMutex);
            return;
        }

        uint8_t buffer[MAX_PAYLOAD_SIZE];
        size_t len = Wire.readBytes(buffer, sizeof(Message));

        Message msg;
        bool success = deserializeMessage(buffer, len, msg);
        if (success)
        {
            switch (msg.type)
            {
            case MessageType::VELOCITY_COMMAND:
            {
                VelocityCommand commands;
                if (msg.length >= sizeof(VelocityCommand))
                {
                    memcpy(&commands, msg.payload, sizeof(VelocityCommand));
                    enqueueVelocityCommand(commands);
                    Serial.println("VelocityCommand received and enqueued.");
                }
                else
                {
                    Serial.println("VelocityCommand payload size mismatch.");
                }
                break;
            }
            case MessageType::SENSOR_DATA:
            {
                SensorData data;
                if (msg.length >= sizeof(SensorData))
                {
                    memcpy(&data, msg.payload, sizeof(SensorData));
                    enqueueSensorData(data);
                    Serial.println("SensorData received and enqueued.");
                }
                else
                {
                    Serial.println("SensorData payload size mismatch.");
                }
                break;
            }
            default:
                Serial.println("Unknown message type received.");
                break;
            }
        }
        else
        {
            Serial.println("Failed to deserialize message.");
        }

        xSemaphoreGive(i2cMutex);
    }
}

// Enqueue received VelocityCommand
void InterESPCommunication::enqueueVelocityCommand(const VelocityCommand &cmd)
{
    velocityCommandQueue.push(cmd);
}

// Enqueue received SensorData
void InterESPCommunication::enqueueSensorData(const SensorData &data)
{
    sensorDataQueue.push(data);
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

// Dequeue SensorData for processing
bool InterESPCommunication::dequeueSensorData(SensorData &data)
{
    if (!sensorDataQueue.empty())
    {
        data = sensorDataQueue.front();
        sensorDataQueue.pop();
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

// Serialize Status into Message and store in serializedStatusBuffer
void InterESPCommunication::serializeStatus(const Status &status, Message &msg)
{
    msg.startByte = 0xAA; // Example start byte
    msg.type = MessageType::STATUS_UPDATE;
    msg.length = sizeof(Status);
    memset(msg.payload, 0, sizeof(msg.payload));
    memcpy(msg.payload, &status, sizeof(Status));
    msg.checksum = calculateCRC16(msg.payload, msg.length);
}

// Deserialize Message from buffer
bool InterESPCommunication::deserializeMessage(const uint8_t *buffer, size_t length, Message &msg)
{
    if (length < sizeof(Message))
    {
        Serial.println("Message size mismatch.");
        return false;
    }

    memcpy(&msg, buffer, sizeof(Message));

    // Verify checksum
    uint16_t computedChecksum = calculateCRC16(msg.payload, msg.length);
    if (computedChecksum != msg.checksum)
    {
        Serial.println("Message checksum mismatch.");
        return false;
    }

    return true;
}

// Receive VelocityCommand from queue
bool InterESPCommunication::receiveVelocityCommands(VelocityCommand &commands)
{
    bool success = false;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
        success = dequeueVelocityCommand(commands);
        xSemaphoreGive(i2cMutex);
    }
    return success;
}

// Receive SensorData from queue
bool InterESPCommunication::receiveSensorData(SensorData &data)
{
    bool success = false;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
        success = dequeueSensorData(data);
        xSemaphoreGive(i2cMutex);
    }
    return success;
}

// Send Status by enqueuing it to the statusQueue and preparing the serializedStatusBuffer
void InterESPCommunication::sendStatus(const Status &status)
{
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
    {
        enqueueStatus(status);

        // Serialize the status and store in the buffer
        Message msg;
        serializeStatus(status, msg);

        if (xSemaphoreTake(statusBufferMutex, portMAX_DELAY) == pdTRUE)
        {
            memcpy(serializedStatusBuffer, &msg, sizeof(Message));
            serializedStatusLength = sizeof(Message);
            xSemaphoreGive(statusBufferMutex);
        }

        xSemaphoreGive(i2cMutex);
    }
}
