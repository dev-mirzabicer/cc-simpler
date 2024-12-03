#include "InterESPCommunication.h"

// Constants for retry mechanism
#define I2C_MAX_RETRIES 3
#define I2C_RETRY_DELAY_MS 50 // Delay between retries in milliseconds

// Constructor
InterESPCommunication::InterESPCommunication()
{
    // Initialize variables if necessary
}

// Initialize I2C communication
void InterESPCommunication::init()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);    // Join I2C bus as master
    Wire.setClock(INTER_ESP_COMM_BAUD_RATE); // Set I2C clock to defined rate
    Serial.println("InterESPCommunication initialized as I2C Master.");
}

// Serialize VelocityCommand into Message
void InterESPCommunication::serializeVelocityCommand(const VelocityCommand &commands, Message &msg)
{
    msg.startByte = 0xAA; // Example start byte
    msg.type = MessageType::VELOCITY_COMMAND;
    msg.length = sizeof(VelocityCommand);
    memset(msg.payload, 0, sizeof(msg.payload));
    memcpy(msg.payload, &commands, sizeof(VelocityCommand));
    msg.checksum = calculateCRC16(msg.payload, msg.length);
}

// Send VelocityCommand to Motor Controller with retry logic
bool InterESPCommunication::sendVelocityCommand(const VelocityCommand &commands)
{
    Message msg;
    serializeVelocityCommand(commands, msg);

    bool transmissionSuccess = false;
    {
        std::lock_guard<std::mutex> lock(i2cMutex); // Protect I2C operations
        for (int attempt = 0; attempt < I2C_MAX_RETRIES; ++attempt)
        {
            Wire.beginTransmission(MOTOR_CONTROLLER_I2C_ADDRESS);
            Wire.write((uint8_t *)&msg, sizeof(Message));
            uint8_t error = Wire.endTransmission();

            if (error == 0)
            {
                transmissionSuccess = true;
                Serial.println("VelocityCommand transmitted successfully.");
                break; // Successful transmission
            }
            else
            {
                Serial.print("I2C Transmission Error to Motor Controller: ");
                Serial.println(error);
                delay(I2C_RETRY_DELAY_MS); // Wait before retrying
            }
        }
    }

    if (!transmissionSuccess)
    {
        Serial.println("Failed to transmit Velocity Command after multiple attempts.");
        // Implement additional error handling if necessary
        return false;
    }

    return true;
}

// Deserialize Status from received Message
bool InterESPCommunication::deserializeStatus(const uint8_t *buffer, size_t length, Status &status)
{
    if (length < sizeof(Message))
    {
        Serial.println("Received data length mismatch for Status.");
        return false;
    }

    Message msg;
    memcpy(&msg, buffer, sizeof(Message));

    // Verify checksum using CRC16
    uint16_t computedChecksum = calculateCRC16(msg.payload, msg.length);
    if (computedChecksum != msg.checksum)
    {
        Serial.println("Status checksum verification failed.");
        return false;
    }

    if (msg.type != MessageType::STATUS_UPDATE)
    {
        Serial.println("Incorrect message type received for Status.");
        return false;
    }

    // Deserialize Status
    if (msg.length > sizeof(Status))
    {
        Serial.println("Status payload size mismatch.");
        return false;
    }
    memcpy(&status, msg.payload, sizeof(Status));

    return true;
}

// Receive Status from Motor Controller
bool InterESPCommunication::receiveStatus(Status &status)
{
    std::lock_guard<std::mutex> lock(i2cMutex);
    Message msg;
    Wire.requestFrom(MOTOR_CONTROLLER_I2C_ADDRESS, sizeof(Message));
    uint8_t buffer[sizeof(Message)] = {0};
    size_t len = Wire.readBytes(buffer, sizeof(Message));

    if (len != sizeof(Message))
    {
        Serial.println("Incomplete Status message received.");
        return false;
    }

    // Deserialize Status
    if (deserializeStatus(buffer, len, status))
    {
        Serial.println("Status received successfully.");
        return true;
    }
    else
    {
        Serial.println("Failed to deserialize Status message.");
        return false;
    }
}
