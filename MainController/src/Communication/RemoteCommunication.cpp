// Communication/RemoteCommunication.cpp
#include "RemoteCommunication.h"

// Constructor
RemoteCommunication::RemoteCommunication(uint8_t acousticTxPin, uint8_t acousticRxPin)
    : acousticComm(acousticTxPin, acousticRxPin), commMethod(CommunicationMethod::I2C) {}

// Initialize RemoteCommunication with selected method
void RemoteCommunication::init(CommunicationMethod method)
{
    std::lock_guard<std::mutex> lock(commMutex);

    // Initialize I2C communication
    interESPComm.init();

    // Initialize Acoustic Communication
    acousticComm.init();

    // Initialize encryption key securely
    // Replace "1234567890abcdef" with a secure key in production
    memcpy(aes_key, AES_KEY, sizeof(aes_key));

    // Initialize Reed-Solomon with standard parameters (RS(255,223))
    rs.init(255, 223);

    // Set communication method
    setCommunicationMethod(method);

    Serial.println("RemoteCommunication initialized.");
}

// Set communication method
void RemoteCommunication::setCommunicationMethod(CommunicationMethod method)
{
    std::lock_guard<std::mutex> lock(commMutex);
    commMethod = method;
    if (commMethod == CommunicationMethod::I2C)
    {
        Serial.println("Communication method set to I2C.");
    }
    else if (commMethod == CommunicationMethod::Acoustic)
    {
        Serial.println("Communication method set to Acoustic.");
    }
}

// Calculate CRC16 checksum
uint16_t RemoteCommunication::calculateCRC16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF; // Initial value

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC16_POLY;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

// Verify CRC16 checksum
bool RemoteCommunication::verifyCRC16Checksum(uint8_t *data, size_t length, uint16_t checksum)
{
    uint16_t computed = calculateCRC16(data, length);
    return (computed == checksum);
}

// Serialize State into byte buffer
void RemoteCommunication::serializeState(const State &state, uint8_t *buffer, size_t &length)
{
    // Convert State struct into a byte array
    memcpy(buffer, &state, sizeof(State));
    length = sizeof(State);
}

// Deserialize Path from buffer
bool RemoteCommunication::deserializePath(const uint8_t *buffer, size_t length, Path &path)
{
    if (length < sizeof(Waypoint))
    {
        Serial.println("Received Path data length mismatch");
        return false;
    }

    // Assuming the buffer contains a series of waypoints
    size_t numWaypoints = length / sizeof(Waypoint);
    path.waypoints.clear();
    for (size_t i = 0; i < numWaypoints; ++i)
    {
        Waypoint wp;
        memcpy(&wp, buffer + (i * sizeof(Waypoint)), sizeof(Waypoint));
        path.waypoints.push_back(wp);
    }

    return true;
}

// Deserialize OccupancyGrid from buffer
bool RemoteCommunication::deserializeOccupancyGrid(const uint8_t *buffer, size_t length, OccupancyGrid &grid)
{
    if (length < sizeof(int) * 3 + sizeof(float))
    {
        Serial.println("Received OccupancyGrid data length mismatch");
        return false;
    }

    memcpy(&grid.sizeX, buffer, sizeof(int));
    memcpy(&grid.sizeY, buffer + sizeof(int), sizeof(int));
    memcpy(&grid.sizeZ, buffer + 2 * sizeof(int), sizeof(int));
    memcpy(&grid.resolution, buffer + 3 * sizeof(int), sizeof(float));

    size_t dataStart = 3 * sizeof(int) + sizeof(float);
    size_t dataLength = length - dataStart;

    // Prevent buffer overflow
    if (dataLength > (AC_MAX_MESSAGE_SIZE - dataStart))
    {
        Serial.println("OccupancyGrid data size exceeds buffer limits");
        return false;
    }

    grid.gridData.clear();
    for (size_t i = 0; i < dataLength; ++i)
    {
        grid.gridData.push_back(buffer[dataStart + i]);
    }

    return true;
}

// Send State data to Remote Computer via selected communication method
bool RemoteCommunication::sendState(const State &state)
{
    std::lock_guard<std::mutex> lock(commMutex);

    uint8_t buffer[AC_MAX_MESSAGE_SIZE];
    size_t length = 0;
    serializeState(state, buffer, length);

    // Encrypt data
    byte encryptedData[AC_MAX_MESSAGE_SIZE];
    aesLib.encrypt(encryptedData, buffer, length, aes_key);

    // Apply Reed-Solomon encoding
    byte encodedData[AC_MAX_MESSAGE_SIZE + 32]; // RS(255,223) adds 32 parity bytes
    size_t encodedLength = rs.encode(encodedData, encryptedData, length);
    if (encodedLength == 0)
    {
        Serial.println("Reed-Solomon encoding failed for State");
        return false;
    }

    // Compute CRC16 checksum
    uint16_t checksum = calculateCRC16(encodedData, encodedLength);

    // Prepare message
    AcousticMessage msg;
    msg.startByte = AC_START_BYTE;
    msg.type = AcousticMessageType::DATA;
    msg.length = encodedLength;
    memset(msg.payload, 0, sizeof(msg.payload));
    memcpy(msg.payload, encodedData, encodedLength);
    msg.checksum = checksum;

    // Send message via selected communication method
    if (commMethod == CommunicationMethod::I2C)
    {
        // Existing I2C send logic
        // Assuming MotorController expects VelocityCommand, adapt accordingly
        // For State, perhaps implement a different I2C message type
        // Placeholder: Send as VelocityCommand with state data
        // You may need to define a new message type for State if necessary
        VelocityCommand cmd;
        // Populate cmd with relevant state data if applicable
        // Otherwise, implement a separate method for sending State via I2C
        // For now, returning false as it's not defined
        Serial.println("I2C sendState not implemented.");
        return false;
    }
    else if (commMethod == CommunicationMethod::Acoustic)
    {
        return acousticComm.sendMessage(msg);
    }

    return false;
}

// Send VelocityCommand to Motor Controller via selected communication method
bool RemoteCommunication::sendVelocityCommand(const VelocityCommand &cmd)
{
    std::lock_guard<std::mutex> lock(commMutex);

    // Serialize VelocityCommand into byte buffer
    uint8_t buffer[AC_MAX_MESSAGE_SIZE];
    size_t length = sizeof(VelocityCommand);
    memcpy(buffer, &cmd, sizeof(VelocityCommand));

    // Encrypt data
    byte encryptedData[AC_MAX_MESSAGE_SIZE];
    aesLib.encrypt(encryptedData, buffer, length, aes_key);

    // Apply Reed-Solomon encoding
    byte encodedData[AC_MAX_MESSAGE_SIZE + 32]; // RS(255,223) adds 32 parity bytes
    size_t encodedLength = rs.encode(encodedData, encryptedData, length);
    if (encodedLength == 0)
    {
        Serial.println("Reed-Solomon encoding failed for VelocityCommand");
        return false;
    }

    // Compute CRC16 checksum
    uint16_t checksum = calculateCRC16(encodedData, encodedLength);

    // Prepare message
    AcousticMessage msg;
    msg.startByte = AC_START_BYTE;
    msg.type = AcousticMessageType::COMMAND;
    msg.length = encodedLength;
    memset(msg.payload, 0, sizeof(msg.payload));
    memcpy(msg.payload, encodedData, encodedLength);
    msg.checksum = checksum;

    // Send message via selected communication method
    if (commMethod == CommunicationMethod::I2C)
    {
        // Existing I2C send logic
        bool success = interESPComm.sendVelocityCommand(cmd);
        if (success)
        {
            Serial.println("VelocityCommand sent via I2C.");
        }
        else
        {
            Serial.println("Failed to send VelocityCommand via I2C.");
        }
        return success;
    }
    else if (commMethod == CommunicationMethod::Acoustic)
    {
        return acousticComm.sendMessage(msg);
    }

    return false;
}

// Receive Status from Motor Controller via selected communication method
bool RemoteCommunication::receiveStatus(Status &status)
{
    std::lock_guard<std::mutex> lock(commMutex);

    if (commMethod == CommunicationMethod::I2C)
    {
        // Existing I2C receive logic
        return interESPComm.receiveStatus(status);
    }
    else if (commMethod == CommunicationMethod::Acoustic)
    {
        // Acoustic communication receive logic
        AcousticMessage msg;
        bool received = acousticComm.receiveMessage(msg);
        if (received && msg.type == AcousticMessageType::DATA)
        {
            // Assuming Status is sent as DATA message
            // Decrypt data
            byte decryptedData[AC_MAX_MESSAGE_SIZE];
            aesLib.decrypt(decryptedData, msg.payload, msg.length, aes_key);

            // Reed-Solomon decoding
            byte decodedData[AC_MAX_MESSAGE_SIZE];
            size_t decodedLength = rs.decode(decodedData, decryptedData, msg.length);
            if (decodedLength == 0)
            {
                Serial.println("Reed-Solomon decoding failed for Status");
                return false;
            }

            // Deserialize Status
            if (decodedLength < sizeof(Status))
            {
                Serial.println("Deserialized Status size mismatch.");
                return false;
            }
            memcpy(&status, decodedData, sizeof(Status));
            return true;
        }
    }

    return false;
}

// Receive Path from Remote Computer via selected communication method
bool RemoteCommunication::receivePath(Path &path)
{
    std::lock_guard<std::mutex> lock(commMutex);

    if (commMethod == CommunicationMethod::I2C)
    {
        // Existing I2C receive logic
        // Placeholder: Implement based on your I2C protocol
        Serial.println("I2C receivePath not implemented.");
        return false;
    }
    else if (commMethod == CommunicationMethod::Acoustic)
    {
        // Acoustic communication receive logic
        AcousticMessage msg;
        bool received = acousticComm.receiveMessage(msg);
        if (received && msg.type == AcousticMessageType::DATA)
        {
            // Decrypt data
            byte decryptedData[AC_MAX_MESSAGE_SIZE];
            aesLib.decrypt(decryptedData, msg.payload, msg.length, aes_key);

            // Reed-Solomon decoding
            byte decodedData[AC_MAX_MESSAGE_SIZE];
            size_t decodedLength = rs.decode(decodedData, decryptedData, msg.length);
            if (decodedLength == 0)
            {
                Serial.println("Reed-Solomon decoding failed for Path");
                return false;
            }

            // Deserialize Path
            return deserializePath(decodedData, decodedLength, path);
        }
    }

    return false;
}

// Receive OccupancyGrid from Remote Computer via selected communication method
bool RemoteCommunication::receiveOccupancyGrid(OccupancyGrid &grid)
{
    std::lock_guard<std::mutex> lock(commMutex);

    if (commMethod == CommunicationMethod::I2C)
    {
        // Existing I2C receive logic
        // Placeholder: Implement based on your I2C protocol
        Serial.println("I2C receiveOccupancyGrid not implemented.");
        return false;
    }
    else if (commMethod == CommunicationMethod::Acoustic)
    {
        // Acoustic communication receive logic
        AcousticMessage msg;
        bool received = acousticComm.receiveMessage(msg);
        if (received && msg.type == AcousticMessageType::DATA)
        {
            // Decrypt data
            byte decryptedData[AC_MAX_MESSAGE_SIZE];
            aesLib.decrypt(decryptedData, msg.payload, msg.length, aes_key);

            // Reed-Solomon decoding
            byte decodedData[AC_MAX_MESSAGE_SIZE];
            size_t decodedLength = rs.decode(decodedData, decryptedData, msg.length);
            if (decodedLength == 0)
            {
                Serial.println("Reed-Solomon decoding failed for OccupancyGrid");
                return false;
            }

            // Deserialize OccupancyGrid
            return deserializeOccupancyGrid(decodedData, decodedLength, grid);
        }
    }

    return false;
}

// Process incoming acoustic data (to be called in the main loop)
void RemoteCommunication::processIncomingData()
{
    if (commMethod == CommunicationMethod::Acoustic)
    {
        acousticComm.processIncomingData();
    }
}
