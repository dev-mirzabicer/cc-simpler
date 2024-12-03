#include "RemoteCommunication.h"

// Define constants for message parsing
#define MESSAGE_START_BYTE 0xAA
#define MAX_MESSAGE_SIZE 512 // Adjust as needed based on maximum expected payload

// Constructor
RemoteCommunication::RemoteCommunication()
    : receiveState(ReceiveState::WAIT_START),
      currentMessageType(0),
      currentPayloadLength(0),
      payloadIndex(0),
      receivedChecksum(0),
      tempChecksum(0),
      headerBytesRead(0),
      checksumBytesRead(0)
{
    // Initialize AES key securely
    // Example: Use a secure method to set the key, such as reading from non-volatile memory or secure storage
    // For demonstration, we'll use a predefined key
    const char *secureKey = AES_KEY; // Ensure AES_KEY is defined securely
    memcpy(aes_key, secureKey, sizeof(aes_key));

    // Initialize Reed-Solomon with appropriate parameters
    // Example: RS(255, 223) for standard RS-255/223
    rs.init(255, 223);
}

// Initialize acoustic communication
void RemoteCommunication::init()
{
    // Initialize acoustic communication hardware (e.g., sonar sensors)
    // Configure TX and RX pins if necessary
    // Example: initialize serial communication for acoustic modem
    Serial1.begin(REMOTE_COMM_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
}

// Serialize State into byte buffer
void RemoteCommunication::serializeState(const State &state, uint8_t *buffer, size_t &length)
{
    // Convert State struct into a byte array
    memcpy(buffer, &state, sizeof(State));
    length = sizeof(State);
}

// Deserialize Path from byte buffer
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

// Deserialize OccupancyGrid from byte buffer
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
    if (dataLength > (MAX_MESSAGE_SIZE - dataStart))
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

// Calculate CRC16 checksum
uint16_t RemoteCommunication::calculateCRC16Checksum(const uint8_t *data, size_t length)
{
    return calculateCRC16(data, length);
}

// Verify CRC16 checksum
bool RemoteCommunication::verifyCRC16Checksum(uint8_t *data, size_t length, uint16_t checksum)
{
    uint16_t computed = calculateCRC16Checksum(data, length);
    return (computed == checksum);
}

// Send State data to Remote Computer
void RemoteCommunication::sendState(const State &state)
{
    uint8_t buffer[MAX_MESSAGE_SIZE];
    size_t length = 0;
    serializeState(state, buffer, length);

    // Encrypt data
    byte encryptedData[MAX_MESSAGE_SIZE];
    {
        std::lock_guard<std::mutex> lock(encryptionMutex);
        aesLib.encrypt(encryptedData, buffer, length, aes_key);
    }

    // Apply Reed-Solomon encoding
    byte encodedData[MAX_MESSAGE_SIZE + 10]; // 10-byte parity (example)
    size_t encodedLength = rs.encode(encodedData, encryptedData, length);
    if (encodedLength == 0)
    {
        Serial.println("Reed-Solomon encoding failed for State");
        return;
    }

    // Compute CRC16 checksum
    uint16_t checksum = calculateCRC16Checksum(encodedData, encodedLength);

    // Prepare message
    Message msg;
    msg.startByte = MESSAGE_START_BYTE;
    msg.type = MessageType::STATUS_UPDATE; // Assuming sending status; adjust as needed
    msg.length = encodedLength;
    memset(msg.payload, 0, sizeof(msg.payload));
    memcpy(msg.payload, encodedData, encodedLength);
    msg.checksum = checksum;

    // Transmit data via Serial1 (acoustic modem)
    Serial1.write((uint8_t *)&msg, sizeof(Message));
}

// Process any received commands
void RemoteCommunication::processReceivedCommands()
{
    while (Serial1.available() > 0)
    {
        uint8_t byte = Serial1.read();

        switch (receiveState)
        {
        case ReceiveState::WAIT_START:
            if (byte == MESSAGE_START_BYTE)
            {
                receiveState = ReceiveState::RECEIVE_HEADER;
                currentMessageType = 0;
                currentPayloadLength = 0;
                payloadIndex = 0;
                memset(payloadBuffer, 0, sizeof(payloadBuffer));
                receivedChecksum = 0;
                tempChecksum = 0;
                headerBytesRead = 0;
                checksumBytesRead = 0;
            }
            break;

        case ReceiveState::RECEIVE_HEADER:
            // Read MessageType (1 byte) and Length (2 bytes)
            if (headerBytesRead == 0)
            {
                currentMessageType = byte;
                headerBytesRead++;
            }
            else if (headerBytesRead == 1)
            {
                currentPayloadLength = byte;
                headerBytesRead++;
            }
            else if (headerBytesRead == 2)
            {
                currentPayloadLength |= ((uint16_t)byte << 8);
                headerBytesRead++;
                if (currentPayloadLength > MAX_PAYLOAD_SIZE)
                {
                    Serial.println("Payload length exceeds maximum limit. Discarding message.");
                    receiveState = ReceiveState::WAIT_START;
                }
                else
                {
                    receiveState = ReceiveState::RECEIVE_PAYLOAD;
                }
            }
            break;

        case ReceiveState::RECEIVE_PAYLOAD:
            payloadBuffer[payloadIndex++] = byte;
            if (payloadIndex >= currentPayloadLength)
            {
                receiveState = ReceiveState::RECEIVE_CHECKSUM;
            }
            break;

        case ReceiveState::RECEIVE_CHECKSUM:
            // Read checksum (2 bytes)
            tempChecksum |= ((uint16_t)byte << (8 * checksumBytesRead));
            checksumBytesRead++;
            if (checksumBytesRead >= 2)
            {
                // Verify checksum
                if (verifyCRC16Checksum(payloadBuffer, currentPayloadLength, tempChecksum))
                {
                    // Decode Reed-Solomon
                    byte decodedData[MAX_MESSAGE_SIZE];
                    size_t decodedLength = rs.decode(decodedData, payloadBuffer, currentPayloadLength);
                    if (decodedLength == 0)
                    {
                        Serial.println("Reed-Solomon decoding failed.");
                        // Reset state
                        receiveState = ReceiveState::WAIT_START;
                        checksumBytesRead = 0;
                        tempChecksum = 0;
                        break;
                    }

                    // Decrypt data
                    byte decryptedData[MAX_MESSAGE_SIZE];
                    {
                        std::lock_guard<std::mutex> lock(encryptionMutex);
                        aesLib.decrypt(decryptedData, decodedData, decodedLength, aes_key);
                    }

                    // Dispatch based on MessageType
                    switch (static_cast<MessageType>(currentMessageType))
                    {
                    case MessageType::PATH_UPDATE:
                    {
                        Path receivedPath;
                        if (deserializePath(decryptedData, decodedLength, receivedPath))
                        {
                            // Enqueue the received path
                            {
                                std::lock_guard<std::mutex> lock(pathQueueMutex);
                                if (pathQueueInternal.size() < 10) // Limit queue size
                                {
                                    pathQueueInternal.push(receivedPath);
                                    Serial.println("Path received and enqueued successfully.");
                                }
                                else
                                {
                                    Serial.println("Path queue is full. Dropping received path.");
                                }
                            }
                        }
                        else
                        {
                            Serial.println("Failed to deserialize Path data.");
                        }
                        break;
                    }
                    case MessageType::OCCUPANCY_GRID:
                    {
                        OccupancyGrid receivedGrid;
                        if (deserializeOccupancyGrid(decryptedData, decodedLength, receivedGrid))
                        {
                            // Enqueue the received occupancy grid
                            {
                                std::lock_guard<std::mutex> lock(gridQueueMutex);
                                if (gridQueueInternal.size() < 10) // Limit queue size
                                {
                                    gridQueueInternal.push(receivedGrid);
                                    Serial.println("OccupancyGrid received and enqueued successfully.");
                                }
                                else
                                {
                                    Serial.println("OccupancyGrid queue is full. Dropping received grid.");
                                }
                            }
                        }
                        else
                        {
                            Serial.println("Failed to deserialize OccupancyGrid data.");
                        }
                        break;
                    }
                    default:
                        Serial.println("Received unsupported message type.");
                        break;
                    }
                }
                else
                {
                    Serial.println("Checksum mismatch. Discarding message.");
                }

                // Reset state
                receiveState = ReceiveState::WAIT_START;
                checksumBytesRead = 0;
                tempChecksum = 0;
            }
            break;

        default:
            // Unknown state, reset
            receiveState = ReceiveState::WAIT_START;
            break;
        }
    }
}

// Receive Path from Remote Computer
bool RemoteCommunication::receivePath(Path &path)
{
    std::lock_guard<std::mutex> lock(pathQueueMutex);
    if (!pathQueueInternal.empty())
    {
        path = pathQueueInternal.front();
        pathQueueInternal.pop();
        return true;
    }
    return false;
}

// Receive OccupancyGrid from Remote Computer
bool RemoteCommunication::receiveOccupancyGrid(OccupancyGrid &grid)
{
    std::lock_guard<std::mutex> lock(gridQueueMutex);
    if (!gridQueueInternal.empty())
    {
        grid = gridQueueInternal.front();
        gridQueueInternal.pop();
        return true;
    }
    return false;
}
