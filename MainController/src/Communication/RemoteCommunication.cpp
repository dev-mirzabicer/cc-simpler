#include "RemoteCommunication.h"

// Initialize static instance pointer
RemoteCommunication *RemoteCommunication::instance = nullptr;

// Constructor
RemoteCommunication::RemoteCommunication()
    : pulseStartTime(0),
      pulseDuration(0),
      pulseReceived(false),
      receivingData(false),
      bitPosition(0),
      currentByte(0),
      receivedDataBuffer(),
      messageQueue(),
      pathQueueInternal(),
      gridQueueInternal()
{
    // Assign the instance pointer for ISR access
    if (instance == nullptr)
    {
        instance = this;
    }
}

// Initialize RemoteCommunication Module
void RemoteCommunication::init()
{
    // Initialize acoustic transmitter pin
    pinMode(ACOUSTIC_TX_PIN, OUTPUT);
    digitalWrite(ACOUSTIC_TX_PIN, LOW); // Ensure transmitter is off initially

    // Initialize acoustic receiver pin
    pinMode(ACOUSTIC_RX_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ACOUSTIC_RX_PIN), RemoteCommunication::acousticPulseISRWrapper, CHANGE);

    Serial.println("RemoteCommunication initialized for Acoustic Communication.");
}

// Acoustic Pulse ISR Wrapper
void IRAM_ATTR RemoteCommunication::acousticPulseISRWrapper()
{
    if (instance)
    {
        instance->acousticPulseISR();
    }
}

// Acoustic Pulse ISR
void IRAM_ATTR RemoteCommunication::acousticPulseISR()
{
    if (digitalRead(ACOUSTIC_RX_PIN) == HIGH)
    {
        pulseStartTime = micros();
    }
    else
    {
        pulseDuration = micros() - pulseStartTime;
        pulseReceived = true;
    }
}

// Send Data Acoustically
void RemoteCommunication::sendDataAcoustically(const uint8_t *data, size_t length)
{
    std::lock_guard<std::mutex> lock(transmissionMutex);

    for (size_t pos = 0; pos < length; ++pos)
    {
        uint8_t ch = data[pos];
        // Send Start Pulse
        tone(ACOUSTIC_TX_PIN, ACOUSTIC_PULSE_FREQ);
        delay(START_PULSE_DURATION);
        noTone(ACOUSTIC_TX_PIN);
        delay(BIT_DELAY);

        // Send each bit
        for (int i = 0; i < 8; ++i)
        {
            bool bit = bitRead(ch, 7 - i);
            if (bit)
            {
                // '1' Bit
                tone(ACOUSTIC_TX_PIN, ACOUSTIC_PULSE_FREQ);
                delay(BIT_ONE_DURATION);
            }
            else
            {
                // '0' Bit
                tone(ACOUSTIC_TX_PIN, ACOUSTIC_PULSE_FREQ);
                delay(BIT_ZERO_DURATION);
            }
            noTone(ACOUSTIC_TX_PIN);
            delay(BIT_DELAY);
        }
    }
}

// Serialize State into Byte Buffer
void RemoteCommunication::serializeState(const State &state, uint8_t *buffer, size_t &length)
{
    // Convert State struct into a byte array
    memcpy(buffer, &state, sizeof(State));
    length = sizeof(State);
}

// Send State Data Acoustically
void RemoteCommunication::sendState(const State &state)
{
    uint8_t buffer[MAX_MESSAGE_SIZE];
    size_t length = 0;
    serializeState(state, buffer, length);

    // Optional: Apply Encryption and Error Correction here
    // Example: Encrypt and encode using Reed-Solomon

    // Encrypt data
    byte encryptedData[MAX_MESSAGE_SIZE];
    {
        std::lock_guard<std::mutex> lock(encryptionMutex);
        aesLib.encrypt(encryptedData, buffer, length, aes_key);
    }

    // Apply Reed-Solomon encoding
    byte encodedData[MAX_MESSAGE_SIZE + 10]; // Extra bytes for Reed-Solomon parity
    size_t encodedLength = rs.encode(encodedData, encryptedData, length);
    if (encodedLength == 0)
    {
        Serial.println("Reed-Solomon encoding failed for State.");
        return;
    }

    // Transmit data acoustically
    sendDataAcoustically(encodedData, encodedLength);
    Serial.println("State data sent acoustically.");
}

// Process Received Commands (Main Loop for Reception)
void RemoteCommunication::processReceivedCommands()
{
    // Check if a pulse was received
    if (pulseReceived)
    {
        pulseReceived = false;
        receiveDataAcoustically();
    }

    // Reconstruct messages from received data
    reconstructMessages();
}

// Receive Data Acoustically by Decoding Pulses
void RemoteCommunication::receiveDataAcoustically()
{
    std::lock_guard<std::mutex> lock(receptionMutex);

    unsigned long duration = pulseDuration;

    if (duration >= START_PULSE_MIN_DURATION_US && duration <= START_PULSE_MAX_DURATION_US)
    {
        // Detected Start Pulse
        receivingData = true;
        bitPosition = 0;
        currentByte = 0;
        receivedDataBuffer.clear();
        Serial.println("Start pulse detected. Beginning data reception.");
    }
    else if (receivingData)
    {
        bool bit = false;
        if (duration >= BIT_ONE_MIN_DURATION_US && duration <= BIT_ONE_MAX_DURATION_US)
        {
            bit = true;
        }
        else if (duration >= BIT_ZERO_MIN_DURATION_US && duration <= BIT_ZERO_MAX_DURATION_US)
        {
            bit = false;
        }
        else
        {
            // Invalid pulse duration
            Serial.println("Invalid pulse duration detected. Resetting reception.");
            receivingData = false;
            return;
        }

        // Shift and set bit
        currentByte = (currentByte << 1) | bit;
        bitPosition++;

        if (bitPosition == 8)
        {
            // Byte complete
            receivedDataBuffer.push_back(currentByte);
            Serial.print("Received Byte: ");
            Serial.println(currentByte, HEX);
            bitPosition = 0;
            currentByte = 0;
        }
    }
}

// Reconstruct Messages from Received Data Buffer
void RemoteCommunication::reconstructMessages()
{
    // Assuming messages are delimited by a special character, e.g., '#'
    // Modify as per your protocol

    while (!receivedDataBuffer.empty())
    {
        uint8_t ch = receivedDataBuffer.front();
        receivedDataBuffer.erase(receivedDataBuffer.begin());

        if (ch == '#')
        {
            // End of message detected
            // Process the accumulated message
            size_t msgLength = receivedDataBuffer.size();
            uint8_t msgBuffer[msgLength];
            memcpy(msgBuffer, receivedDataBuffer.data(), msgLength);

            // Verify checksum if implemented
            // Example: Last two bytes are checksum
            if (msgLength < 2)
            {
                Serial.println("Received message too short for checksum.");
                continue;
            }

            uint16_t receivedChecksum;
            memcpy(&receivedChecksum, &msgBuffer[msgLength - 2], 2);
            size_t dataLength = msgLength - 2;

            if (!verifyCRC16Checksum(msgBuffer, dataLength, receivedChecksum))
            {
                Serial.println("Checksum verification failed for received message.");
                continue;
            }

            // Decrypt data
            byte decryptedData[MAX_MESSAGE_SIZE];
            {
                std::lock_guard<std::mutex> lock(encryptionMutex);
                aesLib.decrypt(decryptedData, msgBuffer, dataLength, aes_key);
            }

            // Decode using Reed-Solomon
            byte decodedData[MAX_MESSAGE_SIZE];
            size_t decodedLength = rs.decode(decodedData, decryptedData, dataLength);
            if (decodedLength == 0)
            {
                Serial.println("Reed-Solomon decoding failed for received message.");
                continue;
            }

            // Deserialize message based on MessageType
            // For example, assume the first byte indicates the message type
            Message receivedMsg;
            memcpy(&receivedMsg, decodedData, sizeof(Message));

            switch (receivedMsg.type)
            {
            case MessageType::PATH_UPDATE:
            {
                Path receivedPath;
                if (deserializePath(receivedMsg.payload, receivedMsg.length, receivedPath))
                {
                    std::lock_guard<std::mutex> lock(pathQueueMutex);
                    pathQueueInternal.push(receivedPath);
                    Serial.println("Path update received and enqueued.");
                }
                else
                {
                    Serial.println("Failed to deserialize Path update.");
                }
                break;
            }
            case MessageType::OCCUPANCY_GRID:
            {
                OccupancyGrid receivedGrid;
                if (deserializeOccupancyGrid(receivedMsg.payload, receivedMsg.length, receivedGrid))
                {
                    std::lock_guard<std::mutex> lock(gridQueueMutex);
                    gridQueueInternal.push(receivedGrid);
                    Serial.println("Occupancy Grid received and enqueued.");
                }
                else
                {
                    Serial.println("Failed to deserialize Occupancy Grid.");
                }
                break;
            }
            default:
                Serial.println("Unknown MessageType received.");
                break;
            }

            // Clear the buffer after processing
            receivedDataBuffer.clear();
        }
        else
        {
            // Accumulate data until end delimiter is found
            // Depending on protocol, implement accordingly
            // For this example, data is processed byte by byte
            // So no action needed here
        }
    }
}

// Reconstruct Messages from Received Data
void RemoteCommunication::reconstructMessages()
{
    // Assuming messages are delimited by a special character, e.g., '#'
    // Modify as per your protocol

    while (!receivedDataBuffer.empty())
    {
        uint8_t ch = receivedDataBuffer.front();
        receivedDataBuffer.erase(receivedDataBuffer.begin());

        if (ch == '#')
        {
            // End of message detected
            // Process the accumulated message
            size_t msgLength = receivedDataBuffer.size();
            uint8_t msgBuffer[msgLength];
            memcpy(msgBuffer, receivedDataBuffer.data(), msgLength);

            // Verify checksum if implemented
            // Example: Last two bytes are checksum
            if (msgLength < 2)
            {
                Serial.println("Received message too short for checksum.");
                continue;
            }

            uint16_t receivedChecksum;
            memcpy(&receivedChecksum, &msgBuffer[msgLength - 2], 2);
            size_t dataLength = msgLength - 2;

            if (!verifyCRC16Checksum(msgBuffer, dataLength, receivedChecksum))
            {
                Serial.println("Checksum verification failed for received message.");
                continue;
            }

            // Decrypt data
            byte decryptedData[MAX_MESSAGE_SIZE];
            {
                std::lock_guard<std::mutex> lock(encryptionMutex);
                aesLib.decrypt(decryptedData, msgBuffer, dataLength, aes_key);
            }

            // Decode using Reed-Solomon
            byte decodedData[MAX_MESSAGE_SIZE];
            size_t decodedLength = rs.decode(decodedData, decryptedData, dataLength);
            if (decodedLength == 0)
            {
                Serial.println("Reed-Solomon decoding failed for received message.");
                continue;
            }

            // Deserialize message based on MessageType
            // For example, assume the first byte indicates the message type
            Message receivedMsg;
            memcpy(&receivedMsg, decodedData, sizeof(Message));

            switch (receivedMsg.type)
            {
            case MessageType::PATH_UPDATE:
            {
                Path receivedPath;
                if (deserializePath(receivedMsg.payload, receivedMsg.length, receivedPath))
                {
                    std::lock_guard<std::mutex> lock(pathQueueMutex);
                    pathQueueInternal.push(receivedPath);
                    Serial.println("Path update received and enqueued.");
                }
                else
                {
                    Serial.println("Failed to deserialize Path update.");
                }
                break;
            }
            case MessageType::OCCUPANCY_GRID:
            {
                OccupancyGrid receivedGrid;
                if (deserializeOccupancyGrid(receivedMsg.payload, receivedMsg.length, receivedGrid))
                {
                    std::lock_guard<std::mutex> lock(gridQueueMutex);
                    gridQueueInternal.push(receivedGrid);
                    Serial.println("Occupancy Grid received and enqueued.");
                }
                else
                {
                    Serial.println("Failed to deserialize Occupancy Grid.");
                }
                break;
            }
            default:
                Serial.println("Unknown MessageType received.");
                break;
            }

            // Clear the buffer after processing
            receivedDataBuffer.clear();
        }
        else
        {
            // Accumulate data until end delimiter is found
            // Depending on protocol, implement accordingly
            // For this example, data is processed byte by byte
            // So no action needed here
        }
    }
}

// Serialize State into Byte Buffer
void RemoteCommunication::serializeState(const State &state, uint8_t *buffer, size_t &length)
{
    // Convert State struct into a byte array
    memcpy(buffer, &state, sizeof(State));
    length = sizeof(State);
}

// Decode and Deserialize Path
bool RemoteCommunication::deserializePath(const uint8_t *buffer, size_t length, Path &path)
{
    if (length < sizeof(Waypoint))
    {
        Serial.println("Received Path data length mismatch.");
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

// Decode and Deserialize OccupancyGrid
bool RemoteCommunication::deserializeOccupancyGrid(const uint8_t *buffer, size_t length, OccupancyGrid &grid)
{
    if (length < sizeof(int) * 3 + sizeof(float))
    {
        Serial.println("Received OccupancyGrid data length mismatch.");
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
        Serial.println("OccupancyGrid data size exceeds buffer limits.");
        return false;
    }

    grid.gridData.clear();
    for (size_t i = 0; i < dataLength; ++i)
    {
        grid.gridData.push_back(buffer[dataStart + i]);
    }

    return true;
}

// Calculate CRC16 Checksum
uint16_t RemoteCommunication::calculateCRC16Checksum(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF; // Initial value
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021; // CRC-CCITT Polynomial
            else
                crc = crc << 1;
        }
    }
    return crc;
}

// Verify CRC16 Checksum
bool RemoteCommunication::verifyCRC16Checksum(const uint8_t *data, size_t length, uint16_t checksum)
{
    uint16_t computedCRC = calculateCRC16Checksum(data, length);
    return (computedCRC == checksum);
}

// Decode Received Pulses and Reconstruct Data
void RemoteCommunication::decodeReceivedPulses()
{
    // This function can be expanded based on specific protocol requirements
    // Currently, reconstructMessages handles message parsing
}

// Example Function to Send Encrypted and Encoded Data
void RemoteCommunication::encodeAndSend(const uint8_t *data, size_t length)
{
    // Encrypt Data
    byte encryptedData[MAX_MESSAGE_SIZE];
    {
        std::lock_guard<std::mutex> lock(encryptionMutex);
        aesLib.encrypt(encryptedData, data, length, aes_key);
    }

    // Apply Reed-Solomon Encoding
    byte encodedData[MAX_MESSAGE_SIZE + 10]; // Extra bytes for parity
    size_t encodedLength = rs.encode(encodedData, encryptedData, length);
    if (encodedLength == 0)
    {
        Serial.println("Reed-Solomon encoding failed.");
        return;
    }

    // Append checksum
    uint16_t checksum = calculateCRC16Checksum(encodedData, encodedLength);
    memcpy(encodedData + encodedLength, &checksum, sizeof(checksum));
    encodedLength += sizeof(checksum);

    // Send Data Acoustically
    sendDataAcoustically(encodedData, encodedLength);
}

// Placeholder: Implement Path and OccupancyGrid Reception
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
