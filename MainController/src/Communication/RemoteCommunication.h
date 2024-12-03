#ifndef REMOTECOMMUNICATION_H
#define REMOTECOMMUNICATION_H

#include <Arduino.h>
#include "../Utils/Message.h"
#include <AESLib.h>
#include <ReedSolomon.h>
#include <mutex>
#include <queue>

class RemoteCommunication
{
public:
    RemoteCommunication();
    void init();
    void sendState(const State &state);
    void processReceivedCommands();
    bool receivePath(Path &path);
    bool receiveOccupancyGrid(OccupancyGrid &grid);
    // Additional methods as needed

private:
    // Encryption
    AESLib aesLib;
    byte aes_key[16]; // 16-byte AES key

    // Reed-Solomon
    ReedSolomon rs;

    // Acoustic communication pins (example)
    const uint8_t TX_PIN = 17; // Transmit pin
    const uint8_t RX_PIN = 16; // Receive pin

    // Mutexes for thread safety
    std::mutex encryptionMutex;
    std::mutex pathQueueMutex;
    std::mutex gridQueueMutex;

    // Internal queues to store received paths and occupancy grids
    std::queue<Path> pathQueueInternal;
    std::queue<OccupancyGrid> gridQueueInternal;

    // Helper functions
    void serializeState(const State &state, uint8_t *buffer, size_t &length);
    bool deserializePath(const uint8_t *buffer, size_t length, Path &path);
    bool deserializeOccupancyGrid(const uint8_t *buffer, size_t length, OccupancyGrid &grid);
    uint16_t calculateCRC16Checksum(const uint8_t *data, size_t length);
    bool verifyCRC16Checksum(uint8_t *data, size_t length, uint16_t checksum);

    // Message parsing state
    enum class ReceiveState
    {
        WAIT_START,
        RECEIVE_HEADER,
        RECEIVE_PAYLOAD,
        RECEIVE_CHECKSUM
    } receiveState;

    uint8_t currentMessageType;
    uint16_t currentPayloadLength;
    uint8_t payloadBuffer[MAX_PAYLOAD_SIZE];
    uint16_t receivedChecksum;
    size_t payloadIndex;

    // Helper variables
    uint16_t tempChecksum;
    int headerBytesRead;
    int checksumBytesRead;
};

#endif // REMOTECOMMUNICATION_H
