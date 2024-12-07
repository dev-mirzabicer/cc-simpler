// Communication/RemoteCommunication.h
#ifndef REMOTECOMMUNICATION_H
#define REMOTECOMMUNICATION_H

#include <Arduino.h>
#include "InterESPCommunication.h"
#include "AcousticComm.h" // Include AcousticComm
#include "../CommonMessageDefinitions/Message.h"
#include <AESLib.h>
#include <ReedSolomon.h>
#include <mutex>
#include <queue>

// Define communication method enumeration
enum class CommunicationMethod
{
    I2C,
    Acoustic
};

class RemoteCommunication
{
public:
    RemoteCommunication(uint8_t acousticTxPin = 17, uint8_t acousticRxPin = 16);
    void init(CommunicationMethod method = CommunicationMethod::I2C);
    void setCommunicationMethod(CommunicationMethod method);

    // Methods to send and receive data
    bool sendState(const State &state);
    bool sendVelocityCommand(const VelocityCommand &cmd);
    bool receiveStatus(Status &status);
    bool receivePath(Path &path);
    bool receiveOccupancyGrid(OccupancyGrid &grid);

    // Acoustic communication processing
    void processIncomingData(); // To be called frequently in the main loop

private:
    // Communication method
    CommunicationMethod commMethod;

    // I2C Communication
    InterESPCommunication interESPComm;

    // Acoustic Communication
    AcousticComm acousticComm;

    // Encryption
    AESLib aesLib;
    byte aes_key[16];

    // Reed-Solomon
    ReedSolomon rs;

    // Mutex for thread safety
    std::mutex commMutex;

    // Internal queues for received data
    std::queue<Path> pathQueueInternal;
    std::queue<OccupancyGrid> gridQueueInternal;

    // Helper functions
    void serializeState(const State &state, uint8_t *buffer, size_t &length);
    bool deserializePath(const uint8_t *buffer, size_t length, Path &path);
    bool deserializeOccupancyGrid(const uint8_t *buffer, size_t length, OccupancyGrid &grid);
    uint16_t calculateCRC16Checksum(const uint8_t *data, size_t length);
    bool verifyCRC16Checksum(uint8_t *data, size_t length, uint16_t checksum);

    // CRC16 calculation
    uint16_t calculateCRC16(const uint8_t *data, size_t length);
};

#endif // REMOTECOMMUNICATION_H
