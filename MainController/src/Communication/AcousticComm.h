// Communication/AcousticComm.h
#ifndef ACOUSTICCOMM_H
#define ACOUSTICCOMM_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include <mutex>
#include <queue>

// Define constants for acoustic communication
#define AC_START_BYTE 0xAA
#define AC_MESSAGE_TYPE_COMMAND 0x01
#define AC_MESSAGE_TYPE_DATA 0x02
#define AC_MAX_MESSAGE_SIZE 256 // Maximum payload size

// Enumeration for message types specific to Acoustic Communication
enum class AcousticMessageType
{
    COMMAND = 0x01,
    DATA = 0x02
};

// Structure for Acoustic Messages
struct AcousticMessage
{
    uint8_t startByte;
    AcousticMessageType type;
    uint16_t length;
    uint8_t payload[AC_MAX_MESSAGE_SIZE];
    uint16_t checksum;
} __attribute__((packed));

class AcousticComm
{
public:
    AcousticComm(uint8_t txPin, uint8_t rxPin);
    void init();
    bool sendMessage(const AcousticMessage &msg);
    bool receiveMessage(AcousticMessage &msg);
    void processIncomingData(); // To be called frequently in the main loop

private:
    uint8_t TX_PIN;
    uint8_t RX_PIN;

    // Mutex for thread safety
    std::mutex acMutex;

    // Internal queue for received messages
    std::queue<AcousticMessage> incomingQueue;

    // Reception state variables
    enum class ReceiveState
    {
        WAIT_START,
        RECEIVE_HEADER,
        RECEIVE_PAYLOAD,
        RECEIVE_CHECKSUM
    } receiveState;

    AcousticMessage currentMsg;
    size_t payloadBytesReceived;
    uint8_t currentBit;
    bool receivingBit;
    unsigned long pulseStartTime;
    unsigned long pulseDuration;

    // Helper functions
    uint16_t calculateCRC16(const uint8_t *data, size_t length);
    bool verifyCRC16(uint8_t *data, size_t length, uint16_t checksum);
    void resetReception();
};

#endif // ACOUSTICCOMM_H
