#ifndef REMOTECOMMUNICATION_H
#define REMOTECOMMUNICATION_H

#include <Arduino.h>
#include "../Utils/Message.h"
#include <AESLib.h>
#include <ReedSolomon.h>
#include <mutex>
#include <queue>
#include <vector>

// Acoustic Communication Constants
#define ACOUSTIC_TX_PIN 17        // GPIO pin connected to ultrasonic transducer (transmitter)
#define ACOUSTIC_RX_PIN 16        // GPIO pin connected to ultrasonic receiver output
#define ACOUSTIC_FREQUENCY 40000  // Frequency in Hz (e.g., 40 kHz)
#define START_PULSE_DURATION_MS 6 // Start pulse duration in milliseconds
#define BIT_ONE_DURATION_MS 2     // '1' bit pulse duration in milliseconds
#define BIT_ZERO_DURATION_MS 4    // '0' bit pulse duration in milliseconds
#define BIT_DELAY_MS 10           // Delay between bits in milliseconds

// Pulse Duration Thresholds (in microseconds) for Reception
#define START_PULSE_MIN_DURATION_US 5500 // Minimum duration for start pulse
#define START_PULSE_MAX_DURATION_US 6500 // Maximum duration for start pulse
#define BIT_ONE_MIN_DURATION_US 1500     // Minimum duration for '1' bit
#define BIT_ONE_MAX_DURATION_US 2500     // Maximum duration for '1' bit
#define BIT_ZERO_MIN_DURATION_US 3500    // Minimum duration for '0' bit
#define BIT_ZERO_MAX_DURATION_US 4500    // Maximum duration for '0' bit

// Maximum Payload Size
#define MAX_MESSAGE_SIZE 256

// Enumeration for Receive State
enum class ReceiveState
{
    WAIT_START,
    RECEIVE_HEADER,
    RECEIVE_PAYLOAD,
    RECEIVE_CHECKSUM
};

// RemoteCommunication Class
class RemoteCommunication
{
public:
    RemoteCommunication();
    void init();

    // Transmit Methods
    void sendState(const State &state);

    // Reception Methods
    void processReceivedCommands();
    bool receivePath(Path &path);
    bool receiveOccupancyGrid(OccupancyGrid &grid);

private:
    // Encryption
    AESLib aesLib;
    byte aes_key[16]; // 16-byte AES key (ensure secure initialization)

    // Reed-Solomon
    ReedSolomon rs;

    // Acoustic Communication Variables
    volatile unsigned long pulseStartTime; // Start time of pulse (in microseconds)
    volatile unsigned long pulseDuration;  // Duration of pulse (in microseconds)
    volatile bool pulseReceived;           // Flag indicating a pulse was received

    bool receivingData;                      // Flag indicating data reception in progress
    uint8_t bitPosition;                     // Current bit position in the byte being received
    uint8_t currentByte;                     // Current byte being reconstructed
    std::vector<uint8_t> receivedDataBuffer; // Buffer to store received bytes

    // Message Buffer for Received Data
    std::queue<Message> messageQueue;

    // Mutexes for Thread Safety
    std::mutex transmissionMutex;
    std::mutex receptionMutex;
    std::mutex pathQueueMutex;
    std::mutex gridQueueMutex;

    // Internal Queues to Store Received Paths and Occupancy Grids
    std::queue<Path> pathQueueInternal;
    std::queue<OccupancyGrid> gridQueueInternal;

    // Helper Functions
    void serializeState(const State &state, uint8_t *buffer, size_t &length);
    bool deserializePath(const uint8_t *buffer, size_t length, Path &path);
    bool deserializeOccupancyGrid(const uint8_t *buffer, size_t length, OccupancyGrid &grid);
    uint16_t calculateCRC16Checksum(const uint8_t *data, size_t length);
    bool verifyCRC16Checksum(const uint8_t *data, size_t length, uint16_t checksum);

    // Acoustic Transmission Functions
    void sendDataAcoustically(const uint8_t *data, size_t length);

    // Acoustic Reception Functions
    static void IRAM_ATTR acousticPulseISRWrapper();
    void IRAM_ATTR acousticPulseISR();
    void receiveDataAcoustically();
    void reconstructMessages();

    // Data Encoding/Decoding
    void encodeAndSend(const uint8_t *data, size_t length);
    void decodeReceivedPulses();

    // ISR Registration
    static RemoteCommunication *instance;

    // Constants for Encoding
    static const unsigned int ACOUSTIC_PULSE_FREQ = ACOUSTIC_FREQUENCY;
    static const unsigned int START_PULSE_DURATION = START_PULSE_DURATION_MS;
    static const unsigned int BIT_ONE_DURATION = BIT_ONE_DURATION_MS;
    static const unsigned int BIT_ZERO_DURATION = BIT_ZERO_DURATION_MS;
    static const unsigned int BIT_DELAY = BIT_DELAY_MS;
};

#endif // REMOTECOMMUNICATION_H
