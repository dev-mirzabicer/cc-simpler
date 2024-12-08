#ifndef ACOUSTICCOMMUNICATION_H
#define ACOUSTICCOMMUNICATION_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "../CommonMessageDefinitions/Message.h"

// Timing and Protocol Parameters
#define ACOUSTIC_SYMBOL_DURATION_MS 10
// Bit encoding (from your example):
// '1' bit -> short tone (~2 ms), '0' bit -> longer tone (~4 ms)
// We'll replicate logic from your code.
#define ACOUSTIC_TONE_FREQUENCY 40000 // 40kHz tone
#define ACOUSTIC_PREAMBLE_THRESHOLD 600
// Thresholds for distinguishing bits
#define ACOUSTIC_BIT_ZERO_MIN 290
#define ACOUSTIC_BIT_ZERO_MAX 600
#define ACOUSTIC_BIT_ONE_MIN 20
#define ACOUSTIC_BIT_ONE_MAX 290

// Frame format
#define ACOUSTIC_PREAMBLE_BYTE 0xAA
#define ACOUSTIC_PREAMBLE_LENGTH 2 // Let's send two bytes 0xAA after preamble signal
#define ACOUSTIC_HEADER_LENGTH 4
#define ACOUSTIC_FRAME_MAX_PAYLOAD 128
#define ACOUSTIC_CRC_LENGTH 2
#define ACOUSTIC_MAX_FRAME_SIZE (ACOUSTIC_PREAMBLE_LENGTH + ACOUSTIC_HEADER_LENGTH + ACOUSTIC_FRAME_MAX_PAYLOAD + ACOUSTIC_CRC_LENGTH)

// Queues
#define ACOUSTIC_TX_QUEUE_LENGTH 10
#define ACOUSTIC_RX_QUEUE_LENGTH 10

// Forward declarations of tasks
void vAcousticTxTask(void *pvParameters);
void vAcousticRxTask(void *pvParameters);

class AcousticCommunication
{
public:
    AcousticCommunication(uint8_t txPin, uint8_t rxPin);
    void init();
    void startTasks(UBaseType_t txTaskPriority, UBaseType_t rxTaskPriority, size_t txStackSize = 4096, size_t rxStackSize = 4096);

    bool sendMessage(const Message &msg);
    bool receiveMessage(Message &msg);

private:
    friend void vAcousticTxTask(void *pvParameters);
    friend void vAcousticRxTask(void *pvParameters);

    uint8_t TX_PIN;
    uint8_t RX_PIN;

    SemaphoreHandle_t txMutex;
    QueueHandle_t txQueue;
    QueueHandle_t rxQueue;

    bool constructFrame(const Message &msg, uint8_t *frame, size_t &frameLen);
    bool parseFrame(const uint8_t *frame, size_t frameLen, Message &msgOut);
    uint16_t computeCRC(const uint8_t *data, size_t length);

    void transmitFrame(const uint8_t *frame, size_t length);
    void sendByte(uint8_t b);
    void sendBit(bool bit);

    // Receiving
    bool waitForFrameStart();
    bool readFrame(uint8_t *frame, size_t &frameLen);

    bool readByte(uint8_t &outByte);
    bool readBit(bool &outBit);

    // Tone generation
    void generateToneMS(unsigned int ms);
    void delayMs(unsigned int ms);

    // Detection logic (based on your code snippet)
    bool detectSymbol(bool &bit, bool &frameStart);
};

#endif
