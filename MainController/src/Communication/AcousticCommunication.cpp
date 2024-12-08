#include "AcousticCommunication.h"
#include <string.h> // for memcpy

static AcousticCommunication *gAcousticCommInstance = nullptr;

AcousticCommunication::AcousticCommunication(uint8_t txPin, uint8_t rxPin)
    : TX_PIN(txPin), RX_PIN(rxPin)
{
}

void AcousticCommunication::init()
{
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT_PULLUP);

    txMutex = xSemaphoreCreateMutex();
    txQueue = xQueueCreate(ACOUSTIC_TX_QUEUE_LENGTH, sizeof(Message));
    rxQueue = xQueueCreate(ACOUSTIC_RX_QUEUE_LENGTH, sizeof(Message));

    gAcousticCommInstance = this;
    Serial.println("AcousticCommunication initialized.");
}

void AcousticCommunication::startTasks(UBaseType_t txTaskPriority, UBaseType_t rxTaskPriority, size_t txStackSize, size_t rxStackSize)
{
    xTaskCreate(vAcousticTxTask, "AcousticTxTask", txStackSize, this, txTaskPriority, NULL);
    xTaskCreate(vAcousticRxTask, "AcousticRxTask", rxStackSize, this, rxTaskPriority, NULL);
}

bool AcousticCommunication::sendMessage(const Message &msg)
{
    if (xSemaphoreTake(txMutex, portMAX_DELAY) == pdTRUE)
    {
        bool success = (xQueueSend(txQueue, &msg, portMAX_DELAY) == pdPASS);
        xSemaphoreGive(txMutex);
        return success;
    }
    return false;
}

bool AcousticCommunication::receiveMessage(Message &msg)
{
    return (xQueueReceive(rxQueue, &msg, 0) == pdPASS);
}

// TX Task
void vAcousticTxTask(void *pvParameters)
{
    AcousticCommunication *comm = (AcousticCommunication *)pvParameters;
    Message msg;
    uint8_t frame[ACOUSTIC_MAX_FRAME_SIZE];
    size_t frameLen = 0;

    while (1)
    {
        if (xQueueReceive(comm->txQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            if (comm->constructFrame(msg, frame, frameLen))
            {
                comm->transmitFrame(frame, frameLen);
                Serial.println("AcousticCommunication: Frame transmitted.");
            }
            else
            {
                Serial.println("AcousticCommunication: Failed to construct frame.");
            }
        }
    }
}

// RX Task
void vAcousticRxTask(void *pvParameters)
{
    AcousticCommunication *comm = (AcousticCommunication *)pvParameters;
    uint8_t frame[ACOUSTIC_MAX_FRAME_SIZE];
    size_t frameLen;
    Message msgOut;

    while (1)
    {
        if (comm->waitForFrameStart())
        {
            if (comm->readFrame(frame, frameLen))
            {
                if (comm->parseFrame(frame, frameLen, msgOut))
                {
                    xQueueSend(comm->rxQueue, &msgOut, portMAX_DELAY);
                    Serial.println("AcousticCommunication: Frame received and parsed.");
                }
                else
                {
                    Serial.println("AcousticCommunication: CRC or parse error in received frame.");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool AcousticCommunication::constructFrame(const Message &msg, uint8_t *frame, size_t &frameLen)
{
    if (msg.length > ACOUSTIC_FRAME_MAX_PAYLOAD)
        return false;

    // Frame format:
    // Start-of-frame: We'll send a special "preamble signal" (a long tone)
    // Then ACOUSTIC_PREAMBLE_LENGTH bytes of 0xAA
    // Then [type(1), length(2), reserved(1)]
    // Payload
    // CRC(2)

    // We'll build the entire frame array (except the preamble signal is sent differently)
    size_t index = 0;
    for (int i = 0; i < ACOUSTIC_PREAMBLE_LENGTH; i++)
        frame[index++] = 0xAA;

    frame[index++] = (uint8_t)msg.type;
    frame[index++] = (uint8_t)((msg.length >> 8) & 0xFF);
    frame[index++] = (uint8_t)(msg.length & 0xFF);
    frame[index++] = 0x00; // reserved

    memcpy(&frame[index], msg.payload, msg.length);
    index += msg.length;

    uint16_t crc = computeCRC(msg.payload, msg.length);
    frame[index++] = (uint8_t)((crc >> 8) & 0xFF);
    frame[index++] = (uint8_t)(crc & 0xFF);

    frameLen = index;
    return true;
}

bool AcousticCommunication::parseFrame(const uint8_t *frame, size_t frameLen, Message &msgOut)
{
    if (frameLen < (ACOUSTIC_PREAMBLE_LENGTH + ACOUSTIC_HEADER_LENGTH + ACOUSTIC_CRC_LENGTH))
        return false;

    size_t index = 0;
    // Check preamble
    for (int i = 0; i < ACOUSTIC_PREAMBLE_LENGTH; i++)
    {
        if (frame[index++] != 0xAA)
            return false;
    }

    uint8_t type = frame[index++];
    uint16_t length = ((uint16_t)frame[index++] << 8) | (uint16_t)frame[index++];
    index++; // reserved

    if (length > ACOUSTIC_FRAME_MAX_PAYLOAD)
        return false;

    if (frameLen < ACOUSTIC_PREAMBLE_LENGTH + ACOUSTIC_HEADER_LENGTH + length + ACOUSTIC_CRC_LENGTH)
        return false;

    memcpy(msgOut.payload, &frame[index], length);
    index += length;

    uint16_t receivedCRC = ((uint16_t)frame[index++] << 8) | (uint16_t)frame[index++];

    uint16_t computed = computeCRC(msgOut.payload, length);
    if (computed != receivedCRC)
        return false;

    msgOut.startByte = 0xAA;
    msgOut.type = (MessageType)type;
    msgOut.length = length;
    msgOut.checksum = receivedCRC;
    return true;
}

uint16_t AcousticCommunication::computeCRC(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

void AcousticCommunication::transmitFrame(const uint8_t *frame, size_t length)
{
    // First send a "frame start" signal: a long tone that receiver interprets as >600 counts
    // We'll send tone for e.g., 20ms continuously to ensure >600 counts
    generateToneMS(20);

    // Now send the frame bytes
    for (size_t i = 0; i < length; i++)
    {
        sendByte(frame[i]);
    }
}

void AcousticCommunication::sendByte(uint8_t b)
{
    for (int bit = 7; bit >= 0; bit--)
    {
        bool bitVal = ((b >> bit) & 0x01);
        sendBit(bitVal);
    }
}

void AcousticCommunication::sendBit(bool bit)
{
    // From your example:
    // If bit=1: short tone ~2 ms
    // If bit=0: longer tone ~4 ms
    // Symbol total: 10 ms

    if (bit)
    {
        generateToneMS(2);
        // remainder of symbol ~8 ms silence
        noTone(TX_PIN);
        delayMs(8);
    }
    else
    {
        generateToneMS(4);
        noTone(TX_PIN);
        delayMs(6);
    }
}

bool AcousticCommunication::waitForFrameStart()
{
    // Continuously monitor for frame start signal (long tone)
    bool frameStart = false;

    while (!frameStart)
    {
        bool bitVal;
        bool start;
        if (!detectSymbol(bitVal, start))
        {
            // If we fail to detect a symbol, just continue
            continue;
        }

        if (start)
        {
            // After detecting start, we expect preamble bytes (0xAA)
            return true;
        }
    }
    return true;
}

bool AcousticCommunication::readFrame(uint8_t *frame, size_t &frameLen)
{
    // After we got frame start, we expect preamble bytes and so forth.
    // We know each byte = 8 bits.
    // Preamble: 2 bytes of 0xAA
    for (int i = 0; i < ACOUSTIC_PREAMBLE_LENGTH; i++)
    {
        uint8_t b = 0;
        if (!readByte(b))
            return false;
        if (b != 0xAA)
            return false;
        frame[i] = b;
    }

    // Read header (4 bytes)
    for (int i = 0; i < ACOUSTIC_HEADER_LENGTH; i++)
    {
        uint8_t b;
        if (!readByte(b))
            return false;
        frame[ACOUSTIC_PREAMBLE_LENGTH + i] = b;
    }

    uint8_t type = frame[ACOUSTIC_PREAMBLE_LENGTH];
    uint16_t length = ((uint16_t)frame[ACOUSTIC_PREAMBLE_LENGTH + 1] << 8) | (uint16_t)frame[ACOUSTIC_PREAMBLE_LENGTH + 2];
    // reserved = frame[ACOUSTIC_PREAMBLE_LENGTH+3]

    if (length > ACOUSTIC_FRAME_MAX_PAYLOAD)
        return false;

    size_t totalLen = ACOUSTIC_PREAMBLE_LENGTH + ACOUSTIC_HEADER_LENGTH + length + ACOUSTIC_CRC_LENGTH;
    if (totalLen > ACOUSTIC_MAX_FRAME_SIZE)
        return false;

    // Read payload
    for (size_t i = 0; i < length; i++)
    {
        uint8_t b;
        if (!readByte(b))
            return false;
        frame[ACOUSTIC_PREAMBLE_LENGTH + ACOUSTIC_HEADER_LENGTH + i] = b;
    }

    // Read CRC
    for (int i = 0; i < ACOUSTIC_CRC_LENGTH; i++)
    {
        uint8_t b;
        if (!readByte(b))
            return false;
        frame[ACOUSTIC_PREAMBLE_LENGTH + ACOUSTIC_HEADER_LENGTH + length + i] = b;
    }

    frameLen = totalLen;
    return true;
}

bool AcousticCommunication::readByte(uint8_t &outByte)
{
    outByte = 0;
    for (int b = 7; b >= 0; b--)
    {
        bool bitVal;
        if (!readBit(bitVal))
            return false;
        if (bitVal)
            outByte |= (1 << b);
    }
    return true;
}

bool AcousticCommunication::readBit(bool &outBit)
{
    // Detect a symbol and interpret as bit.
    // detectSymbol sets outBit and also can indicate frameStart if >600 counts again mid-frame.
    bool frameStart;
    if (!detectSymbol(outBit, frameStart))
        return false;

    // If frameStart occurs mid-frame, handle it accordingly
    if (frameStart)
    {
        // Possible new frame detected; consider frame broken and restart
        Serial.println("AcousticCommunication: Frame start detected during bit reception. Restarting frame reception.");
        return false;
    }

    return true;
}

bool AcousticCommunication::detectSymbol(bool &bit, bool &frameStart)
{
    frameStart = false;
    bit = false;

    // Reset bits count
    unsigned int bitsCount = 0;
    unsigned long start = millis();
    // For ~10ms, count how many times digitalRead(RX_PIN) is HIGH
    while (millis() - start < ACOUSTIC_SYMBOL_DURATION_MS)
    {
        if (digitalRead(RX_PIN))
            bitsCount++;
    }

    // Check thresholds
    if (bitsCount > ACOUSTIC_PREAMBLE_THRESHOLD)
    {
        // This indicates start of frame
        frameStart = true;
        // Not setting bit now because this is preamble signal
        return true;
    }

    // If we are capturing bits (after preamble):
    // bitsCount between 290 and 600 => bit=0
    // bitsCount between 20 and 290 => bit=1
    if (bitsCount > ACOUSTIC_BIT_ZERO_MIN && bitsCount < ACOUSTIC_BIT_ZERO_MAX)
    {
        bit = false;
        return true;
    }
    if (bitsCount > ACOUSTIC_BIT_ONE_MIN && bitsCount < ACOUSTIC_BIT_ONE_MAX)
    {
        bit = true;
        return true;
    }

    // If doesn't fall into known ranges and not frameStart, symbol unreadable
    // Return false to indicate no valid symbol
    return false;
}

void AcousticCommunication::generateToneMS(unsigned int ms)
{
    tone(TX_PIN, ACOUSTIC_TONE_FREQUENCY);
    delayMs(ms);
    noTone(TX_PIN);
}

void AcousticCommunication::delayMs(unsigned int ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
