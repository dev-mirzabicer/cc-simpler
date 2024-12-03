// Communication/AcousticComm.cpp
#include "AcousticComm.h"

// Define the polynomial for CRC-CCITT
#define CRC16_POLY 0x1021

// Constructor
AcousticComm::AcousticComm(uint8_t txPin, uint8_t rxPin)
    : TX_PIN(txPin), RX_PIN(rxPin), receiveState(ReceiveState::WAIT_START),
      payloadBytesReceived(0), currentBit(0), receivingBit(false),
      pulseStartTime(0), pulseDuration(0) {}

// Initialize the Acoustic Communication
void AcousticComm::init()
{
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT_PULLUP);
    noTone(TX_PIN); // Ensure transmitter is off
    Serial.println("AcousticComm initialized.");
}

// Calculate CRC16 checksum
uint16_t AcousticComm::calculateCRC16(const uint8_t *data, size_t length)
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
bool AcousticComm::verifyCRC16(uint8_t *data, size_t length, uint16_t checksum)
{
    uint16_t computed = calculateCRC16(data, length);
    return (computed == checksum);
}

// Reset reception state
void AcousticComm::resetReception()
{
    receiveState = ReceiveState::WAIT_START;
    payloadBytesReceived = 0;
    currentBit = 0;
    receivingBit = false;
    pulseStartTime = 0;
    pulseDuration = 0;
    memset(&currentMsg, 0, sizeof(AcousticMessage));
}

// Send Acoustic Message by encoding bits into sound pulses
bool AcousticComm::sendMessage(const AcousticMessage &msg)
{
    std::lock_guard<std::mutex> lock(acMutex);

    // Construct the bitstream: startByte, type, length, payload, checksum
    std::vector<uint8_t> bitstream;
    bitstream.push_back(msg.startByte);
    bitstream.push_back(static_cast<uint8_t>(msg.type));
    bitstream.push_back((msg.length >> 8) & 0xFF); // High byte
    bitstream.push_back(msg.length & 0xFF);        // Low byte
    for (uint16_t i = 0; i < msg.length; ++i)
    {
        bitstream.push_back(msg.payload[i]);
    }
    bitstream.push_back((msg.checksum >> 8) & 0xFF); // Checksum high byte
    bitstream.push_back(msg.checksum & 0xFF);        // Checksum low byte

    // Transmit each byte as bits with specific pulse durations
    for (size_t i = 0; i < bitstream.size(); ++i)
    {
        uint8_t byte = bitstream[i];
        for (int bit = 7; bit >= 0; --bit)
        { // MSB first
            bool bitValue = (byte >> bit) & 0x01;
            if (bitValue)
            {
                // '1' -> 2ms pulse
                tone(TX_PIN, 40000);     // 40kHz frequency
                delayMicroseconds(2000); // 2ms
            }
            else
            {
                // '0' -> 4ms pulse
                tone(TX_PIN, 40000);     // 40kHz frequency
                delayMicroseconds(4000); // 4ms
            }
            noTone(TX_PIN);
            delayMicroseconds(11000); // 11ms silence between bits (total bit duration ~13ms)
        }
    }

    Serial.println("Acoustic message sent.");
    return true;
}

// Receive Acoustic Messages by decoding sound pulses into bits
bool AcousticComm::receiveMessage(AcousticMessage &msg)
{
    std::lock_guard<std::mutex> lock(acMutex);
    if (!incomingQueue.empty())
    {
        msg = incomingQueue.front();
        incomingQueue.pop();
        return true;
    }
    return false;
}

// Process incoming acoustic data (to be called frequently in the main loop)
void AcousticComm::processIncomingData()
{
    static unsigned long lastPulseTime = 0;
    static bool lastPinState = HIGH;
    bool currentPinState = digitalRead(RX_PIN);

    // Detect falling edge (start of pulse)
    if (lastPinState == HIGH && currentPinState == LOW)
    {
        pulseStartTime = micros();
        receivingBit = true;
    }

    // Detect rising edge (end of pulse)
    if (lastPinState == LOW && currentPinState == HIGH && receivingBit)
    {
        pulseDuration = micros() - pulseStartTime;
        receivingBit = false;

        // Determine bit value based on pulse duration
        bool bitValue;
        if (pulseDuration >= 1500 && pulseDuration < 3000)
        { // 2ms pulse
            bitValue = 1;
        }
        else if (pulseDuration >= 3500 && pulseDuration < 5000)
        { // 4ms pulse
            bitValue = 0;
        }
        else
        {
            // Invalid pulse duration
            Serial.println("Invalid pulse duration detected.");
            resetReception();
            lastPinState = currentPinState;
            return;
        }

        switch (receiveState)
        {
        case ReceiveState::WAIT_START:
            if (bitValue)
            { // Start byte is 0xAA -> 10101010
                // Start byte starts with '1'
                // Start detecting after first '1'
                currentMsg.startByte = (currentMsg.startByte << 1) | bitValue;
                if (currentMsg.startByte == AC_START_BYTE)
                {
                    receiveState = ReceiveState::RECEIVE_HEADER;
                    payloadBytesReceived = 0;
                    currentBit = 0;
                    Serial.println("Start byte detected.");
                }
            }
            break;

        case ReceiveState::RECEIVE_HEADER:
            // Header consists of type (1 byte) and length (2 bytes)
            currentMsg.type = static_cast<AcousticMessageType>(currentMsg.type << 1 | bitValue);
            currentBit++;
            if (currentBit == 8)
            { // Type byte received
                currentBit = 0;
                receiveState = ReceiveState::RECEIVE_HEADER;
                // Next two bytes are length
            }
            break;

        case ReceiveState::RECEIVE_PAYLOAD:
            // Receiving payload bits
            currentMsg.payload[payloadBytesReceived / 8] = (currentMsg.payload[payloadBytesReceived / 8] << 1) | bitValue;
            payloadBytesReceived++;
            if (payloadBytesReceived >= currentMsg.length * 8)
            {
                receiveState = ReceiveState::RECEIVE_CHECKSUM;
            }
            break;

        case ReceiveState::RECEIVE_CHECKSUM:
            // Receiving checksum bits
            currentMsg.checksum = (currentMsg.checksum << 1) | bitValue;
            currentBit++;
            if (currentBit == 16)
            { // Checksum received
                // Verify checksum
                uint16_t computedCRC = calculateCRC16(currentMsg.payload, currentMsg.length);
                if (computedCRC == currentMsg.checksum)
                {
                    // Enqueue the received message
                    incomingQueue.push(currentMsg);
                    Serial.println("Acoustic message received and verified.");
                }
                else
                {
                    Serial.println("Acoustic message checksum mismatch.");
                }
                resetReception();
            }
            break;

        default:
            resetReception();
            break;
        }
    }

    lastPinState = currentPinState;
}
