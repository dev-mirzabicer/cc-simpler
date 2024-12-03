#include "Message.h"

// Polynomial for CRC-CCITT
#define CRC16_POLY 0x1021

// Calculate CRC16 checksum
uint16_t calculateCRC16(const uint8_t *data, size_t length)
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

// Additional helper functions can be implemented here if needed
