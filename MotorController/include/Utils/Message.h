#ifndef MESSAGE_H
#define MESSAGE_H

#include <Arduino.h>
#include <vector>

#define MAX_PAYLOAD_SIZE 256

enum class MessageType
{
    VELOCITY_COMMAND = 1,
    STATUS_UPDATE = 2,

};

struct VelocityCommand
{
    float linearX;
    float linearY;
    float linearZ;
    float angularX;
    float angularY;
    float angularZ;
} __attribute__((packed));

struct MotorCommand
{
    float leftMotorSpeed;
    float rightMotorSpeed;
    float pumpControl;
} __attribute__((packed));

struct Status
{
    bool isOperational;
    float currentLeftMotorSpeed;
    float currentRightMotorSpeed;
    float currentPumpStatus;

} __attribute__((packed));

struct Message
{
    uint8_t startByte;
    MessageType type;
    uint16_t length;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint16_t checksum;
} __attribute__((packed));

uint16_t calculateCRC16(const uint8_t *data, size_t length);

#endif
