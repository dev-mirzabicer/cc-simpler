#ifndef COMMON_MESSAGE_DEFINITIONS_H
#define COMMON_MESSAGE_DEFINITIONS_H

#include <Arduino.h>

// Define maximum payload size
#define MAX_PAYLOAD_SIZE 256

// Enumeration for message types
enum class MessageType
{
    VELOCITY_COMMAND = 1, // Matches MainController
    STATUS_UPDATE = 2,
    // Add additional message types as needed
};

// Struct for velocity commands
struct VelocityCommand
{
    float linearX;  // Forward/backward movement (m/s)
    float linearY;  // Lateral movement (m/s) - Optional
    float linearZ;  // Vertical movement (pump control) (m/s)
    float angularX; // Roll control (degrees/s)
    float angularY; // Pitch control (degrees/s)
    float angularZ; // Yaw control (radians/s)
} __attribute__((packed));

// Struct for motor commands (internal mapping)
struct MotorCommand
{
    float leftMotorSpeed;     // Desired speed for left motor (-1.0 to 1.0 m/s)
    float rightMotorSpeed;    // Desired speed for right motor (-1.0 to 1.0 m/s)
    float pumpIntakeControl;  // Desired pump intake control (0.0 to 1.0)
    float pumpOutflowControl; // Desired pump outflow control (0.0 to 1.0)
} __attribute__((packed));

// Struct for status updates
struct Status
{
    bool isOperational;
    float currentLeftMotorSpeed;    // Current speed of left motor (m/s)
    float currentRightMotorSpeed;   // Current speed of right motor (m/s)
    float currentPumpIntakeStatus;  // Current pump intake status (abstract units)
    float currentPumpOutflowStatus; // Current pump outflow status (abstract units)
    // Add additional status fields as needed
} __attribute__((packed));

// Struct for messages
struct Message
{
    uint8_t startByte;                 // Synchronization byte (e.g., 0xAA)
    MessageType type;                  // Type of message
    uint16_t length;                   // Length of payload
    uint8_t payload[MAX_PAYLOAD_SIZE]; // Payload data
    uint16_t checksum;                 // CRC16 checksum for error detection
} __attribute__((packed));

// Function to calculate CRC16 checksum
uint16_t calculateCRC16(const uint8_t *data, size_t length);

#endif // COMMON_MESSAGE_DEFINITIONS_H
