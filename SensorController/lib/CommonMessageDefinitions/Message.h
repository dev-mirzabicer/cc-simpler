#ifndef COMMON_MESSAGE_DEFINITIONS_H
#define COMMON_MESSAGE_DEFINITIONS_H

#include <Arduino.h>

// Define maximum payload size
#define MAX_PAYLOAD_SIZE 256

// Enumeration for message types
enum class MessageType
{
    VELOCITY_COMMAND = 1, // Controls submarine movement
    SENSOR_DATA = 2,      // Sends sensor data to submarine
    STATUS_UPDATE = 3,    // Status updates from MotorController
    STATE_UPDATE = 4,     // Sends estimated state to remote computer
    OCCUPANCY_GRID_UPDATE = 5,
    DIAGNOSTIC_UPDATE = 6
};

// Struct for velocity commands
struct VelocityCommand
{
    float linearX;  // Forward/backward movement (m/s)
    float angularZ; // Yaw control (rad/s)
} __attribute__((packed));

// Struct for sensor data
struct SensorData
{
    float ax;                    // Acceleration in body X (m/s²)
    float ay;                    // Acceleration in body Y (m/s²)
    float az;                    // Acceleration in body Z (m/s²)
    float gyroZ;                 // Yaw rate (rad/s)
    float yaw;                   // Absolute yaw from magnetometer (rad)
    float depth;                 // Depth from pressure sensor (m)
    float forwardSonarDist;      // m
    float oscillatingSonarDist;  // m
    float oscillatingSonarAngle; // rad [-π/3, π/3]
} __attribute__((packed));

// Struct for status updates
struct Status
{
    bool isOperational;
    float currentLeftMotorSpeed;    // Current speed of left motor (m/s)
    float currentRightMotorSpeed;   // Current speed of right motor (m/s)
    float currentPumpIntakeStatus;  // Current pump intake status (abstract units)
    float currentPumpOutflowStatus; // Current pump outflow status (abstract units)
} __attribute__((packed));

// Struct for state updates (sent to remote computer)
struct StateUpdate
{
    float x;        // Position X (m)
    float y;        // Position Y (m)
    float z;        // Position Z (m)
    float vx;       // Velocity X (m/s)
    float vy;       // Velocity Y (m/s)
    float vz;       // Velocity Z (m/s)
    float yaw;      // Yaw (rad)
    float yaw_rate; // Yaw rate (rad/s)
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

// For OCCUPANCY_GRID_UPDATE, we pack changed cells as int16_t.
struct ChangedCellMessage
{
    uint8_t count; // number of changed cells
    // Followed by count*(x,y,z) as int16_t triplets
    // max 15 cells
} __attribute__((packed));

// For DIAGNOSTIC_UPDATE, 1 byte with bits for sensor health.
struct DiagnosticMessage
{
    // bit0=IMU, bit1=Mag, bit2=Pressure, bit3=ForwardSonar, bit4=OscSonar
    uint8_t healthBits;
} __attribute__((packed));

// Function to calculate CRC16 checksum
uint16_t calculateCRC16(const uint8_t *data, size_t length);

#endif // COMMON_MESSAGE_DEFINITIONS_H
