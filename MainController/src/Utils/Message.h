#ifndef MESSAGE_H
#define MESSAGE_H

#include <Arduino.h>
#include <vector>

// Define maximum payload size
#define MAX_PAYLOAD_SIZE 256

// Enumeration for message types
enum class MessageType
{
    SENSOR_DATA = 0,
    VELOCITY_COMMAND = 1, // Updated from MOTOR_COMMAND to VELOCITY_COMMAND
    PATH_UPDATE = 2,
    STATUS_UPDATE = 3,
    OCCUPANCY_GRID = 4,
    // Add additional message types as needed
};

// Struct for waypoints
struct Waypoint
{
    float x;
    float y;
    float z;
    float yaw; // Orientation at the waypoint

    // Ensure no padding
} __attribute__((packed));

// Struct for path
struct Path
{
    std::vector<Waypoint> waypoints;
} __attribute__((packed));

// Struct for sensor data
struct SensorData
{
    uint32_t timestamp;       // Timestamp in milliseconds
    float imuAcceleration[3]; // X, Y, Z acceleration
    float imuGyro[3];         // X, Y, Z gyroscope
    float magneticField[3];   // X, Y, Z magnetometer
    float pressure;           // Pressure reading
    float depth;              // Depth calculated from pressure
    float sonarDistance[2];   // Distances from two sonar sensors
    // Add additional sensor data fields as needed

} __attribute__((packed));

// Struct for velocity commands (formerly MotorCommand)
struct VelocityCommand
{
    float linearX;  // Forward/backward movement (m/s)
    float linearY;  // Lateral movement (m/s) - Optional
    float linearZ;  // Vertical movement (pump control) (m/s)
    float angularX; // Roll control (degrees/s)
    float angularY; // Pitch control (degrees/s)
    float angularZ; // Yaw control (degrees/s)
} __attribute__((packed));

// Struct for status updates
struct Status
{
    bool isOperational;
    float currentLeftMotorSpeed;  // Current speed of left motor (m/s)
    float currentRightMotorSpeed; // Current speed of right motor (m/s)
    float currentPumpStatus;      // Current pump status (m/s)
    // Add additional status fields as needed

} __attribute__((packed));

// Struct for occupancy grid
struct OccupancyGrid
{
    int sizeX;
    int sizeY;
    int sizeZ;
    float resolution;              // Meters per grid cell
    std::vector<uint8_t> gridData; // 0 = free, 1 = occupied
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

#endif // MESSAGE_H
