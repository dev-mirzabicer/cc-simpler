#ifndef CONFIG_H
#define CONFIG_H

// I2C Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Communication Configuration
#define REMOTE_COMM_BAUD_RATE 115200
#define INTER_ESP_COMM_BAUD_RATE 400000 // Matches Wire.setClock in InterESPCommunication

// I2C Addresses
#define MOTOR_CONTROLLER_I2C_ADDRESS 0x10
#define SENSOR_CONTROLLER_ADDRESS_1 0x08
#define SENSOR_CONTROLLER_ADDRESS_2 0x09
// Add additional I2C addresses as needed

// Encryption Configuration for Remote Communication
#define AES_KEY_SIZE 16            // 16 bytes for AES-128
#define AES_KEY "1234567890abcdef" // Example key (replace with secure key)

// FreeRTOS Task Priorities
#define TASK_PRIORITY_SENSOR_DATA 2
#define TASK_PRIORITY_STATE_ESTIMATION 2
#define TASK_PRIORITY_COMMUNICATION 1
#define TASK_PRIORITY_PATH_PLANNING 2
#define TASK_PRIORITY_PATH_FOLLOWING 2
#define TASK_PRIORITY_MOTOR_CONTROL 3
#define TASK_PRIORITY_LOGGING 1

// FreeRTOS Task Stack Sizes (in words, typically 4 bytes each)
#define TASK_STACK_SIZE_SENSOR_DATA 4096
#define TASK_STACK_SIZE_STATE_ESTIMATION 4096
#define TASK_STACK_SIZE_COMMUNICATION 4096
#define TASK_STACK_SIZE_PATH_PLANNING 4096
#define TASK_STACK_SIZE_PATH_FOLLOWING 4096
#define TASK_STACK_SIZE_MOTOR_CONTROL 4096
#define TASK_STACK_SIZE_LOGGING 2048

// Other global configurations
// e.g., sensor calibration parameters

#endif // CONFIG_H
