#ifndef MOTORCONTROLLER_CONFIG_H
#define MOTORCONTROLLER_CONFIG_H

// I2C Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_SLAVE_ADDRESS 0x10 // MotorController I2C Address (ensure it's unique)

// PWM Configuration for Motors and Pump
#define LEFT_MOTOR_PWM_PIN 18  // PWM pin for Left Motor
#define LEFT_MOTOR_DIR_PIN 17  // Direction pin for Left Motor
#define RIGHT_MOTOR_PWM_PIN 19 // PWM pin for Right Motor
#define RIGHT_MOTOR_DIR_PIN 16 // Direction pin for Right Motor
#define PUMP_PWM_PIN 5         // PWM pin for Pump

// PID Parameters for Motors and Pump
#define LEFT_MOTOR_KP 1.0f
#define LEFT_MOTOR_KI 0.0f
#define LEFT_MOTOR_KD 0.1f

#define RIGHT_MOTOR_KP 1.0f
#define RIGHT_MOTOR_KI 0.0f
#define RIGHT_MOTOR_KD 0.1f

#define PUMP_KP 1.0f
#define PUMP_KI 0.0f
#define PUMP_KD 0.1f

// FreeRTOS Task Priorities
#define TASK_PRIORITY_COMMUNICATION 3
#define TASK_PRIORITY_MOTOR_CONTROL 2
#define TASK_PRIORITY_LOGGING 1

// FreeRTOS Task Stack Sizes (in words)
#define TASK_STACK_SIZE_COMMUNICATION 4096
#define TASK_STACK_SIZE_MOTOR_CONTROL 4096
#define TASK_STACK_SIZE_LOGGING 2048

// PWM Frequency
#define PWM_FREQUENCY 5000 // 5 kHz for smooth motor control

// PWM Resolution
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)

// Motor Calibration Parameters
#define LEFT_MOTOR_MAX_SPEED 10.0f  // Maximum speed in m/s
#define LEFT_MOTOR_MIN_SPEED -10.0f // Minimum speed in m/s

#define RIGHT_MOTOR_MAX_SPEED 10.0f  // Maximum speed in m/s
#define RIGHT_MOTOR_MIN_SPEED -10.0f // Minimum speed in m/s

// Pump Calibration Parameters
#define PUMP_MAX_CONTROL 5.0f  // Maximum pump control in m/s
#define PUMP_MIN_CONTROL -5.0f // Minimum pump control in m/s

// Encoder Configuration
#define ENCODER_LEFT_PIN_A 34  // GPIO pin for Left Motor Encoder A
#define ENCODER_LEFT_PIN_B 35  // GPIO pin for Left Motor Encoder B
#define ENCODER_RIGHT_PIN_A 32 // GPIO pin for Right Motor Encoder A
#define ENCODER_RIGHT_PIN_B 33 // GPIO pin for Right Motor Encoder B

// Encoder Counts per Revolution
#define ENCODER_COUNTS_PER_REV 360

// Wheel Circumference (meters)
#define WHEEL_CIRCUMFERENCE 0.314f // Example: 10 cm diameter wheel

// Pump Sensor Configuration
#define PUMP_SENSOR_PIN 36 // Example ADC pin for Pump feedback

// Additional Global Configurations
// e.g., safety limits, timeout durations

#endif // MOTORCONTROLLER_CONFIG_H
