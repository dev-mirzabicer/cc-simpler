#ifndef MOTORCONTROLLER_CONFIG_H
#define MOTORCONTROLLER_CONFIG_H

// I2C Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_SLAVE_ADDRESS 0x10 // MotorController I2C Address (ensure it's unique)

// PWM Configuration for Motors and Pumps
#define LEFT_MOTOR_PWM_PIN 18  // PWM pin for Left Motor (Channel 0)
#define RIGHT_MOTOR_PWM_PIN 19 // PWM pin for Right Motor (Channel 1)
#define PUMP_INTAKE_PWM_PIN 5  // PWM pin for Pump Intake (Channel 2)
#define PUMP_OUTFLOW_PWM_PIN 4 // PWM pin for Pump Outflow (Channel 3)

// PID Parameters for Motors and Pumps
#define LEFT_MOTOR_KP 1.0f
#define LEFT_MOTOR_KI 0.0f
#define LEFT_MOTOR_KD 0.1f

#define RIGHT_MOTOR_KP 1.0f
#define RIGHT_MOTOR_KI 0.0f
#define RIGHT_MOTOR_KD 0.1f

#define PUMP_INTAKE_KP 1.0f
#define PUMP_INTAKE_KI 0.0f
#define PUMP_INTAKE_KD 0.1f

#define PUMP_OUTFLOW_KP 1.0f
#define PUMP_OUTFLOW_KI 0.0f
#define PUMP_OUTFLOW_KD 0.1f

// FreeRTOS Task Priorities
#define TASK_PRIORITY_COMMUNICATION 3
#define TASK_PRIORITY_MOTOR_CONTROL 2
#define TASK_PRIORITY_LOGGING 1

// FreeRTOS Task Stack Sizes (in words)
#define TASK_STACK_SIZE_COMMUNICATION 4096
#define TASK_STACK_SIZE_MOTOR_CONTROL 4096
#define TASK_STACK_SIZE_LOGGING 2048

// PWM Frequencies
#define PWM_FREQUENCY_MOTORS 50  // 50 Hz for ESCs (Brushless Motors)
#define PWM_FREQUENCY_PUMPS 5000 // 5 kHz for Pumps (DC Control)

// PWM Resolutions
#define PWM_RESOLUTION_MOTORS 16 // 16-bit resolution for ESC signals
#define PWM_RESOLUTION_PUMPS 8   // 8-bit resolution for Pumps

// Motor Calibration Parameters
#define LEFT_MOTOR_MAX_SPEED 1.0f  // Maximum speed in m/s
#define LEFT_MOTOR_MIN_SPEED -1.0f // Minimum speed in m/s

#define RIGHT_MOTOR_MAX_SPEED 1.0f  // Maximum speed in m/s
#define RIGHT_MOTOR_MIN_SPEED -1.0f // Minimum speed in m/s

// Pump Calibration Parameters
#define PUMP_INTAKE_MAX_CONTROL 1.0f // Maximum pump intake control (0.0 to 1.0)
#define PUMP_INTAKE_MIN_CONTROL 0.0f // Minimum pump intake control

#define PUMP_OUTFLOW_MAX_CONTROL 1.0f // Maximum pump outflow control (0.0 to 1.0)
#define PUMP_OUTFLOW_MIN_CONTROL 0.0f // Minimum pump outflow control

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
#define PUMP_INTAKE_SENSOR_PIN 36  // Example ADC pin for Pump Intake feedback
#define PUMP_OUTFLOW_SENSOR_PIN 39 // Example ADC pin for Pump Outflow feedback

// Vehicle Parameters
#define TRACK_WIDTH 0.1f // Distance between propellers in meters

// Additional Global Configurations
// e.g., safety limits, timeout durations

#endif // MOTORCONTROLLER_CONFIG_H
