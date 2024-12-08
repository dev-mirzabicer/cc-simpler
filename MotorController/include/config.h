// config.h
#ifndef CONFIG_H
#define CONFIG_H

// PWM frequencies and resolutions
#define PWM_FREQUENCY_MOTORS 50  // 50 Hz for ESCs
#define PWM_RESOLUTION_MOTORS 16 // 16-bit PWM for motors
#define PWM_FREQUENCY_PUMPS 5000 // 5 kHz for pumps
#define PWM_RESOLUTION_PUMPS 8   // 8-bit PWM for pumps

// I2C addresses
#define I2C_SLAVE_ADDRESS 0x04

// Encoder counts per revolution
#define ENCODER_COUNTS_PER_REV 1000

// Wheel circumference in meters
#define WHEEL_CIRCUMFERENCE 0.314f // Example: 10cm diameter wheel, pi*d = ~0.314m

// Track width in meters (distance between left and right motors)
#define TRACK_WIDTH 0.01f

// Motor speed limits
#define LEFT_MOTOR_MAX_SPEED 1.0f   // m/s
#define LEFT_MOTOR_MIN_SPEED -1.0f  // m/s
#define RIGHT_MOTOR_MAX_SPEED 1.0f  // m/s
#define RIGHT_MOTOR_MIN_SPEED -1.0f // m/s

// Pump control limits
#define PUMP_INTAKE_MAX_CONTROL 1.0f
#define PUMP_OUTFLOW_MAX_CONTROL 1.0f

// PWM and Direction pins for motors
#define LEFT_MOTOR_PWM_PIN 18
#define LEFT_MOTOR_DIR_PIN 19
#define RIGHT_MOTOR_PWM_PIN 21
#define RIGHT_MOTOR_DIR_PIN 22

// PWM and Direction pins for pumps
#define PUMP_INTAKE_PWM_PIN 23
#define PUMP_INTAKE_DIR_PIN 25
#define PUMP_INTAKE_SENSOR_PIN 34

#define PUMP_OUTFLOW_PWM_PIN 26
#define PUMP_OUTFLOW_DIR_PIN 27
#define PUMP_OUTFLOW_SENSOR_PIN 35

// Task stack sizes and priorities
#define TASK_STACK_SIZE_COMMUNICATION 4096
#define TASK_STACK_SIZE_MOTOR_CONTROL 4096
#define TASK_STACK_SIZE_LOGGING 2048
#define TASK_STACK_SIZE_DIAGNOSTICS 2048
#define TASK_STACK_SIZE_SENSOR_READ 4096

#define TASK_PRIORITY_COMMUNICATION 2
#define TASK_PRIORITY_MOTOR_CONTROL 2
#define TASK_PRIORITY_LOGGING 1
#define TASK_PRIORITY_DIAGNOSTICS 1
#define TASK_PRIORITY_SENSOR_READ 3

#endif // CONFIG_H
