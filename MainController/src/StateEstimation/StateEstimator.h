#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <Arduino.h>
#include "../Utils/Message.h"
#include <TinyEKF.h>
#include <mutex> // Include mutex for thread safety

// Define state, measurement, and control dimensions
const size_t STATE_SIZE = 15;      // [x, y, z, vx, vy, vz, roll, pitch, yaw, ax, ay, az, angularVx, angularVy, angularVz]
const size_t MEASUREMENT_SIZE = 7; // [z, roll, pitch, yaw, ax, ay, az]
const size_t CONTROL_SIZE = 6;     // [linearX, linearY, linearZ, angularX, angularY, angularZ]

// Structure for state estimation
struct State
{
    float x;         // X position (meters)
    float y;         // Y position (meters)
    float z;         // Z position (depth in meters)
    float vx;        // Velocity in X (m/s)
    float vy;        // Velocity in Y (m/s)
    float vz;        // Velocity in Z (m/s)
    float roll;      // Roll angle (degrees)
    float pitch;     // Pitch angle (degrees)
    float yaw;       // Yaw angle (degrees)
    float ax;        // Acceleration in X (m/s²)
    float ay;        // Acceleration in Y (m/s²)
    float az;        // Acceleration in Z (m/s²)
    float angularVx; // Roll rate (degrees/s)
    float angularVy; // Pitch rate (degrees/s)
    float angularVz; // Yaw rate (degrees/s)
    float velocity;  // Current speed (m/s)
} __attribute__((packed));

class StateEstimator
{
public:
    StateEstimator();
    void init();
    void estimateState(const SensorData &data, const VelocityCommand &controlCmd);
    State getCurrentState() const;
    // Additional methods as needed

private:
    State currentState;
    // EKF variables
    TinyEKF ekf;

    float stateVec[STATE_SIZE];
    float covarianceMatrix[STATE_SIZE * STATE_SIZE];
    float measurementVec[MEASUREMENT_SIZE];
    float processNoiseMatrix[STATE_SIZE * STATE_SIZE];
    float measurementNoiseMatrix[MEASUREMENT_SIZE * MEASUREMENT_SIZE];
    float kalmanGainMatrix[STATE_SIZE * MEASUREMENT_SIZE];
    float tmpVec[STATE_SIZE];

    // Mutex for protecting state variables
    mutable std::mutex stateMutex;

    // Process model: state transition function
    void processModel(float *x, const float *u, float dt);

    // Measurement model: measurement function
    void measurementModel(const float *x, float *z);

    void populateMeasurement(const SensorData &data);
    void populateControl(const VelocityCommand &controlCmd, float *control);
};

#endif // STATEESTIMATOR_H
