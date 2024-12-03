#include "StateEstimator.h"
#include <cstring> // For memset

// Define time step (dt)
#define TIME_STEP 0.1f // 10 Hz

// Constructor
StateEstimator::StateEstimator()
    : ekf(STATE_SIZE, MEASUREMENT_SIZE, CONTROL_SIZE, stateVec, covarianceMatrix, processNoiseMatrix, measurementNoiseMatrix, kalmanGainMatrix, tmpVec)
{
    // Initialize state and covariance matrices
    memset(stateVec, 0, sizeof(stateVec));
    memset(covarianceMatrix, 0, sizeof(covarianceMatrix));
    for (size_t i = 0; i < STATE_SIZE; ++i)
    {
        covarianceMatrix[i * STATE_SIZE + i] = 1.0f; // Initialize covariance as identity matrix
    }

    // Initialize process and measurement noise matrices
    memset(processNoiseMatrix, 0, sizeof(processNoiseMatrix));
    memset(measurementNoiseMatrix, 0, sizeof(measurementNoiseMatrix));

    // Example noise values (to be tuned)
    for (size_t i = 0; i < STATE_SIZE * STATE_SIZE; ++i)
    {
        processNoiseMatrix[i] = 0.01f; // Low process noise
    }
    for (size_t i = 0; i < MEASUREMENT_SIZE * MEASUREMENT_SIZE; ++i)
    {
        measurementNoiseMatrix[i] = 0.1f; // Measurement noise
    }
}

// Initialize EKF
void StateEstimator::init()
{
    ekf.init();

    // Set initial state if known
    stateVec[0] = 0.0f;  // x
    stateVec[1] = 0.0f;  // y
    stateVec[2] = 0.0f;  // z
    stateVec[3] = 0.0f;  // vx
    stateVec[4] = 0.0f;  // vy
    stateVec[5] = 0.0f;  // vz
    stateVec[6] = 0.0f;  // roll
    stateVec[7] = 0.0f;  // pitch
    stateVec[8] = 0.0f;  // yaw
    stateVec[9] = 0.0f;  // ax
    stateVec[10] = 0.0f; // ay
    stateVec[11] = 0.0f; // az
    stateVec[12] = 0.0f; // angularVx
    stateVec[13] = 0.0f; // angularVy
    stateVec[14] = 0.0f; // angularVz
}

// Populate measurement vector from SensorData
void StateEstimator::populateMeasurement(const SensorData &data)
{
    // Measurement vector consists of:
    // [z, roll, pitch, yaw, ax, ay, az]

    measurementVec[0] = data.depth;              // z position from pressure
    measurementVec[1] = data.imuGyro[0];         // roll from gyro
    measurementVec[2] = data.imuGyro[1];         // pitch from gyro
    measurementVec[3] = data.imuGyro[2];         // yaw from gyro
    measurementVec[4] = data.imuAcceleration[0]; // ax
    measurementVec[5] = data.imuAcceleration[1]; // ay
    measurementVec[6] = data.imuAcceleration[2]; // az
}

// Populate control vector from VelocityCommand
void StateEstimator::populateControl(const VelocityCommand &controlCmd, float *control)
{
    // Control vector consists of:
    // [linearX, linearY, linearZ, angularX, angularY, angularZ]
    control[0] = controlCmd.linearX;
    control[1] = controlCmd.linearY;
    control[2] = controlCmd.linearZ;
    control[3] = controlCmd.angularX;
    control[4] = controlCmd.angularY;
    control[5] = controlCmd.angularZ;
}

// Process model: state transition function
void StateEstimator::processModel(float *x, const float *u, float dt)
{
    // State vector:
    // [x, y, z, vx, vy, vz, roll, pitch, yaw, ax, ay, az, angularVx, angularVy, angularVz]

    // Update position based on velocity and acceleration
    x[0] += x[3] * dt + 0.5f * x[9] * dt * dt;  // x position
    x[1] += x[4] * dt + 0.5f * x[10] * dt * dt; // y position
    x[2] += x[5] * dt + 0.5f * x[11] * dt * dt; // z position (depth)

    // Update velocities based on acceleration
    x[3] += x[9] * dt;  // vx
    x[4] += x[10] * dt; // vy
    x[5] += x[11] * dt; // vz

    // Update orientation based on angular velocities
    x[6] += x[12] * dt; // roll (degrees)
    x[7] += x[13] * dt; // pitch (degrees)
    x[8] += x[14] * dt; // yaw (degrees)

    // Normalize yaw to [0, 360)
    if (x[8] >= 360.0f)
        x[8] -= 360.0f;
    if (x[8] < 0.0f)
        x[8] += 360.0f;

    // Update angular velocities based on angular acceleration (if modeled)
    // Assuming angular accelerations are controlled via commands
    // If not, set angular velocities based on control inputs
    x[12] = u[3]; // angularVx
    x[13] = u[4]; // angularVy
    x[14] = u[5]; // angularVz
}

// Measurement model: measurement function
void StateEstimator::measurementModel(const float *x, float *z)
{
    // Measurement vector:
    // [z, roll, pitch, yaw, ax, ay, az]

    z[0] = x[2];  // z position
    z[1] = x[6];  // roll
    z[2] = x[7];  // pitch
    z[3] = x[8];  // yaw
    z[4] = x[9];  // ax
    z[5] = x[10]; // ay
    z[6] = x[11]; // az
}

// Estimate state using EKF
void StateEstimator::estimateState(const SensorData &data, const VelocityCommand &controlCmd)
{
    // Populate measurement vector
    populateMeasurement(data);

    // Populate control vector
    float control[CONTROL_SIZE];
    populateControl(controlCmd, control);

    // Predict step with control inputs
    ekf.predict(control, TIME_STEP, processModel);

    // Update step with measurements
    ekf.update(measurementVec, measurementModel);

    // Update currentState from state vector
    {
        std::lock_guard<std::mutex> lock(stateMutex); // Protect state variables
        currentState.x = stateVec[0];
        currentState.y = stateVec[1];
        currentState.z = stateVec[2];
        currentState.vx = stateVec[3];
        currentState.vy = stateVec[4];
        currentState.vz = stateVec[5];
        currentState.roll = stateVec[6];
        currentState.pitch = stateVec[7];
        currentState.yaw = stateVec[8];
        currentState.ax = stateVec[9];
        currentState.ay = stateVec[10];
        currentState.az = stateVec[11];
        currentState.angularVx = stateVec[12];
        currentState.angularVy = stateVec[13];
        currentState.angularVz = stateVec[14];
        currentState.velocity = sqrt(stateVec[3] * stateVec[3] + stateVec[4] * stateVec[4] + stateVec[5] * stateVec[5]);
    }
}

// Get current state
State StateEstimator::getCurrentState() const
{
    std::lock_guard<std::mutex> lock(stateMutex); // Ensure thread safety
    return currentState;
}
