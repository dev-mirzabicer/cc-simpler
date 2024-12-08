#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <Arduino.h>

/**
 * @brief StateEstimator implements an Extended Kalman Filter for submarine state estimation.
 *
 * State vector (X): [x, y, z, vx, vy, vz, yaw, yaw_rate]
 * Control input (u): [ax_body, ay_body, az_body, gyro_z]
 * Measurement (Z): [z_measured (depth), yaw_measured (magnetometer yaw)]
 */

class StateEstimator
{
public:
    StateEstimator();
    void init();

    /**
     * @brief Perform the prediction step of the EKF given IMU inputs and a time delta.
     *
     * @param ax_body Acceleration in body X [m/s²]
     * @param ay_body Acceleration in body Y [m/s²]
     * @param az_body Acceleration in body Z [m/s²]
     * @param gyro_z Angular velocity about Z [rad/s]
     * @param dt Time step [s]
     */
    void predict(float ax_body, float ay_body, float az_body, float gyro_z, float dt);

    /**
     * @brief Perform the update step of the EKF given depth and yaw measurements.
     *
     * @param z_measured Depth measurement [m]
     * @param yaw_measured Yaw measurement [rad]
     */
    void update(float z_measured, float yaw_measured);

    /**
     * @brief Get the current estimated state.
     *
     * @param state_out Pointer to array of size 8 to receive [x, y, z, vx, vy, vz, yaw, yaw_rate]
     */
    void getState(float *state_out) const;

    /**
     * @brief Get the current state covariance matrix.
     *
     * @param P_out Pointer to array of size 64 to receive the 8x8 covariance matrix (row-major)
     */
    void getCovariance(float *P_out) const;

private:
    // State dimension
    static const int N = 8;
    // Measurement dimension
    static const int M = 2;

    float X[N];     // State vector
    float P[N * N]; // State covariance

    float Q[N * N]; // Process noise covariance
    float R[M * M]; // Measurement noise covariance

    // Temporary arrays for operations
    float Fx[N * N]; // Jacobian wrt X
    float H[M * N];  // Measurement Jacobian
    float K[N * M];  // Kalman gain
    float I[N * N];  // Identity

    // Standard deviations for process noise
    // Adjust these parameters as needed for tuning:
    float sigma_ax;
    float sigma_ay;
    float sigma_az;
    float sigma_gyro_z;

    // Standard deviations for measurement noise
    float sigma_z;
    float sigma_yaw;

    // Matrix utility functions
    void setIdentity(float *A, int n);
    void setZero(float *A, int rows, int cols);
    void copyMatrix(const float *src, float *dst, int rows, int cols);
    void addMatrix(const float *A, const float *B, float *C, int rows, int cols);
    void subMatrix(const float *A, const float *B, float *C, int rows, int cols);
    void mulMatrix(const float *A, const float *B, float *C, int rA, int cA, int cB);
    void mulTransB(const float *A, const float *B, float *C, int rA, int cA, int cB);
    void mulTransA(const float *A, const float *B, float *C, int rA, int cA, int cB);
    bool invertMatrix(float *A, int n); // In-place matrix inversion

    float clampAngle(float angle);
};

#endif // STATEESTIMATOR_H
