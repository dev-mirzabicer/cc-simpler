#include "StateEstimator.h"
#include <math.h>

// Constructor
StateEstimator::StateEstimator()
{
    // Set standard deviations for process noise
    sigma_ax = 0.1f;      // m/s²
    sigma_ay = 0.1f;      // m/s²
    sigma_az = 0.1f;      // m/s²
    sigma_gyro_z = 0.01f; // rad/s

    // Measurement noise
    sigma_z = 0.1f;    // m
    sigma_yaw = 0.05f; // rad
}

void StateEstimator::init()
{
    // Initialize state to zero
    for (int i = 0; i < N; i++)
        X[i] = 0.0f;

    // Initialize covariance P large
    setZero(P, N, N);
    for (int i = 0; i < N; i++)
        P[i * N + i] = 1e3f; // large uncertainty initially

    // Process noise Q
    setZero(Q, N, N);
    // We model process noise mainly in acceleration and yaw_rate
    // For simplicity, assume process noise adds to velocities and yaw_rate:
    // vx, vy, vz, yaw_rate uncertainty
    float var_ax = sigma_ax * sigma_ax;
    float var_ay = sigma_ay * sigma_ay;
    float var_az = sigma_az * sigma_az;
    float var_gyro = sigma_gyro_z * sigma_gyro_z;

    // Positions affected by double integration: add some small process noise
    Q[0 * N + 0] = 0.01f;
    Q[1 * N + 1] = 0.01f;
    Q[2 * N + 2] = 0.01f;
    // Velocities
    Q[3 * N + 3] = var_ax;
    Q[4 * N + 4] = var_ay;
    Q[5 * N + 5] = var_az;
    // Yaw
    Q[6 * N + 6] = 0.001f;
    // Yaw_rate
    Q[7 * N + 7] = var_gyro;

    // Measurement noise R
    setZero(R, M, M);
    R[0 * M + 0] = sigma_z * sigma_z;
    R[1 * M + 1] = sigma_yaw * sigma_yaw;

    setIdentity(I, N);
}

// Clamp angle to [-pi, pi]
float StateEstimator::clampAngle(float angle)
{
    while (angle > M_PI)
        angle -= 2.0f * M_PI;
    while (angle < -M_PI)
        angle += 2.0f * M_PI;
    return angle;
}

void StateEstimator::predict(float ax_body, float ay_body, float az_body, float gyro_z, float dt)
{
    // Extract state variables
    float x = X[0];
    float y = X[1];
    float z = X[2];
    float vx = X[3];
    float vy = X[4];
    float vz = X[5];
    float yaw = X[6];
    float yaw_rate = X[7];

    // Transform body accelerations to world frame
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    float a_xw = ax_body * cy - ay_body * sy;
    float a_yw = ax_body * sy + ay_body * cy;
    float a_zw = az_body; // no rotation needed for z if we assume no pitch/roll

    // State prediction (nonlinear)
    float x_new = x + vx * dt + 0.5f * a_xw * dt * dt;
    float y_new = y + vy * dt + 0.5f * a_yw * dt * dt;
    float z_new = z + vz * dt + 0.5f * a_zw * dt * dt;
    float vx_new = vx + a_xw * dt;
    float vy_new = vy + a_yw * dt;
    float vz_new = vz + a_zw * dt;
    float yaw_new = yaw + yaw_rate * dt;
    yaw_new = clampAngle(yaw_new);
    float yaw_rate_new = yaw_rate; // yaw_rate evolves with small noise

    X[0] = x_new;
    X[1] = y_new;
    X[2] = z_new;
    X[3] = vx_new;
    X[4] = vy_new;
    X[5] = vz_new;
    X[6] = yaw_new;
    X[7] = yaw_rate_new;

    // Compute Jacobian Fx = dF/dX
    // Partial derivatives of the state w.r.t X
    setIdentity(Fx, N);
    // x depends on vx and a_xw: a_xw depends on yaw, so partial w.r.t yaw:
    // a_xw = ax_body*cy - ay_body*sy
    // da_xw/dyaw = -ax_body*sy - ay_body*cy
    float daxw_dyaw = -ax_body * sy - ay_body * cy;
    float dayw_dyaw = ax_body * cy - ay_body * sy;

    // x_new = x + vx*dt + 0.5*a_xw*dt²
    Fx[0 * N + 3] = dt; // dx/dvx
    Fx[0 * N + 6] = 0.5f * dt * dt * daxw_dyaw;
    // y_new = y + vy*dt + 0.5*a_yw*dt²
    Fx[1 * N + 4] = dt;
    Fx[1 * N + 6] = 0.5f * dt * dt * dayw_dyaw;
    // z_new = z + vz*dt + 0.5*a_zw*dt² (a_zw = az_body no yaw dep)
    Fx[2 * N + 5] = dt;
    // vx_new = vx + a_xw*dt
    Fx[3 * N + 6] = dt * daxw_dyaw;
    // vy_new = vy + a_yw*dt
    Fx[4 * N + 6] = dt * dayw_dyaw;
    // vz_new = vz + a_zw*dt (no yaw dep)
    // yaw_new = yaw + yaw_rate*dt
    Fx[6 * N + 7] = dt;
    // yaw_rate_new = yaw_rate (no state dep changes)

    // Compute P_new = Fx P Fx^T + Q
    float FxP[N * N];
    setZero(FxP, N, N);
    mulMatrix(Fx, P, FxP, N, N, N);
    float P_new[N * N];
    setZero(P_new, N, N);
    mulTransB(FxP, Fx, P_new, N, N, N);

    // P_new += Q
    addMatrix(P_new, Q, P, N, N);
    // P now updated
    copyMatrix(P, P, N, N); // Redundant but okay
}

void StateEstimator::update(float z_measured, float yaw_measured)
{
    // Measurement: Z = [z, yaw]
    // h(X) = [X[2], X[6]] = [z, yaw]

    // Compute residual y = Z - h(X)
    float z_pred = X[2];
    float yaw_pred = X[6];
    // Normalize yaw difference
    float dyaw = yaw_measured - yaw_pred;
    dyaw = clampAngle(dyaw);

    float y_vec[M];
    y_vec[0] = z_measured - z_pred;
    y_vec[1] = dyaw;

    // Compute H = dh/dX
    setZero(H, M, N);
    // h(X) w.r.t X: dh/dz = 1 for z
    H[0 * N + 2] = 1.0f; // dz/dz
    // dh/dyaw = 1 for yaw
    H[1 * N + 6] = 1.0f; // dyaw/dyaw

    // S = H P H^T + R
    float HP[M * N];
    setZero(HP, M, N);
    mulMatrix(H, P, HP, M, N, N);
    float S[M * M];
    setZero(S, M, M);
    mulTransB(HP, H, S, M, N, M);

    // S += R
    addMatrix(S, R, S, M, M);

    // Compute K = P H^T S^-1
    float PHt[N * M];
    setZero(PHt, N, M);
    // P H^T
    mulTransA(P, H, PHt, N, N, M);

    float S_inv[M * M];
    copyMatrix(S, S_inv, M, M);
    if (!invertMatrix(S_inv, M))
    {
        // If inversion fails, no update
        return;
    }

    setZero(K, N, M);
    mulMatrix(PHt, S_inv, K, N, M, M);

    // Update state: X_new = X + K y
    float KY[N];
    setZero(KY, N, 1);
    mulMatrix(K, y_vec, KY, N, M, 1);

    for (int i = 0; i < N; i++)
        X[i] += KY[i];

    // Normalize yaw after update
    X[6] = clampAngle(X[6]);

    // Update covariance: P_new = (I - K H) P
    float KH[N * N];
    setZero(KH, N, N);
    mulMatrix(K, H, KH, N, M, N);

    float IminusKH[N * N];
    setZero(IminusKH, N, N);
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            IminusKH[i * N + j] = I[i * N + j] - KH[i * N + j];

    float P_new[N * N];
    setZero(P_new, N, N);
    mulMatrix(IminusKH, P, P_new, N, N, N);

    // Symmetrize P_new to reduce numeric issues
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            P_new[i * N + j] = 0.5f * (P_new[i * N + j] + P_new[j * N + i]);

    copyMatrix(P_new, P, N, N);
}

void StateEstimator::getState(float *state_out) const
{
    for (int i = 0; i < N; i++)
        state_out[i] = X[i];
}

void StateEstimator::getCovariance(float *P_out) const
{
    for (int i = 0; i < N * N; i++)
        P_out[i] = P[i];
}

void StateEstimator::setIdentity(float *A, int n)
{
    setZero(A, n, n);
    for (int i = 0; i < n; i++)
        A[i * n + i] = 1.0f;
}

void StateEstimator::setZero(float *A, int rows, int cols)
{
    for (int i = 0; i < rows * cols; i++)
        A[i] = 0.0f;
}

void StateEstimator::copyMatrix(const float *src, float *dst, int rows, int cols)
{
    for (int i = 0; i < rows * cols; i++)
        dst[i] = src[i];
}

void StateEstimator::addMatrix(const float *A, const float *B, float *C, int rows, int cols)
{
    for (int i = 0; i < rows * cols; i++)
        C[i] = A[i] + B[i];
}

void StateEstimator::subMatrix(const float *A, const float *B, float *C, int rows, int cols)
{
    for (int i = 0; i < rows * cols; i++)
        C[i] = A[i] - B[i];
}

void StateEstimator::mulMatrix(const float *A, const float *B, float *C, int rA, int cA, int cB)
{
    setZero(C, rA, cB);
    for (int i = 0; i < rA; i++)
    {
        for (int j = 0; j < cB; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < cA; k++)
                sum += A[i * cA + k] * B[k * cB + j];
            C[i * cB + j] = sum;
        }
    }
}

void StateEstimator::mulTransB(const float *A, const float *B, float *C, int rA, int cA, int cB)
{
    // C = A B^T, where B is (rB=cB, cB?), here B is MxN?
    // Actually used for: mulTransB(HP, H, S), H is MxN, HP is MxN, we do HP * H^T -> MxN * NxM = MxM
    // So cB = M in that usage.
    setZero(C, rA, cB);
    for (int i = 0; i < rA; i++)
    {
        for (int j = 0; j < cB; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < cA; k++)
                sum += A[i * cA + k] * B[j * cA + k]; // B^T means swap indices
            C[i * cB + j] = sum;
        }
    }
}

void StateEstimator::mulTransA(const float *A, const float *B, float *C, int rA, int cA, int cB)
{
    // C = A^T B
    // A is NxN, H is MxN -> P H^T or something similar
    setZero(C, cA, cB);
    for (int i = 0; i < cA; i++)
    {
        for (int j = 0; j < cB; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < rA; k++)
                sum += A[k * cA + i] * B[k * cB + j];
            C[i * cB + j] = sum;
        }
    }
}

bool StateEstimator::invertMatrix(float *A, int n)
{
    // Gaussian elimination
    // Create augmented matrix
    float *aug = (float *)malloc(sizeof(float) * n * n * 2);
    if (!aug)
        return false;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            aug[i * (2 * n) + j] = A[i * n + j];
        }
        for (int j = 0; j < n; j++)
        {
            aug[i * (2 * n) + (n + j)] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Forward elimination
    for (int i = 0; i < n; i++)
    {
        // Find pivot
        float maxEl = fabsf(aug[i * (2 * n) + i]);
        int pivot = i;
        for (int k = i + 1; k < n; k++)
        {
            float val = fabsf(aug[k * (2 * n) + i]);
            if (val > maxEl)
            {
                maxEl = val;
                pivot = k;
            }
        }
        if (pivot != i)
        {
            // swap rows
            for (int col = 0; col < 2 * n; col++)
            {
                float temp = aug[i * (2 * n) + col];
                aug[i * (2 * n) + col] = aug[pivot * (2 * n) + col];
                aug[pivot * (2 * n) + col] = temp;
            }
        }

        float diag = aug[i * (2 * n) + i];
        if (fabsf(diag) < 1e-12f)
        {
            free(aug);
            return false;
        }

        // Normalize pivot row
        for (int col = i; col < 2 * n; col++)
            aug[i * (2 * n) + col] /= diag;

        // Eliminate below
        for (int row = 0; row < n; row++)
        {
            if (row != i)
            {
                float factor = aug[row * (2 * n) + i];
                for (int col = i; col < 2 * n; col++)
                    aug[row * (2 * n) + col] -= factor * aug[i * (2 * n) + col];
            }
        }
    }

    // Extract inverse
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            A[i * n + j] = aug[i * (2 * n) + (n + j)];

    free(aug);
    return true;
}
