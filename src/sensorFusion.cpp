//Include The Headers//
#include <Arduino.h>
#include "sensorFusion.h"
#include "imuSensor.h"

#define DEBUG_SENSOR_FUSION True

#define FILTER_DT 0.01      // 10ms (100 Hz update rate)
#define COMP_ALPHA 0.98     // Complementary filter weight (gyro heavy)
#define Q_ANGLE 0.001       // Process noise variance for angle
#define Q_BIAS 0.005          // Process noise variance for bias
#define R_MEASURE 0.03      // Measurement noise variance

// Kalman filter variables
static float rollAngleEst = 0, rollBiasEst = 0;
static float pitchAngleEst = 0, pitchBiasEst = 0;
static float rollP[2][2] = {{0, 0}, {0, 0}}; // Error covariance for roll
static float pitchP[2][2] = {{0, 0}, {0, 0}}; // Error covariance for pitch

void applyComplementaryFilter() {
    computeImuAngles(); //Update The Imu Angles

    static float lastRollAngle = 0, lastPitchAngle = 0;

    // Integrate gyro rates
    float gyroRollAngle = lastRollAngle + IMUDATA.rollRate * FILTER_DT;
    float gyroPitchAngle = lastPitchAngle + IMUDATA.pitchRate * FILTER_DT;

    // Apply complementary filter
    IMUDATA.rollfilteredAngle = COMP_ALPHA * gyroRollAngle + (1.0 - COMP_ALPHA) * IMUDATA.rollAngle;
    IMUDATA.pitchfilteredAngle = COMP_ALPHA * gyroPitchAngle + (1.0 - COMP_ALPHA) * IMUDATA.pitchAngle;

    // Update last angles
    lastRollAngle = IMUDATA.rollfilteredAngle;
    lastPitchAngle = IMUDATA.pitchfilteredAngle;

    #ifdef DEBUG_SENSOR_FUSION
    Serial.printf("Complementary: roll=%.2f deg, pitch=%.2f deg\n",IMUDATA.rollfilteredAngle, IMUDATA.pitchfilteredAngle);
    #endif
}

void applyKalmanFilter() {
    computeImuAngles(); //Update The Imu Angles

    // Roll Kalman filter //

    // Predict
    rollAngleEst += (IMUDATA.rollRate - rollBiasEst) * FILTER_DT;
    rollP[0][0] += FILTER_DT * (rollP[1][1] * FILTER_DT - rollP[0][1] - rollP[1][0]) + Q_ANGLE;
    rollP[0][1] -= rollP[1][1] * FILTER_DT;
    rollP[1][0] -= rollP[1][1] * FILTER_DT;
    rollP[1][1] += Q_BIAS;

    // Update
    float S = rollP[0][0] + R_MEASURE;
    float K[2];
    K[0] = rollP[0][0] / S;
    K[1] = rollP[1][0] / S;

    float y = IMUDATA.rollAngle - rollAngleEst;
    rollAngleEst += K[0] * y;
    rollBiasEst += K[1] * y;

    float P00_temp = rollP[0][0], P01_temp = rollP[0][1];
    rollP[0][0] -= K[0] * P00_temp;
    rollP[0][1] -= K[0] * P01_temp;
    rollP[1][0] -= K[1] * P00_temp;
    rollP[1][1] -= K[1] * P01_temp;

    IMUDATA.rollfilteredAngle = rollAngleEst;

    // Pitch Kalman filter //

    // Predict
    pitchAngleEst += (IMUDATA.pitchRate - pitchBiasEst) * FILTER_DT;
    pitchP[0][0] += FILTER_DT * (pitchP[1][1] * FILTER_DT - pitchP[0][1] - pitchP[1][0]) + Q_ANGLE;
    pitchP[0][1] -= pitchP[1][1] * FILTER_DT;
    pitchP[1][0] -= pitchP[1][1] * FILTER_DT;
    pitchP[1][1] += Q_BIAS;

    // Update
    S = pitchP[0][0] + R_MEASURE;
    K[0] = pitchP[0][0] / S;
    K[1] = pitchP[1][0] / S;

    y = IMUDATA.pitchAngle - pitchAngleEst;
    pitchAngleEst += K[0] * y;
    pitchBiasEst += K[1] * y;

    P00_temp = pitchP[0][0], P01_temp = pitchP[0][1];
    pitchP[0][0] -= K[0] * P00_temp;
    pitchP[0][1] -= K[0] * P01_temp;
    pitchP[1][0] -= K[1] * P00_temp;
    pitchP[1][1] -= K[1] * P01_temp;

    IMUDATA.pitchfilteredAngle = pitchAngleEst;

    #ifdef DEBUG_SENSOR_FUSION
    //Serial.printf("Kalman: roll=%.2f deg, pitch=%.2f deg\n",IMUDATA.rollfilteredAngle, IMUDATA.pitchfilteredAngle);
    #endif
}