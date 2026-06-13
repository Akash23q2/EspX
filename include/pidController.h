#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

struct pidGain {
    float p;        // Proportional gain
    float i;        // Integral gain
    float d;        // Derivative gain
    float maxOutput; // Maximum PID output (+/-)
};

struct pidData {
    float rollOutput;  // PID output for roll
    float pitchOutput; // PID output for pitch
    float yawOutput;   // PID output for yaw
};

// Initialize PID controller
void pidControllerInit();

// Update PID calculations and compute motor speeds
// mode: 0 = rate mode, 1 = angle mode
void pidControllerUpdate(float gyroRoll, float gyroPitch, float gyroYaw,
                         float angleRoll, float anglePitch,
                         int16_t throttle, int16_t rollSetpoint,
                         int16_t pitchSetpoint, int16_t yawSetpoint,
                         bool isArmed, bool mode);

#endif // PID_CONTROLLER_H