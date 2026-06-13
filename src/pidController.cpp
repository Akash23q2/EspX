#include "pidController.h"
#include "pinsDefination.h"
#include "imuSensor.h"
#include "wifiRx.h"
#include "motorControl.h"

#define DEBUG_PID True
#define PID_LOOP_TIME_US 4000 // 4ms loop for 250 Hz
#define PID_MAX_MOTOR 4095     // Max motor throttle (8-bit)
#define PID_MIN_MOTOR 0       // Min motor throttle

// SAFETY LIMITS
#define MAX_ANGLE_DEGREES 30.0f    // Maximum allowed angle before emergency
#define MAX_RATE_DPS 300.0f        // Maximum rate in degrees per second
#define MIN_THROTTLE_FOR_ANGLE_MODE 1100  // Minimum throttle for angle mode

// PID gain structs for rate mode (REDUCED for stability)
static struct pidGain rollRateGain = {0.6, 0.02, 8.0, 100.0};   // Reduced from 1.3, 0.05, 15.0
static struct pidGain pitchRateGain = {0.6, 0.02, 8.0, 100.0};  // Reduced from 1.3, 0.05, 15.0
static struct pidGain yawRateGain = {2.0, 0.01, 0.0, 100.0};    // Reduced from 4.0, 0.02

// PID gain structs for angle mode (REDUCED for stability)
static struct pidGain rollAngleGain = {1.5, 0.0, 0.0, 30.0};    // Reduced from 2.0, limit to ±30°/s
static struct pidGain pitchAngleGain = {1.5, 0.0, 0.0, 30.0};   // Reduced from 2.0, limit to ±30°/s

// PID variables
static float pidIMemRollRate, pidLastRollRateDError;
static float pidIMemPitchRate, pidLastPitchRateDError;
static float pidIMemYawRate, pidLastYawRateDError;
static float pidIMemRollAngle, pidLastRollAngleDError;
static float pidIMemPitchAngle, pidLastPitchAngleDError;

// Safety variables
static bool emergencyStop = false;
static unsigned long emergencyTime = 0;

// Global PID data struct
struct pidData pidData = {0.0, 0.0, 0.0};

void pidControllerInit() {
    // Reset PID variables
    pidIMemRollRate = 0; pidLastRollRateDError = 0;
    pidIMemPitchRate = 0; pidLastPitchRateDError = 0;
    pidIMemYawRate = 0; pidLastYawRateDError = 0;
    pidIMemRollAngle = 0; pidLastRollAngleDError = 0;
    pidIMemPitchAngle = 0; pidLastPitchAngleDError = 0;
    
    emergencyStop = false;

    #ifdef DEBUG_PID
    Serial.println("PID controller initialized");
    #endif
}

bool checkSafetyLimits(float angleRoll, float anglePitch, float rollRate, float pitchRate, float yawRate) {
    // Check angle limits
    if (fabs(angleRoll) > MAX_ANGLE_DEGREES || fabs(anglePitch) > MAX_ANGLE_DEGREES) {
        Serial.printf("EMERGENCY: Angle limit exceeded! Roll=%.1f, Pitch=%.1f\n", angleRoll, anglePitch);
        return false;
    }
    
    // Check rate limits
    if (fabs(rollRate) > MAX_RATE_DPS || fabs(pitchRate) > MAX_RATE_DPS || fabs(yawRate) > MAX_RATE_DPS) {
        Serial.printf("EMERGENCY: Rate limit exceeded! Roll=%.1f, Pitch=%.1f, Yaw=%.1f\n", rollRate, pitchRate, yawRate);
        return false;
    }
    
    return true;
}

void pidControllerUpdate(float gyroRoll, float gyroPitch, float gyroYaw,
                         float angleRoll, float anglePitch,
                         int16_t throttle, int16_t rollSetpoint,
                         int16_t pitchSetpoint, int16_t yawSetpoint,
                         bool isArmed, bool mode) {
    static unsigned long lastLoopTime = 0;

    // Wait for 4ms loop
    while (micros() - lastLoopTime < PID_LOOP_TIME_US);
    lastLoopTime = micros();

    // SAFETY CHECK - Critical!
    // if (!checkSafetyLimits(angleRoll, anglePitch, gyroRoll, gyroPitch, gyroYaw)) {
    //     emergencyStop = true;
    //     emergencyTime = millis();
    //     motorControlSetSpeeds(0, 0, 0, 0);
    //     Serial.println("EMERGENCY STOP ACTIVATED!");
    //     return;
    // }

    // Reset emergency stop after 3 seconds if angles are safe
    // if (emergencyStop && (millis() - emergencyTime > 3000)) {
    //     if (fabs(angleRoll) < 5.0 && fabs(anglePitch) < 5.0) {
    //         emergencyStop = false;
    //         Serial.println("Emergency stop cleared - angles safe");
    //     }
    // }

    // // Don't allow arming if in emergency state
    // if (emergencyStop) {
    //     motorControlSetSpeeds(0, 0, 0, 0);
    //     return;
    // }

    // Convert RC setpoints (1000-2000) to deg/s for rate mode or degrees for angle mode
    float rollSp, pitchSp, yawSp;
    if (rcData.aux2 == 2000) { // Angle mode
        // Minimum throttle check for angle mode
        if (throttle < MIN_THROTTLE_FOR_ANGLE_MODE) {
            motorControlSetSpeeds(0, 0, 0, 0);
            Serial.println("Throttle too low for angle mode");
            return;
        }
        
        rollSp = (rollSetpoint - 1500) / 20.0; // ±25 deg (reduced from ±50)
        pitchSp = (pitchSetpoint - 1500) / 20.0;
    } else { // Rate mode
        rollSp = (rollSetpoint - 1500) / 5.0; // ±100 deg/s (reduced from ±167)
        pitchSp = (pitchSetpoint - 1500) / 5.0;
    }
    yawSp = (yawSetpoint - 1500) / 5.0; // ±100 deg/s for yaw

    // Apply low-pass filter to gyro inputs (FIXED - use proper time constant)
    static float filteredRoll = 0, filteredPitch = 0, filteredYaw = 0;
    float alpha = 0.3; // Increased from 0.2 for faster response
    filteredRoll = filteredRoll * (1.0 - alpha) + gyroRoll * alpha;
    filteredPitch = filteredPitch * (1.0 - alpha) + gyroPitch * alpha;
    filteredYaw = filteredYaw * (1.0 - alpha) + gyroYaw * alpha;

    // Angle mode: Outer PID loop to compute rate setpoints
    if (rcData.aux2 == 2000) {
        // Roll angle PID
        float angleError = rollSp - angleRoll;
        pidIMemRollAngle += rollAngleGain.i * angleError;
        pidIMemRollAngle = constrain(pidIMemRollAngle, -rollAngleGain.maxOutput, rollAngleGain.maxOutput);

        rollSp = rollAngleGain.p * angleError + pidIMemRollAngle +
                 rollAngleGain.d * (angleError - pidLastRollAngleDError);
        rollSp = constrain(rollSp, -rollAngleGain.maxOutput, rollAngleGain.maxOutput);
        pidLastRollAngleDError = angleError;

        // Pitch angle PID
        angleError = pitchSp - anglePitch;
        pidIMemPitchAngle += pitchAngleGain.i * angleError;
        pidIMemPitchAngle = constrain(pidIMemPitchAngle, -pitchAngleGain.maxOutput, pitchAngleGain.maxOutput);

        pitchSp = pitchAngleGain.p * angleError + pidIMemPitchAngle +
                  pitchAngleGain.d * (angleError - pidLastPitchAngleDError);
        pitchSp = constrain(pitchSp, -pitchAngleGain.maxOutput, pitchAngleGain.maxOutput);
        pidLastPitchAngleDError = angleError;
    }

    // Rate mode or angle mode inner loop: Rate PID
    // Roll rate PID
    float error = rollSp - filteredRoll;
    pidIMemRollRate += rollRateGain.i * error;
    pidIMemRollRate = constrain(pidIMemRollRate, -rollRateGain.maxOutput, rollRateGain.maxOutput);

    pidData.rollOutput = rollRateGain.p * error + pidIMemRollRate +
                         rollRateGain.d * (error - pidLastRollRateDError);
    pidData.rollOutput = constrain(pidData.rollOutput, -rollRateGain.maxOutput, rollRateGain.maxOutput);
    pidLastRollRateDError = error;

    // Pitch rate PID
    error = pitchSp - filteredPitch;
    pidIMemPitchRate += pitchRateGain.i * error;
    pidIMemPitchRate = constrain(pidIMemPitchRate, -pitchRateGain.maxOutput, pitchRateGain.maxOutput);

    pidData.pitchOutput = pitchRateGain.p * error + pidIMemPitchRate +
                          pitchRateGain.d * (error - pidLastPitchRateDError);
    pidData.pitchOutput = constrain(pidData.pitchOutput, -pitchRateGain.maxOutput, pitchRateGain.maxOutput);
    pidLastPitchRateDError = error;

    // Yaw rate PID
    error = yawSp - filteredYaw;
    pidIMemYawRate += yawRateGain.i * error;
    pidIMemYawRate = constrain(pidIMemYawRate, -yawRateGain.maxOutput, yawRateGain.maxOutput);

    pidData.yawOutput = yawRateGain.p * error + pidIMemYawRate +
                        yawRateGain.d * (error - pidLastYawRateDError);
    pidData.yawOutput = constrain(pidData.yawOutput, -yawRateGain.maxOutput, yawRateGain.maxOutput);
    pidLastYawRateDError = error;

    // Calculate motor speeds
    Serial.print("isArmed->");
    Serial.println(rcData.aux1 == 2000);
    
    if (rcData.aux1 == 2000) { // Armed
        // CORRECTED Motor mixing for X-configuration
        // Looking from above, motors rotate: M1↻ M2↺ M3↻ M4↺
        int m1 = throttle - pidData.pitchOutput + pidData.rollOutput - pidData.yawOutput; // Front right ↻
        int m2 = throttle + pidData.pitchOutput + pidData.rollOutput + pidData.yawOutput; // Back right ↺  
        int m3 = throttle + pidData.pitchOutput - pidData.rollOutput - pidData.yawOutput; // Back left ↻
        int m4 = throttle - pidData.pitchOutput - pidData.rollOutput + pidData.yawOutput; // Front left ↺

        // Map and constrain motor values
        uint8_t m1_mapped = map(constrain(m1, 1000, 2000), 1000, 2000, PID_MIN_MOTOR, PID_MAX_MOTOR);
        uint8_t m2_mapped = map(constrain(m2, 1000, 2000), 1000, 2000, PID_MIN_MOTOR, PID_MAX_MOTOR);
        uint8_t m3_mapped = map(constrain(m3, 1000, 2000), 1000, 2000, PID_MIN_MOTOR, PID_MAX_MOTOR);
        uint8_t m4_mapped = map(constrain(m4, 1000, 2000), 1000, 2000, PID_MIN_MOTOR, PID_MAX_MOTOR);
        
        // CRITICAL FIX: Actually send motor commands!
        motorControlSetSpeeds(m1_mapped, m2_mapped, m3_mapped, m4_mapped);
        
        Serial.printf("Motor speeds-> M1-%d, M2->%d, M3->%d, M4->%d\n", m1_mapped, m2_mapped, m3_mapped, m4_mapped);
        
        #ifdef DEBUG_PID
        Serial.printf("PID: Roll=%.1f, Pitch=%.1f, Yaw=%.1f | Angles: R=%.1f, P=%.1f\n", 
                      pidData.rollOutput, pidData.pitchOutput, pidData.yawOutput, angleRoll, anglePitch);
        #endif
    } else {
        // Motors off when disarmed
        motorControlSetSpeeds(0, 0, 0, 0);
        
        // Reset I terms when disarmed
        pidIMemRollRate = 0;
        pidIMemPitchRate = 0;
        pidIMemYawRate = 0;
        pidIMemRollAngle = 0;
        pidIMemPitchAngle = 0;
    }
}