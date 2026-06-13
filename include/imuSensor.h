#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include<Arduino.h>

struct IMUDATA{    // Struct To Efficiently Store and Access IMU Sensor Values
    float offsetGyroX=0;
    float offsetGyroY=0;
    float offsetGyroZ=0;
    float offsetAccX=0;
    float offsetAccY=0;
    float offsetAccZ=0;
    float accX=0;
    float accY=0;
    float accZ=0;
    float rollRate=0;     // Initialize With 0 for Safety //
    float pitchRate=0;
    float yawRate=0;
    float rollAngle=0;       // Roll angle from accel (degrees)
    float pitchAngle=0;      // Pitch angle from accel (degrees)
    float rollfilteredAngle=0;   // Roll angle from filter (degrees)
    float pitchfilteredAngle=0;  // Pitch angle from filter (degrees)
};

extern struct IMUDATA IMUDATA; // Make Struct Avilable Globally //

void printImu(
//print the values of IMUDATA//
);

void initImuSensor(
 // Initialize The Imu Sensor
); 

void calibrateImu(
// Calibrate The Imu Sensor
); 

void getImuData(
// Update the IMUDATA1
); 

void computeImuAngles(
// Compute Angles with Acc //
);

void printImu(
//print the IMUDATA values//
);


#endif