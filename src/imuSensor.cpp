/*
 //THIS CODE IS INTENDED FOR THE MPU 6050 //
*/

//NOTE: Wire.requestFrom ==> Wire.begin + Wire.endTrans() at end// starting wire trans b4 end jams the bus//

#include "pinsDefination.h"
#include "ImuSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include "ledControl.h"
#include <math.h>

#define DEBUG_IMU True

#define IMU_I2C_ADDR 0x68
#define I2C_CLK_SPEED 400000 

struct IMUDATA IMUDATA;

#define GYRO_SCALE 65.5
#define ACC_SCALE 4096.0

void initImuSensor(){
    pinMode(I2C_SDA_PIN,INPUT);
    pinMode(I2C_SCL_PIN,INPUT);
    Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN,I2C_CLK_SPEED); 

    //Configure Gyro
    Wire.beginTransmission(IMU_I2C_ADDR); // Start I2C Transmission With Gyro
    Wire.write(0x6B); //WakeUp Device
    Wire.write(0x00); // Set Gyro in Power Mode
    Wire.endTransmission(true);

    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(0x1B); // Gyro_Config Register
    Wire.write(0x08); //500 dps Full Scale
    Wire.endTransmission(true);

    //Check IMU Avilability
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(0x75);  // WHO_AM_I register
    Wire.endTransmission(true);
    Wire.requestFrom(IMU_I2C_ADDR, 1);
    
    if(Wire.available() < 1 || Wire.read() != 0x68) {  // 0x68 is the expected WHO_AM_I response
        Serial.println("ERROR: MPU6050 not detected!");
        digitalWrite(LED_PIN, HIGH);
        while(1) delay(100); // Halt execution
    }
    #ifdef DEBUG_IMU
    Serial.println("MPU6050 detected successfully");
    #endif
    ledControlSetColor(255, 165, 0); // Set Led Color to Orange

    //Configure Accelerometer
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(0x1C); //Acc Config Register
    Wire.write(0x10); //+/- 8g Full Scale
    Wire.endTransmission();
    
    //Setup Digital LPF
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(0x1A); // Enable The Digital LPF
    Wire.write(0x05); //10Hz Smoothening
    Wire.endTransmission(); //End the Transmission

    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(0x1B); 
    Wire.write(0x08);  // Set the senstivity Scale Factor
    Wire.endTransmission();    
    delay(200); // Little delay for color Visibility //
}

/*
//  IMU DATA STRUCTURE //
struct IMUDATA{
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
}
*/

void calibrateImu(){
    for(int count=1; count<=2000; count++){

        ledControlSetColor(255-count%256,0,count%256); //Blink Effect

        Wire.beginTransmission(IMU_I2C_ADDR);
        Wire.write(0x3B); //set Mpu 6050 Pointer starting from Acc_X H OUT 
        Wire.endTransmission(true);
        Wire.requestFrom(IMU_I2C_ADDR,14);
        int16_t AccX = Wire.read() << 8 | Wire.read();
        int16_t AccY = Wire.read() << 8 | Wire.read();
        int16_t AccZ = Wire.read() << 8 | Wire.read();
        int16_t Temp = Wire.read() << 8 | Wire.read();
        int16_t GyroX = Wire.read() << 8 | Wire.read();
        int16_t GyroY = Wire.read() << 8 | Wire.read();
        int16_t GyroZ = Wire.read() << 8 | Wire.read();
        
        //CALIBRATE GYRO
        IMUDATA.offsetGyroX += (((float)GyroX/GYRO_SCALE)-IMUDATA.offsetGyroX)/count; //Convert to Degree Per Sec
        IMUDATA.offsetGyroY += (((float)GyroY/GYRO_SCALE)-IMUDATA.offsetGyroY)/count;  //calculate recursive mean/
        IMUDATA.offsetGyroZ += (((float)GyroZ/GYRO_SCALE)-IMUDATA.offsetGyroZ)/count;

        //CALIBRATE ACC
        IMUDATA.offsetAccX += (((float)AccX/ACC_SCALE)-IMUDATA.offsetAccX)/count;
        IMUDATA.offsetAccY += (((float)AccY/ACC_SCALE)-IMUDATA.offsetAccY)/count;
        IMUDATA.offsetAccZ += ((((float)AccZ/ACC_SCALE)-1)-IMUDATA.offsetAccZ)/count;
        
        #ifdef DEBUG_IMU  
        if(count%200==0){
            Serial.print("Calibrating -> ");
            Serial.print(count/20);
            Serial.println(" %");
        #endif
            ledControlSetColor(0,0,0);
        }

        ; //small delay//
    }
    #ifdef DEBUG_IMU
    Serial.printf("The offsets are:- \nX->%.2f, Y->%.2f, Z->%.2f\n", IMUDATA.offsetGyroX, IMUDATA.offsetGyroY, IMUDATA.offsetGyroZ);
    #endif
}

void getImuData(){
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(0x3B); //set Mpu 6050 Pointer starting from Acc_X H OUT 
    Wire.endTransmission(true);
   // Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.requestFrom(IMU_I2C_ADDR,14);
    int16_t AccX = Wire.read() << 8 | Wire.read();
    int16_t AccY = Wire.read() << 8 | Wire.read();
    int16_t AccZ = Wire.read() << 8 | Wire.read();
    int16_t Temp = Wire.read() << 8 | Wire.read();
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    //CONVERT ACC//
    IMUDATA.accX = ((float)AccX / ACC_SCALE) - IMUDATA.offsetAccX;
    IMUDATA.accY = ((float)AccY / ACC_SCALE) - IMUDATA.offsetAccY;
    IMUDATA.accZ = ((float)AccZ / ACC_SCALE) - IMUDATA.offsetAccZ; //Assuming 1g = 9.8 along the z-axis 

    //CONVERT GYRO//
    IMUDATA.rollRate = ((float)GyroX/GYRO_SCALE) - IMUDATA.offsetGyroX;
    IMUDATA.pitchRate = ((float)GyroY/GYRO_SCALE) - IMUDATA.offsetGyroY;
    IMUDATA.yawRate = ((float)GyroZ/GYRO_SCALE) - IMUDATA.offsetGyroZ;
    
    #ifdef DEBUG_IMU
    //Serial.printf("Raw: X=%d, Y=%d, Z=%d\n", GyroX, GyroY, GyroZ);
    #endif
}

    void computeImuAngles(){
    // Calculate roll: atan2(accY, accZ)
    IMUDATA.rollAngle = atan2(IMUDATA.accY, IMUDATA.accZ) * 180.0 / M_PI;

    // Calculate pitch: atan2(-accX, sqrt(accY^2 + accZ^2))
    float pitchDenom = sqrt(IMUDATA.accY * IMUDATA.accY + IMUDATA.accZ * IMUDATA.accZ);
    IMUDATA.pitchAngle = atan2(-IMUDATA.accX, pitchDenom) * 180.0 / M_PI;

    // Ensure angles are within valid range (-90 to 90 deg)
    IMUDATA.rollAngle = constrain(IMUDATA.rollAngle, -90.0, 90.0);
    IMUDATA.pitchAngle = constrain(IMUDATA.pitchAngle, -90.0, 90.0);

    #ifdef DEBUG_IMU
    Serial.printf("Angles: roll=%.2f deg, pitch=%.2f deg\n",IMUDATA.rollAngle, IMUDATA.pitchAngle);
    #endif
}

void printImu() {
    Serial.println("======= IMU DATA =======");
    Serial.printf("Offset GyroX: %.2f\tGyroY: %.2f\tGyroZ: %.2f\n", IMUDATA.offsetGyroX, IMUDATA.offsetGyroY, IMUDATA.offsetGyroZ);
    Serial.printf("Offset AccX: %.2f\tAccY: %.2f\tAccZ: %.2f\n", IMUDATA.offsetAccX, IMUDATA.offsetAccY, IMUDATA.offsetAccZ);
    Serial.printf("GyroX: %.2f\tY: %.2f\tZ: %.2f\n", IMUDATA.offsetGyroX, IMUDATA.offsetGyroZ, IMUDATA.offsetGyroZ);

    Serial.printf("AccX: %.2f\tAccY: %.2f\tAccZ: %.2f\n", IMUDATA.accX, IMUDATA.accY, IMUDATA.accZ);
    Serial.printf("Roll Rate: %.2f\tPitch Rate: %.2f\tYaw Rate: %.2f\n", IMUDATA.rollRate, IMUDATA.pitchRate, IMUDATA.yawRate);
    Serial.printf("Roll Angle: %.2f\tPitch Angle: %.2f\n", IMUDATA.rollAngle, IMUDATA.pitchAngle);
    Serial.printf("Filtered Roll: %.2f\tFiltered Pitch: %.2f\n", IMUDATA.rollfilteredAngle, IMUDATA.pitchfilteredAngle);
    Serial.println("========================");
}


