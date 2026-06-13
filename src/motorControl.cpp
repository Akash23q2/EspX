#include "motorControl.h"
#include "pinsDefination.h"
#include "ledControl.h"

#define DEBUG_MOTOR True
#define LEDC_FREQ_HZ 1000      // PWM frequency: 25 kHz for coreless motors
#define LEDC_RESOLUTION 12       // 8-bit resolution (0-255)
#define LEDC_CHANNEL_M1 3       // LEDC channel for motor 1
#define LEDC_CHANNEL_M2 4       // LEDC channel for motor 2
#define LEDC_CHANNEL_M3 5       // LEDC channel for motor 3
#define LEDC_CHANNEL_M4 7       // LEDC channel for motor 4

#define DEBUG_MOTOR True

struct MOTORDATA motorData = {0, 0, 0, 0};  // Global motor data struct

void motorControlInit() {
    pinMode(MOTOR1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN, OUTPUT);      // Set GPIO pins as output
    pinMode(MOTOR3_PIN, OUTPUT);
    pinMode(MOTOR4_PIN, OUTPUT);

    ledcSetup(LEDC_CHANNEL_M1, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_M2, LEDC_FREQ_HZ, LEDC_RESOLUTION);  // Setup LEDC for each channel
    ledcSetup(LEDC_CHANNEL_M3, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_M4, LEDC_FREQ_HZ, LEDC_RESOLUTION);

    ledcAttachPin(MOTOR1_PIN, LEDC_CHANNEL_M1);
    ledcAttachPin(MOTOR2_PIN, LEDC_CHANNEL_M2);   // Configure LEDC channels
    ledcAttachPin(MOTOR3_PIN, LEDC_CHANNEL_M3);   
    ledcAttachPin(MOTOR4_PIN, LEDC_CHANNEL_M4);

     for (int duty = 1023; duty <= 4095; duty += 100) {
    ledcWrite(4, duty);
    delay(5000);
  }

    ledcWrite(LEDC_CHANNEL_M1, 0);
    ledcWrite(LEDC_CHANNEL_M2, 0);   // Initialize with motors off
    ledcWrite(LEDC_CHANNEL_M3, 0);
    ledcWrite(LEDC_CHANNEL_M4, 0);

    // Visual verification: Spin each motor briefly at low throttle
    #ifdef DEBUG_MOTOR
    Serial.println("Starting motor verification spin...");
    #endif

    ledControlSetColor(230,230,250);
     //Lavender Color -> Motor Test

    ledcWrite(LEDC_CHANNEL_M1, 150); // Low throttle (20% approx)
    delay(500);
    ledcWrite(LEDC_CHANNEL_M1, 0);
    ledcWrite(LEDC_CHANNEL_M2, 150);
    delay(500);
    ledcWrite(LEDC_CHANNEL_M2, 0);
    ledcWrite(LEDC_CHANNEL_M3, 150);
    delay(500);
    ledcWrite(LEDC_CHANNEL_M3, 0);
    ledcWrite(LEDC_CHANNEL_M4, 150);
    delay(500);
    ledcWrite(LEDC_CHANNEL_M4, 0);

    #ifdef DEBUG_MOTOR
    Serial.println("Motor verification complete. Motors initialized successfully");
    #endif
    ledControlSetColor(0,255,0); //Greem Color -> Motor Verification Done
    delay(500);
}

void motorControlSetSpeeds(uint8_t motor1, uint8_t motor2, uint8_t motor3, uint8_t motor4) {
    motorData.motor1 = motor1;
    motorData.motor2 = motor2;   // Update the motor data struct
    motorData.motor3 = motor3;
    motorData.motor4 = motor4;

    ledcWrite(LEDC_CHANNEL_M1, motor1);
    ledcWrite(LEDC_CHANNEL_M2, motor2);  // Set PWM duty cycle for each channel
    ledcWrite(LEDC_CHANNEL_M3, motor3);
    ledcWrite(LEDC_CHANNEL_M4, motor4);

    #ifdef DEBUG_MOTOR
    Serial.printf("Motor Speeds Set: M1=%d, M2=%d, M3=%d, M4=%d\n", motor1, motor2, motor3, motor4);
    #endif
}

void motorControlGetSpeeds(struct MOTORDATA* motors) {
    motors->motor1 = motorData.motor1;
    motors->motor2 = motorData.motor2;   // Copy current throttle values to the provided struct
    motors->motor3 = motorData.motor3;
    motors->motor4 = motorData.motor4;

    #ifdef DEBUG_MOTOR
    //Serial.printf("Motor Speeds Get: M1=%d, M2=%d, M3=%d, M4=%d\n", motors->motor1, motors->motor2, motors->motor3, motors->motor4);
    #endif
}