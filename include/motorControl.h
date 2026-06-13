#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

struct MOTORDATA {
    uint8_t motor1;
    uint8_t motor2; // Throttle for motors  (0-255)
    uint8_t motor3;
    uint8_t motor4; 
};

extern struct MOTORDATA motorData; // Global motor data struct

void motorControlInit(
// Initialize LEDC for motors with specified GPIO pins and spin for verification
);

void motorControlSetSpeeds(uint8_t motor1, uint8_t motor2, uint8_t motor3, uint8_t motor4
// Set throttle values for each motor (0-255)
);

void motorControlGetSpeeds(struct MOTORDATA* motors
// Get current throttle values from the struct
);

#endif 