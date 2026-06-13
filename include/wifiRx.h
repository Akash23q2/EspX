#ifndef WIFI_RX_H
#define WIFI_RX_H

#include <Arduino.h>

struct RCDATA {
    uint16_t roll;   // Roll command (0-255)
    uint16_t throttle; // Throttle command (0-255)
    uint16_t pitch;  // Pitch command (0-255)
    uint16_t yaw;    // Yaw command (0-255)
    uint16_t aux1;   // Auxiliary channel 1 (0-255)
    uint16_t aux2;   // Auxiliary channel 2 (0-255)
    uint16_t aux3;   // Auxiliary channel 3 (0-255)
    uint16_t aux4;   // Auxiliary channel 4 (0-255)
};

extern struct RCDATA rcData; // Global RC data struct

void rcControlInit(
// Initialize Wi-Fi station and UDP server for receiving RC commands
);

void rcControlUpdate(
// Update RC data by reading incoming UDP packets
);

// void rcControlGetData(
// // Get current RC command values from the struct
// );

#endif