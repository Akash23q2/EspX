#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>

struct RGBDATA {
    uint8_t red;   // Red channel value (0-255)
    uint8_t green; // Green channel value (0-255)
    uint8_t blue;  // Blue channel value (0-255)
};

extern struct RGBDATA rgbData; // Global RGB data struct

void initLedControl(
// Initialize LEDC for RGB LED with specified GPIO pins
);

void ledControlSetColor(uint8_t red, uint8_t green, uint8_t blue
// Set RGB color (0-255 for each channel)
);

void ledControlGetColor(struct RGBDATA* rgb
// Get current RGB values from the struct
);

void ledControlRandomColor(
// Set a random RGB color
);

#endif