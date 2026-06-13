#include "ledControl.h"
#include "pinsDefination.h"

#define LEDC_FREQ_HZ 5000       // PWM frequency: 5 kHz
#define LEDC_RESOLUTION 8       // 8-bit resolution (0-255)
#define LEDC_CHANNEL_RED 0      // LEDC channel for red
#define LEDC_CHANNEL_GREEN 1    // LEDC channel for green
#define LEDC_CHANNEL_BLUE 2     // LEDC channel for blue

//#define DEBUG_LED True

struct RGBDATA rgbData = {0, 0, 0}; // Global RGB data struct

void initLedControl() {
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);  // Set GPIO pins as output 
    pinMode(BLUE_LED_PIN, OUTPUT);

    ledcSetup(LEDC_CHANNEL_RED, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_GREEN, LEDC_FREQ_HZ, LEDC_RESOLUTION);   // Setup LEDC for each channel
    ledcSetup(LEDC_CHANNEL_BLUE, LEDC_FREQ_HZ, LEDC_RESOLUTION);

    ledcAttachPin(RED_LED_PIN, LEDC_CHANNEL_RED);
    ledcAttachPin(GREEN_LED_PIN, LEDC_CHANNEL_GREEN);  // Configure LEDC channels
    ledcAttachPin(BLUE_LED_PIN, LEDC_CHANNEL_BLUE);

    ledcWrite(LEDC_CHANNEL_RED, 0);
    ledcWrite(LEDC_CHANNEL_GREEN, 0);    // Initialize with LED off
    ledcWrite(LEDC_CHANNEL_BLUE, 0);

    #ifdef DEBUG_LED
    Serial.println("RGB LED initialized successfully");
    #endif
}

void ledControlSetColor(uint8_t red, uint8_t green, uint8_t blue) {
    rgbData.red = red;
    rgbData.green = green;  // Update the RGB data struct
    rgbData.blue = blue;

    ledcWrite(LEDC_CHANNEL_RED, red);
    ledcWrite(LEDC_CHANNEL_GREEN, green);  // Set PWM duty cycle for each channel
    ledcWrite(LEDC_CHANNEL_BLUE, blue);

    #ifdef DEBUG_LED
    Serial.printf("RGB Set: R=%d, G=%d, B=%d\n", red, green, blue);
    #endif
}

void ledControlGetColor(struct RGBDATA* rgb) {
    rgb->red = rgbData.red;
    rgb->green = rgbData.green;  // Copy current RGB values to the provided struct
    rgb->blue = rgbData.blue;

    #ifdef DEBUG_LED
    Serial.printf("RGB Get: R=%d, G=%d, B=%d\n", rgb->red, rgb->green, rgb->blue);
    #endif
}

void ledControlRandomColor() {
    uint8_t red = random(125);
    uint8_t green = random(56);   // Generate random RGB values (0-255)
    uint8_t blue = random(250);

    ledControlSetColor(red, green, blue);  // Set the random color using existing function

    #ifdef DEBUG_LED
    Serial.println("Random color set");
    #endif
}