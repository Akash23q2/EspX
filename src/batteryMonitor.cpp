#include "batteryMonitor.h"
#include <Arduino.h>
#include "pinsDefination.h"

#define DEBUG_BATTERY True

#define R1 10000 // First Resistor Value
#define R2 10000 // Second Resistor Value
#define MAX_VOLTAGE_BATTERY 3.7 // Battery Max Voltage
#define MAX_ANALOG_READ 4095 // MCU Max Analog Readable Voltage i.e. 3V3

bool isBatteryLow = false; //Set To False at Start
constexpr float multiplicatonFactor = (3.3 *(R1 + R2)/R2)/(MAX_ANALOG_READ);

/* FUTURE FEATURES
  1. AUTO DISARM
  2. LED INDICATORS
  3. MODULAR ON OFF COMPONENTS
*/

//#ifdef HALT_IF_BATTERY_LOW
 
void initBatteryPin(){
    pinMode(BATTERY_PIN,INPUT);
}

float readBatteryVoltage(){
    float measuredValue=analogRead(BATTERY_PIN);
    #ifdef DEBUG_BATTERY
    Serial.print("Battery %-> ");
    Serial.println(measuredValue*multiplicatonFactor);
    #endif
    return measuredValue*multiplicatonFactor; 
}