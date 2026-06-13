//Include Headers//
#include<Arduino.h>
#include "configEspXC.h"
#include "motorControl.h"
#include "ledControl.h"

#include "soc/soc.h"          // For WRITE_PERI_REG
#include "soc/rtc_cntl_reg.h"  // For RTC_CNTL_BROWN_OUT_REG

#define DEBUG_ESPXC true

void setup(){

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

    #ifdef DEBUG_ESPXC
    Serial.begin(9600);
    Serial.println("Starting ESP32 Dual-Core Drone Controller...");
    #endif
    
    setupEspXC();
    
    #ifdef DEBUG_ESPXC
    Serial.println("Setup complete - Tasks running on both cores");
    #endif
}

void loop(){
    // Minimal main loop - tasks handle everything
    delay(100);
}