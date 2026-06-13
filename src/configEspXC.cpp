// //Include The Headers//
// #include<Arduino.h>
// #include "configEspXC.h"
// #include "batteryMonitor.h"
// #include "ledControl.h"
// #include "motorControl.h"
// #include "imuSensor.h"
// #include "wifiRx.h"
// #include "pidController.h"
// #include "sensorFusion.h"

// #define DEBUG_CONFIG_ESPX True // Enable Serial Debugging //

// void setupEspXC(){
//     initLedControl(); //Initialize Leds
//     initBatteryPin(); //Initialize Battery
//     readBatteryVoltage();
//     initImuSensor(); //Initialize IMU
//     calibrateImu(); //Calibrate Gyro
//     motorControlInit(); //Initialize Motors
//     rcControlInit(); //Initialize  Wifi Rx
//     ledControlSetColor(0,255,0);
//     pidControllerInit();
// }

// void plotImu() {
//     // Output space-separated values for Serial Plotter
//    // Serial.print("Roll:");
//    // Serial.print(IMUDATA.rollAngle);
//    // Serial.print(" Pitch:");
//    // Serial.print(IMUDATA.pitchAngle);
//     Serial.print(" F_Roll:");
//     Serial.print(IMUDATA.rollfilteredAngle);
//     Serial.print(" F_Pitch:");
//     Serial.print(IMUDATA.pitchfilteredAngle);
//     Serial.print(" RollRate:");
//     //Serial.print(IMUDATA.rollRate);
//     //Serial.print(" PitchRate:");
//    // Serial.print(IMUDATA.pitchRate);
//    // Serial.print(" YawRate:");
//    // Serial.println(IMUDATA.yawRate);
// }

// void startEspXC(){
//     ledControlRandomColor(); //Set a random Color//
//     getImuData(); //Update Imu Data
//     // Get RC data
//     applyKalmanFilter();
//     rcControlUpdate();
//   //  Determine mode (e.g., aux2 > 1500 for angle mode)
//     pidControllerUpdate(
//         IMUDATA.rollRate,
//         IMUDATA.pitchRate, 
//         IMUDATA.yawRate,
//         IMUDATA.rollfilteredAngle, 
//         IMUDATA.pitchfilteredAngle,
//         rcData.throttle,
//         rcData.roll, 
//         rcData.pitch,
//         rcData.yaw,
//         rcData.aux1 == 2000, /*Arm*/
//         rcData.aux2==2000 /*Angle Mode*/);
//     printImu();
// }

//Include The Headers//
#include<Arduino.h>
#include "configEspXC.h"
#include "batteryMonitor.h"
#include "ledControl.h"
#include "motorControl.h"
#include "imuSensor.h"
#include "wifiRx.h"
#include "pidController.h"
#include "sensorFusion.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define DEBUG_CONFIG_ESPX true // Enable Serial Debugging //

// Task priorities
#define PID_TASK_PRIORITY 3      // Higher priority for flight control
#define COMM_TASK_PRIORITY 2     // Lower priority for communication

// Task stack sizes (in words, not bytes)
#define PID_TASK_STACK_SIZE  16384    // 16KB stack for PID task
#define COMM_TASK_STACK_SIZE  16384    // 16KB stack for communication task

// Task handles
TaskHandle_t pidTaskHandle = NULL;
TaskHandle_t commTaskHandle = NULL;

// Shared data structures
typedef struct {
    int16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t aux1;
    int16_t aux2;
    bool isValid;
    uint32_t timestamp;
} SharedRCData_t;

typedef struct {
    float rollRate;
    float pitchRate;
    float yawRate;
    float rollfilteredAngle;
    float pitchfilteredAngle;
    bool isValid;
    uint32_t timestamp;
} SharedIMUData_t;

// Shared data variables
SharedRCData_t sharedRCData = {0};
SharedIMUData_t sharedIMUData = {0};

// Synchronization primitives
SemaphoreHandle_t rcDataMutex = NULL;
SemaphoreHandle_t imuDataMutex = NULL;
QueueHandle_t rcDataQueue = NULL;

// Task functions
void pidControlTask(void *pvParameters);
void communicationTask(void *pvParameters);

void setupEspXC(){
    Serial.begin(115200);
    
    // Initialize hardware components
    initLedControl(); //Initialize Leds
    initBatteryPin(); //Initialize Battery
    readBatteryVoltage();
    initImuSensor(); //Initialize IMU
    calibrateImu(); //Calibrate Gyro
    motorControlInit(); //Initialize Motors
    rcControlInit(); //Initialize Wifi Rx
    ledControlSetColor(0,255,0);
    pidControllerInit();
    
    // Create synchronization primitives
    rcDataMutex = xSemaphoreCreateMutex();
    imuDataMutex = xSemaphoreCreateMutex();
    rcDataQueue = xQueueCreate(5, sizeof(SharedRCData_t)); // Queue for 5 RC data packets
    
    if (rcDataMutex == NULL || imuDataMutex == NULL || rcDataQueue == NULL) {
        Serial.println("Failed to create synchronization primitives!");
        while(1) delay(1000);
    }
    
    // Initialize shared data
    sharedRCData.isValid = false;
    sharedIMUData.isValid = false;
    
    // Create tasks
    BaseType_t pidTaskResult = xTaskCreatePinnedToCore(
        pidControlTask,           // Task function
        "PID_Control_Task",       // Task name
        PID_TASK_STACK_SIZE,      // Stack size
        NULL,                     // Parameters
        PID_TASK_PRIORITY,        // Priority
        &pidTaskHandle,           // Task handle
        1                         // Core 1 (APP CPU)
    );
    
    BaseType_t commTaskResult = xTaskCreatePinnedToCore(
        communicationTask,        // Task function
        "Communication_Task",     // Task name
        COMM_TASK_STACK_SIZE,     // Stack size
        NULL,                     // Parameters
        COMM_TASK_PRIORITY,       // Priority
        &commTaskHandle,          // Task handle
        0                         // Core 0 (PRO CPU)
    );
    
    if (pidTaskResult != pdPASS || commTaskResult != pdPASS) {
        Serial.println("Failed to create tasks!");
        while(1) delay(1000);
    }
    
    Serial.println("Dual-core drone controller initialized successfully!");
    Serial.println("PID Task running on Core 1");
    Serial.println("Communication Task running on Core 0");
}

void pidControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 250Hz loop (4ms)
    
    SharedRCData_t localRCData = {0};
    SharedIMUData_t localIMUData = {0};
    
    Serial.println("PID Control Task started on Core 1");
    
    while(1) {
        // Read IMU data
        getImuData();
        applyKalmanFilter();
        
        // Update shared IMU data
        if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            sharedIMUData.rollRate = IMUDATA.rollRate;
            sharedIMUData.pitchRate = IMUDATA.pitchRate;
            sharedIMUData.yawRate = IMUDATA.yawRate;
            sharedIMUData.rollfilteredAngle = IMUDATA.rollfilteredAngle;
            sharedIMUData.pitchfilteredAngle = IMUDATA.pitchfilteredAngle;
            sharedIMUData.isValid = true;
            sharedIMUData.timestamp = millis();
            
            // Copy for local use
            localIMUData = sharedIMUData;
            
            xSemaphoreGive(imuDataMutex);
        }
        
        // Get latest RC data
        if (xSemaphoreTake(rcDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            localRCData = sharedRCData;
            xSemaphoreGive(rcDataMutex);
        }
        
        // Check if RC data is recent (within 100ms)
        bool rcDataRecent = localRCData.isValid && 
                           (millis() - localRCData.timestamp) < 100;
        
        if (localIMUData.isValid && rcDataRecent) {
            // Run PID controller with fresh data
            pidControllerUpdate(
                localIMUData.rollRate,
                localIMUData.pitchRate,
                localIMUData.yawRate,
                localIMUData.rollfilteredAngle,
                localIMUData.pitchfilteredAngle,
                localRCData.throttle,
                localRCData.roll,
                localRCData.pitch,
                localRCData.yaw,
                localRCData.aux1 == 2000, /*Arm*/
                localRCData.aux2 == 2000  /*Angle Mode*/
            );
        } else {
            // Failsafe: If no recent RC data, disarm or hover
            if (!rcDataRecent) {
                pidControllerUpdate(
                    localIMUData.rollRate,
                    localIMUData.pitchRate,
                    localIMUData.yawRate,
                    localIMUData.rollfilteredAngle,
                    localIMUData.pitchfilteredAngle,
                    1000, // Minimum throttle
                    1500, // Neutral roll
                    1500, // Neutral pitch
                    1500, // Neutral yaw
                    false, // Disarm
                    false  // Rate mode
                );
            }
        }
        
        // Maintain precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void communicationTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz loop (20ms)
    
    uint32_t ledUpdateCounter = 0;
    uint32_t batteryCheckCounter = 0;
    
    Serial.println("Communication Task started on Core 0");
    
    while(1) {
        // Update RC data
        rcControlUpdate();
        
        // Update shared RC data
        if (xSemaphoreTake(rcDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            sharedRCData.throttle = rcData.throttle;
            sharedRCData.roll = rcData.roll;
            sharedRCData.pitch = rcData.pitch;
            sharedRCData.yaw = rcData.yaw;
            sharedRCData.aux1 = rcData.aux1;
            sharedRCData.aux2 = rcData.aux2;
            sharedRCData.isValid = true;
            sharedRCData.timestamp = millis();
            
            xSemaphoreGive(rcDataMutex);
        }
        
        // Also send to queue for other potential consumers
        SharedRCData_t queueData = sharedRCData;
        xQueueSend(rcDataQueue, &queueData, 0); // Don't block
        
        // LED control (update every 100ms)
        ledUpdateCounter++;
        if (ledUpdateCounter >= 5) { // 5 * 20ms = 100ms
            ledUpdateCounter = 0;
            
            // Check connection status and update LED accordingly
            bool rcConnected = sharedRCData.isValid && 
                              (millis() - sharedRCData.timestamp) < 200;
            
            if (rcConnected) {
                if (sharedRCData.aux1 == 2000) {
                    ledControlSetColor(255, 0, 0); // Red when armed
                } else {
                    ledControlSetColor(0, 255, 0); // Green when disarmed
                }
            } else {
                ledControlSetColor(255, 255, 0); // Yellow when no RC signal
            }
        }
        
        // Battery monitoring (update every 1 second)
        batteryCheckCounter++;
        if (batteryCheckCounter >= 50) { // 50 * 20ms = 1000ms
            batteryCheckCounter = 0;
            readBatteryVoltage();
            
            // You can add battery voltage to shared data if needed
        }
        
        // Debug output (optional)
        #if DEBUG_CONFIG_ESPX
        static uint32_t debugCounter = 0;
        debugCounter++;
        if (debugCounter >= 25) { // Every 500ms
            debugCounter = 0;
            
            // Print IMU data from shared structure
            if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                Serial.print("F_Roll:");
                Serial.print(sharedIMUData.rollfilteredAngle);
                Serial.print(" F_Pitch:");
                Serial.print(sharedIMUData.pitchfilteredAngle);
                Serial.print(" RC_Throttle:");
                Serial.print(sharedRCData.throttle);
                Serial.print(" RC_Connected:");
                Serial.println(sharedRCData.isValid && 
                              (millis() - sharedRCData.timestamp) < 200);
                
                xSemaphoreGive(imuDataMutex);
            }
        }
        #endif
        
        // Maintain precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void startEspXC(){
    // In dual-core setup, this function is no longer needed
    // as tasks run continuously. You can use this for any 
    // one-time operations or leave it empty.
    
    // Optional: Print system info
    Serial.println("=== ESP32 Dual-Core Drone Controller ===");
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("PID Task running on Core: %d\n", xPortGetCoreID());
    
    // The main loop will be handled by FreeRTOS scheduler
    // Delete the setup task if needed
    vTaskDelete(NULL);
}

// Helper function to get RC data from other parts of code if needed
bool getRCData(SharedRCData_t* rcDataOut) {
    if (xSemaphoreTake(rcDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        *rcDataOut = sharedRCData;
        xSemaphoreGive(rcDataMutex);
        return rcDataOut->isValid;
    }
    return false;
}

// Helper function to get IMU data from other parts of code if needed
bool getIMUData(SharedIMUData_t* imuDataOut) {
    if (xSemaphoreTake(imuDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        *imuDataOut = sharedIMUData;
        xSemaphoreGive(imuDataMutex);
        return imuDataOut->isValid;
    }
    return false;
}

// Function to check task health (optional)
void printTaskInfo() {
    Serial.println("=== Task Information ===");
    Serial.printf("PID Task High Water Mark: %d\n", 
                  uxTaskGetStackHighWaterMark(pidTaskHandle));
    Serial.printf("Comm Task High Water Mark: %d\n", 
                  uxTaskGetStackHighWaterMark(commTaskHandle));
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
}