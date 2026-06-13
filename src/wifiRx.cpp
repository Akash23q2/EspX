#include "wifiRx.h"
#include "pinsDefination.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "ledControl.h"

#define DEBUG_RC True
#define UDP_PORT 4210           // UDP port for receiving RC commands
#define WIFI_SSID "Esp-XC" // ESP32 access point SSID
#define WIFI_PASSWORD "EspXcopter" // ESP32 access point password
#define STATIC_IP {192, 168, 4, 1} // Static IP address (matches app)
#define GATEWAY {192, 168, 4, 1}   // Gateway IP
#define SUBNET {255, 255, 255, 0}  // Subnet mask
#define RC_MIN 1000                // Minimum RC command value
#define RC_MAX 2000                // Maximum RC command value

//#define DEBUG_RC True

// Global RC data struct
struct RCDATA rcData;

// UDP object
static WiFiUDP udp;

void rcControlInit() {
    IPAddress staticIP(STATIC_IP);
    // Configure static IP
    IPAddress gateway(GATEWAY);
    IPAddress subnet(SUBNET);

    // Start Wi-Fi as access point
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(staticIP, gateway, subnet);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

    // Wait for AP to start
    #ifdef DEBUG_RC
    Serial.print("Starting Wi-Fi AP...");
    delay(2000); // Give AP time to initialize
    Serial.println("Wi-Fi AP started");
    Serial.printf("AP IP Address: %s\n", WiFi.softAPIP().toString().c_str());
    #endif

    // Start UDP server
    udp.begin(UDP_PORT);

    #ifdef DEBUG_RC
    Serial.printf("UDP server started on port %d\n", UDP_PORT);
    #endif
}

void rcControlUpdate() {
    // Check for incoming UDP packets
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char packetBuffer[64]; // Buffer for incoming packet
        int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
        packetBuffer[len] = '\0'; // Null-terminate the string

        // Parse format: T:%d,R:%d,P:%d,Y:%d,A1:%d,A2:%d
        uint16_t t, r, p, y, a1, a2;
        int parsed = sscanf(packetBuffer, "T:%hu,R:%hu,P:%hu,Y:%hu,A1:%hu,A2:%hu",
                           &t, &r, &p, &y, &a1, &a2);

        if (parsed == 6) { // Ensure all 6 values were parsed
            // Clamp values to RC_MIN (1000) and RC_MAX (2000)
            t = constrain(t, RC_MIN, RC_MAX);
            r = constrain(r, RC_MIN, RC_MAX);
            p = constrain(p, RC_MIN, RC_MAX);
            y = constrain(y, RC_MIN, RC_MAX);
            a1 = constrain(a1, RC_MIN, RC_MAX);
            a2 = constrain(a2, RC_MIN, RC_MAX);

            // Update RC data struct
            rcData.throttle = t;
            rcData.roll = r;
            rcData.pitch = p;
            rcData.yaw = y;
            rcData.aux1 = a1;
            rcData.aux2 = a2;
            rcData.aux3 = 1000; // Unused, set to neutral
            rcData.aux4 = 1000; // Unused, set to neutral

            #ifdef DEBUG_RC
            Serial.printf("RC Data Updated: throttle=%d, roll=%d, pitch=%d, yaw=%d, aux1=%d, aux2=%d, aux3=%d, aux4=%d\n",
                          t, r, p, y, a1, a2, rcData.aux3, rcData.aux4);
            #endif
        } else {
            #ifdef DEBUG_RC
            Serial.printf("Error: Invalid UDP packet format: %s\n", packetBuffer);
            #endif
        }
    }
}

// void rcControlGetData(struct RCDATA* rc) {
//     // Copy current RC command values to the provided struct
//     rc->roll = rcData.roll;
//     rc->throttle = rcData.throttle;
//     rc->pitch = rcData.pitch;
//     rc->yaw = rcData.yaw;
//     rc->aux1 = rcData.aux1;
//     rc->aux2 = rcData.aux2;
//     rc->aux3 = rcData.aux3;
//     rc->aux4 = rcData.aux4;

//     #ifdef DEBUG_RC
//     Serial.printf("RC Data Get: roll=%d, throttle=%d, pitch=%d, yaw=%d, aux1=%d, aux2=%d, aux3=%d, aux4=%d\n",
//                   rc->roll, rc->throttle, rc->pitch, rc->yaw, rc->aux1, rc->aux2, rc->aux3, rc->aux4);
//     #endif
// }