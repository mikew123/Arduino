#include <Arduino_JSON.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include "String.h"

//#include <esp_mac.h> // For the MAC2STR and MACSTR macros

#include <vector>

/* Definitions */

#define ESPNOW_WIFI_CHANNEL 6


class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
    // Constructor of the class
    ESP_NOW_Peer_Class(const uint8_t *mac_addr,
                       uint8_t channel,
                       wifi_interface_t iface,
                       const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

    // Destructor of the class
    ~ESP_NOW_Peer_Class() {}

    // Function to register the master peer
    bool add_peer() {
        if (!add()) {
            log_e("Failed to register the broadcast peer");
            return false;
        }
        return true;
    }

    // Function to print the received messages from the master
    void onReceive(const uint8_t *data, size_t len, bool broadcast) {
        Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
        Serial.printf("  Message: %s\n", (char *)data);
    }
};

/* Global Variables */

// List of all the masters. It will be populated when a new master is registered
std::vector<ESP_NOW_Peer_Class> masters;

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
    if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
        Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
        Serial.println("Registering the peer as a master");

        ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

        masters.push_back(new_master);
        if (!masters.back().add_peer()) {
            Serial.println("Failed to register the new master");
            return;
        }
    } else {
        // The slave will only receive broadcast messages
        log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
        log_v("Igorning the message");
    }
}


/* Global Variables */

uint32_t msg_count = 0;

// Create a boradcast peer object
ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  Serial.println("ESP32C6 Dev board");

    // Initialize the Wi-Fi module
    WiFi.mode(WIFI_STA);
    WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
    while(!WiFi.STA.started()) delay(100);

    Serial.println("ESP-NOW Example - Broadcast Slave");
    Serial.println("Wi-Fi parameters:");
    Serial.println("  Mode: STA");
    Serial.println("  MAC Address: " + WiFi.macAddress());
    Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

    // Initialize the ESP-NOW protocol
    if (!ESP_NOW.begin()) {
        Serial.println("Failed to initialize ESP-NOW");
        Serial.println("Reeboting in 5 seconds...");
        delay(5000);
        ESP.restart();
    }

    // Register the new peer callback
    ESP_NOW.onNewPeer(register_new_master, NULL);

  Serial.println("ESP NOW initialized");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {

    String str = Serial.readString();
    Serial.print(str);
    // char data[256];
    // str.toCharArray(data, 256);

    // if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
    //     Serial.println("Failed to broadcast message");
    // }

  }
  delay(10);
}
