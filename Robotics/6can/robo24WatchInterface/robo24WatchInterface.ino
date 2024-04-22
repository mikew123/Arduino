/*
    ESP-NOW Broadcast Slave
    Lucas Saavedra Vaz - 2024

    This sketch demonstrates how to receive broadcast messages from a master device using the ESP-NOW protocol.

    The master device will broadcast a message every 5 seconds to all devices within the network.

    The slave devices will receive the broadcasted messages. If they are not from a known master, they will be registered as a new master
    using a callback function.
*/

#include "ESP32_NOW.h"
#include "WiFi.h"

#include <esp_mac.h> // For the MAC2STR and MACSTR macros

#include <vector>

/* Definitions */

#define ESPNOW_WIFI_CHANNEL 6

/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.

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
//        Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
//        Serial.printf("  Message: %s\n", (char *)data);
          Serial.println((char *)data);
    }


    // Function to send a message to all devices within the network
    bool send_message(const uint8_t *data, size_t len) {
        if (!send(data, len)) {
            log_e("Failed to broadcast message");
            return false;
        }
        return true;
    }

};

/* Global Variables */

// List of all the masters. It will be populated when a new master is registered
std::vector<ESP_NOW_Peer_Class> masters;

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
    if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
//        Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
//        Serial.println("Registering the peer as a master");

        ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

        masters.push_back(new_master);
        if (!masters.back().add_peer()) {
            Serial.println("Failed to register the new master");
            return;
        }

        // Send the message to the robot over USB serial
        Serial.println((char *)data);

    } else {
        // The slave will only receive broadcast messages
        log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
        log_v("Igorning the message");
    }
}

//uint8_t peer_mac[6] = {0x84, 0xFC, 0xE6, 0x50, 0x99, 0x37};
uint8_t peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Create a boradcast peer object
ESP_NOW_Peer_Class broadcast_peer(peer_mac, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

/* Main */

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

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

    if(!broadcast_peer.add_peer()) {
      Serial.println("Failed to add broadcast_peer");
    }

    // Register the new peer callback
    ESP_NOW.onNewPeer(register_new_master, NULL);

    Serial.println("Setup complete. Waiting for a master to broadcast a message...");
}

void loop() {

  // Copy text line from serial USB to ESP-NOW on Robo24 watch controller
  while (Serial.available() > 0) {

    String str = Serial.readString();
    unsigned char data[256];
    str.getBytes(data, sizeof(data));
    Serial.printf("Broadcasting message: %s\n", data);
    if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
         Serial.println("Failed to broadcast message");
    }

  }
  delay(10);
}