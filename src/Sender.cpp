#include "Sender.h"
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>
#include <Arduino.h>

uint8_t receiverMAC[] = {0x3C, 0x8A, 0x1F, 0xB0, 0x13, 0xF0}; // Feeder MAC adresses u put the reciver mac address in the sender

typedef struct {
  char text[32];
} esp_message_t;

esp_message_t msg;

// Send callback
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void Sender_Init() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

}

void Send(const char* message) {
  strncpy(msg.text, message, sizeof(msg.text) - 1);
  msg.text[sizeof(msg.text) - 1] = '\0';

  esp_now_send(receiverMAC, (uint8_t*)&msg, sizeof(msg));
  Serial.print("Sending: ");
  Serial.println(message);
}