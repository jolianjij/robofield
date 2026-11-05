// RoboField gateway — USB serial <-> ESP-NOW bridge (any ESP32; reference: ESP32-C3 Super Mini)
// Serial frame: 0xAA 0x55 <len> <packet>. Packets are forwarded unchanged (docs/protocol.md).
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"

static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Ring buffer for packets coming from cars (filled in the WiFi task)
#define RXQ 16
static uint8_t rxBuf[RXQ][32];
static uint8_t rxLen[RXQ];
static volatile uint8_t rxHead = 0, rxTail = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len <= 0 || len > 32) return;
  portENTER_CRITICAL(&mux);
  uint8_t next = (rxHead + 1) % RXQ;
  if (next != rxTail) {               // drop if full
    memcpy(rxBuf[rxHead], data, len);
    rxLen[rxHead] = len;
    rxHead = next;
  }
  portEXIT_CRITICAL(&mux);
}

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(921600);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(RF_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = RF_ESPNOW_CHANNEL;
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  // Serial -> ESP-NOW: tiny state machine for 0xAA 0x55 <len> <payload>
  static uint8_t state = 0, len = 0, idx = 0, pkt[32];
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (state) {
      case 0: state = (b == 0xAA) ? 1 : 0; break;
      case 1: state = (b == 0x55) ? 2 : (b == 0xAA ? 1 : 0); break;
      case 2: len = b; idx = 0; state = (len > 0 && len <= sizeof(pkt)) ? 3 : 0; break;
      case 3:
        pkt[idx++] = b;
        if (idx == len) { esp_now_send(BROADCAST, pkt, len); state = 0; }
        break;
    }
  }
  // ESP-NOW -> Serial
  while (rxTail != rxHead) {
    uint8_t hdr[3] = {0xAA, 0x55, rxLen[rxTail]};
    Serial.write(hdr, 3);
    Serial.write(rxBuf[rxTail], rxLen[rxTail]);
    rxTail = (rxTail + 1) % RXQ;
  }
}
