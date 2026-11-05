// Packet definitions — must match docs/protocol.md and robofield/comm/protocol.py
#pragma once
#include <stdint.h>
#include <stddef.h>

#define RF_MAGIC 0xA5
#define RF_VERSION 1
#define RF_TYPE_COMMAND 0x01
#define RF_TYPE_STATUS 0x02
#define RF_BROADCAST_ID 255

#define RF_FLAG_ENABLE 0x01
#define RF_FLAG_BRAKE 0x02
#define RF_STATUS_WATCHDOG 0x01
#define RF_STATUS_LOW_BATTERY 0x02

#define RF_TYPE_CONFIG 0x03        // set the robot id (sent to one car's IP)

#define RF_ESPNOW_CHANNEL 1
#define RF_UDP_CMD_PORT 4210        // cars listen here for commands / config
#define RF_UDP_STATUS_PORT 4211     // cars broadcast status here

typedef struct __attribute__((packed)) {
  uint8_t magic, version, type, robot_id;
  uint16_t seq;
  int16_t left, right;   // -1000..1000
  uint8_t servo;         // 0..180
  uint8_t leds[9];       // RGB x3
  uint8_t flags;
  uint8_t crc;
} rf_command_t;

typedef struct __attribute__((packed)) {
  uint8_t magic, version, type, robot_id;
  uint16_t seq_ack;
  uint16_t battery_mv;
  uint8_t fw_version;
  uint8_t status;
  int8_t rssi;
  uint8_t crc;
} rf_status_t;

typedef struct __attribute__((packed)) {
  uint8_t magic, version, type;   // type = RF_TYPE_CONFIG
  uint8_t new_id;                 // 0..19
  uint8_t reserved;
  uint8_t crc;
} rf_config_t;

static_assert(sizeof(rf_command_t) == 22, "command packet must be 22 bytes");
static_assert(sizeof(rf_config_t) == 6, "config packet must be 6 bytes");
static_assert(sizeof(rf_status_t) == 12, "status packet must be 12 bytes");

// CRC-8, polynomial 0x07, init 0x00
static inline uint8_t rf_crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}
