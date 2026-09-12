#pragma once

#include <Arduino.h>

namespace config {

constexpr uint8_t DEFAULT_ESPNOW_CHANNEL = 1;
constexpr uint8_t BROADCAST_MAC[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

constexpr uint32_t SERIAL_BAUD_RATE = 921600;
constexpr size_t SERIAL_LINE_BUFFER_SIZE = 1024;
constexpr uint32_t STATUS_INTERVAL_MS = 1000;
constexpr uint32_t STREAM_HEARTBEAT_TIMEOUT_MS = 1500;

constexpr size_t MAX_ESPNOW_PACKET_SIZE = 250;
constexpr uint8_t ESPNOW_RX_QUEUE_DEPTH = 16;

}  // namespace config
