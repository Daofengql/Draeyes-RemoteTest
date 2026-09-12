#pragma once

#include <Arduino.h>

namespace config {

// The bridge deliberately uses the same broadcast/channel contract as RC Mini.
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint8_t RECEIVER_MAC[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

constexpr uint32_t CONTROL_INTERVAL_MS = 10;
constexpr uint32_t PAIRING_LONG_PRESS_MS = 3000;
constexpr uint32_t PAIRING_RETRY_INTERVAL_MS = 500;
constexpr uint32_t MULTI_PRESS_WINDOW_MS = 1000;
constexpr uint32_t RUNTIME_STATE_TIMEOUT_MS = 1200;

// USB CDC ignores the physical UART baud, but keeping this value high avoids
// throttling the TX/RX frame log when the remote sends controller packets at
// 100 Hz.
constexpr uint32_t SERIAL_BAUD_RATE = 921600;
constexpr size_t SERIAL_LINE_BUFFER_SIZE = 768;
constexpr uint32_t STATUS_INTERVAL_MS = 1000;

constexpr size_t MAX_ESPNOW_PACKET_SIZE = 250;
constexpr uint8_t ESPNOW_RX_QUEUE_DEPTH = 12;
constexpr uint32_t RECEIVER_PAIRING_TIMEOUT_MS = 180000;

constexpr float DEFAULT_LIGHT_VALUE = 1.0f;
constexpr char DEVICE_TYPE[] = "remote";
constexpr char DEVICE_NAME[] = "Draeye RC Mini (CDC)";

}  // namespace config
