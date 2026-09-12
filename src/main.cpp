#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cstring>

#include "config.h"

namespace {

enum class BridgeRole : uint8_t { Remote = 0, Receiver };

struct RxFrame {
    uint8_t mac[6] = {};
    uint16_t length = 0;
    uint8_t data[config::MAX_ESPNOW_PACKET_SIZE] = {};
};

QueueHandle_t rxQueue = nullptr;
volatile uint32_t rxDroppedCount = 0;

BridgeRole bridgeRole = BridgeRole::Remote;
uint8_t activeChannel = config::DEFAULT_ESPNOW_CHANNEL;
bool espNowReady = false;

uint32_t txFrameCount = 0;
uint32_t rxFrameCount = 0;
uint32_t sendErrorCount = 0;
uint32_t lastStatusAt = 0;

uint8_t streamFrame[config::MAX_ESPNOW_PACKET_SIZE] = {};
size_t streamFrameLength = 0;
uint16_t streamIntervalMs = 0;
uint32_t lastStreamHeartbeatAt = 0;
uint32_t lastStreamSentAt = 0;
bool streamActive = false;

char serialLine[config::SERIAL_LINE_BUFFER_SIZE] = {};
size_t serialLineLength = 0;

const char *roleName(BridgeRole role) {
    return role == BridgeRole::Remote ? "remote" : "receiver";
}

String macToString(const uint8_t *mac) {
    if (mac == nullptr) return String("--");
    char value[18] = {};
    snprintf(value,
             sizeof(value),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0],
             mac[1],
             mac[2],
             mac[3],
             mac[4],
             mac[5]);
    return String(value);
}

void printJson(JsonDocument &document) {
    serializeJson(document, Serial);
    Serial.println();
}

void emitMessage(const char *message) {
    JsonDocument event;
    event["event"] = "message";
    event["message"] = message;
    printJson(event);
}

void emitRadioEvent(const char *direction,
                    const uint8_t *mac,
                    const uint8_t *data,
                    size_t length,
                    bool ok,
                    int errorMask = 0) {
    JsonDocument event;
    event["event"] = "radio";
    event["direction"] = direction;
    event["mac"] = macToString(mac);
    event["len"] = length;
    event["ok"] = ok;
    if (!ok) event["error"] = errorMask;

    char hex[config::MAX_ESPNOW_PACKET_SIZE * 2 + 1] = {};
    const size_t copyLength = min(length, config::MAX_ESPNOW_PACKET_SIZE);
    for (size_t i = 0; i < copyLength; ++i) {
        snprintf(hex + i * 2, 3, "%02X", data[i]);
    }
    event["hex"] = hex;
    printJson(event);
}

void printStatus() {
    JsonDocument status;
    status["event"] = "status";
    status["role"] = roleName(bridgeRole);
    status["channel"] = activeChannel;
    status["mac"] = WiFi.macAddress();
    status["espnow"] = espNowReady;
    status["stream"] = streamActive;
    status["stream_interval_ms"] = streamIntervalMs;
    status["tx_count"] = txFrameCount;
    status["rx_count"] = rxFrameCount;
    status["rx_dropped"] = rxDroppedCount;
    status["errors"] = sendErrorCount;
    printJson(status);
}

bool hexNibble(char value, uint8_t &result) {
    if (value >= '0' && value <= '9') {
        result = static_cast<uint8_t>(value - '0');
        return true;
    }
    if (value >= 'A' && value <= 'F') {
        result = static_cast<uint8_t>(value - 'A' + 10);
        return true;
    }
    if (value >= 'a' && value <= 'f') {
        result = static_cast<uint8_t>(value - 'a' + 10);
        return true;
    }
    return false;
}

bool decodeHex(const char *hex, uint8_t *target, size_t capacity, size_t &length) {
    length = 0;
    if (hex == nullptr) return false;
    const size_t hexLength = strlen(hex);
    if (hexLength == 0 || (hexLength % 2) != 0 || hexLength / 2 > capacity) return false;

    for (size_t i = 0; i < hexLength; i += 2) {
        uint8_t high = 0;
        uint8_t low = 0;
        if (!hexNibble(hex[i], high) || !hexNibble(hex[i + 1], low)) return false;
        target[length++] = static_cast<uint8_t>((high << 4) | low);
    }
    return true;
}

bool sendRawFrame(const uint8_t *data, size_t length) {
    if (data == nullptr || length == 0 || length > config::MAX_ESPNOW_PACKET_SIZE) {
        return false;
    }
    if (!espNowReady || bridgeRole != BridgeRole::Remote) {
        emitRadioEvent("tx",
                       config::BROADCAST_MAC,
                       data,
                       length,
                       false,
                       ESP_ERR_INVALID_STATE);
        return false;
    }

    const esp_err_t result = esp_now_send(config::BROADCAST_MAC, data, length);
    ++txFrameCount;
    if (result != ESP_OK) ++sendErrorCount;
    emitRadioEvent("tx",
                   config::BROADCAST_MAC,
                   data,
                   length,
                   result == ESP_OK,
                   result);
    return result == ESP_OK;
}

void stopRawStream(bool announce) {
    if (streamActive && announce) emitMessage("raw stream stopped");
    streamActive = false;
    streamFrameLength = 0;
    streamIntervalMs = 0;
    lastStreamHeartbeatAt = 0;
    lastStreamSentAt = 0;
}

void updateRawStream(const char *hex, int requestedIntervalMs) {
    if (bridgeRole != BridgeRole::Remote) {
        emitMessage("raw stream rejected in receiver mode");
        return;
    }

    uint8_t decoded[config::MAX_ESPNOW_PACKET_SIZE] = {};
    size_t length = 0;
    if (!decodeHex(hex, decoded, sizeof(decoded), length)) {
        emitMessage("invalid raw frame hex");
        return;
    }

    const bool wasActive = streamActive;
    memcpy(streamFrame, decoded, length);
    streamFrameLength = length;
    streamIntervalMs = static_cast<uint16_t>(constrain(requestedIntervalMs, 1, 60000));
    streamActive = true;
    lastStreamHeartbeatAt = millis();
    if (!wasActive) {
        lastStreamSentAt = 0;
        sendRawFrame(streamFrame, streamFrameLength);
    }
}

bool reconfigureBroadcastPeer() {
    if (esp_now_is_peer_exist(config::BROADCAST_MAC)) {
        esp_now_del_peer(config::BROADCAST_MAC);
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, config::BROADCAST_MAC, sizeof(config::BROADCAST_MAC));
    peer.channel = activeChannel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    return esp_now_add_peer(&peer) == ESP_OK;
}

bool setChannel(int channel) {
    if (channel < 1 || channel > 13) return false;
    if (esp_wifi_set_channel(static_cast<uint8_t>(channel), WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        return false;
    }
    activeChannel = static_cast<uint8_t>(channel);
    return reconfigureBroadcastPeer();
}

void onEspNowReceive(const uint8_t *mac, const uint8_t *incomingData, int length) {
    if (rxQueue == nullptr || mac == nullptr || incomingData == nullptr || length <= 0) return;
    RxFrame frame = {};
    memcpy(frame.mac, mac, 6);
    frame.length = static_cast<uint16_t>(min(
        static_cast<size_t>(length), config::MAX_ESPNOW_PACKET_SIZE));
    memcpy(frame.data, incomingData, frame.length);
    if (xQueueSend(rxQueue, &frame, 0) != pdTRUE) ++rxDroppedCount;
}

bool initializeEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_wifi_set_channel(activeChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        Serial.println("[bridge] Wi-Fi channel setup failed");
        return false;
    }
    if (esp_now_init() != ESP_OK) {
        Serial.println("[bridge] ESP-NOW initialization failed");
        return false;
    }
    rxQueue = xQueueCreate(config::ESPNOW_RX_QUEUE_DEPTH, sizeof(RxFrame));
    if (rxQueue == nullptr) {
        Serial.println("[bridge] RX queue allocation failed");
        esp_now_deinit();
        return false;
    }
    esp_now_register_recv_cb(onEspNowReceive);
    if (!reconfigureBroadcastPeer()) {
        Serial.println("[bridge] failed to add broadcast peer");
        esp_now_unregister_recv_cb();
        vQueueDelete(rxQueue);
        rxQueue = nullptr;
        esp_now_deinit();
        return false;
    }
    return true;
}

void handleSerialCommand(const char *line) {
    JsonDocument document;
    const DeserializationError error = deserializeJson(document, line);
    if (error) {
        emitMessage("invalid CDC JSON");
        return;
    }

    const char *command = document["cmd"] | "";
    if (strcmp(command, "ping") == 0 || strcmp(command, "status") == 0) {
        printStatus();
    } else if (strcmp(command, "set_role") == 0 || strcmp(command, "role") == 0) {
        const char *role = document["role"] | "remote";
        bridgeRole = strcmp(role, "receiver") == 0 ? BridgeRole::Receiver : BridgeRole::Remote;
        if (bridgeRole == BridgeRole::Receiver) stopRawStream(false);
        printStatus();
    } else if (strcmp(command, "set_channel") == 0) {
        const int channel = document["channel"] | activeChannel;
        if (!setChannel(channel)) emitMessage("failed to set Wi-Fi channel");
        printStatus();
    } else if (strcmp(command, "raw_tx") == 0) {
        uint8_t decoded[config::MAX_ESPNOW_PACKET_SIZE] = {};
        size_t length = 0;
        if (!decodeHex(document["hex"] | "", decoded, sizeof(decoded), length)) {
            emitMessage("invalid raw_tx hex");
        } else {
            sendRawFrame(decoded, length);
        }
    } else if (strcmp(command, "raw_stream") == 0) {
        const int interval = document["repeat_ms"] | 10;
        updateRawStream(document["hex"] | "", interval);
    } else if (strcmp(command, "stream_stop") == 0 || strcmp(command, "raw_stop") == 0) {
        stopRawStream(true);
        printStatus();
    } else if (strcmp(command, "reset") == 0) {
        stopRawStream(true);
        printStatus();
    } else {
        emitMessage("unknown CDC command");
    }
}

void pollSerial() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n') {
            serialLine[serialLineLength] = '\0';
            if (serialLineLength > 0) handleSerialCommand(serialLine);
            serialLineLength = 0;
        } else if (c != '\r') {
            if (serialLineLength + 1 < sizeof(serialLine)) {
                serialLine[serialLineLength++] = c;
            } else {
                serialLineLength = 0;
                emitMessage("CDC command too long");
            }
        }
    }
}

void pollEspNowFrames() {
    if (rxQueue == nullptr) return;
    RxFrame frame = {};
    while (xQueueReceive(rxQueue, &frame, 0) == pdTRUE) {
        ++rxFrameCount;
        emitRadioEvent("rx", frame.mac, frame.data, frame.length, true);
    }
}

void serviceRawStream(uint32_t now) {
    if (!streamActive) return;

    if (now - lastStreamHeartbeatAt > config::STREAM_HEARTBEAT_TIMEOUT_MS) {
        stopRawStream(true);
        return;
    }
    if (now - lastStreamSentAt < streamIntervalMs) return;

    lastStreamSentAt = now;
    sendRawFrame(streamFrame, streamFrameLength);
}

}  // namespace

void setup() {
    Serial.begin(config::SERIAL_BAUD_RATE);
    const uint32_t serialWaitStartedAt = millis();
    while (!Serial && millis() - serialWaitStartedAt < 1500) delay(10);

    espNowReady = initializeEspNow();
    Serial.println("[bridge] raw ESP-NOW passthrough ready");
    if (!espNowReady) Serial.println("[bridge] ESP-NOW initialization failed");
    printStatus();
}

void loop() {
    pollSerial();
    pollEspNowFrames();

    const uint32_t now = millis();
    serviceRawStream(now);

    if (now - lastStatusAt >= config::STATUS_INTERVAL_MS) {
        lastStatusAt = now;
        printStatus();
    }
    delay(1);
}
