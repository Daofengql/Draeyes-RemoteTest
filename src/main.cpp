#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <cmath>
#include <cstring>

#include "config.h"

namespace {

enum class BridgeRole : uint8_t { Remote = 0, Receiver };
enum class ReceiverProfile : uint8_t { C3 = 0, S3 };
enum class ControlMode : uint8_t { Manual = 0, AutoFull, AutoBlink };

struct InputState {
    uint8_t x = 127;
    uint8_t y = 127;
    bool joystick = false;
    bool lb = false;
    bool rb = false;
    bool r1 = false;
    bool r2 = false;
};

struct BlinkEffect {
    bool active = false;
    uint32_t startedAt = 0;
    uint32_t durationMs = 0;
    float start = 1.0f;
    float stop = 1.0f;
};

struct RxFrame {
    uint8_t mac[6] = {};
    uint16_t length = 0;
    uint8_t data[config::MAX_ESPNOW_PACKET_SIZE] = {};
};

struct ReceiverCandidate {
    bool pending = false;
    uint8_t mac[6] = {};
    char type[16] = "unknown";
    char name[32] = "device";
};

BridgeRole bridgeRole = BridgeRole::Remote;
ReceiverProfile receiverProfile = ReceiverProfile::C3;

// Mini remote emulator state.
InputState input;
const InputState safeInput;
ControlMode currentMode = ControlMode::Manual;
bool joystickLocked = false;
uint8_t lockedJoystickX = 127;
uint8_t lockedJoystickY = 127;
bool eyelidControlLinked = false;

bool lastR1 = false;
bool lastR2 = false;
bool lastLB = false;
bool lastRB = false;
bool lastEffectLB = false;
bool lastEffectRB = false;
bool lastEffectLinked = false;
uint8_t r1PressCount = 0;
uint32_t lastR1PressAt = 0;
bool r1LongPressTracking = false;
bool r2LongPressTracking = false;
bool r1LongPressHandled = false;
bool r2LongPressHandled = false;
uint32_t r1LongPressStartedAt = 0;
uint32_t r2LongPressStartedAt = 0;

bool physicalPairTracking = false;
bool physicalPairHandled = false;
bool physicalPairStarted = false;
uint32_t physicalPairStartedAt = 0;
bool pairingActive = false;
uint32_t lastPairingSendAt = 0;
uint32_t pairingNonce = 0;

BlinkEffect leftBlink;
BlinkEffect rightBlink;
BlinkEffect linkedBlink;
float currentLeftBlink = 1.0f;
float currentRightBlink = 1.0f;
uint8_t autoBlinkPhase = 0;
uint32_t autoBlinkStartedAt = 0;
uint32_t autoBlinkDurationMs = 70;
uint32_t autoNextBlinkAt = 0;

// C3/S3 receiver emulator state.
bool receiverHasPair = false;
bool receiverPairIsPersistent = false;
uint8_t receiverPairedMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool receiverPairingWindow = false;
uint32_t receiverPairingEndsAt = 0;
ReceiverCandidate receiverCandidate;
uint8_t receiverX = 127;
uint8_t receiverY = 127;
float receiverLeftBlink = 1.0f;
float receiverRightBlink = 1.0f;
float receiverLight = 1.0f;
char receiverLastRequest[24] = "";
char receiverLastAction[24] = "";

QueueHandle_t rxQueue = nullptr;
volatile uint32_t rxDroppedCount = 0;

bool espNowReady = false;
uint32_t lastControlSentAt = 0;
uint32_t lastStatusAt = 0;
uint32_t sendErrorCount = 0;
uint32_t txFrameCount = 0;
uint32_t rxFrameCount = 0;

char serialLine[config::SERIAL_LINE_BUFFER_SIZE] = {};
size_t serialLineLength = 0;
bool serialStateSeen = false;
bool serialFailsafeApplied = false;
uint32_t lastSerialStateAt = 0;

const char *roleName(BridgeRole role) {
    return role == BridgeRole::Remote ? "remote" : "receiver";
}

const char *profileName(ReceiverProfile profile) {
    return profile == ReceiverProfile::C3 ? "c3" : "s3";
}

const char *modeName(ControlMode mode) {
    switch (mode) {
        case ControlMode::Manual: return "manual";
        case ControlMode::AutoFull: return "auto_full";
        case ControlMode::AutoBlink: return "auto_blink";
    }
    return "unknown";
}

bool timeReached(uint32_t now, uint32_t target) {
    return static_cast<int32_t>(now - target) >= 0;
}

void copyText(char *target, size_t targetSize, const char *source) {
    if (target == nullptr || targetSize == 0) return;
    if (source == nullptr) source = "";
    strncpy(target, source, targetSize - 1);
    target[targetSize - 1] = '\0';
}

bool macEquals(const uint8_t *left, const uint8_t *right) {
    return left != nullptr && right != nullptr && memcmp(left, right, 6) == 0;
}

bool macIsInvalid(const uint8_t *mac) {
    if (mac == nullptr) return true;
    bool allZero = true;
    bool allFF = true;
    for (int i = 0; i < 6; ++i) {
        allZero = allZero && mac[i] == 0x00;
        allFF = allFF && mac[i] == 0xFF;
    }
    return allZero || allFF;
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

String optionalMacToString(const uint8_t *mac) {
    return macIsInvalid(mac) ? String("--") : macToString(mac);
}

void printJson(JsonDocument &document) {
    serializeJson(document, Serial);
    Serial.println();
}

bool isPrintablePacket(const uint8_t *data, size_t length) {
    if (data == nullptr) return false;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t value = data[i];
        if (value != '\t' && value != '\r' && value != '\n' &&
            (value < 0x20 || value > 0x7E)) {
            return false;
        }
    }
    return true;
}

void emitRadioLog(const char *direction,
                  const uint8_t *mac,
                  const uint8_t *data,
                  size_t length,
                  bool ok,
                  esp_err_t error = ESP_OK) {
    JsonDocument event;
    event["event"] = "radio";
    event["direction"] = direction;
    event["mac"] = macToString(mac);
    event["len"] = length;
    event["ok"] = ok;
    if (error != ESP_OK) event["error"] = static_cast<int>(error);

    if (data == nullptr) {
        event["data"] = "";
    } else if (isPrintablePacket(data, length)) {
        char text[config::MAX_ESPNOW_PACKET_SIZE + 1] = {};
        const size_t copyLength = min(length, config::MAX_ESPNOW_PACKET_SIZE);
        memcpy(text, data, copyLength);
        text[copyLength] = '\0';
        event["data"] = text;
    } else {
        char hex[config::MAX_ESPNOW_PACKET_SIZE * 2 + 1] = {};
        const size_t copyLength = min(length, config::MAX_ESPNOW_PACKET_SIZE);
        for (size_t i = 0; i < copyLength; ++i) {
            snprintf(hex + i * 2, 3, "%02X", data[i]);
        }
        event["hex"] = hex;
    }
    printJson(event);
}

void emitBridgeMessage(const char *message) {
    JsonDocument event;
    event["event"] = "message";
    event["message"] = message;
    printJson(event);
}

bool addBroadcastPeer() {
    if (esp_now_is_peer_exist(config::RECEIVER_MAC)) {
        esp_now_del_peer(config::RECEIVER_MAC);
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, config::RECEIVER_MAC, sizeof(config::RECEIVER_MAC));
    peer.channel = config::ESPNOW_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    return esp_now_add_peer(&peer) == ESP_OK;
}

void onEspNowReceive(const uint8_t *mac, const uint8_t *incomingData, int length) {
    if (rxQueue == nullptr || mac == nullptr || incomingData == nullptr || length <= 0) {
        return;
    }
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
    if (esp_wifi_set_channel(config::ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
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
    if (!addBroadcastPeer()) {
        Serial.println("[bridge] failed to add broadcast peer");
        esp_now_unregister_recv_cb();
        vQueueDelete(rxQueue);
        rxQueue = nullptr;
        esp_now_deinit();
        return false;
    }
    Serial.printf("[bridge] ESP-NOW ready channel=%u mac=%s\n",
                  config::ESPNOW_CHANNEL,
                  WiFi.macAddress().c_str());
    return true;
}

bool sendDocument(JsonDocument &document) {
    char payload[config::MAX_ESPNOW_PACKET_SIZE + 1] = {};
    const size_t payloadLength = serializeJson(document, payload, sizeof(payload));
    if (payloadLength == 0 || payloadLength > config::MAX_ESPNOW_PACKET_SIZE) {
        Serial.println("[bridge] ESP-NOW JSON payload too large");
        return false;
    }
    if (!espNowReady || bridgeRole != BridgeRole::Remote) {
        emitRadioLog("tx",
                     config::RECEIVER_MAC,
                     reinterpret_cast<const uint8_t *>(payload),
                     payloadLength,
                     false,
                     ESP_ERR_INVALID_STATE);
        return false;
    }

    const esp_err_t result = esp_now_send(
        config::RECEIVER_MAC,
        reinterpret_cast<const uint8_t *>(payload),
        payloadLength);
    ++txFrameCount;
    if (result != ESP_OK) ++sendErrorCount;
    emitRadioLog("tx",
                 config::RECEIVER_MAC,
                 reinterpret_cast<const uint8_t *>(payload),
                 payloadLength,
                 result == ESP_OK,
                 result);
    return result == ESP_OK;
}

bool sendRequest(const char *request) {
    JsonDocument document;
    document["req"] = request;
    if (strcmp(request, "pairing") == 0) {
        document["type"] = config::DEVICE_TYPE;
        document["name"] = config::DEVICE_NAME;
        document["channel"] = config::ESPNOW_CHANNEL;
        document["nonce"] = pairingNonce;
    }
    return sendDocument(document);
}

void stopPairingSession() {
    if (pairingActive) Serial.println("[pairing] stopped");
    pairingActive = false;
    lastPairingSendAt = 0;
}

void startPairingSession() {
    if (bridgeRole != BridgeRole::Remote) {
        emitBridgeMessage("当前为接收端，不能发送 pairing");
        return;
    }
    pairingNonce = esp_random();
    if (pairingNonce == 0) pairingNonce = 1;
    pairingActive = true;
    lastPairingSendAt = millis();
    sendRequest("pairing");
    Serial.printf("[pairing] started nonce=%lu retry=%lums\n",
                  static_cast<unsigned long>(pairingNonce),
                  static_cast<unsigned long>(config::PAIRING_RETRY_INTERVAL_MS));
}

void servicePairing(uint32_t now) {
    if (bridgeRole != BridgeRole::Remote || !pairingActive) return;
    if (now - lastPairingSendAt >= config::PAIRING_RETRY_INTERVAL_MS) {
        lastPairingSendAt = now;
        sendRequest("pairing");
    }
}

void startBlink(BlinkEffect &effect, bool pressed, uint32_t durationMs, uint32_t now) {
    effect.active = true;
    effect.startedAt = now;
    effect.durationMs = durationMs;
    effect.start = pressed ? 1.0f : 0.0f;
    effect.stop = pressed ? 0.0f : 1.0f;
}

float updateBlink(BlinkEffect &effect, float current, uint32_t now) {
    if (!effect.active) return current;
    if (effect.durationMs == 0 || now - effect.startedAt >= effect.durationMs) {
        effect.active = false;
        return effect.stop;
    }
    const float progress = static_cast<float>(now - effect.startedAt) /
                           static_cast<float>(effect.durationMs);
    return effect.start + (effect.stop - effect.start) * progress;
}

void updateManualEyelids(uint32_t now) {
    if (currentMode != ControlMode::Manual || input.joystick) {
        lastEffectLB = false;
        lastEffectRB = false;
        lastEffectLinked = false;
        return;
    }
    if (eyelidControlLinked) {
        if (input.lb != lastEffectLinked) {
            startBlink(linkedBlink, input.lb, input.lb ? 70 : 80, now);
        }
        lastEffectLinked = input.lb;
        lastEffectLB = false;
        lastEffectRB = false;
        currentLeftBlink = updateBlink(linkedBlink, currentLeftBlink, now);
        currentRightBlink = currentLeftBlink;
        return;
    }
    if (input.lb != lastEffectLB) {
        startBlink(leftBlink, input.lb, input.lb ? 70 : 80, now);
    }
    if (input.rb != lastEffectRB) {
        startBlink(rightBlink, input.rb, input.rb ? 40 : 80, now);
    }
    lastEffectLB = input.lb;
    lastEffectRB = input.rb;
    lastEffectLinked = false;
    currentLeftBlink = updateBlink(leftBlink, currentLeftBlink, now);
    currentRightBlink = updateBlink(rightBlink, currentRightBlink, now);
}

float updateAutomaticBlink(uint32_t now) {
    if (autoBlinkPhase == 0) {
        if (autoNextBlinkAt == 0) autoNextBlinkAt = now + 2500;
        if (timeReached(now, autoNextBlinkAt)) {
            autoBlinkPhase = 1;
            autoBlinkStartedAt = now;
            autoBlinkDurationMs = static_cast<uint32_t>(random(50, 101));
        }
    }
    if (autoBlinkPhase == 1) {
        if (now - autoBlinkStartedAt >= autoBlinkDurationMs) {
            autoBlinkPhase = 2;
            autoBlinkStartedAt = now;
        }
        const float progress = min(
            1.0f,
            static_cast<float>(now - autoBlinkStartedAt) /
                static_cast<float>(autoBlinkDurationMs));
        return 1.0f - progress;
    }
    if (autoBlinkPhase == 2) {
        const uint32_t openingDuration = autoBlinkDurationMs * 2U;
        if (now - autoBlinkStartedAt >= openingDuration) {
            autoBlinkPhase = 0;
            autoNextBlinkAt = now + autoBlinkDurationMs * 3U +
                              static_cast<uint32_t>(random(2000, 6001));
            return 1.0f;
        }
        return min(1.0f,
                   static_cast<float>(now - autoBlinkStartedAt) /
                       static_cast<float>(openingDuration));
    }
    return 1.0f;
}

void getJoystickOutput(uint32_t now, uint8_t &x, uint8_t &y) {
    if (currentMode == ControlMode::AutoFull) {
        const float phaseX = static_cast<float>(now % 3200U) / 3200.0f;
        const float phaseY = static_cast<float>((now + 900U) % 2600U) / 2600.0f;
        x = static_cast<uint8_t>(constrain(
            static_cast<int>(127.0f + sinf(phaseX * 2.0f * PI) * 60.0f), 0, 255));
        y = static_cast<uint8_t>(constrain(
            static_cast<int>(127.0f + sinf(phaseY * 2.0f * PI) * 45.0f), 0, 255));
    } else if (joystickLocked) {
        x = lockedJoystickX;
        y = lockedJoystickY;
    } else {
        x = input.x;
        y = input.y;
    }
}

void sendController(uint32_t now) {
    uint8_t x = 127;
    uint8_t y = 127;
    getJoystickOutput(now, x, y);
    float left = currentLeftBlink;
    float right = currentRightBlink;
    if (currentMode != ControlMode::Manual) {
        const float open = updateAutomaticBlink(now);
        left = open;
        right = open;
    }
    JsonDocument document;
    document["req"] = "controller";
    JsonObject data = document["data"].to<JsonObject>();
    data["light"] = config::DEFAULT_LIGHT_VALUE;
    data["j1PotX"] = x;
    data["j1PotY"] = y;
    data["bkl"] = left;
    data["bkr"] = right;
    sendDocument(document);
}

void cycleMode() {
    if (currentMode == ControlMode::Manual) currentMode = ControlMode::AutoFull;
    else if (currentMode == ControlMode::AutoFull) currentMode = ControlMode::AutoBlink;
    else currentMode = ControlMode::Manual;
    Serial.printf("[mini] mode=%s\n", modeName(currentMode));
}

void toggleJoystickLock(uint32_t now) {
    if (currentMode != ControlMode::Manual && currentMode != ControlMode::AutoBlink) {
        Serial.println("[mini] joystick lock unavailable in full-auto mode");
        return;
    }
    if (joystickLocked) {
        joystickLocked = false;
        Serial.println("[mini] joystick unlocked");
    } else {
        getJoystickOutput(now, lockedJoystickX, lockedJoystickY);
        joystickLocked = true;
        Serial.printf("[mini] joystick locked at %u,%u\n", lockedJoystickX, lockedJoystickY);
    }
}

void toggleEyelidLink() {
    eyelidControlLinked = !eyelidControlLinked;
    if (eyelidControlLinked) {
        currentRightBlink = currentLeftBlink;
        Serial.println("[mini] eyelids linked");
    } else {
        currentLeftBlink = currentRightBlink;
        Serial.println("[mini] eyelids independent");
    }
}

void processRawGestures(uint32_t now) {
    const bool allFour = input.r1 && input.r2 && input.lb && input.rb;
    const bool pairingPressed = input.r1 && input.r2 && !input.lb && !input.rb;

    if (pairingPressed) {
        if (!physicalPairTracking) {
            physicalPairTracking = true;
            physicalPairStartedAt = now;
            physicalPairHandled = false;
        } else if (!physicalPairHandled && now - physicalPairStartedAt >= config::PAIRING_LONG_PRESS_MS) {
            physicalPairHandled = true;
            physicalPairStarted = true;
            startPairingSession();
        }
    } else {
        physicalPairTracking = false;
        physicalPairHandled = false;
        if (physicalPairStarted) {
            physicalPairStarted = false;
            stopPairingSession();
        }
    }

    if (!allFour && !pairingPressed && input.joystick) {
        if (input.lb && !lastLB) sendRequest("cp");
        if (input.rb && !lastRB) sendRequest("cs");
    }

    if (!allFour && !pairingPressed && input.r1 && !input.r2) {
        if (!r1LongPressTracking) {
            r1LongPressTracking = true;
            r1LongPressStartedAt = now;
            r1LongPressHandled = false;
        } else if (!r1LongPressHandled && now - r1LongPressStartedAt >= config::PAIRING_LONG_PRESS_MS) {
            toggleJoystickLock(now);
            r1LongPressHandled = true;
        }
    } else {
        r1LongPressTracking = false;
        r1LongPressHandled = false;
    }

    if (!allFour && !pairingPressed && input.r2 && !input.r1) {
        if (!r2LongPressTracking) {
            r2LongPressTracking = true;
            r2LongPressStartedAt = now;
            r2LongPressHandled = false;
        } else if (!r2LongPressHandled && now - r2LongPressStartedAt >= config::PAIRING_LONG_PRESS_MS) {
            toggleEyelidLink();
            r2LongPressHandled = true;
        }
    } else {
        r2LongPressTracking = false;
        r2LongPressHandled = false;
    }

    if (input.r1 && !lastR1 && !input.r2) {
        r1PressCount = (r1PressCount > 0 && now - lastR1PressAt <= config::MULTI_PRESS_WINDOW_MS)
            ? static_cast<uint8_t>(r1PressCount + 1)
            : 1;
        lastR1PressAt = now;
        if (r1PressCount >= 3) {
            cycleMode();
            r1PressCount = 0;
        }
    }

    lastR1 = input.r1;
    lastR2 = input.r2;
    lastLB = input.lb;
    lastRB = input.rb;
}

void updateInputFromDocument(JsonDocument &document) {
    if (document["x"].is<int>()) input.x = static_cast<uint8_t>(constrain(document["x"].as<int>(), 0, 255));
    if (document["y"].is<int>()) input.y = static_cast<uint8_t>(constrain(document["y"].as<int>(), 0, 255));
    if (document["joy"].is<bool>() || document["joy"].is<int>()) input.joystick = document["joy"].as<bool>();
    if (document["lb"].is<bool>() || document["lb"].is<int>()) input.lb = document["lb"].as<bool>();
    if (document["rb"].is<bool>() || document["rb"].is<int>()) input.rb = document["rb"].as<bool>();
    if (document["r1"].is<bool>() || document["r1"].is<int>()) input.r1 = document["r1"].as<bool>();
    if (document["r2"].is<bool>() || document["r2"].is<int>()) input.r2 = document["r2"].as<bool>();
    lastSerialStateAt = millis();
    serialStateSeen = true;
    serialFailsafeApplied = false;
}

// ------------------------------- Receiver ----------------------------------
void clearReceiverCandidate() {
    receiverCandidate = ReceiverCandidate();
}

void clearReceiverRuntimePairing() {
    receiverHasPair = false;
    receiverPairIsPersistent = false;
    memset(receiverPairedMac, 0xFF, sizeof(receiverPairedMac));
}

void loadPersistentReceiverPairing() {
    clearReceiverRuntimePairing();
    if (receiverProfile != ReceiverProfile::S3) return;
    Preferences preferences;
    if (!preferences.begin("rxsim", true)) return;
    if (preferences.getBool("valid", false) && preferences.getBytesLength("mac") == 6) {
        preferences.getBytes("mac", receiverPairedMac, 6);
        receiverHasPair = !macIsInvalid(receiverPairedMac);
        receiverPairIsPersistent = receiverHasPair;
    }
    preferences.end();
}

void savePersistentReceiverPairing() {
    Preferences preferences;
    if (!preferences.begin("rxsim", false)) return;
    preferences.clear();
    if (receiverHasPair && receiverPairIsPersistent) {
        preferences.putBool("valid", true);
        preferences.putBytes("mac", receiverPairedMac, 6);
    }
    preferences.end();
}

void resetReceiverStateForProfile(ReceiverProfile profile) {
    receiverProfile = profile;
    receiverPairingWindow = false;
    receiverPairingEndsAt = 0;
    clearReceiverCandidate();
    clearReceiverRuntimePairing();
    receiverX = 127;
    receiverY = 127;
    receiverLeftBlink = 1.0f;
    receiverRightBlink = 1.0f;
    receiverLight = 1.0f;
    receiverLastRequest[0] = '\0';
    receiverLastAction[0] = '\0';
    loadPersistentReceiverPairing();
}

void bindReceiverMac(const uint8_t *mac, bool persistent) {
    if (mac == nullptr || macIsInvalid(mac)) return;
    memcpy(receiverPairedMac, mac, 6);
    receiverHasPair = true;
    receiverPairIsPersistent = persistent && receiverProfile == ReceiverProfile::S3;
    clearReceiverCandidate();
    receiverPairingWindow = false;
    receiverPairingEndsAt = 0;
    if (receiverPairIsPersistent) savePersistentReceiverPairing();
    Serial.printf("[receiver] bound mac=%s persistence=%s profile=%s\n",
                  macToString(mac).c_str(),
                  receiverPairIsPersistent ? "permanent" : "runtime",
                  profileName(receiverProfile));
}

void clearReceiverBinding(bool clearPersistentStorage) {
    clearReceiverRuntimePairing();
    clearReceiverCandidate();
    if (clearPersistentStorage) {
        Preferences preferences;
        if (preferences.begin("rxsim", false)) {
            preferences.clear();
            preferences.end();
        }
    }
    Serial.printf("[receiver] binding cleared storage=%s\n",
                  clearPersistentStorage ? "yes" : "no");
}

void openReceiverPairingWindow() {
    if (receiverProfile != ReceiverProfile::S3) {
        emitBridgeMessage("C3 模拟器自动绑定首个 pairing，不需要配对窗口");
        return;
    }
    receiverPairingWindow = true;
    receiverPairingEndsAt = millis() + config::RECEIVER_PAIRING_TIMEOUT_MS;
    clearReceiverCandidate();
    Serial.println("[receiver] S3 pairing window opened");
}

void closeReceiverPairingWindow() {
    receiverPairingWindow = false;
    receiverPairingEndsAt = 0;
    clearReceiverCandidate();
    Serial.println("[receiver] pairing window closed");
}

void acceptReceiverCandidate(bool persistent) {
    if (!receiverCandidate.pending) {
        emitBridgeMessage("当前没有待确认 pairing");
        return;
    }
    bindReceiverMac(receiverCandidate.mac, persistent);
}

void serviceReceiverPairingWindow(uint32_t now) {
    if (receiverPairingWindow && timeReached(now, receiverPairingEndsAt)) {
        closeReceiverPairingWindow();
    }
}

void handleReceiverPacket(const uint8_t *mac, JsonDocument &document) {
    const char *request = document["req"] | "";
    if (request[0] == '\0') return;

    if (strcmp(request, "pairing") == 0) {
        if (receiverProfile == ReceiverProfile::C3) {
            // C3: first pairing is an immediate RAM-only binding. No response
            // or handshake is sent back to the Mini.
            if (!receiverHasPair) bindReceiverMac(mac, false);
            else if (!macEquals(mac, receiverPairedMac)) {
                Serial.printf("[receiver] C3 pairing ignored from %s; already bound to %s\n",
                              macToString(mac).c_str(),
                              macToString(receiverPairedMac).c_str());
            }
            return;
        }

        // S3: pairing is a local candidate only while its UI window is open.
        // Accepting it locally still sends no ESP-NOW response.
        if (!receiverPairingWindow) {
            Serial.printf("[receiver] S3 pairing ignored outside window from %s\n",
                          macToString(mac).c_str());
            return;
        }
        receiverCandidate.pending = true;
        memcpy(receiverCandidate.mac, mac, 6);
        copyText(receiverCandidate.type, sizeof(receiverCandidate.type), document["type"] | "unknown");
        copyText(receiverCandidate.name, sizeof(receiverCandidate.name), document["name"] | "device");
        Serial.printf("[receiver] S3 pairing candidate mac=%s type=%s name=%s\n",
                      macToString(mac).c_str(),
                      receiverCandidate.type,
                      receiverCandidate.name);
        return;
    }

    if (!receiverHasPair || !macEquals(mac, receiverPairedMac)) {
        Serial.printf("[receiver] %s ignored unauthorized mac=%s\n",
                      request,
                      macToString(mac).c_str());
        return;
    }

    copyText(receiverLastRequest, sizeof(receiverLastRequest), request);
    if (strcmp(request, "controller") == 0) {
        JsonObject data = document["data"];
        if (data["j1PotX"].is<int>() || data["j1PotX"].is<float>()) {
            receiverX = static_cast<uint8_t>(constrain(data["j1PotX"].as<int>(), 0, 255));
        }
        if (data["j1PotY"].is<int>() || data["j1PotY"].is<float>()) {
            receiverY = static_cast<uint8_t>(constrain(data["j1PotY"].as<int>(), 0, 255));
        }
        if (data["light"].is<int>() || data["light"].is<float>()) receiverLight = constrain(data["light"].as<float>(), 0.0f, 1.0f);
        if (data["bkl"].is<int>() || data["bkl"].is<float>()) receiverLeftBlink = constrain(data["bkl"].as<float>(), 0.0f, 1.0f);
        if (data["bkr"].is<int>() || data["bkr"].is<float>()) receiverRightBlink = constrain(data["bkr"].as<float>(), 0.0f, 1.0f);
        copyText(receiverLastAction, sizeof(receiverLastAction), "controller");
    } else {
        copyText(receiverLastAction, sizeof(receiverLastAction), request);
    }
}

void pollEspNowFrames() {
    if (rxQueue == nullptr) return;
    RxFrame frame = {};
    while (xQueueReceive(rxQueue, &frame, 0) == pdTRUE) {
        ++rxFrameCount;
        JsonDocument document;
        const DeserializationError error = deserializeJson(document, frame.data, frame.length);
        emitRadioLog("rx", frame.mac, frame.data, frame.length, !error,
                     error ? ESP_ERR_INVALID_ARG : ESP_OK);
        if (!error) handleReceiverPacket(frame.mac, document);
        else Serial.printf("[receiver] RX JSON parse failed len=%u error=%s\n",
                           frame.length, error.c_str());
    }
}

// ------------------------------- CDC ----------------------------------------
void printStatus() {
    JsonDocument status;
    status["event"] = "status";
    status["role"] = roleName(bridgeRole);
    status["profile"] = profileName(receiverProfile);
    status["espnow"] = espNowReady;
    status["channel"] = config::ESPNOW_CHANNEL;
    status["mac"] = WiFi.macAddress();
    status["mode"] = modeName(currentMode);
    status["x"] = input.x;
    status["y"] = input.y;
    status["pairing"] = pairingActive;
    status["linked"] = eyelidControlLinked;
    status["locked"] = joystickLocked;
    status["errors"] = sendErrorCount;
    status["tx_count"] = txFrameCount;
    status["rx_count"] = rxFrameCount;
    status["rx_dropped"] = rxDroppedCount;
    status["receiver_paired"] = receiverHasPair;
    status["receiver_persistent"] = receiverPairIsPersistent;
    status["receiver_mac"] = optionalMacToString(receiverPairedMac);
    status["receiver_pairing_window"] = receiverPairingWindow;
    status["receiver_candidate"] = receiverCandidate.pending;
    status["receiver_candidate_mac"] = optionalMacToString(receiverCandidate.mac);
    status["receiver_candidate_type"] = receiverCandidate.type;
    status["receiver_candidate_name"] = receiverCandidate.name;
    status["receiver_x"] = receiverX;
    status["receiver_y"] = receiverY;
    status["receiver_bkl"] = receiverLeftBlink;
    status["receiver_bkr"] = receiverRightBlink;
    status["receiver_last_request"] = receiverLastRequest;
    status["receiver_last_action"] = receiverLastAction;
    printJson(status);
}

void setBridgeRole(const char *role, const char *profile) {
    stopPairingSession();
    bridgeRole = strcmp(role, "receiver") == 0 ? BridgeRole::Receiver : BridgeRole::Remote;
    receiverProfile = strcmp(profile, "s3") == 0 ? ReceiverProfile::S3 : ReceiverProfile::C3;
    resetReceiverStateForProfile(receiverProfile);
    Serial.printf("[bridge] role=%s profile=%s\n", roleName(bridgeRole), profileName(receiverProfile));
}

void resetRemoteInput() {
    input = InputState();
    lastR1 = false;
    lastR2 = false;
    lastLB = false;
    lastRB = false;
    physicalPairTracking = false;
    physicalPairHandled = false;
    physicalPairStarted = false;
    stopPairingSession();
    serialStateSeen = false;
    serialFailsafeApplied = false;
}

void handleDirectAction(const char *action) {
    const uint32_t now = millis();
    if (strcmp(action, "mode_cycle") == 0) cycleMode();
    else if (strcmp(action, "joystick_lock_toggle") == 0) toggleJoystickLock(now);
    else if (strcmp(action, "eyelid_link_toggle") == 0) toggleEyelidLink();
    else if (strcmp(action, "cp") == 0 || strcmp(action, "cs") == 0) sendRequest(action);
    else if (strcmp(action, "pairing_start") == 0) startPairingSession();
    else if (strcmp(action, "pairing_stop") == 0) stopPairingSession();
    else if (strcmp(action, "pairing_once") == 0) {
        pairingNonce = esp_random();
        if (pairingNonce == 0) pairingNonce = 1;
        sendRequest("pairing");
    } else if (strcmp(action, "receiver_pairing_open") == 0) openReceiverPairingWindow();
    else if (strcmp(action, "receiver_pairing_close") == 0) closeReceiverPairingWindow();
    else if (strcmp(action, "receiver_accept_temp") == 0) acceptReceiverCandidate(false);
    else if (strcmp(action, "receiver_accept_perm") == 0) acceptReceiverCandidate(true);
    else if (strcmp(action, "receiver_reject") == 0) {
        clearReceiverCandidate();
        Serial.println("[receiver] pairing candidate rejected");
    } else if (strcmp(action, "receiver_clear_runtime") == 0) clearReceiverBinding(false);
    else if (strcmp(action, "receiver_clear_all") == 0) clearReceiverBinding(true);
    else emitBridgeMessage("未知的直接操作");
}

void handleSerialCommand(const char *line) {
    JsonDocument document;
    const DeserializationError error = deserializeJson(document, line);
    if (error) {
        Serial.printf("[bridge] bad CDC JSON: %s\n", error.c_str());
        return;
    }
    const char *command = document["cmd"] | "";
    if (strcmp(command, "state") == 0) {
        updateInputFromDocument(document);
    } else if (strcmp(command, "role") == 0 || strcmp(command, "set_role") == 0) {
        setBridgeRole(document["role"] | "remote", document["profile"] | "c3");
        printStatus();
    } else if (strcmp(command, "action") == 0) {
        handleDirectAction(document["action"] | "");
        printStatus();
    } else if (strcmp(command, "ping") == 0 || strcmp(command, "status") == 0) {
        printStatus();
    } else if (strcmp(command, "reset") == 0) {
        resetRemoteInput();
        Serial.println("[bridge] remote input reset");
    } else if (strcmp(command, "pair_once") == 0) {
        pairingNonce = esp_random();
        if (pairingNonce == 0) pairingNonce = 1;
        sendRequest("pairing");
    } else {
        emitBridgeMessage("未知 CDC 命令");
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
            if (serialLineLength + 1 < sizeof(serialLine)) serialLine[serialLineLength++] = c;
            else {
                serialLineLength = 0;
                Serial.println("[bridge] CDC command too long");
            }
        }
    }
}

void applySerialFailsafe(uint32_t now) {
    if (!serialStateSeen || serialFailsafeApplied ||
        now - lastSerialStateAt <= config::RUNTIME_STATE_TIMEOUT_MS) return;
    input = safeInput;
    serialFailsafeApplied = true;
    Serial.println("[bridge] CDC state timeout; inputs released and centered");
}

}  // namespace

void setup() {
    Serial.begin(config::SERIAL_BAUD_RATE);
    const uint32_t serialWaitStartedAt = millis();
    while (!Serial && millis() - serialWaitStartedAt < 1500) delay(10);
    randomSeed(esp_random());
    espNowReady = initializeEspNow();
    autoNextBlinkAt = millis() + 2500;
    resetReceiverStateForProfile(ReceiverProfile::C3);
    Serial.println("[bridge] Mini remote / C3-S3 receiver simulator ready");
    Serial.println("[bridge] default role=remote; no ESP-NOW handshake is used");
    printStatus();
}

void loop() {
    pollSerial();
    pollEspNowFrames();
    const uint32_t now = millis();
    serviceReceiverPairingWindow(now);

    if (bridgeRole == BridgeRole::Remote) {
        applySerialFailsafe(now);
        processRawGestures(now);
        updateManualEyelids(now);
        servicePairing(now);
        if (espNowReady && now - lastControlSentAt >= config::CONTROL_INTERVAL_MS) {
            lastControlSentAt = now;
            sendController(now);
        }
    }

    if (now - lastStatusAt >= config::STATUS_INTERVAL_MS) {
        lastStatusAt = now;
        printStatus();
    }
    delay(1);
}
