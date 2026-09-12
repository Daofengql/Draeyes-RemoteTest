# Draeyes RemoteTest Firmware

ESP32-S3 USB CDC to ESP-NOW raw-frame transport for Draeyes Web Console.

The firmware is a transparent payload bridge. It does not understand remote
controller models, binding rules, button gestures, or RC Mini packet fields.
All protocol behavior lives in the Web Console RemoteTest model files.

## Architecture

```text
Web Console model
  -> raw ESP-NOW payload bytes
  -> USB CDC JSON control frame
ESP32-S3 RemoteTest firmware
  -> esp_now_send

ESP-NOW payload
  -> USB CDC radio event with hexadecimal payload
Web Console model
  -> parse and apply model-specific behavior
```

## Build and flash

```powershell
git clone https://github.com/Daofengql/Draeyes-RemoteTest.git
cd Draeyes-RemoteTest
pio run
pio run --target upload --upload-port COM26
```

Replace `COM26` with the ESP32-S3 native USB CDC port.

## CDC control protocol

Control messages are UTF-8 JSON followed by `\n`. Raw payload bytes are encoded
as hexadecimal strings.

```json
{"cmd":"ping"}
{"cmd":"set_role","role":"remote"}
{"cmd":"set_role","role":"receiver"}
{"cmd":"set_channel","channel":1}
{"cmd":"raw_tx","hex":"7B22726571223A226370227D"}
{"cmd":"raw_stream","hex":"7B22726571223A22636F6E74726F6C6C6572227D","repeat_ms":10}
{"cmd":"stream_stop"}
```

`raw_stream` stores the latest raw frame and repeats it with the ESP32 hardware
timer. The Web Console refreshes the frame when model state changes. If the Web
Console stops refreshing for 1.5 seconds, streaming stops automatically.

## CDC events

Every event is newline-delimited JSON.

```json
{"event":"status","role":"remote","channel":1,"mac":"30:ED:A0:BE:55:58","espnow":true,"stream":false,"stream_interval_ms":0,"tx_count":0,"rx_count":0,"rx_dropped":0,"errors":0}
{"event":"radio","direction":"rx","mac":"AA:BB:CC:DD:EE:FF","len":3,"ok":true,"hex":"7B7D"}
{"event":"radio","direction":"tx","mac":"FF:FF:FF:FF:FF:FF","len":3,"ok":true,"hex":"7B7D"}
{"event":"message","message":"raw stream stopped"}
```

The `radio` event always carries the complete payload as hexadecimal. The Web
Console converts it back to bytes and lets the selected model parse it.

## Limits

- ESP-NOW payload maximum: 250 bytes.
- Remote mode sends to broadcast MAC on the configured Wi-Fi channel.
- Receiver mode forwards every received frame to the CDC connection.
- Binding and filtering are entirely controlled by the Web Console.
