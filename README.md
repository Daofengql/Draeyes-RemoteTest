# Draeyes RemoteTest Firmware

ESP32-S3 USB CDC / ESP-NOW remote and receiver simulator firmware for the
Draeyes Web Console RemoteTest workspace.

```text
Draeyes Web Console
  <--USB CDC / newline-delimited JSON-->
ESP32-S3 RemoteTest firmware
  <--ESP-NOW channel 1-->
RC Mini / C3 / S3 devices
```

The firmware has no ESP-NOW handshake. Receiver mode never sends
`pair_accept`, `link_hello`, `link_ok`, or another acknowledgement.

## Build and flash

```powershell
git clone https://github.com/Daofengql/Draeyes-RemoteTest.git
cd Draeyes-RemoteTest
pio run
pio run --target upload --upload-port COM26
```

The board uses the ESP32-S3 native USB CDC interface. Replace `COM26` when the
assigned port differs.

## Test roles

### Remote simulator

The default role broadcasts the RC Mini protocol on ESP-NOW channel 1:

- controller data every 10 ms
- mode switch, joystick lock, and eyelid link actions
- previous/next eyelid requests (`cp`, `cs`)
- optional pairing requests

### Receiver simulator

The receiver role displays data received from a real remote:

- C3: RAM-only automatic binding to the first sender
- S3: local pairing window with temporary or persistent binding
- TX/RX frame logs, MAC addresses, lengths, and raw JSON

The binding controls are intentionally shown only in receiver mode.

## USB CDC protocol

Commands are UTF-8 JSON followed by `\n`.

```json
{"cmd":"state","x":127,"y":127,"joy":false,"lb":false,"rb":false,"r1":false,"r2":false}
{"cmd":"role","role":"remote","profile":"c3"}
{"cmd":"action","action":"mode_cycle"}
{"cmd":"ping"}
{"cmd":"reset"}
```

Firmware events are also newline-delimited JSON:

- `status`: role, mode, input state, counters, receiver and binding state
- `radio`: TX/RX direction, MAC, length, result, and raw payload
- `message`: system or error message

## ESP-NOW packets

```json
{"req":"controller","data":{"light":1.0,"j1PotX":127,"j1PotY":127,"bkl":1.0,"bkr":1.0}}
{"req":"pairing","type":"remote","name":"Draeye RC Mini (CDC)","channel":1,"nonce":123456}
{"req":"cp"}
{"req":"cs"}
```

## Web Console integration

The browser opens the ESP32-S3 USB CDC port with Web Serial. No desktop serial
tool or Python environment is required. Use the `RemoteTest` workspace at:

```text
/console/RemoteTest
```
