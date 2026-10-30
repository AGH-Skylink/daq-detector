# daq-detector

Firmware for the **Audio Detection System** — a portable 6-microphone array that localizes speakers (azimuth, elevation, distance) and counts active speakers. Results are sent over BLE to a phone app; raw recordings and logs go to a microSD card.

## Hardware

| Module | Part |
|---|---|
| MCU | ESP32-S3-DevKitC-1 N8R8 (2× LX7 240 MHz, 8 MB PSRAM, 8 MB flash, BLE 5) |
| Microphones | 6× Adafruit 3492 (ST MP34DT01-M, PDM) |
| Power | Waveshare UPS HAT (C), LiPo 1000 mAh, INA219 battery monitor (I2C) |
| Storage | Pololu 2597 microSD breakout |

Array geometry: triangle M1–M3 (60 mm side) on the body, M4–M6 on three arms (350 mm radius, 120 mm below the body). Exact coordinates: see the project documentation.

## Planned structure

```
main/          application entry point
components/    audio capture (I2S0 PDM RX), SD logging, BLE GATT "AudioLoc", localization
```

## Build

Requires ESP-IDF (v5.x).

```
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

## Related

- `daq-sim` — offline simulation and algorithm development (Python).
