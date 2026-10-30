# daq-detector

Firmware for the **Audio Detection System**: a portable 6-microphone array that localizes speakers (azimuth, elevation, distance) and counts active speakers.

Current stage (test T0 from the project documentation): **acquisition only**. The firmware captures 6 synchronous channels and
- records them to the microSD card as WAV files,
- streams them to a PC over native USB,
- prints a level / health status line on the UART console.

- exposes a BLE GATT server "AudioLoc" for the Android app (`daq-app`): live diagnostics, device info and control are real, localization results and the SRP map are placeholders until the localization module exists. Protocol: [`docs/ble_gatt.md`](docs/ble_gatt.md).

Localization will be added on top of this pipeline and will publish its output through `ble_publish_result()` / `ble_publish_map()`.

## Hardware

| Module | Part | Connection |
|---|---|---|
| MCU | ESP32-S3-DevKitC-1 N8R8 (2× LX7 240 MHz, 8 MB octal PSRAM, 8 MB flash) | — |
| Microphones | 6× Adafruit 3492 (ST MP34DT01-M, PDM, top port) | I2S0 PDM RX, 1 clock + 3 data lines, 6 SEL GPIOs |
| Storage | Pololu 2597 microSD breakout | SPI2 |
| Power | Waveshare UPS HAT (C), LiPo 1000 mAh, INA219 | 5 V → DevKitC 5V pin (INA219 not used yet) |
| Button | momentary switch to GND | GPIO16 |

Array geometry: triangle M1–M3 (60 mm side) on the body, M4–M6 on three arms (350 mm radius, 120 mm below the body). Exact coordinates are in the project documentation.

### Wiring

| Signal | GPIO | Notes |
|---|---|---|
| PDM CLK | 4 | shared by all 6 mics; 33–100 Ω series resistor per branch (body + 3 arms) |
| PDM DATA0 | 5 | M1 (body, left) + M4 (arm, right) |
| PDM DATA1 | 6 | M2 (body, left) + M5 (arm, right) |
| PDM DATA2 | 7 | M3 (body, left) + M6 (arm, right) |
| SEL M1 / M2 / M3 | 17 / 18 / 8 | body mics, driven low (left slot) |
| SEL M4 / M5 / M6 | 9 / 14 / 21 | arm mics, driven high (right slot) |
| SD SCK | 12 | Pololu SCLK |
| SD MOSI | 11 | Pololu DI |
| SD MISO | 13 | Pololu DO |
| SD CS | 10 | Pololu nCS |
| Button | 16 | to GND, internal pull-up |
| I2C SDA / SCL | 1 / 2 | INA219 on the UPS HAT, reserved |
| LED | 48 | on-board RGB (GPIO38 on DevKitC-1 v1.1), reserved |

Each arm cable has 5 wires: 3V, GND, CLK, DAT, SEL (CLK and DAT twisted with GND).
Microphones and the SD card are powered from the DevKitC 3V3 pin. The Pololu board has no pull-ups: the firmware enables the weak internal ones, external 10 kΩ to 3.3 V on DI, DO, nCS are recommended.

Avoid: GPIO35–37 (octal PSRAM), GPIO0, 3, 45, 46 (strapping), GPIO19–20 (native USB), GPIO43–44 (UART0 console).

## Data flow

```
 MP34DT01-M ×6 ──PDM──▶ I2S0 (HW PDM→PCM, 16 bit) ──DMA──▶ capture task (core 1)
                                                            │ remap to M1..M6, levels
                                       ┌────────────────────┴────────────────────┐
                                       ▼                                         ▼
                           SD ring buffer (512 kB PSRAM)            USB ring buffer (128 kB PSRAM)
                                       │                                         │
                           storage task (core 0)                     stream task (core 0)
                           32 kB fwrite → RECnnnnn.WAV               packets → native USB → PC
```

Each block is 320 frames (6.67 ms at 48 kHz, 3840 bytes). The two ring buffers are independent: if one consumer is too slow, only its blocks are dropped (and counted); the other path is not affected.

## Files

### `CMakeLists.txt`
Standard ESP-IDF project file; project name `daq-detector`.

### `sdkconfig.defaults`
Configuration applied when `sdkconfig` is generated (`sdkconfig` itself is not tracked in git):

| Option | Value | Why |
|---|---|---|
| `CONFIG_IDF_TARGET` | `esp32s3` | target chip |
| `CONFIG_ESPTOOLPY_FLASHSIZE_8MB` | y | N8R8 module has 8 MB flash |
| `CONFIG_SPIRAM`, `CONFIG_SPIRAM_MODE_OCT`, `CONFIG_SPIRAM_SPEED_80M` | y | enables the 8 MB octal PSRAM used for ring buffers |
| `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240` | y | full CPU speed |
| `CONFIG_FREERTOS_HZ` | 1000 | 1 ms tick, finer task timing |
| `CONFIG_ESP_CONSOLE_UART_DEFAULT` | y | console and logs on UART0 (CP2102N port) |
| `CONFIG_ESP_CONSOLE_SECONDARY_NONE` | y | no logs on native USB, so the binary stream stays clean |
| `CONFIG_BT_ENABLED`, `CONFIG_BT_NIMBLE_ENABLED` | y | BLE with the NimBLE host (smaller than Bluedroid) |
| `CONFIG_BT_NIMBLE_MAX_CONNECTIONS` | 1 | one phone at a time |
| `CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU` | 247 | SRP map in 3 notifications instead of ~30 |
| `CONFIG_BT_NIMBLE_ROLE_CENTRAL`, `_OBSERVER` | not set | peripheral only, saves RAM |
| `CONFIG_BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL` | y | NimBLE buffers in PSRAM, internal RAM stays for DMA |
| `CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE` | y | 1.5 MB app partition, the image with BLE does not fit in 1 MB |

After changing this file, delete `sdkconfig` (or run `idf.py fullclean`) so the defaults are applied again.

### `main/CMakeLists.txt`
Registers the `main` component with its five source files.

### `main/config.h`
All tunable settings in one place.

| Setting | Default | Meaning |
|---|---|---|
| `DAQ_SAMPLE_RATE_HZ` | 48000 | sample rate; PDM clock = 64 × fs = 3.072 MHz (mic limit 3.25 MHz). Use 32000 if the clock is unstable on long wires. |
| `DAQ_NUM_CHANNELS` | 6 | number of microphones |
| `DAQ_BLOCK_FRAMES` | 320 | frames per DMA buffer and per read; `frames × 6 × 2 B` must stay below 4092 B |
| `DAQ_DMA_DESC_NUM` | 16 | number of DMA buffers (~107 ms of slack before `drop dma` grows) |
| `DAQ_WARMUP_MS` | 100 | audio discarded after start (mic turn-on time and high-pass filter settling) |
| `DAQ_PIN_PDM_CLK`, `DAQ_PIN_PDM_DIN0..2` | 4, 5, 6, 7 | PDM pins (variant A: 3 data lines) |
| `DAQ_PIN_MIC_SEL` | `{17,18,8,9,14,21}` | SEL pin of M1..M6 |
| `DAQ_MIC_SEL_LEVEL` | `{0,0,0,1,1,1}` | SEL level of M1..M6 (0 = left slot, 1 = right slot), set before the clock starts |
| `DAQ_CHANNEL_MAP` | `{0,2,4,1,3,5}` | raw I2S slot used for M1..M6; **confirm with the tap test** |
| `DAQ_PIN_SD_*` | 12, 11, 13, 10 | SPI pins of the microSD card |
| `DAQ_SD_FREQ_KHZ` | 20000 | SPI clock of the card; lower it (e.g. 10000) if mounting or writes fail |
| `DAQ_SD_MOUNT_POINT` | `/sdcard` | VFS mount point |
| `DAQ_SD_RING_BYTES` | 512 kB | SD ring buffer in PSRAM (~0.9 s of audio, absorbs card write stalls) |
| `DAQ_SD_CHUNK_BYTES` | 32 kB | size of one `fwrite` |
| `DAQ_SD_FILE_SECONDS` | 300 | length of one WAV file before gapless rotation |
| `DAQ_USB_RING_BYTES` | 128 kB | USB ring buffer in PSRAM |
| `DAQ_USB_CHUNK_BYTES` | 4096 | bytes passed to the USB driver per write |
| `DAQ_USB_TX_BUFFER_BYTES` | 16 kB | USB-Serial-JTAG driver TX buffer |
| `DAQ_USB_WRITE_TIMEOUT_MS` | 20 | write timeout when the PC does not read |
| `DAQ_PIN_BUTTON` | 16 | record toggle button (active low) |
| `DAQ_STATUS_PERIOD_MS` | 1000 | period of the console status line |
| `DAQ_PIN_I2C_SDA`, `DAQ_PIN_I2C_SCL`, `DAQ_PIN_LED` | 1, 2, 48 | reserved for INA219 and the RGB LED, not driven yet (reported to the app) |
| `DAQ_FW_VERSION_*` | 0.2.0 | firmware version reported over BLE |
| `DAQ_BLE_NAME_PREFIX` | `AudioLoc-` | advertised name prefix, + last 2 bytes of the MAC |
| `DAQ_BLE_HOP_MS` | 32 | Result notification period |
| `DAQ_BLE_MAX_SOURCES` | 2 | sources per Result frame |
| `DAQ_BLE_MAP_AZ_BINS`, `DAQ_BLE_MAP_EL_BINS` | 36, 18 | SRP map size (10° cells) |
| `DAQ_BLE_MAP_RATE_HZ` | 5 | default map rate, changed from the app |
| `DAQ_BLE_TEST_PATTERN` | 0 | 1 = send one synthetic moving source and its map, to test the app and the link |
| `DAQ_MIC_XYZ_MM` | doc v0.2 | mic coordinates reported to the app; **confirm on the device** |

### `main/daq.h`
Shared definitions used by all modules:
- `daq_state_t g_daq`: global runtime state, i.e. flags (`recording`, `record_request`, `streaming`, `sd_ready`, `ble_connected`), statistics (captured frames, dropped blocks, bytes written/sent, SD errors) and the last per-mic levels (`level_dbfs`).
- `daq_packet_header_t`: header of a USB stream packet (format below).
- `daq_source_t`: one localized source (azimuth, elevation, distance, confidence) passed to the BLE module.
- Function prototypes of `capture.c`, `storage.c`, `stream.c` and `ble.c`.

### `main/capture.c`
Audio acquisition.
- `mic_sel_init()` drives the SEL pin of each mic to its level from `DAQ_MIC_SEL_LEVEL` (before the clock starts).
- `pdm_init()` configures I2S0 as PDM RX master: 3 data lines, left + right slot on each line (6 channels), hardware PDM→PCM conversion to 16-bit with the built-in high-pass filter (35.5 Hz), and a callback that counts DMA overflows.
- `capture_task` (core 1, priority 10) discards the warm-up, then for every block:
  1. reads 320 frames from DMA,
  2. reorders slots to M1..M6 using `DAQ_CHANNEL_MAP`,
  3. accumulates per-channel energy for the level display,
  4. pushes the block to the SD ring buffer (if recording) and to the USB ring buffer with a packet header (if streaming).
  The packet `seq` is incremented for every block, so the PC sees gaps when blocks are dropped.
- `capture_get_levels()` returns the RMS level of each channel in dBFS since the previous call and resets the accumulators.

### `main/storage.c`
Recording to the microSD card.
- `sd_mount()` initialises SPI2, enables internal pull-ups and mounts the FAT card at `/sdcard` (32 kB allocation unit, no formatting).
- `storage_task` (core 0, priority 5):
  - on a record request opens the next free `RECnnnnn.WAV` (8.3 names, numbering continues after existing files) and writes a WAV header with `data_size = 0`,
  - writes 32 kB chunks from the ring buffer,
  - every `DAQ_SD_FILE_SECONDS` closes the file and opens the next one without losing samples,
  - on stop drains the buffer, writes the final header with the correct sizes and closes the file,
  - stops recording on a write error.
- `storage_push()` adds a block to the ring buffer; returns `false` (block dropped) if there is no room.

WAV format: PCM, 6 channels, 48 kHz, 16-bit, interleaved M1..M6. A file interrupted by a power loss keeps `data_size = 0`; `daq_sim.recording.load_recording` reads it using the file size.

### `main/stream.c`
Streaming to the PC over native USB (USB-Serial-JTAG, the DevKitC port labelled USB).
- `stream_init()` installs the USB-Serial-JTAG driver and starts `stream_task` (core 0, priority 6).
- `stream_task` reads host commands (`S` = start, `X` = stop) and sends the ring buffer content in 4 kB writes. If the PC does not read, writes time out and new blocks are dropped (`drop usb`).
- `stream_set_enabled()` starts or stops streaming (also from the console key `u`).
- `stream_push()` adds header + payload atomically, or drops the block if there is no room.

Packet format (little-endian, 24-byte header + payload):

| Field | Type | Meaning |
|---|---|---|
| `magic` | u32 | `0x31514144` ("DAQ1"), used by the host to resynchronise |
| `seq` | u32 | block number, +1 per captured block |
| `first_frame` | u64 | index of the first frame since capture start |
| `sample_rate` | u32 | 48000 |
| `channels` | u16 | 6 |
| `frames` | u16 | 320 |
| payload | i16 × frames × channels | samples interleaved M1..M6 |

### `main/ble.c`
BLE GATT server "AudioLoc" (NimBLE, peripheral). Full protocol in [`docs/ble_gatt.md`](docs/ble_gatt.md).
- `ble_init()` starts NimBLE, registers the service (Result, SRP map, Diagnostics, Device info, Control) and starts `ble_task` (core 0, priority 4). Failure is not fatal: acquisition keeps running.
- Advertises the service UUID, name `AudioLoc-XXXX` in the scan response; re-advertises after a disconnect.
- `ble_task` runs every 32 ms and, for the subscribed characteristics, sends the Result frame, Diagnostics every 1 s, and the SRP map at the rate set by the app (chunked to the negotiated MTU).
- `ble_publish_result()` / `ble_publish_map()`: entry points for the future localization module. A result older than 500 ms is reported as not valid.
- Control writes: start/stop SD recording, start/stop USB stream, SRP map rate, identify (placeholder).
- Placeholders: Result without sources, no map, battery unknown (INA219 not read), identify only logged.

### `main/main.c`
- `app_main()` initialises storage (continues without SD if the card is missing), the USB stream, the capture and BLE (continues without it), then starts the UI task.
- `ui_task` (core 0, priority 3) polls the UART console and the button every 20 ms and prints the status line.

Console keys: `r` start/stop SD recording, `u` start/stop USB stream, `h` help. The button on GPIO16 also toggles recording.

Status line example:
```
t=42s | dBFS M1: -52.3 M2: -51.8 M3: -53.0 M4: -50.9 M5: -52.1 M6: -51.5 | rec=ON  REC00003 12.4MB err=0 | usb=ON  20.1MB | ble=ON  | drop dma=0 sd=0 usb=0
```
- `dBFS`: RMS level of each mic over the last second (≈ −120 means no signal),
- `rec`: recording state, current file, total bytes written, write errors,
- `usb`: stream state and total bytes sent,
- `ble`: phone connected,
- `drop dma / sd / usb`: lost blocks in capture / SD path / USB path; all should stay 0.

## Build and run

Requires ESP-IDF v5.3 or newer.

```
idf.py set-target esp32s3
idf.py build
idf.py -p <UART_PORT> flash monitor
```

Connect both DevKitC ports to the PC:
- **UART port (CP2102N)**: flashing and console (`idf.py monitor`),
- **USB port (native)**: binary stream, read with `daq-sim/tools/usb_record.py`.

## First run checklist

1. **Tap test**: tap M1..M6 one by one, see which level rises and fix `DAQ_CHANNEL_MAP` in `config.h`.
2. **Capture**: `drop dma` stays 0.
3. **SD**: record a few minutes (`r`); `drop sd` and `err` stay 0. If not, lower `DAQ_SD_FREQ_KHZ`, add external pull-ups or move to SDMMC (same wiring).
4. **BLE**: connect with the app, check that Diagnostics shows the same levels as the console and `drop dma` stays 0 with BLE connected. Set `DAQ_BLE_TEST_PATTERN` to 1 to see a moving source in the Live view.
5. **USB**: run the PC script; `drop usb` and `lost` stay 0 at ~576 kB/s. If not, native USB-Serial-JTAG is too slow: switch to TinyUSB CDC or stream fewer channels / a lower rate.

## Related

- `daq-app`: Android app (Kotlin, Jetpack Compose) — live view, diagnostics, device control.
- `daq-sim`: PC tools (USB receiver, recording loader) and offline algorithm development in Python.
