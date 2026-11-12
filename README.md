# daq-detector

Firmware for the **Audio Detection System**: a portable 6-microphone array that localizes speakers (azimuth, elevation, distance) and counts active speakers.

Current stage (test T0 from the project documentation): **acquisition only**. The firmware captures 6 synchronous channels and
- records them to the microSD card as WAV files,
- streams them to a PC over native USB,
- prints a level / health status line on the UART console.

- exposes a BLE GATT server "AudioLoc" for the Android app (`daq-app`): live diagnostics (levels, drops, battery, CPU load), device info, control, tracked speakers, per-frame sources and the SRP map. Protocol v3: [`docs/ble_gatt.md`](docs/ble_gatt.md),
- monitors the battery (INA219 on the UPS HAT), shows the state on the on-board RGB LED and keeps settings (mic geometry, speed of sound, algorithm, map rate, tracker mode, BLE outputs) in NVS,
- localizes speakers (firmware 0.6.0) with algorithm A (GCC-PHAT + least squares) or B (SRP-PHAT, up to 2 sources, power map) and sends
  the result to the app over BLE; when the fit is poor only the direction is reported (distance unknown),
- localizes with algorithm C (firmware 0.8.0): broadband MUSIC on the small triangle M1–M3, azimuth only, up to 2 sources, count from MDL,
- counts and follows speakers over time (firmware 0.7.0): direction histogram + tracker with three selectable association modes
  (azimuth only, azimuth × elevation, elliptical gate); BLE sends the tracked speakers and/or the per-frame sources (each switchable),
- runs the DSP front end (firmware 0.4.0): 32 ms frames, voice activity detection and GCC-PHAT time differences for the 15 mic pairs, logged to CSV next to the WAV; a replay mode runs a WAV from the card through the same code for comparison with the Python reference in `daq-sim`.

Development plan: [`docs/firmware_plan.md`](docs/firmware_plan.md).


## Hardware

| Module | Part | Connection |
|---|---|---|
| MCU | ESP32-S3-DevKitC-1 N8R8 (2× LX7 240 MHz, 8 MB octal PSRAM, 8 MB flash) | — |
| Microphones | 6× Adafruit 3492 (ST MP34DT01-M, PDM, top port) | I2S0 PDM RX, 1 clock + 3 data lines, 6 SEL GPIOs |
| Storage | Pololu 2597 microSD breakout | SPI2 |
| Power | Waveshare UPS HAT (C), LiPo 1000 mAh, INA219 | 5 V → DevKitC 5V pin, INA219 on I2C (0x43) |
| Button | momentary switch to GND | GPIO16 |

Array geometry (pinout DAQ-PIN-01 is the reference): triangle M1–M3 (60 mm side) at 90°/210°/330°, arms M4/M5/M6 at 150°/270°/30°, radius 350 mm, 161 mm below the M1–M3 plane. Default coordinates are in `config.h` (`DAQ_MIC_XYZ_MM`); measured ones are set from the console and kept in NVS.

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
| I2C SDA / SCL | 1 / 2 | INA219 on the UPS HAT (HAT connector pins 3 / 5) |
| LED | 48 | on-board RGB (GPIO38 on DevKitC-1 v1.1) |

**UPS HAT connector:** the HAT is mirrored compared to the Raspberry Pi numbering used in the pinout. Before wiring, measure which pin carries 5 V with the HAT switched on; 5 V on the 3V3 line destroys the ESP32, mics and card.

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
ESP-IDF project file; project name `daq-detector`. `MINIMAL_BUILD ON` builds only the components that `main` requires
(faster, and it skips `esp_lcd`, whose RGB panel driver crashes GCC 15.2 of ESP-IDF v6.1 at `-Og`).

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
| `CONFIG_FREERTOS_USE_TRACE_FACILITY`, `CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS` | y | idle task run time per core, used for the CPU load |
| `CONFIG_COMPILER_OPTIMIZATION_PERF` | y | `-O2`: with the default `-Og` a DSP frame takes longer than 32 ms (`drop dsp` grows) |
| `CONFIG_BT_NIMBLE_LOG_LEVEL_WARNING` | y | NimBLE logs every notification at INFO (31 Hz × 2 lines) and floods the console |

After changing this file, delete `sdkconfig` (or run `idf.py fullclean`) so the defaults are applied again.

### `main/CMakeLists.txt`
Registers the `main` component with its ten source files and the ESP-IDF components it uses (`PRIV_REQUIRES`: GPIO, I2C, I2S,
SPI, SD over SPI, USB-Serial-JTAG, timer, FATFS, SDMMC, BT, NVS); with `MINIMAL_BUILD` nothing else is built.

### `main/idf_component.yml`
Managed dependencies, downloaded by `idf.py` on the first build: `espressif/led_strip` (^3.0.0, WS2812 LED) and `espressif/esp-dsp` (^1.4.0, FFT).

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
| `DAQ_SD_CSV_RING_BYTES` | 32 kB | PSRAM buffer for CSV log lines from the DSP (~3 s) |
| `DAQ_USB_RING_BYTES` | 128 kB | USB ring buffer in PSRAM |
| `DAQ_USB_CHUNK_BYTES` | 4096 | bytes passed to the USB driver per write |
| `DAQ_USB_TX_BUFFER_BYTES` | 16 kB | USB-Serial-JTAG driver TX buffer |
| `DAQ_USB_WRITE_TIMEOUT_MS` | 20 | write timeout when the PC does not read |
| `DAQ_PIN_BUTTON` | 16 | record toggle button (active low) |
| `DAQ_STATUS_PERIOD_MS` | 1000 | period of the console status line |
| `DAQ_PIN_I2C_SDA`, `DAQ_PIN_I2C_SCL` | 1, 2 | I2C bus of the INA219 |
| `DAQ_INA219_ADDR` | 0x43 | INA219 address on the UPS HAT (confirm with an I2C scan) |
| `DAQ_INA219_SHUNT_MOHM` | 10 | shunt resistor on the HAT |
| `DAQ_BATTERY_CURRENT_SIGN` | −1 | sign so that + = charging; **check on the device** (plug the charger: current must go positive) |
| `DAQ_BATTERY_RINT_MOHM` | 150 | battery + protection resistance, corrects the voltage under load before the state-of-charge lookup |
| `DAQ_PIN_LED` | 48 | on-board WS2812 (38 on DevKitC-1 v1.1) |
| `DAQ_LED_BRIGHTNESS` | 24 | LED brightness 0..255 |
| `DAQ_IDENTIFY_MS` | 3000 | identify blink duration |
| `DAQ_FW_VERSION_*` | 0.8.0 | firmware version reported over BLE |
| `DAQ_BLE_NAME_PREFIX` | `AudioLoc-` | advertised name prefix, + last 2 bytes of the MAC |
| `DAQ_BLE_HOP_MS` | 32 | Result notification period |
| `DAQ_BLE_MAX_SOURCES` | 2 | sources per Result frame |
| `DAQ_BLE_MAP_AZ_BINS`, `DAQ_BLE_MAP_EL_BINS` | 36, 18 | SRP map size (10° cells) |
| `DAQ_BLE_MAP_RATE_HZ` | 5 | default map rate (runtime value in NVS, changed from the app) |
| `DAQ_BLE_TEST_PATTERN` | 0 | 1 = send one synthetic moving source and its map, to test the app and the link |
| `DAQ_SPEED_OF_SOUND_MPS` | 343.0 | default speed of sound (runtime value in NVS, changed from the app) |
| `DAQ_ALGORITHM_DEFAULT` | 1 (GCC-PHAT + LS) | default algorithm (runtime value in NVS) |
| `DAQ_TRACKER_MODE_DEFAULT` | 1 (azimuth only) | default tracker association mode (runtime value in NVS) |
| `DAQ_OUTPUT_MASK_DEFAULT` | 3 (tracked + raw) | default BLE outputs (runtime value in NVS) |
| `DAQ_TRACK_MAX` | 4 | tracks kept by the tracker |
| `DAQ_MIC_XYZ_MM` | pinout DAQ-PIN-01 | default mic coordinates in mm, 0.1 mm resolution (runtime values in NVS) |

Values marked "runtime value in NVS" are only defaults: after the first change from the app or the console the stored value wins. `defaults` in the console restores them.

### `main/daq.h`
Shared definitions used by all modules:
- `daq_state_t g_daq`: global runtime state, i.e. flags (`recording`, `record_request`, `streaming`, `sd_ready`, `ble_connected`), statistics (captured frames, dropped blocks, bytes written/sent, SD errors) and the last per-mic levels (`level_dbfs`).
- `daq_packet_header_t`: header of a USB stream packet (format below).
- `daq_source_t`: one localized source or track (azimuth, elevation, distance, confidence, track id) passed to the BLE module.
- `daq_settings_t g_settings`: persistent settings (mic coordinates, speed of sound, algorithm, map rate, tracker mode, output mask); change them only through `settings_set_*()`.
- `daq_track_mode_t` (1 azimuth, 2 azimuth × elevation, 3 elliptical gate), `DAQ_OUT_TRACKED` / `DAQ_OUT_RAW`, `track_out_t` (one confirmed track).
- `daq_algorithm_t`: 0 off, 1 GCC-PHAT + LS, 2 SRP-PHAT, 3 MUSIC.
- `dsp_result_t`: one DSP frame (first sample, VAD, energy, τ and peak for the 15 pairs); `DSP_FRAME_LEN` 2048, `DSP_HOP` 1536, `DSP_NUM_PAIRS` 15.
- `loc_source_t` / `loc_result_t`: result of `localize_a()` / `localize_b()` (up to 2 sources: azimuth, elevation, distance, confidence, kind `P` / `D`; fit residual and pairs for A).
- `dsp_result_t` also points to the band spectra of every mic (`xr`, `xi`), used by MUSIC.
- Function prototypes of `capture.c`, `storage.c`, `stream.c`, `ble.c`, `system.c`, `dsp.c`, `localize.c`, `music.c` and `track.c`.

### `main/capture.c`
Audio acquisition.
- `mic_sel_init()` drives the SEL pin of each mic to its level from `DAQ_MIC_SEL_LEVEL` (before the clock starts).
- `pdm_init()` configures I2S0 as PDM RX master: 3 data lines, left + right slot on each line (6 channels), hardware PDM→PCM conversion to 16-bit with the built-in high-pass filter (35.5 Hz), and a callback that counts DMA overflows.
- `capture_task` (core 1, priority 10) discards the warm-up, then for every block:
  1. reads 320 frames from DMA,
  2. reorders slots to M1..M6 using `DAQ_CHANNEL_MAP`,
  3. accumulates per-channel energy for the level display,
  4. passes the block to the DSP (`dsp_push()`, with its frame index), to the SD ring buffer (if recording; the frame index of the first recorded block is kept as `rec_first_frame`) and to the USB ring buffer with a packet header (if streaming).
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
  - stops recording on a write error,
  - next to every `RECnnnnn.WAV` writes `RECnnnnn.CSV` with the DSP results (lines from `storage_push_csv()`; header written with the first lines).
- `storage_push()` adds a block to the ring buffer; returns `false` (block dropped) if there is no room.

WAV format: PCM, 6 channels, 48 kHz, 16-bit, interleaved M1..M6. A file interrupted by a power loss keeps `data_size = 0`; `daq_sim.recording.load_recording` reads it using the file size.

### `main/stream.c`
Streaming to the PC over native USB (USB-Serial-JTAG, the DevKitC port labelled USB).
- `stream_init()` installs the USB-Serial-JTAG driver and starts `stream_task` (core 0, priority 6).
- `stream_task` reads host commands (`S` = start, `X` = stop) and sends the ring buffer content in 4 kB writes. If the PC does not read, writes time out and new blocks are dropped (`drop usb`).
- `stream_set_enabled()` starts or stops streaming (also from the console key `u`).
- `stream_push()` adds header + payload atomically, or drops the block if there is no room.
- `stream_push_text()` adds a text packet (`DAQT`) the same way. Audio (capture task) and text (DSP task) are written
  under one mutex, so a packet is never split by another.

While the stream is on, the DSP task also sends the **detection log**: the CSV header block once per stream start, then
one CSV line per DSP frame (32 ms) with the same columns as the SD card CSV (`dsp.c`): frame, VAD, energy, 15 time
differences and peaks, algorithm result (up to 2 sources), tracker output. `frame` is the capture sample index, the
same clock as `first_frame` of the audio packets. A line that does not fit in the ring counts in `drop csv`.

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

Text packet (detection log), 8-byte header + ASCII:

| Field | Type | Meaning |
|---|---|---|
| `magic` | u32 | `0x54514144` ("DAQT") |
| `length` | u16 | number of text bytes |
| `reserved` | u16 | 0 |
| text | `length` bytes | CSV header block (`# daq-detector … wav=usb first_frame=0 …` + column line) or one CSV line with `\n` |

### `main/ble.c`
BLE GATT server "AudioLoc" (NimBLE, peripheral). Full protocol in [`docs/ble_gatt.md`](docs/ble_gatt.md).
- `ble_init()` starts NimBLE, registers the service (Result, SRP map, Diagnostics, Device info, Control, Raw result) and starts `ble_task` (core 0, priority 4). Failure is not fatal: acquisition keeps running.
- Advertises the service UUID, name `AudioLoc-XXXX` in the scan response; re-advertises after a disconnect.
- `ble_task` runs every 32 ms and, for the subscribed characteristics, sends the Result and Raw result frames, Diagnostics every 1 s, and the SRP map at the rate set by the app (chunked to the negotiated MTU).
- `ble_publish_result()` (tracked speakers, track id in byte 7), `ble_publish_raw()` (sources of the frame) and `ble_publish_map()`, called from `dsp.c`. A result older than 500 ms is reported as not valid.
- Notifications are sent only when they fit in the negotiated MTU (nothing is sent before the MTU exchange).
- Control writes: start/stop SD recording, start/stop USB stream, SRP map rate, identify, algorithm (`07`), speed of sound (`08`), outputs (`09`), tracker mode (`0A`).

### `main/system.c`
System services, task `system` (core 0, priority 2, every 100 ms):
- **Settings**: `settings_init()` initialises NVS (also needed by BLE) and loads `daq_settings_t` (blob version 2; a version 1 blob from firmware ≤ 0.6.0 is kept and the new fields take their defaults), falling back to the defaults from `config.h`. Every `settings_set_*()` validates the value and saves it immediately.
- **Battery**: INA219 at 0x43, 12-bit with 128-sample averaging, read every 1 s. Current from the shunt voltage (10 µV / 10 mΩ = 1 mA per LSB), voltage from the bus register (4 mV per LSB). State of charge: open-circuit estimate `V − I·R_int` on a LiPo curve, smoothed. If the INA219 does not answer, battery data stays unknown.
- **LED** (WS2812): white blink = identify (3 s), magenta = new lost blocks or SD error (2 s), red blink = recording, cyan = USB stream, blue = phone connected, yellow = no SD card, green = idle.
- **CPU load**: busy time of each core over the last second, from the idle task run time.

### `main/dsp.c`
DSP front end, task `dsp` (core 1, priority 8, below capture). It implements `daq-sim/daq_sim/tdoa.py` step by step:
- frames of 2048 samples with hop 1536 (32 ms), periodic Hann window, samples scaled by 1/32768,
- 3 complex FFTs (esp-dsp) per frame, two real channels packed in one transform; the FFT sign convention is checked at start,
- band 300–3500 Hz (bins 13–149),
- VAD: band energy averaged over the mics; noise floor follows the minimum and rises at most 0.05 dB per frame; voice = 9 dB above the floor,
- GCC-PHAT for the 15 pairs (12, 13, …, 56) evaluated directly at integer lags within the physical range + 2 samples
  (from the mic geometry and speed of sound in NVS), parabolic refinement; τ_ij = t_i − t_j in µs and the peak value.

Live mode: blocks from `capture_task` come through a queue (24 blocks, PSRAM); a lost block restarts the frame window. While recording,
one CSV line per frame goes to `RECnnnnn.CSV`. `g_daq.dsp_frame_us` and `dsp_voice` show the processing time and the VAD.

Replay mode (`play N` in the console, not while recording): reads `RECnnnnn.WAV` from the card (also a file copied from the PC,
e.g. from `simulate_wav.py`; any 6-channel 16-bit 48 kHz WAV), processes it frame by frame (not real time; live input is ignored)
and writes `PLYnnnnn.CSV` with `first_frame=0`. Compare on the PC: `python tools/localize_wav.py RECnnnnn.WAV --compare PLYnnnnn.CSV`
(target max |Δτ| ≤ 1 µs).

CSV format (both modes): header line `# daq-detector 0.8.0 wav=RECnnnnn.WAV first_frame=… fs=48000 c=… alg=A|B|C trk=AZ|AZEL|ELLIPSE mics=x,y,z;…` (geometry,
speed of sound, algorithm and tracker mode actually used, mm), column line, then
`frame, vad, energy_db, t12…t56 (µs), p12…p56, alg, kind, speakers, az_deg, el_deg, dist_m, conf, res_us, pairs, az2_deg, el2_deg, dist2_m, conf2,
tracks, t1_id, t1_az_deg, t1_el_deg, t1_dist_m, t1_conf, t2_id, t2_az_deg, t2_el_deg, t2_dist_m, t2_conf`.
`frame` is the capture frame index of the frame start; `frame − first_frame` is the sample index in the WAV file.
`kind`: `P` position, `D` direction only (`dist_m` = 0), `-` no result; `res_us` / `pairs` only for algorithm A; the second source only for B and C;
C reports azimuths only (`el_deg` = 0, `dist_m` = 0).
Localization columns are empty for frames without voice; tracker columns are on every line (`tracks` = confirmed tracks = speakers,
then the two most confident). A replay starts with an empty tracker (as `localize_wav.py`), live tracking restarts after it.

For voice frames with algorithm A or B selected (`alg 1` / `alg 2`, BLE `07 01` / `07 02`) the DSP calls `localize_a()` / `localize_b()`;
the sources go to BLE (Raw result; distance 0 = unknown) and, for B, the 36 × 18 power map (SRP map characteristic) at the rate set by the app.
Every frame (also without voice) goes through the tracker (`track.c`); its confirmed tracks go to BLE Result.

Host check: `dsp.c` compiled on a PC with a reference DFT in place of esp-dsp and run in replay on `simulate_wav.py --source 60 20 2.0`
matches `daq-sim` within 0.001 µs on all 78 voice frames × 15 pairs (same VAD decisions).

### `main/localize.c`
Ports of `daq-sim/daq_sim/ls.py` and `srp.py` (same steps and constants; least squares in double precision).

**Algorithm A** — `localize_a()`:
1. linear start with M1 as reference (unknowns: position and distance to M1, 5 equations),
2. Gauss–Newton on the used pairs (at most 10 iterations, stop when the step is below 10 µm),
3. while the worst residual exceeds 25 µs, drop that pair and refit (at most 2 pairs),
4. the fit is ok if finite, RMS ≤ 25 µs and the position within 8 m → kind `P`,
5. otherwise (decision e) a plane-wave fit τ_ij = (m_j − m_i)·v with the same pair rejection; accepted if RMS ≤ 25 µs and |v|·c within
   ±30 % of 1 → kind `D` (direction only, distance 0); otherwise no result.

Confidence = mean GCC-PHAT peak of the used pairs × (1 − RMS / 25 µs).

**Algorithm B (SRP-PHAT)** — `localize_b()`:
1. coarse grid = the BLE map grid (azimuth 0–350° step 10°, elevation −85…85° step 10°) × 8 distances (0.5–6 m); the delay table
   (5184 points × 15 pairs, PSRAM) is rebuilt when the geometry or speed of sound changes; power = sum of the pair correlations at the grid
   delays (linear interpolation); map = maximum over distances, scaled to 0–255,
2. up to 2 local maxima of the map: the first if power / 15 ≥ 0.15, a second one if ≥ 0.8 × the first and ≥ 40° away,
3. pattern search around each maximum on the exact correlation at fractional delays (steps 5° / 5° / ×1.25, halved down to 1°),
4. SRP-guided time differences (best correlation value within ±3 samples of the delays predicted by the refined point) and the
   least-squares fit of algorithm A → kind `P` if the fit is ok, otherwise the SRP direction only (kind `D`).

Confidence: of the least-squares fit for `P`, power / 15 for `D`.

Simulation results (`daq-sim`, source at 60° / 20° / 2 m, SNR 30 dB, 4 s of speech-like signal):

| Conditions | Algorithm | Frames with a result | Azimuth (median, p10–p90) | Elevation | Distance (kind P) |
|---|---|---|---|---|---|
| free field | A or B | 78 / 78 (all P) | 60.0° (59.9–60.1) | 20.0° (19.6–20.5) | 2.0 m (1.9–2.1) |
| room 6 × 5 × 3 m, RT60 0.5 s | A | 15 / 89 (8 P, 7 D) | 59.5° (56.8–64.8) | 22° (−4…28) | 0.9 m (0.6–2.5) |
| same room | B | 80 / 89 (33 P, 47 D) | 60.0° (58.6–80.0) | 21° (−5…25) | 1.8 m (0.6–3.7) |

With two sources in the room (60° / 220°) single frames are not reliable for counting (two sources in 28 of 89 frames, sometimes a reflection);
counting is done over time by `track.c`.

Host check: `dsp.c` + `localize.c` compiled on a PC (reference DFT in place of esp-dsp) and run in replay on the four simulated recordings match
`daq-sim` exactly (same kind and number of sources in every frame, directions and distances equal to the printed precision).
Device cost not measured yet: B evaluates the exact correlation a few hundred times per source; check `loc` time in the status line
(must leave room in the 32 ms frame together with `dsp`).

### `main/music.c`
Algorithm C, port of `daq-sim/daq_sim/music.py` (decisions 02.11.2026: separate algorithm, azimuth only, ~0.5 s averaging).
`localize_c()` is called by `dsp.c` for **every** frame (the covariance also decays without voice) when algorithm 3 is selected
(`alg 3`, BLE `07 03`); a result row is produced for voice frames:
1. 40 single FFT bins spread over 300–2800 Hz (below the 2.9 kHz phase-ambiguity limit of the 60 mm triangle),
2. per bin the M1–M3 snapshot normalised to unit length (spectral whitening), 3 × 3 covariance averaged exponentially
   over ~16 frames (λ = 15/16, voice frames only, all frames decay); a result needs a weight ≥ 4 (≈ 4 recent voice frames),
3. eigen-decomposition of each 3 × 3 Hermitian matrix (complex Jacobi, double); speaker count K = 1 or 2 from MDL summed over the
   bins (`MDL_PENALTY` 1 = standard MDL),
4. pseudospectrum on a 1° azimuth grid (elevation 0; the planar triangle cannot tell +el from −el), mean over bins of
   1 / |E<sub>n</sub><sup>H</sup> a|² (steering table 40 × 360 × 3 in PSRAM, rebuilt after a geometry or speed-of-sound change),
5. the K largest local maxima with parabolic refinement; confidence = 1 − mean projection on the noise subspace.

Sources go to the tracker and BLE like those of A and B (kind `D`, elevation 0, distance 0). No SRP map with C.
Host check: C replay identical to `daq-sim` on the three simulated recordings (same counts, azimuths within 0.01°).
Simulation (`tools/evaluate.py`, see `daq-sim`): azimuth of the matched sources within ~1–7° on average, but the MDL count reports 2
for a single speaker in most rooms (the floor reflection is coherent with the source), so C counts speakers poorly.

### `main/track.c`
Port of `daq-sim/daq_sim/track.py` (decision 1c: direction histogram + tracker), in double precision; `track_update()` is called by
`dsp.c` for every frame with the sources of the frame (none without voice):
1. histogram of directions with a 1 s half-life; each source adds 1 (centre 1, neighbours 0.5 / 0.25); AZ mode: 36 azimuth bins,
   AZEL / ELLIPSE: the 36 × 18 map grid,
2. each source (most confident first) updates the nearest track within the 20° gate (smoothing 0.3 of direction, distance, confidence);
   otherwise a new tentative track is born if the histogram at its cell is ≥ 1.5 (at most 4 tracks, ids 1–255),
3. a track is confirmed after 4 hits; a tentative track dies after 0.3 s without an update, a confirmed one after 1.5 s,
4. tracks closer than the merge distance are merged (the older one stays): 30° in azimuth for AZ, 10° otherwise,
5. output: confirmed tracks by decreasing confidence; their number = speakers.

Modes (console `trk N`, BLE `0A nn`, stored in NVS; a change restarts the tracker):

| Mode | Distance | Room, 2 speakers (60° / 220°) | Room, 1 speaker | Free field | Trade-off |
|---|---|---|---|---|---|
| 1 AZ (default) | wrapped azimuth difference | 2 tracks in 91 / 124 frames | 1 track | 1 track | robust to unstable elevation; speakers < 30° apart in azimuth count as one |
| 2 AZEL | great-circle angle | 2nd speaker in 5 frames | ghost track (3 tracks in 52 frames) | 1 track | separates heights; needs a reliable elevation |
| 3 ELLIPSE | 20° × max(Δaz / 15°, Δel / 45°) | 2nd speaker in 14 frames | ghost persists (2 tracks in 85 frames) | 1 track | compromise |

(simulation, algorithm B, RT60 0.5 s; to be repeated on real recordings with `play N` + `localize_wav.py --track …`.)
Host check: C replay identical to Python in all three modes (same counts and ids in every frame).

### `main/main.c`
- `app_main()` initialises settings/NVS, storage (continues without SD if the card is missing), the USB stream, the capture, BLE (continues without it) and the system task, then starts the UI task.
- `ui_task` (core 0, priority 3) polls the UART console and the button every 20 ms and prints the status line.

Console keys (act immediately): `r` start/stop SD recording, `u` start/stop USB stream, `i` identify, `h` help. The button on GPIO16 also toggles recording.

Console line commands (typed with echo, end with Enter), saved to NVS immediately:

| Command | Effect |
|---|---|
| `mic` | list mic positions |
| `mic N X Y Z` | set position of mic N (1..6) in mm, e.g. `mic 4 -301.5 176.2 -160.0` (geometry calibration) |
| `c` / `c 341.5` | show / set speed of sound (300..380 m/s) |
| `alg` / `alg 2` | show / set algorithm (0 off, 1 GCC-PHAT + LS, 2 SRP-PHAT, 3 MUSIC on M1–M3) |
| `trk` / `trk 2` | show / set tracker mode (1 azimuth, 2 azimuth × elevation, 3 elliptical gate) |
| `out` / `out 1` | show / set BLE outputs: 1 tracked Result, 2 Raw result, 3 both, 0 none |
| `defaults` | restore all settings from `config.h` |
| `play N` | replay `RECnnnnn.WAV` through the DSP → `PLYnnnnn.CSV` (not while recording); progress in the status line |

Status line example:
```
t=42s | dBFS M1: -52.3 M2: -51.8 M3: -53.0 M4: -50.9 M5: -52.1 M6: -51.5 | rec=ON  REC00003 12.4MB err=0 | usb=ON  20.1MB | ble=ON  | bat=78% 3912mV -182mA | cpu=21/38% | dsp=9.8ms vad=1 | src az=60 el=20 d=2.0m n=1 (0.6ms) | spk=1 az=60 | drop dma=0 sd=0 usb=0 dsp=0 csv=0
```
- `dBFS`: RMS level of each mic over the last second (≈ −120 means no signal),
- `rec`: recording state, current file, total bytes written, write errors,
- `usb`: stream state and total bytes sent,
- `ble`: phone connected,
- `bat`: state of charge, voltage, current (+ charging, − discharging), only if the INA219 answers,
- `cpu`: load of core 0 / core 1,
- `dsp`: processing time of the last frame (must stay well below 32 ms) and its VAD decision; during a replay `replay NN%`,
- `src`: last result (azimuth, elevation, distance — 0 = direction only, number of sources) and the localization time, shown for 1 s after the last result,
- `spk`: confirmed tracks (speakers) and the direction of the most confident one,
- `drop dma / sd / usb / dsp / csv`: lost blocks in capture / SD path / USB path / DSP queue, lost CSV lines (SD and USB detection log); all should stay 0.

## Build and run

Requires ESP-IDF v5.3 or newer; built and run with ESP-IDF v6.1 (GCC 15.2). In VS Code (ESP-IDF extension): *Set Espressif
Device Target* → `esp32s3`, *Select Flash Method* → UART, *Select Port* → the CP210x port, then *Build*, *Flash*, *Monitor*.
On Windows keep the project in a path without spaces or non-ASCII characters.

```
idf.py set-target esp32s3
idf.py build
idf.py -p <UART_PORT> flash monitor
```

Connect both DevKitC ports to the PC:
- **UART port (CP2102N)**: flashing and console (`idf.py monitor`),
- **USB port (native)**: binary stream, read with `tools/daq_term.py` (below) or `daq-sim/tools/usb_record.py`.

### `tools/daq_term.py`
Terminal monitor for both ports at once: the USB stream (packets, throughput, lost packets / resyncs, RMS and peak level of
M1..M6, optional WAV on the PC) and the UART console (status line and log; keys `r`, `i`, `h` go to the ESP).

```
pip install pyserial numpy rich
python tools/daq_term.py --uart COM3 --usb COM4            # COM numbers from the Device Manager
python tools/daq_term.py --uart COM3 --usb COM4 --plain --duration 30   # one line per second, no UI
```

Keys: `s` stream on/off (on at start), `w` WAV on the PC (`recordings/`), `l` detection CSV on the PC
(`recordings/det_*.csv`), `r` SD recording, `i` identify, `h` ESP help, `c` clear counters, `q` quit.
`--csv` writes the detection CSV from the start. The CSV has the columns of the SD card CSV plus `pc_time_s` (PC receive
time, Unix seconds) to align it with other logs, e.g. the source position of `daq-sim/tools/live_source.py`. The
`detect` row shows the last result and tracks, the number of lines and missing frames (`gaps`).

```
python tools/daq_term.py --uart COM3 --usb COM4 --csv                    # detection CSV from the start
python tools/daq_term.py --uart "" --usb COM4 --plain --csv --duration 60   # 60 s, no UI
``` Run it with the Windows Python (PowerShell), not from WSL: WSL does not see `COMx` ports.
One program per port: close the VS Code monitor first.

## First run checklist

1. **Tap test**: tap M1..M6 one by one, see which level rises and fix `DAQ_CHANNEL_MAP` in `config.h`.
2. **Capture**: `drop dma` stays 0.
3. **SD**: record a few minutes (`r`); `drop sd` and `err` stay 0. If not, lower `DAQ_SD_FREQ_KHZ`, add external pull-ups or move to SDMMC (same wiring).
4. **BLE**: connect with the app, check that Diagnostics shows the same levels as the console and `drop dma` stays 0 with BLE connected. Set `DAQ_BLE_TEST_PATTERN` to 1 to see a moving source in the Live view.
5. **DSP and localization**: `dsp=` time well below 32 ms, `drop dsp=0`; `vad=1` and a plausible `src` when talking near the array;
   the Live view of the app shows the source. Then record a few seconds, run `play N`
   and compare `PLYnnnnn.CSV` with `daq-sim` (`localize_wav.py --compare`). A synthetic WAV from `simulate_wav.py` copied to the card as
   `RECnnnnn.WAV` checks the whole chain without microphones.
6. **Battery**: `bat=` appears in the status line; plug the charger and check that the current goes positive (otherwise flip `DAQ_BATTERY_CURRENT_SIGN`).
7. **USB**: run `tools/daq_term.py`; `drop usb` and `lost` stay 0 at ~576 kB/s. If not, native USB-Serial-JTAG is too slow: switch to TinyUSB CDC or stream fewer channels / a lower rate.
8. **Hardware in the loop** (without microphones): a Raspberry Pi Pico 2 with `daq-sim/pico_emu` feeds PDM streams of a known source
   (CLK GPIO4 → GP2, GP3/4/5 → GPIO5/6/7, common GND); guide in `daq-sim/docs/hardware_in_the_loop.md`. The firmware needs no changes.

## Related

- `daq-app`: Android app (Kotlin, Jetpack Compose) — live view, diagnostics, device control.
- `daq-sim` (repository `daq-simulator`): PC tools (USB receiver, recording loader), offline algorithm development in Python,
  evaluation of T1–T5 in simulation and the Pico 2 PDM emulator for hardware-in-the-loop tests.
