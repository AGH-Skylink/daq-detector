# BLE GATT protocol — "AudioLoc"

Contract between the firmware (`main/ble.c`) and the Android app (`daq-app`).
Both sides must be changed together; bump the `version` byte of a frame when its layout changes.

Status: **v3** (firmware 0.7.0+). Result (`0002`) carries the **tracked speakers** (confirmed tracks of the speaker
tracker, with a track id), Raw result (`0007`) the **sources of every frame** from algorithm A (one source),
B (up to two sources) or C (firmware 0.8.0, MUSIC on M1–M3: up to two azimuths, elevation 0, distance 0); each output can
be switched off (`09`). B sends the SRP map. The layout did not change in 0.8.0.
`distance_mm` = 0 means the direction is known but the distance is not (decision e). Diagnostics (with battery and CPU load), device info and control are real.

Changes from v1: Diagnostics v2 (52 → 56 B, CPU load, real battery), Device info v2 (68 → 71 B, mic coordinates in 0.1 mm,
speed of sound, algorithm), Control commands `07` (algorithm) and `08` (speed of sound), `06` (identify) implemented,
notifications are skipped until the MTU is large enough.

Changes from v2 (firmware 0.7.0): Result v2 (byte 7 of a source = track id), new characteristic Raw result `0007`
(same 26 B layout, track id 0), Device info v3 (71 → 73 B: output mask, tracker mode), Control `09` (outputs) and
`0A` (tracker mode). The app still reads v1 / v2 frames.

## Link

| Item | Value |
|---|---|
| Role | device = peripheral (GATT server), phone = central |
| Advertised name | `AudioLoc-XXXX` (XXXX = last 2 bytes of the BT MAC), in the scan response |
| Advertised UUID | service UUID (128-bit) in the advertising data; the app scans with this filter |
| Connections | 1 |
| MTU | the app requests 247; the firmware prefers 247. A notification is sent only if it fits (`len + 3 ≤ MTU`), so nothing is sent before the MTU exchange; Device info (73 B) is a read and works at any MTU (long read) |
| Byte order | little-endian everywhere |
| Pairing | none (v1) |

## UUIDs

Base: `da0cXXXX-7e4b-4c8a-9f2e-3b5d1a6c0e01`

| XXXX | Name | Properties | Size | Rate |
|---|---|---|---|---|
| `0001` | AudioLoc service | — | — | — |
| `0002` | Result (tracked speakers) | read, notify | 26 B | every hop (32 ms, ~31 Hz) |
| `0003` | SRP map | notify | chunks, 8 B header + payload | `map_rate_hz` (default 5 Hz, 0 = off) |
| `0004` | Diagnostics | read, notify | 56 B | 1 Hz |
| `0005` | Device info | read | 73 B | static (changes after `07` / `08` / `09` / `0A` / geometry calibration) |
| `0006` | Control | write, write without response | 1–3 B | on demand |
| `0007` | Raw result (sources of the frame) | read, notify | 26 B | every hop (32 ms, ~31 Hz) |

Notifications are enabled through the standard CCCD (`0x2902`).

## Coordinate system

- Origin: centre of the M1–M3 triangle.
- `x` = azimuth 0°, `y` = azimuth 90°, `z` = up. Azimuth grows counter-clockwise seen from above, range 0–360°.
- Elevation: −90° (down) … +90° (up), 0° = horizontal plane of M1–M3.

## Result (`0002`) and Raw result (`0007`), 26 B

Same layout. **Result** = confirmed tracks of the speaker tracker (`main/track.c`, reference `daq-sim/daq_sim/track.py`):
published every frame while the output is on and localization runs (algorithm 1 or 2), also with 0 tracks
(`valid = 1`, `speaker_count = 0` = nobody speaks); `speaker_count` = all confirmed tracks (up to 4), `n_sources` = the
two most confident. **Raw result** = the sources of the last voice frame (`speaker_count` = `n_sources`), valid for
500 ms after the last voice frame; for debugging and learning. A disabled output is reported as `valid = 0`.

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | `version` | 2 (v1: byte 7 of a source reserved) |
| 1 | u8 | `flags` | bit0 `valid` (localization running), bit1 `test_pattern` (synthetic data) |
| 2 | u16 | `seq` | +1 per hop, wraps; gaps = lost notifications |
| 4 | u32 | `timestamp_ms` | since capture start |
| 8 | u8 | `speaker_count` | estimated number of active speakers (may exceed `n_sources`) |
| 9 | u8 | `n_sources` | sources present in this frame, 0–2, strongest first |
| 10 | source | `source[0]` | 8 B, see below |
| 18 | source | `source[1]` | 8 B, see below |

Source (8 B):

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u16 | `azimuth_cdeg` | 0–35999 (0.01°) |
| 2 | i16 | `elevation_cdeg` | −9000…9000 (0.01°) |
| 4 | u16 | `distance_mm` | 0 = unknown |
| 6 | u8 | `confidence` | 0–100 % |
| 7 | u8 | `track_id` | Result: track id 1–255 (stable while the track lives, then wraps); Raw result: 0 |

Unused source slots are zero-filled.

## SRP map (`0003`)

A coarse SRP-PHAT power map over the full sphere, `az_bins × el_bins` bytes (default 36 × 18 = 648 B, 10° cells).

- Cell `(i, j)` is stored at `j * az_bins + i`.
- Azimuth of column `i`: `i * 360 / az_bins` (cell centre).
- Elevation of row `j`: `-90 + (j + 0.5) * 180 / el_bins` (row 0 = bottom).
- Value: u8, linear power normalised to the maximum of this map (255 = maximum).

One map is split into chunks that each fit in one notification (`MTU − 3` bytes):

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | `version` | 1 |
| 1 | u8 | `map_id` | +1 per map, wraps; all chunks of a map share it |
| 2 | u8 | `chunk_index` | 0 … `chunk_count − 1` |
| 3 | u8 | `chunk_count` | |
| 4 | u8 | `az_bins` | |
| 5 | u8 | `el_bins` | |
| 6 | u16 | `offset` | byte offset of this payload in the map |
| 8 | u8[] | payload | |

The receiver assembles a map when it has all `chunk_count` chunks of one `map_id`; a new `map_id` discards an incomplete map.
With MTU 247 the map is 3 notifications (236 B payload each).

## Diagnostics (`0004`), 56 B

| Offset | Type | Field | Source in firmware |
|---|---|---|---|
| 0 | u8 | `version` (2) | |
| 1 | u8 | `flags` | bit0 `sd_ready`, bit1 `recording`, bit2 `streaming` (USB), bit3 `localization_active`, bit4 `battery_valid` |
| 2 | u8 | `battery_pct` | 255 = unknown |
| 3 | u8 | reserved | |
| 4 | u32 | `uptime_s` | `frames_captured / fs` |
| 8 | i16[6] | `level_cdbfs` | RMS level M1..M6 over the last second, 0.01 dBFS (−12000 = no signal) |
| 20 | u32 | `dma_overflows` | `g_daq.dma_overflows` |
| 24 | u32 | `sd_dropped_blocks` | `g_daq.sd_dropped_blocks` |
| 28 | u32 | `usb_dropped_blocks` | `g_daq.usb_dropped_blocks` |
| 32 | u32 | `sd_write_errors` | `g_daq.sd_write_errors` |
| 36 | u32 | `sd_file_index` | current / last `RECnnnnn.WAV` |
| 40 | u32 | `sd_kbytes_written` | `sd_bytes_written / 1024` |
| 44 | u32 | `usb_kbytes_sent` | `usb_bytes_sent / 1024` |
| 48 | u16 | `battery_mv` | INA219 bus voltage, 0 = unknown |
| 50 | i16 | `battery_ma` | INA219, + charging, − discharging |
| 52 | u8 | `cpu0_pct` | load of core 0 (BLE, SD, USB, UI) over the last second |
| 53 | u8 | `cpu1_pct` | load of core 1 (capture, DSP) over the last second |
| 54 | u16 | reserved | 0 |

`battery_pct`: state of charge from the open-circuit voltage estimate `V − I·R_int` (R_int = 150 mΩ) on a LiPo curve, smoothed.

## Device info (`0005`), 73 B

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | `version` (3) | v2: 71 B (up to `algorithm`), v1: 68 B |
| 1 | u8×3 | `fw_major`, `fw_minor`, `fw_patch` | |
| 4 | u32 | `sample_rate` | 48000 |
| 8 | u8 | `channels` | 6 |
| 9 | u8 | `az_bins` | map size |
| 10 | u8 | `el_bins` | |
| 11 | u8 | `hop_ms` | Result period, 32 |
| 12 | i16[6][3] | `mic_xyz_dmm` | M1..M6, x, y, z in **0.1 mm** (v1: mm) |
| 48 | u8[19] | `pins` | GPIO numbers, 255 = unused: PDM CLK, DIN0, DIN1, DIN2, DIN3, SD SCK, SD MOSI, SD MISO, SD CS, button, I2C SDA, I2C SCL, LED, SEL M1, SEL M2, SEL M3, SEL M4, SEL M5, SEL M6 |
| 67 | u8 | `features` | bit0 localization, bit1 SRP map, bit2 battery (INA219 found), bit3 identify (LED) |
| 68 | u16 | `speed_of_sound_dmps` | 0.1 m/s, default 3430 |
| 70 | u8 | `algorithm` | 0 off, 1 GCC-PHAT + LS, 2 SRP-PHAT, 3 MUSIC |
| 71 | u8 | `output_mask` | bit0 Result (tracked), bit1 Raw result; default 3 |
| 72 | u8 | `tracker_mode` | 1 azimuth only (default), 2 azimuth × elevation, 3 elliptical gate |

The app shows features that are not implemented as placeholders.

## Control (`0006`)

| Bytes | Command |
|---|---|
| `01` | start SD recording (ignored without a card) |
| `02` | stop SD recording |
| `03` | start USB stream |
| `04` | stop USB stream |
| `05 nn` | set SRP map rate to `nn` Hz (0 = off, max 10), stored in NVS |
| `06` | identify: LED blinks white for 3 s |
| `07 nn` | select localization algorithm `nn` (0 off, 1 GCC-PHAT + LS, 2 SRP-PHAT, 3 MUSIC), stored in NVS |
| `08 lo hi` | set speed of sound, u16 in 0.1 m/s (3000–3800), stored in NVS |
| `09 nn` | BLE outputs, bit mask: bit0 Result (tracked), bit1 Raw result (0–3), stored in NVS |
| `0A nn` | tracker mode `nn` (1 azimuth only, 2 azimuth × elevation, 3 elliptical gate), stored in NVS; restarts the tracker |

After `05`, `07`, `08`, `09`, `0A` the app re-reads Device info to show the stored values.

An unknown command or a wrong length returns ATT error `0x80`.

## Placeholders (firmware 0.8.0)

| Data | Sent now |
|---|---|
| Result | tracked speakers with algorithms 1, 2 and 3; algorithm 0: `flags.valid = 0` (unless `DAQ_BLE_TEST_PATTERN` = 1) |
| Raw result | algorithm 1 (GCC-PHAT + LS): one source; algorithm 2 (SRP-PHAT): up to two sources; algorithm 3 (MUSIC): up to two azimuths (elevation 0, distance 0); valid while the last voice frame is less than 500 ms old |
| SRP map | real with algorithm 2 (one map per voice frame, sent at `map_rate_hz`); nothing with algorithms 1 / 3 (unless `DAQ_BLE_TEST_PATTERN` = 1) |
| Battery | real if the INA219 answers; otherwise `battery_valid = 0`, `battery_pct = 255`, `battery_mv = 0` |
| Algorithm | 1, 2 and 3 run on the device |

`DAQ_BLE_TEST_PATTERN` (in `config.h`) makes the firmware send one synthetic source moving around the array
and a matching map, to test the BLE link and the app with the capture running.

## Mic coordinates (defaults, pinout DAQ-PIN-01 is the reference)

Triangle side 60 mm (circumradius 34.6 mm), arms 350 mm, 161 mm below the M1–M3 plane, rotated 60°.
Defaults from `config.h`; measured values are set from the console (`mic N X Y Z`), stored in NVS and reported in Device info.

| Mic | Angle | x [mm] | y [mm] | z [mm] | Data line | SEL GPIO / level |
|---|---|---|---|---|---|---|
| M1 | 90° | 0.0 | 34.6 | 0.0 | DIN0 (GPIO5) | 17 / low |
| M2 | 210° | −30.0 | −17.3 | 0.0 | DIN1 (GPIO6) | 18 / low |
| M3 | 330° | 30.0 | −17.3 | 0.0 | DIN2 (GPIO7) | 8 / low |
| M4 | 150° | −303.1 | 175.0 | −161.0 | DIN0 (GPIO5) | 9 / high |
| M5 | 270° | 0.0 | −350.0 | −161.0 | DIN1 (GPIO6) | 14 / high |
| M6 | 30° | 303.1 | 175.0 | −161.0 | DIN2 (GPIO7) | 21 / high |

Each data line carries one body mic (short wire, left slot) and one arm mic (long wire, right slot).

Check: |M1M2| = 60.0, |M1M4| = 370.8, |M1M5| = 417.0, |M4M5| = 606.2 mm (match the documentation).
The angular placement of each mic must be confirmed on the built device together with `DAQ_CHANNEL_MAP`.
