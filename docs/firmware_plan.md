# Firmware development plan

Status: 0.8.0 written (not yet built with ESP-IDF or run on hardware): algorithm A with decision e, algorithm B (SRP-PHAT) with the map, algorithm C (MUSIC on M1–M3), speaker tracker with three selectable modes, BLE tracked + raw outputs (switchable). `dsp.c` + `localize.c` + `music.c` + `track.c` checked on a PC against `daq-sim` (identical results on the simulated recordings). Simulated evaluation of T1–T5 with proposed threshold changes: see "Evaluation 02.11.2026" below.

## Milestones

| Version | Scope | Done when |
|---|---|---|
| 0.2.x | T0 on hardware: tap test, `drop dma/sd/usb` = 0, BLE with the app | 6 channels recorded without loss, app shows diagnostics |
| **0.3.0** | `system.c`: NVS settings, INA219 battery, LED + identify, CPU load; BLE protocol v2 (0.1 mm geometry, speed of sound, algorithm, MTU gate); console line commands | all placeholders except localization and map replaced by real data |
| **0.4.0** | `dsp.c`: frames 2048 / hop 1536 from capture blocks, VAD (300–3500 Hz energy), 6 forward FFTs, GCC-PHAT for 15 pairs with parabolic interpolation; replay mode (WAV from SD through the same pipeline); TDOAs to CSV on SD | device TDOAs match `daq-sim` on the same recordings within 1 µs |
| **0.5.0** | `localize.c`: algorithm A (GCC-PHAT + least squares, Gauss–Newton, pair rejection) → `ble_publish_result()`; results to CSV | live position in the app, test T1 |
| **0.6.0** | algorithm B (SRP-PHAT, coarse + fine grid, delay tables in PSRAM) → 36×18 map, up to 2 sources | map in the app, test T2 |
| **0.7.0** | `track.c`: direction histogram + tracker (decision 1c), modes AZ / AZEL / ELLIPSE; BLE Result = tracks, Raw result = per-frame sources, both switchable (decision 2); protocol v3 | speaker count in the app, test T3, choice of the tracker mode |
| **0.8.0** | `music.c`: algorithm C (MUSIC on M1–M3, 40 bins, 3×3 covariance over ~16 frames, MDL count, azimuth only) | test T3, comparison with B |
| 1.0.0 | threshold tuning, tests T4–T5, documentation | results in the project documentation |

Each algorithm is first written and validated in Python (`daq-sim`), then ported to C and compared on the same recordings through the replay mode.

## Decisions (30.10.2026)

| # | Topic | Decision |
|---|---|---|
| D1 | Mic labels | pinout DAQ-PIN-01 is the reference (M4 150°, M5 270°, M6 30°, origin in the M1–M3 plane); project documentation to be updated |
| D2 | Coordinate precision | 0.1 mm in firmware and in BLE (Device info v2) |
| D3 | Algorithm development | Python in `daq-sim` first, then port to C |
| D4 | Replay mode | yes: WAV from the SD card through the DSP pipeline |
| D5 | First algorithm on the device | A, then B |
| D6 | Algorithm selection | BLE control `07 nn` (and console `alg N`), stored in NVS |
| D7 | Speed of sound | set from the app (BLE `08`), default 343 m/s, stored in NVS |
| D8 | Geometry calibration | console `mic N X Y Z`, stored in NVS |
| D9 | Result log | CSV next to the WAV files on the SD card |
| D10 | Battery state of charge | voltage corrected by current (`V − I·R_int`) on a LiPo curve |
| D11 | Notifications before the MTU exchange | skipped |

## Module layout

| File | Content |
|---|---|
| `system.c` (0.3.0) | NVS settings, INA219, LED, identify, CPU load |
| `dsp.c` (0.4.0) | frame window fed by `capture_task`, VAD, FFT (esp-dsp), GCC-PHAT, replay, CSV lines |
| `localize.c` (0.5.0+) | geometry, algorithms A/B/C, tracking, counting, `ble_publish_*`, CSV log |

Core 1: capture (priority 10) and DSP (lower priority). Core 0: BLE, SD, USB, UI, system.

## Compute budget (estimate, to be measured in 0.4.0 via `cpu1_pct`)

- 0.4.0 implementation: 3 complex 2048-point FFTs (two channels per transform) + GCC-PHAT evaluated directly at the needed lags (15 pairs × up to 177 lags × 137 bins ≈ 0.36 M multiply-adds): estimated 5–12 ms per 32 ms hop; measured value in the status line (`dsp=`).
- SRP coarse grid: 36 × 18 × ~5 distances ≈ 3 200 points × 15 lookups, < 1 ms.
- MUSIC: 40 eigen-decompositions of 3×3 matrices per hop, < 1 ms.
- Memory: SRP delay tables ~0.4 MB, spectra ~0.2 MB, in PSRAM.

## Open items

- Done: `daq-app` on protocol v2, project documentation v0.4 (D1), `daq-sim` geometry + reference GCC-PHAT + LS.
- Done (0.5.0): `localize.c` (port of `ls.py`), positions to BLE (Result) and CSV.
- Done (0.6.0): algorithm A + direction-only fallback (e), algorithm B (SRP-PHAT, 2 sources, map) — see the decision below.
- Done (0.7.0): speaker tracker (3 modes), BLE tracked + raw outputs — see the decision below.
- Done (0.8.0): algorithm C (MUSIC) in daq-sim and firmware; `daq-sim/tools/evaluate.py` (simulated T1–T5, threshold sweep); project documentation v0.5.
- Next (hardware):
  1. ~~first ESP-IDF build~~ done with ESP-IDF v6.1: `MINIMAL_BUILD`, `PRIV_REQUIRES`, `-O2` (at `-Og` the DSP frame exceeded 32 ms),
  2. before power-up: measure the 5 V pin of the UPS HAT connector (mirrored), 10 kΩ pull-ups on the SD lines,
  3. T0: tap test of every mic (order M1–M6 / `DAQ_CHANNEL_MAP`), `drop` counters 0, SD recording, USB stream,
  4. INA219 current sign (`DAQ_BATTERY_CURRENT_SIGN`) and battery percentage,
  4b. hardware in the loop with the Pico 2 emulator (`daq-sim/docs/hardware_in_the_loop.md`): channel mapping (`tone N`),
      live source vs the ESP result for A, B, C, static and rotating,
  5. `dsp` + `loc` times: B must fit in 32 ms together with the DSP, otherwise make the B refinement cheaper,
  6. measured geometry of the built device (`mic N x y z`),
  7. T1: one speaker at known positions, `play N` + `localize_wav.py --compare`,
  8. T2/T3: two speakers, choice of the tracker mode (AZ / AZEL / ELLIPSE) on real recordings,
  9. app: `./gradlew test`, run on a phone with the device.
- Open decisions from the evaluation (below): tracker `CONFIRM_HITS`, MUSIC `MDL_PENALTY`.
- Later: thresholds confirmed on real recordings T1–T5, documentation 1.0 with measured results (1.0.0).

## Decision 02.11.2026: algorithm A in reverberation → d + e

Simulation (room 6 × 5 × 3 m, RT60 0.5 s, source 60° / 20° / 2 m): algorithm A gave a result in 15 of 89 voice frames.
Chosen: **d** (algorithm B, SRP-PHAT) and **e** (direction only when the fit is poor). With both: B gives a result in 80 of 89 frames
(33 positions, 47 directions), azimuth median 60.0°; A keeps working as before plus direction-only results.
B uses SRP-PHAT to find the source and then SRP-guided time differences + the least-squares fit of A for the exact position.

Next:
- measure `dsp` + `loc` time on the device (B refinement may need to be made cheaper),
- ~~tracking and speaker counting over 1–2 s~~ done in 0.7.0,
- ~~algorithm C (MUSIC) for counting~~ done in 0.8.0.

## Decision 02.11.2026: speaker counting and BLE output → 1c, 2 = both

1c: direction histogram (1 s half-life) + tracker (gate 20°, confirm after 4 hits, timeouts 0.3 / 1.5 s, up to 4 tracks); speakers =
confirmed tracks. Spec: `daq-sim/daq_sim/track.py`, port: `main/track.c`.
2: BLE Result carries the tracked speakers (for the app), the new Raw result `0007` the per-frame sources (debug, learning); each
switchable with Control `09` / console `out N`.

In the reverberant simulation the elevation of the weaker source jumps between −65° and +55° while its azimuth is stable, and a single
speaker produces a ghost source ~15° away in azimuth with a mirrored elevation. Three association modes are therefore implemented and
selectable (Control `0A`, console `trk N`, `localize_wav.py --track`), to be compared on real recordings:

| Mode | Room, 2 speakers | Room, 1 speaker | Free field |
|---|---|---|---|
| AZ, azimuth only (default) | 2 tracks in 91 / 124 frames | 1 track | 1 track |
| AZEL, azimuth × elevation | 2nd speaker in 5 frames | ghost: 3 tracks in 52 frames | 1 track |
| ELLIPSE, ±15° az / ±45° el | 2nd speaker in 14 frames | ghost: 2 tracks in 85 frames | 1 track |

The histogram uses unit weights: the confidences of positions (0.002–0.38) and directions (0.15–0.28) are not comparable.

## Decision 02.11.2026: algorithm C (MUSIC)

Separate algorithm (`alg 3`, like A and B), azimuth only (the M1–M3 triangle is planar: +el and −el are identical), covariance
averaged over ~16 frames (0.5 s). Implementation detail found during development: one FFT bin per band (grouping neighbouring bins
made the signal subspace rank 2 for a single source in free field); speaker count K = 1 or 2 (voice frames only).

## Evaluation 02.11.2026 (simulation, `daq-sim/tools/evaluate.py`, quick set of 14 scenarios)

Room simulation (image sources), array 1.2 m above the floor, speech-like signals, 6 s per scenario. Directions: errors of the
sources within 20° of an active speaker; "false" = sources farther than 20° (great circle; for A and B mostly elevation errors).

| Test | A: az / el error, detection | B: az / el error, detection | C: az error, detection, false | Counting (tracker AZ, current thresholds) A / B / C |
|---|---|---|---|---|
| T1 one speaker, 1 and 3 m | 0.6° / 4.5°, 0.72 | 0.6° / 2.5°, 0.70 | 1.1°, 0.95, 0.34 | 1.00 / 0.90 / 0.00 |
| T2 two speakers, 45° and 180° | 1.4° / 5.2°, 0.67 | 0.6° / 2.7°, 1.00 | 11.0°, 0.74, 0.25 | 1.00 / 1.00 / 0.50 |
| T3 three speakers, conversation | 0.2° / 2.9°, 0.16 | 0.3° / 0.9°, 0.26 | 6.2°, 0.83, 0.18 | 0.96 / 0.99 / 0.89 |
| T4 office RT60 0.3 s, hall 1.0 s | 1.6° / 6.7°, 0.29 | 1.3° / 4.4°, 0.89 | 6.6°, 0.85, 0.34 | 1.00 / 1.00 / 0.50 |
| T5 ventilation, music, steps | 1.8° / 6.1°, 1.00 | 0.7° / 2.7°, 1.00 | 5.4°, 0.89, 0.25 | 1.00 / 0.71 / 0.39 |

Findings:
- Azimuth is reliable for A and B (≤ 2° mean on matched sources). Elevation is the weak coordinate in rooms: the floor reflection
  pulls it down (A at 3 m: −30…−40° instead of +8°), which is also why detection at 3 m is low.
- Distance: ~0.1 m at 1 m, 1.5–2.5 m error at 3 m in a room (positions beyond ~2 m are not reliable).
- B gives a result in far more frames than A in reverberation (office 0.95 vs 0.42, hall 0.82 vs 0.16).
- B + tracker: a ghost track at 3 m (T1) and the footsteps counted as a speaker (T5) with `CONFIRM_HITS` 4.
- C: azimuth of the matched sources 1–11°, but the MDL count says 2 for one speaker in most rooms (the floor reflection is coherent
  with the source; eigenvalue ratio λ2/λ3 ≈ 14 in T1), so C alone counts speakers poorly.

Proposals (not applied, to be decided; confirm on real recordings first):

| # | Change | Effect in the simulation | Cost |
|---|---|---|---|
| E1 | tracker `CONFIRM_HITS` 4 → 6 | B counting 0.92 → 0.99 (no ghost, no footsteps) | A counting 0.99 → 0.84 (A gives a result in fewer frames, its tracks confirm too late); new speaker shown after ≥ 6 hits (≥ 0.2 s) |
| E1b | `CONFIRM_HITS` per algorithm (A 4, B 6) | best of both | one more setting |
| E2 | MUSIC `MDL_PENALTY` 1 → 2 | one speaker in T4/T5: share of K = 2 drops from 0.2–1.0 to 0.0–0.4, false sources 0.16–0.50 → 0.0–0.34 | two speakers at 45° are never resolved (already 0.04 at penalty 1), at 180° K = 2 in 45% of frames instead of 95%; T1 unchanged (always 2) |
| E3 | tracker mode stays AZ | best mode for A, B and C in every test | — |
