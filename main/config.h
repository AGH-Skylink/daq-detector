#pragma once

#include "driver/gpio.h"

/* ---------- Audio capture (I2S0, PDM RX) ---------- */
#define DAQ_SAMPLE_RATE_HZ      48000   /* PDM clock = 64 * fs = 3.072 MHz (MP34DT01-M max 3.25 MHz) */
#define DAQ_NUM_CHANNELS        6
#define DAQ_BLOCK_FRAMES        320     /* frames per DMA buffer / read: 320 * 6 ch * 2 B = 3840 B (< 4092 B limit) */
#define DAQ_DMA_DESC_NUM        16      /* 16 * 320 frames = ~107 ms of DMA buffering */
#define DAQ_WARMUP_MS           100     /* discarded after start: mic turn-on (10 ms) + HP filter settling */

/*
 * Variant A: common clock, 3 data lines, each line shared by one body mic (short wire)
 * and one arm mic (long wire). SEL low = left slot, SEL high = right slot.
 */
#define DAQ_PIN_PDM_CLK         GPIO_NUM_4
#define DAQ_PIN_PDM_DIN0        GPIO_NUM_5   /* M1 (body) + M4 (arm) */
#define DAQ_PIN_PDM_DIN1        GPIO_NUM_6   /* M2 (body) + M5 (arm) */
#define DAQ_PIN_PDM_DIN2        GPIO_NUM_7   /* M3 (body) + M6 (arm) */

/* SEL of every mic is driven by its own GPIO, set before the clock starts */
#define DAQ_PIN_MIC_SEL         { GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_14, GPIO_NUM_21 }  /* M1..M6 */
#define DAQ_MIC_SEL_LEVEL       { 0, 0, 0, 1, 1, 1 }   /* body mics left slot, arm mics right slot */

/*
 * Output channel k (M1..M6) is taken from raw slot DAQ_CHANNEL_MAP[k] of the I2S frame.
 * Raw slots: 0 = line0 L, 1 = line0 R, 2 = line1 L, 3 = line1 R, 4 = line2 L, 5 = line2 R.
 * With the SEL levels above: M1 = 0, M2 = 2, M3 = 4, M4 = 1, M5 = 3, M6 = 5.
 * Confirm with a tap test: tap M1, M2, ... one by one and watch which level rises in the status line.
 */
#define DAQ_CHANNEL_MAP         { 0, 2, 4, 1, 3, 5 }

/* ---------- microSD (SPI, Pololu 2597) ---------- */
#define DAQ_PIN_SD_SCK          GPIO_NUM_12
#define DAQ_PIN_SD_MOSI         GPIO_NUM_11
#define DAQ_PIN_SD_MISO         GPIO_NUM_13
#define DAQ_PIN_SD_CS           GPIO_NUM_10
#define DAQ_SD_FREQ_KHZ         20000
#define DAQ_SD_MOUNT_POINT      "/sdcard"
#define DAQ_SD_RING_BYTES       (512 * 1024)  /* PSRAM, ~0.9 s of audio, absorbs card write stalls */
#define DAQ_SD_CHUNK_BYTES      (32 * 1024)   /* size of a single fwrite */
#define DAQ_SD_FILE_SECONDS     300           /* gapless rotation to a new WAV file */
#define DAQ_SD_CSV_RING_BYTES   (32 * 1024)   /* PSRAM, CSV log lines from dsp.c (~3 s at 31 lines/s) */

/* ---------- USB stream (native USB, USB-Serial-JTAG) ---------- */
#define DAQ_USB_RING_BYTES      (128 * 1024)  /* PSRAM */
#define DAQ_USB_CHUNK_BYTES     4096
#define DAQ_USB_TX_BUFFER_BYTES (16 * 1024)
#define DAQ_USB_WRITE_TIMEOUT_MS 20

/* ---------- UI ---------- */
#define DAQ_PIN_BUTTON          GPIO_NUM_16   /* active low, toggles recording */
#define DAQ_STATUS_PERIOD_MS    1000

/* ---------- System: battery monitor (INA219 on the UPS HAT), status LED ---------- */
#define DAQ_PIN_I2C_SDA         GPIO_NUM_1
#define DAQ_PIN_I2C_SCL         GPIO_NUM_2
#define DAQ_INA219_ADDR         0x43          /* Waveshare UPS HAT (C) default, confirm with an I2C scan */
#define DAQ_INA219_SHUNT_MOHM   10            /* R9 on the HAT */
#define DAQ_BATTERY_CURRENT_SIGN (-1)         /* chosen so that + = charging; flip if the sign is wrong on the device */
#define DAQ_BATTERY_RINT_MOHM   150           /* cell + protection resistance, used to correct voltage under load */
#define DAQ_PIN_LED             GPIO_NUM_48   /* on-board WS2812 (DevKitC-1 v1.0; v1.1 uses GPIO38) */
#define DAQ_LED_BRIGHTNESS      24            /* 0..255 */
#define DAQ_IDENTIFY_MS         3000

/* ---------- BLE (NimBLE, GATT "AudioLoc", see docs/ble_gatt.md) ---------- */
#define DAQ_FW_VERSION_MAJOR    0
#define DAQ_FW_VERSION_MINOR    8
#define DAQ_FW_VERSION_PATCH    0
#define DAQ_BLE_NAME_PREFIX     "AudioLoc-"   /* + last 2 bytes of the BT MAC */
#define DAQ_BLE_HOP_MS          32            /* Result notification period */
#define DAQ_BLE_MAX_SOURCES     2             /* sources per Result frame */
#define DAQ_BLE_MAP_AZ_BINS     36            /* SRP map: 10 deg cells */
#define DAQ_BLE_MAP_EL_BINS     18
#define DAQ_BLE_MAP_RATE_HZ     5             /* default, changed by the app (0 = off, max 10), stored in NVS */
#define DAQ_BLE_TEST_PATTERN    0             /* 1 = send a synthetic moving source and map */

/* ---------- Localization defaults (runtime values live in NVS, see daq_settings_t) ---------- */
#define DAQ_SPEED_OF_SOUND_MPS  343.0f        /* changed from the app (BLE control 0x08) */
#define DAQ_ALGORITHM_DEFAULT   DAQ_ALG_A_GCC_LS
#define DAQ_TRACKER_MODE_DEFAULT DAQ_TRACK_AZ          /* speaker tracker association (BLE control 0x0A, `trk N`) */
#define DAQ_OUTPUT_MASK_DEFAULT (DAQ_OUT_TRACKED | DAQ_OUT_RAW)   /* BLE outputs (BLE control 0x09, `out N`) */
#define DAQ_TRACK_MAX           4             /* tracks kept by the tracker (track.c) */

/*
 * Mic positions M1..M6 in mm, 0.1 mm resolution (pinout DAQ-PIN-01 is the reference):
 * origin = centre of the M1-M3 triangle, x = azimuth 0, y = azimuth 90, z = up.
 * Body: triangle side 60 mm (circumradius 34.64 mm) at 90/210/330 deg.
 * Arms: radius 350 mm at 150 (M4) / 270 (M5) / 30 (M6) deg, 161 mm below the M1-M3 plane.
 * Measured values are entered from the console (`mic N x y z`) and stored in NVS.
 */
#define DAQ_MIC_XYZ_MM { \
    {    0.0f,   34.6f,    0.0f }, \
    {  -30.0f,  -17.3f,    0.0f }, \
    {   30.0f,  -17.3f,    0.0f }, \
    { -303.1f,  175.0f, -161.0f }, \
    {    0.0f, -350.0f, -161.0f }, \
    {  303.1f,  175.0f, -161.0f }, \
}
