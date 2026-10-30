#pragma once

#include "driver/gpio.h"

/* ---------- Audio capture (I2S0, PDM RX) ---------- */
#define DAQ_SAMPLE_RATE_HZ      48000   /* PDM clock = 64 * fs = 3.072 MHz (MP34DT01-M max 3.25 MHz) */
#define DAQ_NUM_CHANNELS        6
#define DAQ_BLOCK_FRAMES        320     /* frames per DMA buffer / read: 320 * 6 ch * 2 B = 3840 B (< 4092 B limit) */
#define DAQ_DMA_DESC_NUM        16      /* 16 * 320 frames = ~107 ms of DMA buffering */
#define DAQ_WARMUP_MS           100     /* discarded after start: mic turn-on (10 ms) + HP filter settling */

/* Variant A: 3 data lines, 2 mics per line (SEL low = left slot, SEL high = right slot) */
#define DAQ_PIN_PDM_CLK         GPIO_NUM_4
#define DAQ_PIN_PDM_DIN0        GPIO_NUM_5   /* M1 (SEL low), M2 (SEL high) */
#define DAQ_PIN_PDM_DIN1        GPIO_NUM_6   /* M3 (SEL low), M4 (SEL high) */
#define DAQ_PIN_PDM_DIN2        GPIO_NUM_7   /* M5 (SEL low), M6 (SEL high) */

/*
 * Output channel k (M1..M6) is taken from raw slot DAQ_CHANNEL_MAP[k] of the I2S frame.
 * The raw slot order must be confirmed with a tap test: tap M1, M2, ... one by one
 * and watch which level rises in the status line, then fix this table.
 */
#define DAQ_CHANNEL_MAP         { 0, 1, 2, 3, 4, 5 }

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

/* ---------- USB stream (native USB, USB-Serial-JTAG) ---------- */
#define DAQ_USB_RING_BYTES      (128 * 1024)  /* PSRAM */
#define DAQ_USB_CHUNK_BYTES     4096
#define DAQ_USB_TX_BUFFER_BYTES (16 * 1024)
#define DAQ_USB_WRITE_TIMEOUT_MS 20

/* ---------- UI ---------- */
#define DAQ_PIN_BUTTON          GPIO_NUM_16   /* active low, toggles recording */
#define DAQ_STATUS_PERIOD_MS    1000
