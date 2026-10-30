#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Shared runtime state. Counters are statistics only, so plain volatile access is enough. */
typedef struct {
    /* capture */
    volatile uint64_t frames_captured;
    volatile uint32_t dma_overflows;      /* DMA blocks lost before the capture task read them */
    volatile uint32_t sd_dropped_blocks;  /* blocks not accepted by the SD ring buffer */
    volatile uint32_t usb_dropped_blocks; /* blocks not accepted by the USB ring buffer */
    /* storage */
    volatile bool     sd_ready;
    volatile bool     record_request;     /* requested by UI */
    volatile bool     recording;          /* actual state, owned by the storage task */
    volatile uint32_t sd_file_index;
    volatile uint64_t sd_bytes_written;
    volatile uint32_t sd_write_errors;
    /* stream */
    volatile bool     streaming;          /* started/stopped by the host ('S' / 'X') or UI key 'u' */
    volatile uint64_t usb_bytes_sent;
} daq_state_t;

extern daq_state_t g_daq;

/*
 * USB stream packet: header followed by `frames * channels` int16 samples,
 * interleaved M1..M6, little-endian. The host resynchronises on `magic`
 * and detects lost packets by gaps in `seq`.
 */
#define DAQ_STREAM_MAGIC 0x31514144u   /* "DAQ1" */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint64_t first_frame;
    uint32_t sample_rate;
    uint16_t channels;
    uint16_t frames;
} daq_packet_header_t;

/* capture.c */
esp_err_t capture_start(void);
void capture_get_levels(float out_dbfs[], size_t count);  /* RMS since the previous call */

/* storage.c */
esp_err_t storage_init(void);
bool storage_push(const void *data, size_t len);

/* stream.c */
esp_err_t stream_init(void);
void stream_set_enabled(bool enabled);
bool stream_push(const daq_packet_header_t *header, const void *data, size_t len);
