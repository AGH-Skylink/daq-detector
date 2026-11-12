#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "config.h"

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
    /* ui */
    volatile float    level_dbfs[DAQ_NUM_CHANNELS];  /* RMS over the last status period, updated by the UI task */
    /* ble */
    volatile bool     ble_connected;
    /* system (system.c) */
    volatile bool     battery_present;    /* INA219 answered on I2C */
    volatile bool     battery_valid;      /* last reading succeeded */
    volatile uint16_t battery_mv;
    volatile int16_t  battery_ma;         /* + charging, - discharging */
    volatile uint8_t  battery_pct;        /* 255 = unknown */
    volatile uint8_t  cpu_load_pct[2];    /* per core, over the last second */
    volatile bool     led_present;
    /* dsp (dsp.c) */
    volatile uint64_t rec_first_frame;    /* capture frame index of the first sample of the current recording */
    volatile uint32_t dsp_frame_us;       /* processing time of the last frame */
    volatile bool     dsp_voice;          /* VAD of the last frame */
    volatile uint32_t dsp_dropped_blocks; /* blocks not accepted by the DSP queue */
    volatile uint32_t csv_dropped_lines;  /* CSV lines not accepted by the storage ring */
    volatile uint8_t  replay_progress_pct;
    /* localization (localize.c), last valid position */
    volatile uint32_t loc_us;             /* time of the last localization call */
    volatile uint32_t settings_version;   /* incremented on every settings change (geometry tables) */
    volatile uint8_t  loc_speakers;
    volatile float    loc_az_deg, loc_el_deg, loc_dist_m;
    volatile uint64_t loc_frame;          /* frame of the last valid position */
    volatile uint8_t  trk_count;          /* confirmed tracks (speakers) after the last frame */
    volatile float    trk_az_deg;         /* direction of the most confident track */
} daq_state_t;

extern daq_state_t g_daq;

/* Localization algorithm selected at runtime (BLE control 0x07, console `alg N`) */
typedef enum {
    DAQ_ALG_OFF = 0,
    DAQ_ALG_A_GCC_LS = 1,   /* GCC-PHAT + least squares */
    DAQ_ALG_B_SRP = 2,      /* SRP-PHAT */
    DAQ_ALG_C_MUSIC = 3,    /* MUSIC on M1-M3 */
} daq_algorithm_t;

/* Speaker tracker association mode (track.c, reference daq-sim/daq_sim/track.py; BLE control 0x0A, console `trk N`) */
typedef enum {
    DAQ_TRACK_AZ = 1,       /* azimuth only (default) */
    DAQ_TRACK_AZEL = 2,     /* azimuth x elevation */
    DAQ_TRACK_ELLIPSE = 3,  /* azimuth x elevation, elliptical gate */
} daq_track_mode_t;

/* BLE localization outputs (bit mask; BLE control 0x09, console `out N`) */
#define DAQ_OUT_TRACKED 0x01  /* Result (0002): confirmed tracks */
#define DAQ_OUT_RAW     0x02  /* Raw result (0007): sources of every frame */

/* Persistent settings (NVS), defaults from config.h. Change only through the settings_set_*() functions. */
typedef struct {
    float   mic_xyz_mm[DAQ_NUM_CHANNELS][3];
    float   speed_of_sound_mps;
    uint8_t algorithm;      /* daq_algorithm_t */
    uint8_t map_rate_hz;    /* SRP map notifications, 0 = off */
    uint8_t tracker_mode;   /* daq_track_mode_t */
    uint8_t output_mask;    /* DAQ_OUT_* */
} daq_settings_t;

extern daq_settings_t g_settings;

/*
 * USB stream packet: header followed by `frames * channels` int16 samples,
 * interleaved M1..M6, little-endian. The host resynchronises on `magic`
 * and detects lost packets by gaps in `seq`.
 */
#define DAQ_STREAM_MAGIC 0x31514144u   /* "DAQ1" */
#define DAQ_TEXT_MAGIC   0x54514144u   /* "DAQT": text packet, one CSV line of the detection log */

/* Text packet on the USB stream: header followed by `length` bytes of ASCII (a CSV line with '\n', or the
 * CSV header block). Sent while the stream is on, between audio packets, never inside one. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t length;
    uint16_t reserved;
} daq_text_header_t;

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
bool storage_push_csv(const char *line, size_t len);   /* CSV log next to the current WAV file */

/* stream.c */
esp_err_t stream_init(void);
void stream_set_enabled(bool enabled);
bool stream_push(const daq_packet_header_t *header, const void *data, size_t len);
bool stream_push_text(const char *text, size_t len);   /* DAQT packet; false if the ring is full or len > 65535 */

/* system.c — NVS settings, battery (INA219), status LED, CPU load */
esp_err_t settings_init(void);   /* also initialises NVS (needed by BLE) */
void settings_reset(void);
esp_err_t settings_set_mic(int index, float x_mm, float y_mm, float z_mm);   /* index 0..5 */
esp_err_t settings_set_speed_of_sound(float mps);   /* 300..380 m/s */
esp_err_t settings_set_algorithm(uint8_t algorithm);
esp_err_t settings_set_map_rate(uint8_t hz);       /* 0..10 */
esp_err_t settings_set_tracker_mode(uint8_t mode);  /* daq_track_mode_t */
esp_err_t settings_set_output_mask(uint8_t mask);   /* DAQ_OUT_* */
esp_err_t system_start(void);    /* battery, LED and CPU load task */
void system_identify(void);      /* blink the LED for DAQ_IDENTIFY_MS */

/* dsp.c — frames, VAD, GCC-PHAT (reference: daq-sim/daq_sim/tdoa.py) */
#define DSP_FRAME_LEN   2048
#define DSP_HOP         1536          /* 32 ms at 48 kHz */
#define DSP_NUM_PAIRS   15            /* (1,2), (1,3), ..., (5,6) */

typedef struct {
    uint64_t frame;                   /* first sample of the frame (capture index, or WAV index in replay) */
    bool     voice;
    float    energy_db;
    float    tau_us[DSP_NUM_PAIRS];   /* tau_ij = t_i - t_j */
    float    peak[DSP_NUM_PAIRS];
    /* for SRP-PHAT (valid until the next frame): correlation r(n), n = -(L+1)..L+1, and PHAT cross spectra */
    const float *corr[DSP_NUM_PAIRS];
    int16_t  max_lag[DSP_NUM_PAIRS];
    const float *wr[DSP_NUM_PAIRS], *wi[DSP_NUM_PAIRS];
    /* band spectra X_c[k] of every mic (MUSIC), bins k_lo .. k_lo + n_band - 1 */
    const float *xr[DAQ_NUM_CHANNELS], *xi[DAQ_NUM_CHANNELS];
    int      k_lo, n_band;
} dsp_result_t;

esp_err_t dsp_start(void);
void dsp_push(const int16_t *block, uint64_t first_frame);   /* called by capture_task for every block */
esp_err_t dsp_replay_start(int index);                       /* RECnnnnn.WAV -> PLYnnnnn.CSV */
bool dsp_replay_running(void);
int dsp_csv_header(char *buf, size_t size, const char *wav_name, uint64_t first_frame);

/* localize.c — algorithms A and B (reference: daq-sim/daq_sim/ls.py, srp.py) */
typedef struct {
    float az_deg, el_deg;
    float dist_m;                 /* 0 = unknown (direction only) */
    float confidence;             /* 0..1 */
    char  kind;                   /* 'P' position, 'D' direction only */
} loc_source_t;

typedef struct {
    char         alg;             /* 'A', 'B' or 'C' */
    char         kind;            /* kind of the first source, '-' = no result */
    uint8_t      n_sources;       /* 0..2 (A: at most 1) */
    loc_source_t src[2];
    bool         has_fit;         /* A: rms_us / n_used are valid */
    float        rms_us;
    uint8_t      n_used;
} loc_result_t;

void localize_a(const dsp_result_t *r, loc_result_t *out);
void localize_b(const dsp_result_t *r, loc_result_t *out, uint8_t map[DAQ_BLE_MAP_AZ_BINS * DAQ_BLE_MAP_EL_BINS]);

/* music.c — algorithm C, MUSIC on M1-M3 (reference: daq-sim/daq_sim/music.py). Call for EVERY frame;
 * returns true when `out` is a result row (voice frame), sources: azimuth only (elevation 0, distance 0, kind 'D'). */
bool localize_c(const dsp_result_t *r, loc_result_t *out);
void music_reset(void);

/* track.c — speaker tracking over time (reference: daq-sim/daq_sim/track.py) */
typedef struct {
    uint8_t id;                   /* 1..255 */
    float   az_deg, el_deg;
    float   dist_m;               /* 0 = unknown */
    float   confidence;
} track_out_t;

void track_reset(uint8_t mode);   /* clears tracks and histogram; ids restart at 1 */
/* Call for every frame (loc NULL or without sources when there is no result); returns the number of confirmed
 * tracks written to out (by decreasing confidence). */
int track_update(uint64_t frame, const loc_result_t *loc, track_out_t out[DAQ_TRACK_MAX]);

/* ble.c — localization output sent over BLE (docs/ble_gatt.md) */
typedef struct {
    float azimuth_deg;     /* 0..360, counter-clockwise from x */
    float elevation_deg;   /* -90..90 */
    float distance_m;      /* 0 = unknown */
    float confidence;      /* 0..1 */
    uint8_t track_id;      /* 0 = not a track (raw result) */
} daq_source_t;

esp_err_t ble_init(void);
void ble_publish_result(const daq_source_t *sources, int n_sources, int speaker_count);  /* tracks, strongest first */
void ble_publish_raw(const daq_source_t *sources, int n_sources, int speaker_count);     /* per-frame sources */
void ble_publish_map(const uint8_t *map, size_t len);  /* DAQ_BLE_MAP_AZ_BINS * DAQ_BLE_MAP_EL_BINS bytes */
