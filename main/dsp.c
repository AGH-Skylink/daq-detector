/*
 * DSP front end (firmware 0.4.0): frames, VAD and GCC-PHAT time differences for the 15 mic pairs.
 *
 * The processing follows the reference implementation daq-sim/daq_sim/tdoa.py step by step
 * (same constants, window, band, VAD rule, lag range and parabolic refinement), so the replay mode
 * can be compared with Python: tools/localize_wav.py --compare PLYnnnnn.CSV (target <= 1 us).
 *
 * Live mode:   capture_task -> dsp_push() -> queue -> dsp_task (core 1); while recording, one CSV line per
 *              hop goes to the SD card next to the WAV file (storage.c, RECnnnnn.CSV).
 * Replay mode: dsp_replay_start(n) reads RECnnnnn.WAV from the card through the same processing and
 *              writes PLYnnnnn.CSV (not real time; live input is ignored meanwhile).
 *
 * Localization (0.6.0): for voice frames with algorithm A or B selected, localize_a() / localize_b()
 * (localize.c) or C (music.c, azimuth only) give up to 2 sources; the tracker (track.c) turns them into speakers over time (every frame).
 * Live output: BLE Result = confirmed tracks, Raw result = sources of the frame (each switchable,
 * g_settings.output_mask), SRP map for B. Both modes write sources and tracks to the CSV; live, the CSV goes to the SD
 * card while recording and to the native USB stream (DAQT text packets) while the stream is on.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "dsps_fft2r.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "config.h"
#include "daq.h"

static const char *TAG = "dsp";

#define N               DSP_FRAME_LEN
#define NCH             DAQ_NUM_CHANNELS
#define BAND_LO_HZ      300.0f
#define BAND_HI_HZ      3500.0f
#define VAD_MARGIN_DB   9.0f
#define VAD_RISE_DB     0.05f
#define MAX_LAG_LIMIT   200          /* > any physical lag + 2 for arrays up to ~1.4 m */
#define QUEUE_BLOCKS    24           /* ~160 ms of audio */
#define CORR_LEN        (2 * MAX_LAG_LIMIT + 3)

typedef struct {
    uint64_t first_frame;
    int16_t data[DAQ_BLOCK_FRAMES * NCH];
} dsp_block_t;

static QueueHandle_t s_queue;
static StaticQueue_t s_queue_struct;

static int16_t *s_win;               /* N frames x 6 channels, interleaved */
static float *s_fft;                 /* 2 * N floats, complex interleaved, 16-byte aligned */
static float s_hann[N];
static float *s_cos, *s_sin;         /* cos / sin (2 pi m / N) */
static int s_k_lo, s_k_hi, s_nb;
static float *s_re, *s_im;           /* band spectra [NCH][s_nb] */
static float *s_wr, *s_wi;           /* PHAT-weighted cross spectra [DSP_NUM_PAIRS][s_nb] */
static float *s_corr;                /* correlations [DSP_NUM_PAIRS][CORR_LEN] */
static uint8_t s_map[DAQ_BLE_MAP_AZ_BINS * DAQ_BLE_MAP_EL_BINS];
static bool s_fft_mirrored;          /* FFT sign convention check, see fft_self_test() */
static bool s_floor_valid;
static float s_floor_db;

static volatile int s_replay_request = -1;
static volatile bool s_replay_active;

/* ============================== processing ============================== */

/* The 15 pairs (i, j), i < j, in the order of daq_sim.geometry.PAIRS and the CSV columns t12 ... t56 */
static const uint8_t PAIR_I[DSP_NUM_PAIRS] = { 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 4 };
static const uint8_t PAIR_J[DSP_NUM_PAIRS] = { 1, 2, 3, 4, 5, 2, 3, 4, 5, 3, 4, 5, 4, 5, 5 };

/* Same rule as Geometry.max_lag_samples(): ceil(d / c * fs) + 2 */
static int max_lag(int i, int j)
{
    const float *a = g_settings.mic_xyz_mm[i], *b = g_settings.mic_xyz_mm[j];
    float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    float d_m = sqrtf(dx * dx + dy * dy + dz * dz) / 1000.0f;
    int lag = (int)ceilf(d_m / g_settings.speed_of_sound_mps * DAQ_SAMPLE_RATE_HZ) + 2;
    return lag > MAX_LAG_LIMIT ? MAX_LAG_LIMIT : lag;
}

/* Two real channels per complex FFT: z = a + j b, then A[k] = (Z[k] + Z*[N-k]) / 2, B[k] = (Z[k] - Z*[N-k]) / 2j */
static void compute_spectra(void)
{
    const float scale = 1.0f / 32768.0f;
    for (int pair = 0; pair < NCH / 2; pair++) {
        int ca = 2 * pair, cb = 2 * pair + 1;
        for (int n = 0; n < N; n++) {
            s_fft[2 * n] = s_win[n * NCH + ca] * scale * s_hann[n];
            s_fft[2 * n + 1] = s_win[n * NCH + cb] * scale * s_hann[n];
        }
        dsps_fft2r_fc32(s_fft, N);
        dsps_bit_rev_fc32(s_fft, N);
        for (int k = s_k_lo; k <= s_k_hi; k++) {
            /* Z[k] and Z[N-k] of the e^{-j} transform; with an e^{+j} FFT they sit at N-k and k */
            int kp = s_fft_mirrored ? N - k : k;
            int kn = N - kp;
            float zr = s_fft[2 * kp], zi = s_fft[2 * kp + 1];
            float nr = s_fft[2 * kn], ni = s_fft[2 * kn + 1];
            int b = k - s_k_lo;
            s_re[ca * s_nb + b] = 0.5f * (zr + nr);
            s_im[ca * s_nb + b] = 0.5f * (zi - ni);
            s_re[cb * s_nb + b] = 0.5f * (zi + ni);
            s_im[cb * s_nb + b] = -0.5f * (zr - nr);
        }
    }
}

static bool vad(float *energy_db_out)
{
    float e = 0.0f;
    for (int c = 0; c < NCH; c++) {
        for (int b = 0; b < s_nb; b++) {
            float re = s_re[c * s_nb + b], im = s_im[c * s_nb + b];
            e += re * re + im * im;
        }
    }
    e /= NCH;
    float db = 10.0f * log10f(e + 1e-20f);
    if (!s_floor_valid) {
        s_floor_db = db;
        s_floor_valid = true;
    } else {
        s_floor_db = fminf(db, s_floor_db + VAD_RISE_DB);
    }
    *energy_db_out = db;
    return db > s_floor_db + VAD_MARGIN_DB;
}

static void gcc_phat(int p, int i, int j, int lag, float *tau_us, float *peak)
{
    float *s_r = &s_corr[p * CORR_LEN];
    float *wr = &s_wr[p * s_nb], *wi = &s_wi[p * s_nb];
    const float *ar = &s_re[i * s_nb], *ai = &s_im[i * s_nb];
    const float *br = &s_re[j * s_nb], *bi = &s_im[j * s_nb];
    for (int b = 0; b < s_nb; b++) {
        float cr = ar[b] * br[b] + ai[b] * bi[b];
        float ci = ai[b] * br[b] - ar[b] * bi[b];
        float mag = sqrtf(cr * cr + ci * ci) + 1e-20f;
        wr[b] = cr / mag;
        wi[b] = ci / mag;
    }
    /* r(n) = Re(sum W[k] e^{j 2 pi k n / N}) / n_band for n = -lag-1 .. lag+1 */
    const float inv_nb = 1.0f / s_nb;
    for (int n = -lag - 1; n <= lag + 1; n++) {
        int step = ((n % N) + N) % N;
        int idx = (int)(((int64_t)s_k_lo * step) % N);
        float sum = 0.0f;
        for (int b = 0; b < s_nb; b++) {
            sum += wr[b] * s_cos[idx] - wi[b] * s_sin[idx];
            idx += step;
            if (idx >= N) {
                idx -= N;
            }
        }
        s_r[n + lag + 1] = sum * inv_nb;
    }
    int m = 1;
    for (int t = 2; t <= 2 * lag + 1; t++) {
        if (s_r[t] > s_r[m]) {
            m = t;
        }
    }
    float y0 = s_r[m], ym = s_r[m - 1], yp = s_r[m + 1];
    float den = ym - 2.0f * y0 + yp;
    float delta = den != 0.0f ? 0.5f * (ym - yp) / den : 0.0f;
    *tau_us = ((m - lag - 1) + delta) * (1e6f / DAQ_SAMPLE_RATE_HZ);
    *peak = y0 - 0.25f * (ym - yp) * delta;
}

/* Processes the frame in s_win */
static void process_frame(uint64_t frame, dsp_result_t *out)
{
    int64_t t0 = esp_timer_get_time();
    compute_spectra();
    out->frame = frame;
    out->voice = vad(&out->energy_db);
    out->k_lo = s_k_lo;
    out->n_band = s_nb;
    for (int p = 0; p < DSP_NUM_PAIRS; p++) {
        int lag = max_lag(PAIR_I[p], PAIR_J[p]);
        gcc_phat(p, PAIR_I[p], PAIR_J[p], lag, &out->tau_us[p], &out->peak[p]);
        out->corr[p] = &s_corr[p * CORR_LEN];
        out->max_lag[p] = (int16_t)lag;
        out->wr[p] = &s_wr[p * s_nb];
        out->wi[p] = &s_wi[p * s_nb];
    }
    for (int c = 0; c < NCH; c++) {
        out->xr[c] = &s_re[c * s_nb];
        out->xi[c] = &s_im[c * s_nb];
    }
    g_daq.dsp_frame_us = (uint32_t)(esp_timer_get_time() - t0);
    g_daq.dsp_voice = out->voice;
}

/* ============================== CSV ============================== */

int dsp_csv_header(char *buf, size_t size, const char *wav_name, uint64_t first_frame)
{
    static const char alg_name[] = { '-', 'A', 'B', 'C' };
    static const char *const trk_name[] = { "-", "AZ", "AZEL", "ELLIPSE" };
    int n = snprintf(buf, size, "# daq-detector %d.%d.%d wav=%s first_frame=%llu fs=%d c=%.1f alg=%c trk=%s mics=",
                     DAQ_FW_VERSION_MAJOR, DAQ_FW_VERSION_MINOR, DAQ_FW_VERSION_PATCH, wav_name,
                     (unsigned long long)first_frame, DAQ_SAMPLE_RATE_HZ, g_settings.speed_of_sound_mps,
                     alg_name[g_settings.algorithm & 3], trk_name[g_settings.tracker_mode & 3]);
    for (int m = 0; m < NCH && n < (int)size; m++) {
        n += snprintf(buf + n, size - n, "%s%.1f,%.1f,%.1f", m ? ";" : "", g_settings.mic_xyz_mm[m][0],
                      g_settings.mic_xyz_mm[m][1], g_settings.mic_xyz_mm[m][2]);
    }
    n += snprintf(buf + n, size - n, "\nframe,vad,energy_db");
    for (int p = 0; p < DSP_NUM_PAIRS && n < (int)size; p++) {
        n += snprintf(buf + n, size - n, ",t%d%d", PAIR_I[p] + 1, PAIR_J[p] + 1);
    }
    for (int p = 0; p < DSP_NUM_PAIRS && n < (int)size; p++) {
        n += snprintf(buf + n, size - n, ",p%d%d", PAIR_I[p] + 1, PAIR_J[p] + 1);
    }
    n += snprintf(buf + n, size - n, ",alg,kind,speakers,az_deg,el_deg,dist_m,conf,res_us,pairs,az2_deg,el2_deg,dist2_m,conf2"
                                     ",tracks,t1_id,t1_az_deg,t1_el_deg,t1_dist_m,t1_conf,t2_id,t2_az_deg,t2_el_deg,t2_dist_m,t2_conf\n");
    return n < (int)size ? n : (int)size - 1;
}

/* loc == NULL: localization columns left empty (no voice or localization off); tracker columns on every line.
 * Same format as daq-sim localize_wav.py */
static int csv_tracks(char *buf, size_t size, const track_out_t *trk, int n_trk)
{
    int n = snprintf(buf, size, ",%d", n_trk);
    for (int t = 0; t < 2 && n < (int)size; t++) {
        if (t < n_trk) {
            n += snprintf(buf + n, size - n, ",%u,%.2f,%.2f,%.3f,%.3f", trk[t].id, trk[t].az_deg, trk[t].el_deg,
                          trk[t].dist_m, trk[t].confidence);
        } else {
            n += snprintf(buf + n, size - n, ",,,,,");
        }
    }
    n += snprintf(buf + n, size - n, "\n");
    return n;
}

static int csv_line(char *buf, size_t size, const dsp_result_t *r, const loc_result_t *loc,
                    const track_out_t *trk, int n_trk)
{
    int n = snprintf(buf, size, "%llu,%d,%.2f", (unsigned long long)r->frame, r->voice ? 1 : 0, r->energy_db);
    for (int p = 0; p < DSP_NUM_PAIRS && n < (int)size; p++) {
        n += snprintf(buf + n, size - n, ",%.3f", r->tau_us[p]);
    }
    for (int p = 0; p < DSP_NUM_PAIRS && n < (int)size; p++) {
        n += snprintf(buf + n, size - n, ",%.4f", r->peak[p]);
    }
    if (!loc) {
        n += snprintf(buf + n, size - n, ",,,,,,,,,,,,,");
        if (n < (int)size) {
            n += csv_tracks(buf + n, size - n, trk, n_trk);
        }
        return n < (int)size ? n : (int)size - 1;
    }
    n += snprintf(buf + n, size - n, ",%c,%c,%u", loc->alg, loc->kind, loc->n_sources);
    for (int s = 0; s < 2 && n < (int)size; s++) {
        const loc_source_t *src = &loc->src[s];
        if (s < loc->n_sources) {
            n += snprintf(buf + n, size - n, ",%.2f,%.2f,%.3f,%.3f", src->az_deg, src->el_deg, src->dist_m, src->confidence);
        } else {
            n += snprintf(buf + n, size - n, ",,,,");
        }
        if (s == 0) {
            if (loc->has_fit) {
                n += snprintf(buf + n, size - n, ",%.2f,%u", loc->rms_us, loc->n_used);
            } else {
                n += snprintf(buf + n, size - n, ",,");
            }
        }
    }
    if (n < (int)size) {
        n += csv_tracks(buf + n, size - n, trk, n_trk);
    }
    return n < (int)size ? n : (int)size - 1;
}

/* Tracker on every frame; the mode is taken from the settings (a change restarts the tracker) */
static uint8_t s_track_mode;

static int track(const dsp_result_t *r, const loc_result_t *loc, track_out_t trk[DAQ_TRACK_MAX])
{
    if (g_settings.tracker_mode != s_track_mode) {
        s_track_mode = g_settings.tracker_mode;
        track_reset(s_track_mode);
    }
    return track_update(r->frame, loc, trk);
}

/* Algorithm A or B on a voice frame, C on every frame (its covariance decays without voice);
 * NULL if there is no result row. *map_fresh: B produced a new SRP map */
static const loc_result_t *locate(const dsp_result_t *r, loc_result_t *loc, bool *map_fresh)
{
    static uint8_t last_alg;
    *map_fresh = false;
    if (g_settings.algorithm != last_alg) {
        last_alg = g_settings.algorithm;
        music_reset();
    }
    if (g_settings.algorithm == DAQ_ALG_C_MUSIC) {
        int64_t t0 = esp_timer_get_time();
        bool row = localize_c(r, loc);
        if (r->voice) {
            g_daq.loc_us = (uint32_t)(esp_timer_get_time() - t0);
        }
        return row ? loc : NULL;
    }
    if (!r->voice || (g_settings.algorithm != DAQ_ALG_A_GCC_LS && g_settings.algorithm != DAQ_ALG_B_SRP)) {
        return NULL;
    }
    int64_t t0 = esp_timer_get_time();
    if (g_settings.algorithm == DAQ_ALG_A_GCC_LS) {
        localize_a(r, loc);
    } else {
        localize_b(r, loc, s_map);
        *map_fresh = true;
    }
    g_daq.loc_us = (uint32_t)(esp_timer_get_time() - t0);
    return loc;
}

/* Live output: BLE results and the status line values */
static void publish(const dsp_result_t *r, const loc_result_t *loc, bool map_fresh, const track_out_t *trk, int n_trk)
{
    if (map_fresh) {
        ble_publish_map(s_map, sizeof(s_map));
    }
    daq_source_t src[DAQ_BLE_MAX_SOURCES];
    g_daq.trk_count = (uint8_t)n_trk;
    g_daq.trk_az_deg = n_trk ? trk[0].az_deg : 0.0f;
    bool localizing = g_settings.algorithm >= DAQ_ALG_A_GCC_LS && g_settings.algorithm <= DAQ_ALG_C_MUSIC;
    if ((g_settings.output_mask & DAQ_OUT_TRACKED) && localizing) {
        /* every frame, also with 0 tracks: "nobody speaks" is a valid tracked result */
        int n = n_trk < DAQ_BLE_MAX_SOURCES ? n_trk : DAQ_BLE_MAX_SOURCES;
        for (int t = 0; t < n; t++) {
            src[t] = (daq_source_t) {
                .azimuth_deg = trk[t].az_deg,
                .elevation_deg = trk[t].el_deg,
                .distance_m = trk[t].dist_m,
                .confidence = trk[t].confidence,
                .track_id = trk[t].id,
            };
        }
        ble_publish_result(src, n, n_trk);
    }
    if (!loc || loc->n_sources == 0) {
        return;   /* BLE reports the raw result as not valid once it is older than 500 ms */
    }
    if (g_settings.output_mask & DAQ_OUT_RAW) {
        for (int s = 0; s < loc->n_sources; s++) {
            src[s] = (daq_source_t) {
                .azimuth_deg = loc->src[s].az_deg,
                .elevation_deg = loc->src[s].el_deg,
                .distance_m = loc->src[s].dist_m,     /* 0 = unknown (direction only) */
                .confidence = loc->src[s].confidence,
                .track_id = 0,
            };
        }
        ble_publish_raw(src, loc->n_sources, loc->n_sources);
    }
    g_daq.loc_az_deg = loc->src[0].az_deg;
    g_daq.loc_el_deg = loc->src[0].el_deg;
    g_daq.loc_dist_m = loc->src[0].dist_m;
    g_daq.loc_speakers = loc->n_sources;
    g_daq.loc_frame = r->frame;
}

/* ============================== replay ============================== */

/* Finds the data chunk of a 16-bit 6-channel WAV; returns the number of frames (data_size 0 -> to EOF) */
static long wav_open_data(FILE *f)
{
    uint8_t h[12];
    if (fread(h, 1, 12, f) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) {
        return -1;
    }
    uint16_t channels = 0, bits = 0;
    uint32_t rate = 0;
    while (true) {
        uint8_t ch[8];
        if (fread(ch, 1, 8, f) != 8) {
            return -1;
        }
        uint32_t size = ch[4] | ch[5] << 8 | ch[6] << 16 | (uint32_t)ch[7] << 24;
        if (!memcmp(ch, "fmt ", 4)) {
            uint8_t fmt[16];
            if (size < 16 || fread(fmt, 1, 16, f) != 16) {
                return -1;
            }
            channels = fmt[2] | fmt[3] << 8;
            rate = fmt[4] | fmt[5] << 8 | fmt[6] << 16 | (uint32_t)fmt[7] << 24;
            bits = fmt[14] | fmt[15] << 8;
            fseek(f, size - 16 + (size & 1), SEEK_CUR);
        } else if (!memcmp(ch, "data", 4)) {
            if (channels != NCH || bits != 16 || rate != DAQ_SAMPLE_RATE_HZ) {
                ESP_LOGE(TAG, "replay needs %d ch, 16 bit, %d Hz (got %u, %u, %lu)", NCH, DAQ_SAMPLE_RATE_HZ,
                         channels, bits, (unsigned long)rate);
                return -1;
            }
            long pos = ftell(f);
            fseek(f, 0, SEEK_END);
            long avail = ftell(f) - pos;
            fseek(f, pos, SEEK_SET);
            long bytes = (size == 0 || size > (uint32_t)avail) ? avail : (long)size;
            return bytes / (NCH * 2);
        } else {
            fseek(f, size + (size & 1), SEEK_CUR);
        }
    }
}

static void run_replay(int index)
{
    char wav[48], csv[48], name[24], line[800];   /* sized for any int index (-Wformat-truncation at -O2) */
    snprintf(name, sizeof(name), "REC%05d.WAV", index);
    snprintf(wav, sizeof(wav), DAQ_SD_MOUNT_POINT "/%s", name);
    snprintf(csv, sizeof(csv), DAQ_SD_MOUNT_POINT "/PLY%05d.CSV", index);

    FILE *in = fopen(wav, "rb");
    long frames = in ? wav_open_data(in) : -1;
    FILE *out = frames >= N ? fopen(csv, "w") : NULL;
    if (!out) {
        ESP_LOGE(TAG, "replay: cannot use %s", wav);
        if (in) {
            fclose(in);
        }
        return;
    }
    char *header = malloc(1024);
    if (header) {
        dsp_csv_header(header, 1024, name, 0);
        fputs(header, out);
        free(header);
    }

    ESP_LOGI(TAG, "replay %s (%.1f s) -> %s", name, (float)frames / DAQ_SAMPLE_RATE_HZ, csv);
    s_floor_valid = false;
    s_track_mode = g_settings.tracker_mode;
    track_reset(s_track_mode);            /* replay starts from an empty tracker, as localize_wav.py */
    music_reset();
    dsp_result_t r;
    loc_result_t loc;
    track_out_t trk[DAQ_TRACK_MAX];
    bool map_fresh;
    long pos = 0, n_frames = 0, n_voice = 0;
    int64_t busy_us = 0;
    size_t got = fread(s_win, NCH * 2, N, in);
    while (got == N) {
        g_daq.replay_progress_pct = (uint8_t)(100 * (pos + N) / frames);
        process_frame((uint64_t)pos, &r);
        busy_us += g_daq.dsp_frame_us;
        n_frames++;
        n_voice += r.voice;
        const loc_result_t *lp = locate(&r, &loc, &map_fresh);
        int n_trk = track(&r, lp, trk);
        fwrite(line, 1, csv_line(line, sizeof(line), &r, lp, trk, n_trk), out);
        if (pos + DSP_HOP + N > frames) {
            break;
        }
        memmove(s_win, s_win + DSP_HOP * NCH, (N - DSP_HOP) * NCH * 2);
        got = (N - DSP_HOP) + fread(s_win + (N - DSP_HOP) * NCH, NCH * 2, DSP_HOP, in);
        pos += DSP_HOP;
    }
    fclose(out);
    fclose(in);
    s_floor_valid = false;
    track_reset(s_track_mode);            /* live tracking restarts after the replay */
    music_reset();
    ESP_LOGI(TAG, "replay done: %ld frames, %ld with voice, %.2f ms per frame", n_frames, n_voice,
             n_frames ? busy_us / 1000.0 / n_frames : 0.0);
}

/* ============================== task ============================== */

static void dsp_task(void *arg)
{
    static dsp_block_t blk;
    static char line[800];
    static char header[1024];
    bool usb_header_sent = false;
    int have = 0;              /* frames in s_win */
    int blk_pos = DAQ_BLOCK_FRAMES;
    uint64_t win_start = 0, expected = 0;
    dsp_result_t r;
    loc_result_t loc;
    track_out_t trk[DAQ_TRACK_MAX];
    bool map_fresh;

    while (true) {
        int idx = s_replay_request;
        if (idx >= 0) {
            s_replay_active = true;
            run_replay(idx);
            s_replay_request = -1;
            xQueueReset(s_queue);
            have = 0;
            blk_pos = DAQ_BLOCK_FRAMES;
            s_replay_active = false;
            continue;
        }

        /* Fill the window from capture blocks; a gap (lost block) restarts the window */
        while (have < N) {
            if (blk_pos == DAQ_BLOCK_FRAMES) {
                if (xQueueReceive(s_queue, &blk, pdMS_TO_TICKS(100)) != pdTRUE) {
                    break;
                }
                if (have > 0 && blk.first_frame != expected) {
                    have = 0;
                }
                expected = blk.first_frame + DAQ_BLOCK_FRAMES;
                blk_pos = 0;
            }
            if (have == 0) {
                win_start = blk.first_frame + blk_pos;
            }
            int n = DAQ_BLOCK_FRAMES - blk_pos;
            if (n > N - have) {
                n = N - have;
            }
            memcpy(&s_win[have * NCH], &blk.data[blk_pos * NCH], n * NCH * sizeof(int16_t));
            have += n;
            blk_pos += n;
        }
        if (have < N) {
            continue;   /* no data (timeout) or replay requested */
        }

        process_frame(win_start, &r);
        const loc_result_t *lp = locate(&r, &loc, &map_fresh);
        int n_trk = track(&r, lp, trk);
        publish(&r, lp, map_fresh, trk, n_trk);
        bool to_sd = g_daq.recording && win_start >= g_daq.rec_first_frame;
        bool to_usb = g_daq.streaming;
        if (!to_usb) {
            usb_header_sent = false;
        }
        if (to_sd || to_usb) {
            int len = csv_line(line, sizeof(line), &r, lp, trk, n_trk);
            if (to_sd && !storage_push_csv(line, len)) {
                g_daq.csv_dropped_lines++;
            }
            if (to_usb) {
                /* detection log on the USB stream (DAQT packets): CSV header once per stream start, then every
                 * frame; `frame` = capture sample index, the same clock as first_frame of the audio packets */
                if (!usb_header_sent) {
                    usb_header_sent = stream_push_text(header, dsp_csv_header(header, sizeof(header), "usb", 0));
                }
                if (!usb_header_sent || !stream_push_text(line, len)) {
                    g_daq.csv_dropped_lines++;
                }
            }
        }
        memmove(s_win, s_win + DSP_HOP * NCH, (N - DSP_HOP) * NCH * sizeof(int16_t));
        have -= DSP_HOP;
        win_start += DSP_HOP;
    }
}

/* ============================== init / API ============================== */

/* esp-dsp does not document the sign of its FFT: transform e^{+j 2 pi n / N} and see where the energy goes */
static void fft_self_test(void)
{
    for (int n = 0; n < N; n++) {
        s_fft[2 * n] = s_cos[n];
        s_fft[2 * n + 1] = s_sin[n];
    }
    dsps_fft2r_fc32(s_fft, N);
    dsps_bit_rev_fc32(s_fft, N);
    float bin1 = hypotf(s_fft[2], s_fft[3]);
    float binm = hypotf(s_fft[2 * (N - 1)], s_fft[2 * (N - 1) + 1]);
    s_fft_mirrored = binm > bin1;
    ESP_LOGI(TAG, "FFT convention %s", s_fft_mirrored ? "e^{+j} (mirrored)" : "e^{-j}");
}

bool dsp_fft_mirrored(void)
{
    return s_fft_mirrored;
}

esp_err_t dsp_start(void)
{
    s_k_lo = (int)ceilf(BAND_LO_HZ * N / DAQ_SAMPLE_RATE_HZ);
    s_k_hi = (int)floorf(BAND_HI_HZ * N / DAQ_SAMPLE_RATE_HZ);
    s_nb = s_k_hi - s_k_lo + 1;

    s_win = heap_caps_malloc(N * NCH * sizeof(int16_t), MALLOC_CAP_INTERNAL);
    s_fft = heap_caps_aligned_alloc(16, 2 * N * sizeof(float), MALLOC_CAP_INTERNAL);
    s_cos = heap_caps_malloc(N * sizeof(float), MALLOC_CAP_INTERNAL);
    s_sin = heap_caps_malloc(N * sizeof(float), MALLOC_CAP_INTERNAL);
    s_re = heap_caps_malloc(NCH * s_nb * sizeof(float), MALLOC_CAP_INTERNAL);
    s_im = heap_caps_malloc(NCH * s_nb * sizeof(float), MALLOC_CAP_INTERNAL);
    s_wr = heap_caps_malloc(DSP_NUM_PAIRS * s_nb * sizeof(float), MALLOC_CAP_INTERNAL);
    s_wi = heap_caps_malloc(DSP_NUM_PAIRS * s_nb * sizeof(float), MALLOC_CAP_INTERNAL);
    s_corr = heap_caps_malloc(DSP_NUM_PAIRS * CORR_LEN * sizeof(float), MALLOC_CAP_INTERNAL);
    uint8_t *q_mem = heap_caps_malloc(QUEUE_BLOCKS * sizeof(dsp_block_t), MALLOC_CAP_SPIRAM);
    if (!s_win || !s_fft || !s_cos || !s_sin || !s_re || !s_im || !s_wr || !s_wi || !s_corr || !q_mem) {
        return ESP_ERR_NO_MEM;
    }
    s_queue = xQueueCreateStatic(QUEUE_BLOCKS, sizeof(dsp_block_t), q_mem, &s_queue_struct);

    for (int n = 0; n < N; n++) {
        s_hann[n] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * n / N);   /* periodic Hann */
        s_cos[n] = cosf(2.0f * (float)M_PI * n / N);
        s_sin[n] = sinf(2.0f * (float)M_PI * n / N);
    }
    ESP_RETURN_ON_ERROR(dsps_fft2r_init_fc32(NULL, N), TAG, "fft init");
    fft_self_test();

    BaseType_t ok = xTaskCreatePinnedToCore(dsp_task, "dsp", 6144, NULL, 8, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void dsp_push(const int16_t *block, uint64_t first_frame)
{
    if (!s_queue || s_replay_active) {
        return;
    }
    static dsp_block_t b;   /* called only from capture_task */
    b.first_frame = first_frame;
    memcpy(b.data, block, sizeof(b.data));
    if (xQueueSend(s_queue, &b, 0) != pdTRUE) {
        g_daq.dsp_dropped_blocks++;
    }
}

esp_err_t dsp_replay_start(int index)
{
    if (g_daq.recording || g_daq.record_request) {
        return ESP_ERR_INVALID_STATE;   /* the card is busy with recording */
    }
    if (s_replay_active || s_replay_request >= 0 || !g_daq.sd_ready || index < 1 || index > 99999) {
        return ESP_ERR_INVALID_ARG;
    }
    g_daq.replay_progress_pct = 0;
    s_replay_request = index;
    return ESP_OK;
}

bool dsp_replay_running(void)
{
    return s_replay_active || s_replay_request >= 0;
}
