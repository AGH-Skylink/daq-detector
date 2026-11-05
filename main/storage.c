#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "config.h"
#include "daq.h"

static const char *TAG = "storage";

static StreamBufferHandle_t s_ring;
static StaticStreamBuffer_t s_ring_struct;
static StreamBufferHandle_t s_csv_ring;      /* text lines from dsp.c */
static StaticStreamBuffer_t s_csv_ring_struct;
static char s_csv_chunk[2048];
static uint8_t *s_chunk;          /* internal, DMA-capable buffer for fwrite */
static sdmmc_card_t *s_card;

typedef struct __attribute__((packed)) {
    char     riff[4];
    uint32_t riff_size;
    char     wave[4];
    char     fmt[4];
    uint32_t fmt_size;
    uint16_t format;              /* 1 = PCM */
    uint16_t channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char     data[4];
    uint32_t data_size;
} wav_header_t;

static void wav_header_fill(wav_header_t *h, uint32_t data_bytes)
{
    const uint16_t block_align = DAQ_NUM_CHANNELS * sizeof(int16_t);
    memcpy(h->riff, "RIFF", 4);
    h->riff_size = 36 + data_bytes;
    memcpy(h->wave, "WAVE", 4);
    memcpy(h->fmt, "fmt ", 4);
    h->fmt_size = 16;
    h->format = 1;
    h->channels = DAQ_NUM_CHANNELS;
    h->sample_rate = DAQ_SAMPLE_RATE_HZ;
    h->byte_rate = DAQ_SAMPLE_RATE_HZ * block_align;
    h->block_align = block_align;
    h->bits_per_sample = 16;
    memcpy(h->data, "data", 4);
    h->data_size = data_bytes;
}

static esp_err_t sd_mount(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = DAQ_PIN_SD_MOSI,
        .miso_io_num = DAQ_PIN_SD_MISO,
        .sclk_io_num = DAQ_PIN_SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO), TAG, "spi_bus_initialize");

    /* Pololu 2597 has no pull-ups; internal ones are weak, external 10 kOhm are recommended */
    gpio_set_pull_mode(DAQ_PIN_SD_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(DAQ_PIN_SD_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(DAQ_PIN_SD_CS, GPIO_PULLUP_ONLY);

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = DAQ_SD_FREQ_KHZ;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = DAQ_PIN_SD_CS;
    slot.host_id = SPI2_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,   /* WAV + CSV while recording, WAV + CSV in replay */
        .allocation_unit_size = 32 * 1024,
    };
    ESP_RETURN_ON_ERROR(esp_vfs_fat_sdspi_mount(DAQ_SD_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card),
                        TAG, "mount");
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

/* Opens the next free RECnnnnn.WAV (8.3 name, no LFN needed) and writes a placeholder header */
static FILE *open_next_file(void)
{
    char path[32];
    struct stat st;
    for (uint32_t i = g_daq.sd_file_index + 1; i < 100000; i++) {
        snprintf(path, sizeof(path), DAQ_SD_MOUNT_POINT "/REC%05lu.WAV", (unsigned long)i);
        if (stat(path, &st) == 0) {
            continue;
        }
        FILE *f = fopen(path, "wb");
        if (!f) {
            ESP_LOGE(TAG, "cannot create %s", path);
            return NULL;
        }
        wav_header_t h;
        wav_header_fill(&h, 0);   /* size 0 until close; readers fall back to file size */
        fwrite(&h, sizeof(h), 1, f);
        g_daq.sd_file_index = i;
        ESP_LOGI(TAG, "recording to %s", path);
        return f;
    }
    return NULL;
}

static void close_file(FILE *f, uint32_t data_bytes)
{
    wav_header_t h;
    wav_header_fill(&h, data_bytes);
    fseek(f, 0, SEEK_SET);
    fwrite(&h, sizeof(h), 1, f);
    fclose(f);
    ESP_LOGI(TAG, "closed REC%05lu.WAV, %.1f MB", (unsigned long)g_daq.sd_file_index, data_bytes / 1e6);
}

static bool write_chunk(FILE *f, size_t n, uint32_t *data_bytes)
{
    if (fwrite(s_chunk, 1, n, f) != n) {
        g_daq.sd_write_errors++;
        return false;
    }
    *data_bytes += n;
    g_daq.sd_bytes_written += n;
    return true;
}

/* CSV log next to the WAV file (same number). The header needs the frame index of the WAV start,
 * which capture sets with the first block, so it is written together with the first lines. */
typedef struct {
    FILE *f;
    bool header_pending;
    uint64_t file_first_byte;   /* audio bytes of this recording session before this WAV file */
} csv_log_t;

static void csv_open(csv_log_t *log, uint64_t session_bytes)
{
    char path[32];
    snprintf(path, sizeof(path), DAQ_SD_MOUNT_POINT "/REC%05lu.CSV", (unsigned long)g_daq.sd_file_index);
    log->f = fopen(path, "w");
    log->header_pending = true;
    log->file_first_byte = session_bytes;
}

static void csv_drain(csv_log_t *log)
{
    size_t n;
    while ((n = xStreamBufferReceive(s_csv_ring, s_csv_chunk, sizeof(s_csv_chunk), 0)) > 0) {
        if (!log->f) {
            continue;   /* no file: discard */
        }
        if (log->header_pending) {
            char name[16], header[1024];
            snprintf(name, sizeof(name), "REC%05lu.WAV", (unsigned long)g_daq.sd_file_index);
            uint64_t first = g_daq.rec_first_frame + log->file_first_byte / (DAQ_NUM_CHANNELS * sizeof(int16_t));
            fwrite(header, 1, dsp_csv_header(header, sizeof(header), name, first), log->f);
            log->header_pending = false;
        }
        fwrite(s_csv_chunk, 1, n, log->f);
    }
}

static void csv_close(csv_log_t *log)
{
    csv_drain(log);
    if (log->f) {
        fclose(log->f);
        log->f = NULL;
    }
}

static void storage_task(void *arg)
{
    FILE *f = NULL;
    csv_log_t csv = { 0 };
    uint32_t data_bytes = 0;
    uint64_t session_bytes = 0;
    TickType_t file_start = 0;

    while (true) {
        if (!f) {
            if (!g_daq.record_request) {
                xStreamBufferReset(s_csv_ring);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            f = open_next_file();
            if (!f) {
                g_daq.record_request = false;
                continue;
            }
            data_bytes = 0;
            session_bytes = 0;
            csv_open(&csv, session_bytes);
            file_start = xTaskGetTickCount();
            g_daq.recording = true;   /* capture starts pushing from now on */
        }

        size_t n = xStreamBufferReceive(s_ring, s_chunk, DAQ_SD_CHUNK_BYTES, pdMS_TO_TICKS(100));
        bool ok = (n == 0) || write_chunk(f, n, &data_bytes);
        csv_drain(&csv);

        if (!ok || !g_daq.record_request) {
            /* Stop: block new data, drain what is buffered, finalise the files */
            g_daq.recording = false;
            while (ok && (n = xStreamBufferReceive(s_ring, s_chunk, DAQ_SD_CHUNK_BYTES, 0)) > 0) {
                ok = write_chunk(f, n, &data_bytes);
            }
            xStreamBufferReset(s_ring);
            close_file(f, data_bytes);
            csv_close(&csv);
            f = NULL;
            g_daq.record_request = false;
        } else if (xTaskGetTickCount() - file_start >= pdMS_TO_TICKS(DAQ_SD_FILE_SECONDS * 1000)) {
            /* Gapless rotation: capture keeps pushing, the ring buffer absorbs the reopen time */
            close_file(f, data_bytes);
            csv_close(&csv);
            session_bytes += data_bytes;
            f = open_next_file();
            data_bytes = 0;
            file_start = xTaskGetTickCount();
            if (f) {
                csv_open(&csv, session_bytes);
            } else {
                g_daq.recording = false;
                g_daq.record_request = false;
                xStreamBufferReset(s_ring);
            }
        }
    }
}

esp_err_t storage_init(void)
{
    uint8_t *ring_mem = heap_caps_malloc(DAQ_SD_RING_BYTES + 1, MALLOC_CAP_SPIRAM);
    s_chunk = heap_caps_malloc(DAQ_SD_CHUNK_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!ring_mem || !s_chunk) {
        return ESP_ERR_NO_MEM;
    }
    s_ring = xStreamBufferCreateStatic(DAQ_SD_RING_BYTES, DAQ_SD_CHUNK_BYTES, ring_mem, &s_ring_struct);
    uint8_t *csv_mem = heap_caps_malloc(DAQ_SD_CSV_RING_BYTES + 1, MALLOC_CAP_SPIRAM);
    if (!csv_mem) {
        return ESP_ERR_NO_MEM;
    }
    s_csv_ring = xStreamBufferCreateStatic(DAQ_SD_CSV_RING_BYTES, 1, csv_mem, &s_csv_ring_struct);

    ESP_RETURN_ON_ERROR(sd_mount(), TAG, "sd_mount");
    g_daq.sd_ready = true;

    BaseType_t ok = xTaskCreatePinnedToCore(storage_task, "storage", 4096, NULL, 5, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

bool storage_push(const void *data, size_t len)
{
    /* Single producer: if there is room now, the whole block fits */
    if (xStreamBufferSpacesAvailable(s_ring) < len) {
        return false;
    }
    return xStreamBufferSend(s_ring, data, len, 0) == len;
}

bool storage_push_csv(const char *line, size_t len)
{
    /* Single producer (dsp_task): a line goes in whole or not at all */
    if (!s_csv_ring || xStreamBufferSpacesAvailable(s_csv_ring) < len) {
        return false;
    }
    return xStreamBufferSend(s_csv_ring, line, len, 0) == len;
}
