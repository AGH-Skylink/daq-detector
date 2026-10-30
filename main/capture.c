#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "config.h"
#include "daq.h"

#define BLOCK_SAMPLES  (DAQ_BLOCK_FRAMES * DAQ_NUM_CHANNELS)
#define BLOCK_BYTES    (BLOCK_SAMPLES * sizeof(int16_t))
#define WARMUP_BLOCKS  ((DAQ_SAMPLE_RATE_HZ / 1000 * DAQ_WARMUP_MS) / DAQ_BLOCK_FRAMES)

static const char *TAG = "capture";

static i2s_chan_handle_t s_rx_chan;
static const uint8_t s_channel_map[DAQ_NUM_CHANNELS] = DAQ_CHANNEL_MAP;

/* Level preview accumulators, read and reset by the UI task */
static portMUX_TYPE s_level_lock = portMUX_INITIALIZER_UNLOCKED;
static double s_sum_sq[DAQ_NUM_CHANNELS];
static uint32_t s_level_frames;

static IRAM_ATTR bool on_recv_overflow(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    g_daq.dma_overflows++;
    return false;
}

/* Drive the SEL pin of every mic (slot selection) before the PDM clock starts */
static esp_err_t mic_sel_init(void)
{
    static const gpio_num_t pins[DAQ_NUM_CHANNELS] = DAQ_PIN_MIC_SEL;
    static const uint8_t levels[DAQ_NUM_CHANNELS] = DAQ_MIC_SEL_LEVEL;
    for (int m = 0; m < DAQ_NUM_CHANNELS; m++) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << pins[m],
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "SEL M%d", m + 1);
        ESP_RETURN_ON_ERROR(gpio_set_level(pins[m], levels[m]), TAG, "SEL M%d level", m + 1);
    }
    return ESP_OK;
}

static esp_err_t pdm_init(void)
{
    ESP_RETURN_ON_ERROR(mic_sel_init(), TAG, "mic_sel_init");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DAQ_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = DAQ_BLOCK_FRAMES;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan), TAG, "i2s_new_channel");

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(DAQ_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .clk = DAQ_PIN_PDM_CLK,
            .dins = { DAQ_PIN_PDM_DIN0, DAQ_PIN_PDM_DIN1, DAQ_PIN_PDM_DIN2, I2S_GPIO_UNUSED },
            .invert_flags = { .clk_inv = false },
        },
    };
    /* 3 lines x (left + right) = 6 channels; the hardware PDM->PCM filter outputs 16-bit PCM */
    pdm_cfg.slot_cfg.slot_mask =
        I2S_PDM_RX_LINE0_SLOT_LEFT | I2S_PDM_RX_LINE0_SLOT_RIGHT |
        I2S_PDM_RX_LINE1_SLOT_LEFT | I2S_PDM_RX_LINE1_SLOT_RIGHT |
        I2S_PDM_RX_LINE2_SLOT_LEFT | I2S_PDM_RX_LINE2_SLOT_RIGHT;
    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(s_rx_chan, &pdm_cfg), TAG, "init_pdm_rx_mode");

    i2s_event_callbacks_t cbs = { .on_recv_q_ovf = on_recv_overflow };
    ESP_RETURN_ON_ERROR(i2s_channel_register_event_callback(s_rx_chan, &cbs, NULL), TAG, "register_callback");

    return i2s_channel_enable(s_rx_chan);
}

static void capture_task(void *arg)
{
    static int16_t raw[BLOCK_SAMPLES];
    static int16_t out[BLOCK_SAMPLES];
    uint32_t seq = 0;
    size_t got = 0;

    for (int i = 0; i < WARMUP_BLOCKS; i++) {
        i2s_channel_read(s_rx_chan, raw, BLOCK_BYTES, &got, portMAX_DELAY);
    }
    ESP_LOGI(TAG, "capturing %d ch @ %d Hz", DAQ_NUM_CHANNELS, DAQ_SAMPLE_RATE_HZ);

    while (true) {
        if (i2s_channel_read(s_rx_chan, raw, BLOCK_BYTES, &got, portMAX_DELAY) != ESP_OK || got != BLOCK_BYTES) {
            continue;
        }

        /* Reorder raw slots to M1..M6 and accumulate energy (integer math, no soft-float doubles in the loop) */
        int64_t sum_sq[DAQ_NUM_CHANNELS] = { 0 };
        for (int f = 0; f < DAQ_BLOCK_FRAMES; f++) {
            const int16_t *src = &raw[f * DAQ_NUM_CHANNELS];
            int16_t *dst = &out[f * DAQ_NUM_CHANNELS];
            for (int c = 0; c < DAQ_NUM_CHANNELS; c++) {
                int16_t s = src[s_channel_map[c]];
                dst[c] = s;
                sum_sq[c] += (int32_t)s * s;
            }
        }
        portENTER_CRITICAL(&s_level_lock);
        for (int c = 0; c < DAQ_NUM_CHANNELS; c++) {
            s_sum_sq[c] += (double)sum_sq[c];
        }
        s_level_frames += DAQ_BLOCK_FRAMES;
        portEXIT_CRITICAL(&s_level_lock);

        if (g_daq.recording && !storage_push(out, BLOCK_BYTES)) {
            g_daq.sd_dropped_blocks++;
        }
        if (g_daq.streaming) {
            daq_packet_header_t header = {
                .magic = DAQ_STREAM_MAGIC,
                .seq = seq,
                .first_frame = g_daq.frames_captured,
                .sample_rate = DAQ_SAMPLE_RATE_HZ,
                .channels = DAQ_NUM_CHANNELS,
                .frames = DAQ_BLOCK_FRAMES,
            };
            if (!stream_push(&header, out, BLOCK_BYTES)) {
                g_daq.usb_dropped_blocks++;
            }
        }
        seq++;  /* incremented for every block, so the host sees gaps caused by drops */
        g_daq.frames_captured += DAQ_BLOCK_FRAMES;
    }
}

esp_err_t capture_start(void)
{
    ESP_RETURN_ON_ERROR(pdm_init(), TAG, "pdm_init");
    BaseType_t ok = xTaskCreatePinnedToCore(capture_task, "capture", 4096, NULL, 10, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void capture_get_levels(float out_dbfs[], size_t count)
{
    double sums[DAQ_NUM_CHANNELS];
    uint32_t frames;

    portENTER_CRITICAL(&s_level_lock);
    memcpy(sums, s_sum_sq, sizeof(sums));
    frames = s_level_frames;
    memset(s_sum_sq, 0, sizeof(s_sum_sq));
    s_level_frames = 0;
    portEXIT_CRITICAL(&s_level_lock);

    for (size_t c = 0; c < count && c < DAQ_NUM_CHANNELS; c++) {
        double rms = frames ? sqrt(sums[c] / frames) : 0.0;
        out_dbfs[c] = rms > 0.0 ? (float)(20.0 * log10(rms / 32768.0)) : -120.0f;
    }
}
