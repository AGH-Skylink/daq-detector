#include <inttypes.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "config.h"
#include "daq.h"

static const char *TAG = "main";

daq_state_t g_daq;

static void toggle_recording(void)
{
    if (!g_daq.sd_ready) {
        ESP_LOGW(TAG, "SD card not available");
        return;
    }
    g_daq.record_request = !g_daq.record_request;
    ESP_LOGI(TAG, "recording %s requested", g_daq.record_request ? "start" : "stop");
}

static void print_help(void)
{
    printf("keys: r = start/stop SD recording, u = start/stop USB stream, h = help\n");
}

static void print_status(void)
{
    float levels[DAQ_NUM_CHANNELS];
    capture_get_levels(levels, DAQ_NUM_CHANNELS);
    for (int c = 0; c < DAQ_NUM_CHANNELS; c++) {
        g_daq.level_dbfs[c] = levels[c];   /* shared with the BLE diagnostics */
    }

    printf("t=%" PRIu32 "s | dBFS", (uint32_t)(g_daq.frames_captured / DAQ_SAMPLE_RATE_HZ));
    for (int c = 0; c < DAQ_NUM_CHANNELS; c++) {
        printf(" M%d:%6.1f", c + 1, levels[c]);
    }
    printf(" | rec=%s REC%05" PRIu32 " %.1fMB err=%" PRIu32,
           g_daq.recording ? "ON " : "off", g_daq.sd_file_index,
           g_daq.sd_bytes_written / 1e6, g_daq.sd_write_errors);
    printf(" | usb=%s %.1fMB", g_daq.streaming ? "ON " : "off", g_daq.usb_bytes_sent / 1e6);
    printf(" | ble=%s", g_daq.ble_connected ? "ON " : "off");
    printf(" | drop dma=%" PRIu32 " sd=%" PRIu32 " usb=%" PRIu32 "\n",
           g_daq.dma_overflows, g_daq.sd_dropped_blocks, g_daq.usb_dropped_blocks);
}

/* UART0 console (CP2102N port): key commands, button and 1 s status line */
static void ui_task(void *arg)
{
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << DAQ_PIN_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn);

    int last_btn = 1;
    TickType_t last_status = xTaskGetTickCount();
    print_help();

    while (true) {
        int ch = getchar();   /* non-blocking on the default UART VFS */
        if (ch == EOF) {
            clearerr(stdin);
        } else if (ch == 'r') {
            toggle_recording();
        } else if (ch == 'u') {
            stream_set_enabled(!g_daq.streaming);
        } else if (ch == 'h') {
            print_help();
        }

        int level = gpio_get_level(DAQ_PIN_BUTTON);   /* 20 ms polling also debounces */
        if (last_btn == 1 && level == 0) {
            toggle_recording();
        }
        last_btn = level;

        if (xTaskGetTickCount() - last_status >= pdMS_TO_TICKS(DAQ_STATUS_PERIOD_MS)) {
            last_status = xTaskGetTickCount();
            print_status();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    if (storage_init() != ESP_OK) {
        ESP_LOGW(TAG, "SD card not available, recording disabled");
    }
    ESP_ERROR_CHECK(stream_init());
    ESP_ERROR_CHECK(capture_start());
    if (ble_init() != ESP_OK) {
        ESP_LOGW(TAG, "BLE not available");
    }
    xTaskCreatePinnedToCore(ui_task, "ui", 4096, NULL, 3, NULL, 0);
}
