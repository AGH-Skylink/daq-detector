#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    printf("keys:  r = start/stop SD recording, u = start/stop USB stream, i = identify, h = help\n"
           "lines (end with Enter):\n"
           "  mic                 list mic positions [mm]\n"
           "  mic N X Y Z         set position of mic N (1..6) in mm, saved to NVS\n"
           "  c [M/S]             show / set speed of sound (300..380 m/s)\n"
           "  alg [N]             show / set algorithm (0 off, 1 GCC-PHAT+LS, 2 SRP-PHAT, 3 MUSIC)\n"
           "  trk [N]             show / set tracker mode (1 azimuth, 2 azimuth x elevation, 3 elliptical gate)\n"
           "  out [N]             show / set BLE outputs (bit 1 = tracked Result, bit 2 = Raw result; 0..3)\n"
           "  defaults            restore settings from config.h\n"
           "  play N              replay RECnnnnn.WAV through the DSP -> PLYnnnnn.CSV (not while recording)\n");
}

static void print_settings(void)
{
    for (int m = 0; m < DAQ_NUM_CHANNELS; m++) {
        printf("M%d: %8.1f %8.1f %8.1f mm\n", m + 1, g_settings.mic_xyz_mm[m][0],
               g_settings.mic_xyz_mm[m][1], g_settings.mic_xyz_mm[m][2]);
    }
    printf("c = %.1f m/s, algorithm = %u, map rate = %u Hz, tracker = %u, outputs = %u\n",
           g_settings.speed_of_sound_mps, g_settings.algorithm, g_settings.map_rate_hz,
           g_settings.tracker_mode, g_settings.output_mask);
}

static void report(esp_err_t err)
{
    printf(err == ESP_OK ? "ok\n" : "error: %s\n", esp_err_to_name(err));
}

/* Line commands for settings (geometry calibration, speed of sound, algorithm, tracker, BLE outputs) */
static void handle_line(char *line)
{
    char *cmd = strtok(line, " \t");
    if (!cmd) {
        return;
    }
    char *a1 = strtok(NULL, " \t");
    if (strcmp(cmd, "mic") == 0) {
        if (!a1) {
            print_settings();
            return;
        }
        char *a2 = strtok(NULL, " \t"), *a3 = strtok(NULL, " \t"), *a4 = strtok(NULL, " \t");
        if (!a2 || !a3 || !a4) {
            printf("usage: mic N X Y Z\n");
            return;
        }
        report(settings_set_mic(atoi(a1) - 1, strtof(a2, NULL), strtof(a3, NULL), strtof(a4, NULL)));
    } else if (strcmp(cmd, "c") == 0) {
        if (a1) {
            report(settings_set_speed_of_sound(strtof(a1, NULL)));
        }
        printf("c = %.1f m/s\n", g_settings.speed_of_sound_mps);
    } else if (strcmp(cmd, "alg") == 0) {
        if (a1) {
            report(settings_set_algorithm((uint8_t)atoi(a1)));
        }
        printf("algorithm = %u\n", g_settings.algorithm);
    } else if (strcmp(cmd, "trk") == 0) {
        if (a1) {
            report(settings_set_tracker_mode((uint8_t)atoi(a1)));
        }
        printf("tracker = %u (1 AZ, 2 AZEL, 3 ELLIPSE)\n", g_settings.tracker_mode);
    } else if (strcmp(cmd, "out") == 0) {
        if (a1) {
            report(settings_set_output_mask((uint8_t)atoi(a1)));
        }
        printf("outputs = %u (tracked %s, raw %s)\n", g_settings.output_mask,
               g_settings.output_mask & DAQ_OUT_TRACKED ? "on" : "off", g_settings.output_mask & DAQ_OUT_RAW ? "on" : "off");
    } else if (strcmp(cmd, "play") == 0) {
        if (!a1) {
            printf("usage: play N   (file RECnnnnn.WAV)\n");
            return;
        }
        report(dsp_replay_start(atoi(a1)));
    } else if (strcmp(cmd, "defaults") == 0) {
        settings_reset();
        print_settings();
    } else {
        printf("unknown command, h = help\n");
    }
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
    if (g_daq.battery_valid) {
        printf(" | bat=%u%% %umV %dmA", g_daq.battery_pct, g_daq.battery_mv, g_daq.battery_ma);
    }
    printf(" | cpu=%u/%u%%", g_daq.cpu_load_pct[0], g_daq.cpu_load_pct[1]);
    if (dsp_replay_running()) {
        printf(" | replay %u%%", g_daq.replay_progress_pct);
    } else {
        printf(" | dsp=%.1fms vad=%d", g_daq.dsp_frame_us / 1000.0, g_daq.dsp_voice);
        if (g_daq.frames_captured - g_daq.loc_frame < DAQ_SAMPLE_RATE_HZ && g_daq.loc_frame > 0) {
            printf(" | src az=%.0f el=%.0f d=%.1fm n=%u (%.1fms)", g_daq.loc_az_deg, g_daq.loc_el_deg,
                   g_daq.loc_dist_m, g_daq.loc_speakers, g_daq.loc_us / 1000.0);
        }
        printf(" | spk=%u", g_daq.trk_count);
        if (g_daq.trk_count) {
            printf(" az=%.0f", g_daq.trk_az_deg);
        }
    }
    printf(" | drop dma=%" PRIu32 " sd=%" PRIu32 " usb=%" PRIu32 " dsp=%" PRIu32 " csv=%" PRIu32 "\n",
           g_daq.dma_overflows, g_daq.sd_dropped_blocks, g_daq.usb_dropped_blocks,
           g_daq.dsp_dropped_blocks, g_daq.csv_dropped_lines);
}

/*
 * UART0 console (CP2102N port), button and 1 s status line.
 * Single keys (r, u, i, h) act immediately when no line is being typed;
 * anything else is collected (with echo) until Enter and parsed by handle_line().
 */
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
    char line[64];
    size_t line_len = 0;
    print_help();

    while (true) {
        int ch;
        while ((ch = getchar()) != EOF) {   /* non-blocking on the default UART VFS */
            if (line_len == 0 && (ch == 'r' || ch == 'u' || ch == 'i' || ch == 'h')) {
                if (ch == 'r') {
                    toggle_recording();
                } else if (ch == 'u') {
                    stream_set_enabled(!g_daq.streaming);
                } else if (ch == 'i') {
                    system_identify();
                } else {
                    print_help();
                }
            } else if (ch == '\r' || ch == '\n') {
                if (line_len > 0) {
                    putchar('\n');
                    line[line_len] = '\0';
                    line_len = 0;
                    handle_line(line);
                }
            } else if ((ch == '\b' || ch == 0x7f) && line_len > 0) {
                line_len--;
                printf("\b \b");
            } else if (ch >= ' ' && line_len < sizeof(line) - 1) {
                line[line_len++] = (char)ch;
                putchar(ch);
            }
            fflush(stdout);
        }
        clearerr(stdin);

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
    ESP_ERROR_CHECK(settings_init());   /* NVS first: BLE needs it for PHY calibration data */
    if (storage_init() != ESP_OK) {
        ESP_LOGW(TAG, "SD card not available, recording disabled");
    }
    ESP_ERROR_CHECK(stream_init());
    ESP_ERROR_CHECK(dsp_start());       /* before capture, so no block is lost */
    ESP_ERROR_CHECK(capture_start());
    if (ble_init() != ESP_OK) {
        ESP_LOGW(TAG, "BLE not available");
    }
    ESP_ERROR_CHECK(system_start());
    xTaskCreatePinnedToCore(ui_task, "ui", 4096, NULL, 3, NULL, 0);
}
