#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
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
           "  play N              replay RECnnnnn.WAV through the DSP -> PLYnnnnn.CSV (not while recording)\n"
           "  diag                run the self-test again (DIAG lines, also printed after boot)\n");
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

/* ============================== self-test ============================== */

/* Thresholds of the self-test (tune on the device: measure healthy mics in a quiet room) */
#define DIAG_LEVEL_DEAD_DBFS   (-100.0f)   /* below: no signal (≈ -120 = all zeros) */
#define DIAG_LEVEL_STUCK_DBFS  (-10.0f)    /* above: stuck / clipping data line */
#define DIAG_BATTERY_LOW_MV    3400
#define DIAG_SD_LOW_FREE_MB    100

typedef enum { DIAG_OK, DIAG_WARN, DIAG_FAIL } diag_level_t;
static int s_diag_warn, s_diag_fail;

static void diag(diag_level_t level, const char *name, const char *fmt, ...)
{
    static const char *const tag[] = { "OK  ", "WARN", "FAIL" };
    if (level == DIAG_WARN) {
        s_diag_warn++;
    } else if (level == DIAG_FAIL) {
        s_diag_fail++;
    }
    printf("DIAG %s %-7s ", tag[level], name);
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
}

static const char *reset_reason_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout (supply / battery)";
    case ESP_RST_EXT:      return "external pin";
    case ESP_RST_USB:      return "USB";
    default:               return "other";
    }
}

/*
 * Self-test printed on the UART console ~1.5 s after boot and on the `diag` command.
 * Every line starts with "DIAG"; the last one is "DIAG result OK|FAIL (n fail, m warn)".
 * Takes ~0.5 s (sample rate and mic levels are measured live).
 * Not covered: swapped mic order (DAQ_CHANNEL_MAP) - use the tap test.
 */
static void run_diag(void)
{
    s_diag_warn = s_diag_fail = 0;
    printf("DIAG begin fw %d.%d.%d\n", DAQ_FW_VERSION_MAJOR, DAQ_FW_VERSION_MINOR, DAQ_FW_VERSION_PATCH);

    esp_reset_reason_t rr = esp_reset_reason();
    bool bad_reset = rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT ||
                     rr == ESP_RST_WDT || rr == ESP_RST_BROWNOUT;
    diag(bad_reset ? DIAG_WARN : DIAG_OK, "reset", "%s", reset_reason_name(rr));

    size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    diag(psram >= 7u * 1024 * 1024 ? DIAG_OK : DIAG_FAIL, "psram", "%u kB, free internal %u kB",
         (unsigned)(psram / 1024), (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    /* capture: samples must arrive at the sample rate (PDM clock on GPIO DAQ_PIN_PDM_CLK) */
    uint32_t dma0 = g_daq.dma_overflows;
    capture_get_levels(NULL, 0);   /* reset the level accumulators: levels below cover exactly this 0.5 s */
    uint64_t f0 = g_daq.frames_captured;
    vTaskDelay(pdMS_TO_TICKS(500));
    uint32_t df = (uint32_t)(g_daq.frames_captured - f0);
    uint32_t expected = DAQ_SAMPLE_RATE_HZ / 2;
    if (g_daq.capture_init_err != ESP_OK) {
        diag(DIAG_FAIL, "capture", "init failed: %s", esp_err_to_name(g_daq.capture_init_err));
    } else {
        diag(df > expected * 9 / 10 && df < expected * 11 / 10 ? DIAG_OK : DIAG_FAIL, "capture",
             "%" PRIu32 " frames / 0.5 s (expected %" PRIu32 ")", df, expected);
    }
    diag(g_daq.dma_overflows == 0 ? DIAG_OK : DIAG_WARN, "dma", "overflows %" PRIu32 " (+%" PRIu32 " during test)",
         g_daq.dma_overflows, g_daq.dma_overflows - dma0);

    /* microphones: RMS level over the last 0.5 s */
    static const int sel_pins[DAQ_NUM_CHANNELS] = DAQ_PIN_MIC_SEL;
    float lv[DAQ_NUM_CHANNELS];
    capture_get_levels(lv, DAQ_NUM_CHANNELS);
    for (int m = 0; m < DAQ_NUM_CHANNELS; m++) {
        char name[4];
        snprintf(name, sizeof(name), "M%d", m + 1);
        if (lv[m] < DIAG_LEVEL_DEAD_DBFS) {
            diag(DIAG_FAIL, name, "%6.1f dBFS - no signal (check 3V/GND/DAT wire, SEL GPIO %d)", lv[m],
                 sel_pins[m]);
        } else if (lv[m] > DIAG_LEVEL_STUCK_DBFS) {
            diag(DIAG_FAIL, name, "%6.1f dBFS - stuck / clipping (DAT line floating or SEL conflict?)", lv[m]);
        } else {
            diag(DIAG_OK, name, "%6.1f dBFS", lv[m]);
        }
    }

    if (g_daq.dsp_init_err != ESP_OK) {
        diag(DIAG_FAIL, "dsp", "init failed: %s", esp_err_to_name(g_daq.dsp_init_err));
    } else if (g_daq.dsp_frame_us == 0) {
        diag(DIAG_FAIL, "dsp", "no frame processed yet");
    } else {
        diag(g_daq.dsp_frame_us > 25000 ? DIAG_WARN : DIAG_OK, "dsp", "%.1f ms per 32 ms frame, FFT %s, dropped %" PRIu32,
             g_daq.dsp_frame_us / 1000.0, dsp_fft_mirrored() ? "mirrored" : "normal", g_daq.dsp_dropped_blocks);
    }

    if (g_daq.sd_ready) {
        uint64_t total = 0, free_b = 0;
        if (esp_vfs_fat_info(DAQ_SD_MOUNT_POINT, &total, &free_b) == ESP_OK) {
            diag(free_b < DIAG_SD_LOW_FREE_MB * 1000000ULL ? DIAG_WARN : DIAG_OK, "sd", "%.0f / %.0f MB free, write errors %" PRIu32,
                 free_b / 1e6, total / 1e6, g_daq.sd_write_errors);
        } else {
            diag(DIAG_WARN, "sd", "mounted, but free space unknown");
        }
    } else {
        diag(DIAG_FAIL, "sd", "not available: %s (recording disabled)", esp_err_to_name(g_daq.sd_init_err));
    }

    if (!g_daq.battery_present) {
        diag(DIAG_WARN, "battery", "INA219 not found at 0x%02x (SDA GPIO %d, SCL GPIO %d)",
             DAQ_INA219_ADDR, DAQ_PIN_I2C_SDA, DAQ_PIN_I2C_SCL);
    } else if (!g_daq.battery_valid) {
        diag(DIAG_WARN, "battery", "INA219 found, no reading yet");
    } else {
        diag(g_daq.battery_mv < DIAG_BATTERY_LOW_MV ? DIAG_WARN : DIAG_OK, "battery", "%u mV %d mA %u%%",
             g_daq.battery_mv, g_daq.battery_ma, g_daq.battery_pct);
    }

    diag(g_daq.led_present ? DIAG_OK : DIAG_WARN, "led", g_daq.led_present ? "ok" : "not available");
    diag(g_daq.stream_init_err == ESP_OK ? DIAG_OK : DIAG_FAIL, "usb", "%s", esp_err_to_name(g_daq.stream_init_err));
    diag(g_daq.ble_init_err == ESP_OK ? DIAG_OK : DIAG_WARN, "ble", "%s%s", esp_err_to_name(g_daq.ble_init_err),
         g_daq.ble_connected ? ", phone connected" : "");

    g_daq.diag_failed = s_diag_fail > 0;
    printf("DIAG result %s (%d fail, %d warn)\n", s_diag_fail ? "FAIL" : "OK", s_diag_fail, s_diag_warn);
    fflush(stdout);
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
    } else if (strcmp(cmd, "diag") == 0) {
        run_diag();
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
    vTaskDelay(pdMS_TO_TICKS(1500));   /* capture warm-up, first DSP frames, first INA219 reading */
    run_diag();
    last_status = xTaskGetTickCount();

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
    /* The other modules are not fatal: a failure is kept and reported by the self-test (run_diag),
     * instead of a reboot loop that never reaches the console. */
    g_daq.sd_init_err = storage_init();
    if (g_daq.sd_init_err != ESP_OK) {
        ESP_LOGW(TAG, "SD card not available, recording disabled");
    }
    g_daq.stream_init_err = stream_init();
    if (g_daq.stream_init_err != ESP_OK) {
        ESP_LOGE(TAG, "USB stream not available: %s", esp_err_to_name(g_daq.stream_init_err));
    }
    g_daq.dsp_init_err = dsp_start();   /* before capture, so no block is lost */
    if (g_daq.dsp_init_err != ESP_OK) {
        ESP_LOGE(TAG, "DSP not available: %s", esp_err_to_name(g_daq.dsp_init_err));
    }
    g_daq.capture_init_err = capture_start();
    if (g_daq.capture_init_err != ESP_OK) {
        ESP_LOGE(TAG, "capture not available: %s", esp_err_to_name(g_daq.capture_init_err));
    }
    g_daq.ble_init_err = ble_init();
    if (g_daq.ble_init_err != ESP_OK) {
        ESP_LOGW(TAG, "BLE not available");
    }
    ESP_ERROR_CHECK(system_start());
    xTaskCreatePinnedToCore(ui_task, "ui", 4096, NULL, 3, NULL, 0);
}
