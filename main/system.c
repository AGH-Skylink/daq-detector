/*
 * System services:
 *  - persistent settings in NVS (mic geometry, speed of sound, algorithm, map rate, tracker mode, BLE outputs),
 *  - battery monitor: INA219 on the UPS HAT (voltage, current, state of charge),
 *  - status LED (on-board WS2812) and identify,
 *  - CPU load per core.
 */
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "config.h"
#include "daq.h"

static const char *TAG = "system";

daq_settings_t g_settings;

/* ============================== settings ============================== */

#define NVS_NAMESPACE        "daq"
#define NVS_KEY_SETTINGS     "settings"
#define SETTINGS_VERSION     2   /* 2: + tracker_mode, output_mask (version 1 blobs are upgraded) */

typedef struct {
    uint16_t version;
    uint16_t size;
    daq_settings_t data;
} settings_blob_t;

static portMUX_TYPE s_settings_lock = portMUX_INITIALIZER_UNLOCKED;

static void settings_defaults(daq_settings_t *s)
{
    static const float mic[DAQ_NUM_CHANNELS][3] = DAQ_MIC_XYZ_MM;
    memcpy(s->mic_xyz_mm, mic, sizeof(mic));
    s->speed_of_sound_mps = DAQ_SPEED_OF_SOUND_MPS;
    s->algorithm = DAQ_ALGORITHM_DEFAULT;
    s->map_rate_hz = DAQ_BLE_MAP_RATE_HZ;
    s->tracker_mode = DAQ_TRACKER_MODE_DEFAULT;
    s->output_mask = DAQ_OUTPUT_MASK_DEFAULT;
}

static esp_err_t settings_save(void)
{
    g_daq.settings_version++;
    settings_blob_t blob = { .version = SETTINGS_VERSION, .size = sizeof(daq_settings_t) };
    portENTER_CRITICAL(&s_settings_lock);
    blob.data = g_settings;
    portEXIT_CRITICAL(&s_settings_lock);

    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h), TAG, "nvs_open");
    esp_err_t err = nvs_set_blob(h, NVS_KEY_SETTINGS, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "settings not saved: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t settings_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init");

    settings_defaults(&g_settings);
    g_daq.settings_version = 1;

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        settings_blob_t blob;
        size_t len = sizeof(blob);
        /* version 1 had the same fields up to map_rate_hz (the struct padding keeps the blob size) */
        size_t v1_size = offsetof(daq_settings_t, tracker_mode);
        bool read = nvs_get_blob(h, NVS_KEY_SETTINGS, &blob, &len) == ESP_OK && len == sizeof(blob);
        if (read && blob.version == SETTINGS_VERSION && blob.size == sizeof(daq_settings_t)) {
            g_settings = blob.data;
            ESP_LOGI(TAG, "settings loaded from NVS");
        } else if (read && blob.version == 1 && blob.size >= v1_size) {
            memcpy(&g_settings, &blob.data, v1_size);   /* new fields keep their defaults */
            ESP_LOGI(TAG, "settings version 1 loaded from NVS (tracker and outputs: defaults)");
        } else {
            ESP_LOGI(TAG, "no valid settings in NVS, using defaults");
        }
        nvs_close(h);
    }
    return ESP_OK;
}

void settings_reset(void)
{
    portENTER_CRITICAL(&s_settings_lock);
    settings_defaults(&g_settings);
    portEXIT_CRITICAL(&s_settings_lock);
    settings_save();
}

esp_err_t settings_set_mic(int index, float x_mm, float y_mm, float z_mm)
{
    if (index < 0 || index >= DAQ_NUM_CHANNELS ||
        x_mm < -3000 || x_mm > 3000 || y_mm < -3000 || y_mm > 3000 || z_mm < -3000 || z_mm > 3000) {
        return ESP_ERR_INVALID_ARG;   /* +-3000 mm keeps 0.1 mm values inside int16 for BLE */
    }
    portENTER_CRITICAL(&s_settings_lock);
    g_settings.mic_xyz_mm[index][0] = x_mm;
    g_settings.mic_xyz_mm[index][1] = y_mm;
    g_settings.mic_xyz_mm[index][2] = z_mm;
    portEXIT_CRITICAL(&s_settings_lock);
    return settings_save();
}

esp_err_t settings_set_speed_of_sound(float mps)
{
    if (mps < 300.0f || mps > 380.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    g_settings.speed_of_sound_mps = mps;
    return settings_save();
}

esp_err_t settings_set_algorithm(uint8_t algorithm)
{
    if (algorithm > DAQ_ALG_C_MUSIC) {
        return ESP_ERR_INVALID_ARG;
    }
    g_settings.algorithm = algorithm;
    return settings_save();
}

esp_err_t settings_set_tracker_mode(uint8_t mode)
{
    if (mode < DAQ_TRACK_AZ || mode > DAQ_TRACK_ELLIPSE) {
        return ESP_ERR_INVALID_ARG;
    }
    g_settings.tracker_mode = mode;
    return settings_save();
}

esp_err_t settings_set_output_mask(uint8_t mask)
{
    if (mask & ~(DAQ_OUT_TRACKED | DAQ_OUT_RAW)) {
        return ESP_ERR_INVALID_ARG;
    }
    g_settings.output_mask = mask;
    return settings_save();
}

esp_err_t settings_set_map_rate(uint8_t hz)
{
    if (hz > 10) {
        return ESP_ERR_INVALID_ARG;
    }
    g_settings.map_rate_hz = hz;
    return settings_save();
}

/* ============================== battery (INA219) ============================== */

#define INA219_REG_CONFIG    0x00
#define INA219_REG_SHUNT     0x01
#define INA219_REG_BUS       0x02
/* 16 V bus range, PGA /1 (+-40 mV = +-4 A on 10 mOhm), 12-bit with 128-sample averaging, continuous shunt + bus */
#define INA219_CONFIG        0x07FF

static i2c_master_dev_handle_t s_ina;

/* LiPo open-circuit voltage -> state of charge */
static const struct { uint16_t mv; uint8_t pct; } s_soc_curve[] = {
    { 3300, 0 }, { 3500, 5 }, { 3610, 10 }, { 3670, 20 }, { 3710, 30 }, { 3750, 40 },
    { 3790, 50 }, { 3850, 60 }, { 3920, 70 }, { 4000, 80 }, { 4100, 90 }, { 4200, 100 },
};

static esp_err_t ina_write(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)value };
    return i2c_master_transmit(s_ina, buf, sizeof(buf), 50);
}

static esp_err_t ina_read(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_ina, &reg, 1, buf, 2, 50), TAG, "ina read");
    *value = (uint16_t)(buf[0] << 8 | buf[1]);
    return ESP_OK;
}

static esp_err_t battery_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = DAQ_PIN_I2C_SDA,
        .scl_io_num = DAQ_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "i2c bus");
    esp_err_t err = i2c_master_probe(bus, DAQ_INA219_ADDR, 50);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INA219 not found at 0x%02x", DAQ_INA219_ADDR);
        i2c_del_master_bus(bus);
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DAQ_INA219_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &s_ina), TAG, "i2c device");
    ESP_RETURN_ON_ERROR(ina_write(INA219_REG_CONFIG, INA219_CONFIG), TAG, "INA219 config");
    g_daq.battery_present = true;
    return ESP_OK;
}

static uint8_t soc_from_mv(int mv)
{
    const int n = sizeof(s_soc_curve) / sizeof(s_soc_curve[0]);
    if (mv <= s_soc_curve[0].mv) {
        return 0;
    }
    for (int i = 1; i < n; i++) {
        if (mv <= s_soc_curve[i].mv) {
            int dv = s_soc_curve[i].mv - s_soc_curve[i - 1].mv;
            int dp = s_soc_curve[i].pct - s_soc_curve[i - 1].pct;
            return (uint8_t)(s_soc_curve[i - 1].pct + dp * (mv - s_soc_curve[i - 1].mv) / dv);
        }
    }
    return 100;
}

static void battery_update(void)
{
    static float soc_filtered = -1.0f;
    uint16_t shunt_raw, bus_raw;
    if (ina_read(INA219_REG_SHUNT, &shunt_raw) != ESP_OK || ina_read(INA219_REG_BUS, &bus_raw) != ESP_OK) {
        g_daq.battery_valid = false;
        return;
    }
    /* shunt LSB = 10 uV, bus LSB = 4 mV (bits 15..3) */
    int ma = DAQ_BATTERY_CURRENT_SIGN * (int)(int16_t)shunt_raw * 10 / DAQ_INA219_SHUNT_MOHM;
    int mv = (bus_raw >> 3) * 4;

    /* Terminal voltage -> open-circuit voltage: rises while charging (+), sags while discharging (-) */
    int ocv_mv = mv - ma * DAQ_BATTERY_RINT_MOHM / 1000;
    float soc = soc_from_mv(ocv_mv);
    soc_filtered = soc_filtered < 0.0f ? soc : soc_filtered + 0.1f * (soc - soc_filtered);

    g_daq.battery_mv = (uint16_t)mv;
    g_daq.battery_ma = (int16_t)(ma > 32767 ? 32767 : (ma < -32768 ? -32768 : ma));
    g_daq.battery_pct = (uint8_t)(soc_filtered + 0.5f);
    g_daq.battery_valid = true;
}

/* ============================== status LED ============================== */

static led_strip_handle_t s_led;
static volatile int64_t s_identify_until_us;

static esp_err_t led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = DAQ_PIN_LED,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led), TAG, "led_strip");
    g_daq.led_present = true;
    return ESP_OK;
}

void system_identify(void)
{
    s_identify_until_us = esp_timer_get_time() + DAQ_IDENTIFY_MS * 1000LL;
}

/*
 * Priority: identify (white blink) > error (magenta) > recording (red blink) >
 * USB stream (cyan) > BLE connected (blue) > no SD card (yellow) > idle (green)
 */
static void led_update(bool error)
{
    if (!s_led) {
        return;
    }
    const uint8_t b = DAQ_LED_BRIGHTNESS;
    int64_t now = esp_timer_get_time();
    bool blink_fast = (now / 125000) & 1;
    bool blink_slow = (now / 500000) & 1;
    uint8_t r = 0, g = 0, bl = 0;

    if (now < s_identify_until_us) {
        r = g = bl = blink_fast ? b : 0;
    } else if (error) {
        r = b; bl = b;
    } else if (g_daq.recording) {
        r = blink_slow ? b : 0;
    } else if (g_daq.streaming) {
        g = b; bl = b;
    } else if (g_daq.ble_connected) {
        bl = b;
    } else if (!g_daq.sd_ready) {
        r = b; g = b;
    } else {
        g = b;
    }
    led_strip_set_pixel(s_led, 0, r, g, bl);
    led_strip_refresh(s_led);
}

/* ============================== CPU load ============================== */

static void cpu_load_update(void)
{
    static uint32_t last_idle[2];
    static int64_t last_us;
    int64_t now = esp_timer_get_time();
    for (int core = 0; core < 2; core++) {
        TaskStatus_t st;
        vTaskGetInfo(xTaskGetIdleTaskHandleForCore(core), &st, pdFALSE, eRunning);
        uint32_t idle = st.ulRunTimeCounter;   /* us, esp_timer clock */
        if (last_us) {
            float busy = 1.0f - (float)(idle - last_idle[core]) / (float)(now - last_us);
            busy = busy < 0.0f ? 0.0f : (busy > 1.0f ? 1.0f : busy);
            g_daq.cpu_load_pct[core] = (uint8_t)(busy * 100.0f + 0.5f);
        }
        last_idle[core] = idle;
    }
    last_us = now;
}

/* ============================== task ============================== */

static void system_task(void *arg)
{
    uint32_t last_drops = 0;
    int64_t error_until_us = 0;
    int tick = 0;

    while (true) {
        /* error = any new lost block or SD write error, shown for 2 s */
        uint32_t drops = g_daq.dma_overflows + g_daq.sd_dropped_blocks + g_daq.usb_dropped_blocks + g_daq.sd_write_errors;
        if (drops != last_drops) {
            last_drops = drops;
            error_until_us = esp_timer_get_time() + 2000000;
        }
        led_update(esp_timer_get_time() < error_until_us);

        if (++tick >= 10) {   /* every 1 s */
            tick = 0;
            if (g_daq.battery_present) {
                battery_update();
            }
            cpu_load_update();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t system_start(void)
{
    g_daq.battery_pct = 255;
    if (battery_init() != ESP_OK) {
        ESP_LOGW(TAG, "battery monitor not available");
    }
    if (led_init() != ESP_OK) {
        ESP_LOGW(TAG, "status LED not available");
    }
    BaseType_t ok = xTaskCreatePinnedToCore(system_task, "system", 3072, NULL, 2, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
