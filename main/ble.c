/*
 * BLE GATT server "AudioLoc" (NimBLE). Protocol: docs/ble_gatt.md.
 *
 * Diagnostics, device info and control use the real firmware state.
 * Localization output comes from dsp.c: ble_publish_result() (confirmed tracks, Result 0002),
 * ble_publish_raw() (sources of every frame, Raw result 0007) and ble_publish_map() (SRP map, algorithm B).
 * Which of the two results is produced is chosen by g_settings.output_mask (control 0x09).
 * DAQ_BLE_TEST_PATTERN 1 sends a synthetic moving source instead.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "config.h"
#include "daq.h"

static const char *TAG = "ble";

/* Frame layout versions (docs/ble_gatt.md); bump when a layout changes */
#define RESULT_VERSION     2       /* 2: byte 7 of a source = track id */
#define MAP_VERSION        1
#define DIAG_VERSION       2
#define INFO_VERSION       3       /* 3: + output mask, tracker mode */
#define RESULT_LEN         26
#define DIAG_LEN           56
#define INFO_LEN           73
#define MAP_HEADER_LEN     8
#define MAP_LEN            (DAQ_BLE_MAP_AZ_BINS * DAQ_BLE_MAP_EL_BINS)
#define MAP_CHUNK_MAX      236     /* payload per notification at MTU 247 */
#define RESULT_TIMEOUT_MS  500     /* result older than this is reported as not valid */
#define DIAG_PERIOD_MS     1000
#define ATT_ERR_APP        0x80

#define FEATURE_LOCALIZATION 0x01
#define FEATURE_SRP_MAP      0x02
#define FEATURE_BATTERY      0x04
#define FEATURE_IDENTIFY     0x08

/* da0cXXXX-7e4b-4c8a-9f2e-3b5d1a6c0e01, bytes in little-endian order */
#define AUDIOLOC_UUID(x) BLE_UUID128_INIT(0x01, 0x0e, 0x6c, 0x1a, 0x5d, 0x3b, 0x2e, 0x9f, \
                                          0x8a, 0x4c, 0x4b, 0x7e, (x), 0x00, 0x0c, 0xda)

static const ble_uuid128_t s_uuid_svc     = AUDIOLOC_UUID(0x01);
static const ble_uuid128_t s_uuid_result  = AUDIOLOC_UUID(0x02);
static const ble_uuid128_t s_uuid_map     = AUDIOLOC_UUID(0x03);
static const ble_uuid128_t s_uuid_diag    = AUDIOLOC_UUID(0x04);
static const ble_uuid128_t s_uuid_info    = AUDIOLOC_UUID(0x05);
static const ble_uuid128_t s_uuid_control = AUDIOLOC_UUID(0x06);
static const ble_uuid128_t s_uuid_raw     = AUDIOLOC_UUID(0x07);

enum { CHR_RESULT, CHR_MAP, CHR_DIAG, CHR_INFO, CHR_CONTROL, CHR_RAW };

static uint16_t s_h_result, s_h_map, s_h_diag, s_h_raw;

/* Connection state, owned by the NimBLE host task */
static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile uint16_t s_mtu = BLE_ATT_MTU_DFLT;
static volatile bool s_sub_result, s_sub_map, s_sub_diag, s_sub_raw;
static uint8_t s_own_addr_type;
static char s_name[24] = DAQ_BLE_NAME_PREFIX "0000";

static volatile uint16_t s_seq;
static volatile uint32_t s_notify_errors;

/* Latest localization output, written by ble_publish_*() */
typedef struct {
    daq_source_t sources[DAQ_BLE_MAX_SOURCES];
    int n_sources;
    int speaker_count;
    TickType_t tick;
    bool seen;
} result_slot_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static result_slot_t s_tracked, s_raw;   /* Result (0002), Raw result (0007) */
static uint8_t s_map[MAP_LEN];
static bool s_map_fresh;

static void advertise(void);

/* ---------- encoding ---------- */

static inline void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static bool result_is_valid(const result_slot_t *r)
{
    return r->seen && (xTaskGetTickCount() - r->tick) < pdMS_TO_TICKS(RESULT_TIMEOUT_MS);
}

static uint32_t uptime_ms(void)
{
    return (uint32_t)(g_daq.frames_captured * 1000ULL / DAQ_SAMPLE_RATE_HZ);
}

static void build_result(const result_slot_t *r, uint8_t out[RESULT_LEN], uint16_t seq)
{
    daq_source_t src[DAQ_BLE_MAX_SOURCES];
    int n = 0, speakers = 0;

    portENTER_CRITICAL(&s_lock);
    bool valid = result_is_valid(r);
    if (valid) {
        n = r->n_sources;
        speakers = r->speaker_count;
        memcpy(src, r->sources, sizeof(src));
    }
    portEXIT_CRITICAL(&s_lock);

    memset(out, 0, RESULT_LEN);
    out[0] = RESULT_VERSION;
    out[1] = (valid ? 0x01 : 0) | (DAQ_BLE_TEST_PATTERN ? 0x02 : 0);
    put_u16(&out[2], seq);
    put_u32(&out[4], uptime_ms());
    out[8] = (uint8_t)(speakers > 255 ? 255 : speakers);
    out[9] = (uint8_t)n;

    for (int k = 0; k < n; k++) {
        uint8_t *p = &out[10 + 8 * k];
        float az = fmodf(src[k].azimuth_deg, 360.0f);
        if (az < 0.0f) {
            az += 360.0f;
        }
        put_u16(&p[0], (uint16_t)(lroundf(az * 100.0f) % 36000));
        put_u16(&p[2], (uint16_t)(int16_t)lroundf(clampf(src[k].elevation_deg, -90.0f, 90.0f) * 100.0f));
        put_u16(&p[4], (uint16_t)lroundf(clampf(src[k].distance_m * 1000.0f, 0.0f, 65535.0f)));
        p[6] = (uint8_t)lroundf(clampf(src[k].confidence * 100.0f, 0.0f, 100.0f));
        p[7] = src[k].track_id;   /* 0 in the raw result */
    }
}

static void build_diag(uint8_t out[DIAG_LEN])
{
    memset(out, 0, DIAG_LEN);
    out[0] = DIAG_VERSION;
    out[1] = (g_daq.sd_ready ? 0x01 : 0) |
             (g_daq.recording ? 0x02 : 0) |
             (g_daq.streaming ? 0x04 : 0) |
             (result_is_valid(&s_tracked) || result_is_valid(&s_raw) ? 0x08 : 0) |
             (g_daq.battery_valid ? 0x10 : 0);
    out[2] = g_daq.battery_valid ? g_daq.battery_pct : 255;
    put_u32(&out[4], uptime_ms() / 1000);
    for (int c = 0; c < DAQ_NUM_CHANNELS; c++) {
        float cdb = clampf(g_daq.level_dbfs[c] * 100.0f, -12000.0f, 0.0f);
        put_u16(&out[8 + 2 * c], (uint16_t)(int16_t)lroundf(cdb));
    }
    put_u32(&out[20], g_daq.dma_overflows);
    put_u32(&out[24], g_daq.sd_dropped_blocks);
    put_u32(&out[28], g_daq.usb_dropped_blocks);
    put_u32(&out[32], g_daq.sd_write_errors);
    put_u32(&out[36], g_daq.sd_file_index);
    put_u32(&out[40], (uint32_t)(g_daq.sd_bytes_written / 1024));
    put_u32(&out[44], (uint32_t)(g_daq.usb_bytes_sent / 1024));
    put_u16(&out[48], g_daq.battery_valid ? g_daq.battery_mv : 0);
    put_u16(&out[50], g_daq.battery_valid ? (uint16_t)g_daq.battery_ma : 0);
    out[52] = g_daq.cpu_load_pct[0];
    out[53] = g_daq.cpu_load_pct[1];
    /* 54..55 reserved */
}

static void build_info(uint8_t out[INFO_LEN])
{
    static const gpio_num_t sel[DAQ_NUM_CHANNELS] = DAQ_PIN_MIC_SEL;
    const int pins[19] = {
        DAQ_PIN_PDM_CLK, DAQ_PIN_PDM_DIN0, DAQ_PIN_PDM_DIN1, DAQ_PIN_PDM_DIN2, -1,
        DAQ_PIN_SD_SCK, DAQ_PIN_SD_MOSI, DAQ_PIN_SD_MISO, DAQ_PIN_SD_CS,
        DAQ_PIN_BUTTON, DAQ_PIN_I2C_SDA, DAQ_PIN_I2C_SCL, DAQ_PIN_LED,
        sel[0], sel[1], sel[2], sel[3], sel[4], sel[5],
    };

    memset(out, 0, INFO_LEN);
    out[0] = INFO_VERSION;
    out[1] = DAQ_FW_VERSION_MAJOR;
    out[2] = DAQ_FW_VERSION_MINOR;
    out[3] = DAQ_FW_VERSION_PATCH;
    put_u32(&out[4], DAQ_SAMPLE_RATE_HZ);
    out[8] = DAQ_NUM_CHANNELS;
    out[9] = DAQ_BLE_MAP_AZ_BINS;
    out[10] = DAQ_BLE_MAP_EL_BINS;
    out[11] = DAQ_BLE_HOP_MS;
    for (int m = 0; m < DAQ_NUM_CHANNELS; m++) {
        for (int a = 0; a < 3; a++) {
            /* 0.1 mm units (protocol v2) */
            put_u16(&out[12 + (m * 3 + a) * 2], (uint16_t)(int16_t)lroundf(g_settings.mic_xyz_mm[m][a] * 10.0f));
        }
    }
    for (int i = 0; i < 19; i++) {
        out[48 + i] = pins[i] < 0 ? 255 : (uint8_t)pins[i];
    }
    out[67] = (DAQ_BLE_TEST_PATTERN ? (FEATURE_LOCALIZATION | FEATURE_SRP_MAP) : 0) |
              (g_settings.algorithm >= DAQ_ALG_A_GCC_LS && g_settings.algorithm <= DAQ_ALG_C_MUSIC ? FEATURE_LOCALIZATION : 0) |
              (g_settings.algorithm == DAQ_ALG_B_SRP ? FEATURE_SRP_MAP : 0) |
              (g_daq.battery_present ? FEATURE_BATTERY : 0) |
              (g_daq.led_present ? FEATURE_IDENTIFY : 0);
    put_u16(&out[68], (uint16_t)lroundf(g_settings.speed_of_sound_mps * 10.0f));   /* 0.1 m/s */
    out[70] = g_settings.algorithm;
    out[71] = g_settings.output_mask;
    out[72] = g_settings.tracker_mode;
}

/* ---------- GATT ---------- */

static int handle_control(const uint8_t *cmd, uint16_t n)
{
    if (n < 1) {
        return ATT_ERR_APP;
    }
    switch (cmd[0]) {
    case 0x01:
        if (n != 1 || !g_daq.sd_ready) {
            return ATT_ERR_APP;
        }
        g_daq.record_request = true;
        break;
    case 0x02:
        if (n != 1) {
            return ATT_ERR_APP;
        }
        g_daq.record_request = false;
        break;
    case 0x03:
    case 0x04:
        if (n != 1) {
            return ATT_ERR_APP;
        }
        stream_set_enabled(cmd[0] == 0x03);
        break;
    case 0x05:
        if (n != 2 || settings_set_map_rate(cmd[1] > 10 ? 10 : cmd[1]) != ESP_OK) {
            return ATT_ERR_APP;
        }
        ESP_LOGI(TAG, "map rate %u Hz", g_settings.map_rate_hz);
        break;
    case 0x06:
        if (n != 1) {
            return ATT_ERR_APP;
        }
        system_identify();
        break;
    case 0x07:
        if (n != 2 || settings_set_algorithm(cmd[1]) != ESP_OK) {
            return ATT_ERR_APP;
        }
        ESP_LOGI(TAG, "algorithm %u", cmd[1]);
        break;
    case 0x08: {
        if (n != 3) {
            return ATT_ERR_APP;
        }
        uint16_t dmps = (uint16_t)(cmd[1] | cmd[2] << 8);   /* 0.1 m/s */
        if (settings_set_speed_of_sound(dmps / 10.0f) != ESP_OK) {
            return ATT_ERR_APP;
        }
        ESP_LOGI(TAG, "speed of sound %.1f m/s", g_settings.speed_of_sound_mps);
        break;
    }
    case 0x09:
        if (n != 2 || settings_set_output_mask(cmd[1]) != ESP_OK) {
            return ATT_ERR_APP;
        }
        ESP_LOGI(TAG, "outputs 0x%02x", cmd[1]);
        break;
    case 0x0A:
        if (n != 2 || settings_set_tracker_mode(cmd[1]) != ESP_OK) {
            return ATT_ERR_APP;
        }
        ESP_LOGI(TAG, "tracker mode %u", cmd[1]);
        break;
    default:
        return ATT_ERR_APP;
    }
    return 0;
}

static int chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int id = (int)(intptr_t)arg;
    uint8_t buf[INFO_LEN];   /* largest readable value */
    size_t len = 0;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        switch (id) {
        case CHR_RESULT: build_result(&s_tracked, buf, s_seq); len = RESULT_LEN; break;
        case CHR_RAW:    build_result(&s_raw, buf, s_seq);     len = RESULT_LEN; break;
        case CHR_DIAG:   build_diag(buf);          len = DIAG_LEN;   break;
        case CHR_INFO:   build_info(buf);          len = INFO_LEN;   break;
        default:         return BLE_ATT_ERR_READ_NOT_PERMITTED;
        }
        return os_mbuf_append(ctxt->om, buf, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && id == CHR_CONTROL) {
        uint8_t cmd[4];
        uint16_t n = 0;
        if (OS_MBUF_PKTLEN(ctxt->om) > sizeof(cmd)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        if (ble_hs_mbuf_to_flat(ctxt->om, cmd, sizeof(cmd), &n) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        return handle_control(cmd, n);
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_uuid_svc.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_uuid_result.u,
                .access_cb = chr_access,
                .arg = (void *)(intptr_t)CHR_RESULT,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_h_result,
            },
            {
                .uuid = &s_uuid_map.u,
                .access_cb = chr_access,
                .arg = (void *)(intptr_t)CHR_MAP,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_h_map,
            },
            {
                .uuid = &s_uuid_diag.u,
                .access_cb = chr_access,
                .arg = (void *)(intptr_t)CHR_DIAG,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_h_diag,
            },
            {
                .uuid = &s_uuid_info.u,
                .access_cb = chr_access,
                .arg = (void *)(intptr_t)CHR_INFO,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &s_uuid_control.u,
                .access_cb = chr_access,
                .arg = (void *)(intptr_t)CHR_CONTROL,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &s_uuid_raw.u,
                .access_cb = chr_access,
                .arg = (void *)(intptr_t)CHR_RAW,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_h_raw,
            },
            { 0 },
        },
    },
    { 0 },
};

/* ---------- GAP ---------- */

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn = event->connect.conn_handle;
            s_mtu = BLE_ATT_MTU_DFLT;
            g_daq.ble_connected = true;
            ESP_LOGI(TAG, "connected");
        } else {
            advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected (reason 0x%x)", event->disconnect.reason);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_sub_result = s_sub_map = s_sub_diag = s_sub_raw = false;
        g_daq.ble_connected = false;
        advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_h_result) {
            s_sub_result = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_h_map) {
            s_sub_map = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_h_diag) {
            s_sub_diag = event->subscribe.cur_notify;
        } else if (event->subscribe.attr_handle == s_h_raw) {
            s_sub_raw = event->subscribe.cur_notify;
        }
        break;
    case BLE_GAP_EVENT_MTU:
        s_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU %u", event->mtu.value);
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;
    default:
        break;
    }
    return 0;
}

static void advertise(void)
{
    /* Service UUID in the advertising data (scan filter), name in the scan response (31 B limit) */
    struct ble_hs_adv_fields fields = { 0 };
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&s_uuid_svc;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields: %d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp = { 0 };
    rsp.name = (uint8_t *)s_name;
    rsp.name_len = strlen(s_name);
    rsp.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields: %d", rc);
        return;
    }

    struct ble_gap_adv_params params = { 0 };
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv_start: %d", rc);
    }
}

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_own_addr_type);

    uint8_t addr[6] = { 0 };
    ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
    snprintf(s_name, sizeof(s_name), DAQ_BLE_NAME_PREFIX "%02X%02X", addr[1], addr[0]);
    ble_svc_gap_device_name_set(s_name);

    ESP_LOGI(TAG, "advertising as %s", s_name);
    advertise();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset (reason %d)", reason);
}

static void host_task(void *arg)
{
    nimble_port_run();   /* returns only after nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ---------- publishing ---------- */

static void notify(uint16_t handle, const uint8_t *data, size_t len)
{
    if (len + 3 > s_mtu) {
        return;   /* MTU not negotiated yet: skip instead of sending a truncated frame */
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        s_notify_errors++;
        return;
    }
    if (ble_gatts_notify_custom(s_conn, handle, om) != 0) {   /* om is consumed in any case */
        s_notify_errors++;
    }
}

static bool send_map(void)
{
    static uint8_t map_id;
    static uint8_t snapshot[MAP_LEN];
    uint8_t chunk[MAP_HEADER_LEN + MAP_CHUNK_MAX];

    portENTER_CRITICAL(&s_lock);
    bool fresh = s_map_fresh;
    if (fresh) {
        memcpy(snapshot, s_map, MAP_LEN);
        s_map_fresh = false;
    }
    portEXIT_CRITICAL(&s_lock);
    if (!fresh) {
        return false;
    }

    int payload_max = (int)s_mtu - 3 - MAP_HEADER_LEN;
    if (payload_max <= 0) {
        return false;
    }
    if (payload_max > MAP_CHUNK_MAX) {
        payload_max = MAP_CHUNK_MAX;
    }
    int count = (MAP_LEN + payload_max - 1) / payload_max;
    if (count > 255) {
        return false;
    }

    for (int idx = 0; idx < count; idx++) {
        int offset = idx * payload_max;
        int n = MAP_LEN - offset < payload_max ? MAP_LEN - offset : payload_max;
        chunk[0] = MAP_VERSION;
        chunk[1] = map_id;
        chunk[2] = (uint8_t)idx;
        chunk[3] = (uint8_t)count;
        chunk[4] = DAQ_BLE_MAP_AZ_BINS;
        chunk[5] = DAQ_BLE_MAP_EL_BINS;
        put_u16(&chunk[6], (uint16_t)offset);
        memcpy(&chunk[MAP_HEADER_LEN], &snapshot[offset], n);
        notify(s_h_map, chunk, MAP_HEADER_LEN + n);
    }
    map_id++;
    return true;
}

#if DAQ_BLE_TEST_PATTERN
/* One synthetic source circling the array at 20 deg/s, elevation +-20 deg, and a matching map */
static void test_pattern_update(void)
{
    static float t;
    static uint8_t map[MAP_LEN];
    static TickType_t last_map;
    const float deg = (float)M_PI / 180.0f;

    t += DAQ_BLE_HOP_MS / 1000.0f;
    daq_source_t src = {
        .azimuth_deg = fmodf(t * 20.0f, 360.0f),
        .elevation_deg = 20.0f * sinf(t * 0.5f),
        .distance_m = 3.0f,
        .confidence = 0.9f,
        .track_id = 1,
    };
    ble_publish_result(&src, 1, 1);
    src.track_id = 0;
    ble_publish_raw(&src, 1, 1);

    TickType_t now = xTaskGetTickCount();
    if (now - last_map < pdMS_TO_TICKS(100)) {
        return;
    }
    last_map = now;
    float sx = cosf(src.elevation_deg * deg) * cosf(src.azimuth_deg * deg);
    float sy = cosf(src.elevation_deg * deg) * sinf(src.azimuth_deg * deg);
    float sz = sinf(src.elevation_deg * deg);
    for (int j = 0; j < DAQ_BLE_MAP_EL_BINS; j++) {
        float el = (-90.0f + (j + 0.5f) * 180.0f / DAQ_BLE_MAP_EL_BINS) * deg;
        for (int i = 0; i < DAQ_BLE_MAP_AZ_BINS; i++) {
            float az = (i * 360.0f / DAQ_BLE_MAP_AZ_BINS) * deg;
            float dot = cosf(el) * cosf(az) * sx + cosf(el) * sinf(az) * sy + sinf(el) * sz;
            float d = acosf(clampf(dot, -1.0f, 1.0f)) / (25.0f * deg);   /* 25 deg spot */
            map[j * DAQ_BLE_MAP_AZ_BINS + i] = (uint8_t)(255.0f * expf(-d * d));
        }
    }
    ble_publish_map(map, MAP_LEN);
}
#endif

static void ble_task(void *arg)
{
    uint8_t buf[DIAG_LEN];
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_diag = last_wake;
    TickType_t last_map = last_wake;

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DAQ_BLE_HOP_MS));
#if DAQ_BLE_TEST_PATTERN
        test_pattern_update();
#endif
        uint16_t seq = s_seq++;
        if (s_conn == BLE_HS_CONN_HANDLE_NONE) {
            continue;
        }
        TickType_t now = xTaskGetTickCount();

        if (s_sub_result) {
            build_result(&s_tracked, buf, seq);
            notify(s_h_result, buf, RESULT_LEN);
        }
        if (s_sub_raw) {
            build_result(&s_raw, buf, seq);
            notify(s_h_raw, buf, RESULT_LEN);
        }
        if (s_sub_diag && now - last_diag >= pdMS_TO_TICKS(DIAG_PERIOD_MS)) {
            last_diag = now;
            build_diag(buf);
            notify(s_h_diag, buf, DIAG_LEN);
        }
        uint8_t rate = g_settings.map_rate_hz;
        if (s_sub_map && rate > 0 && now - last_map >= pdMS_TO_TICKS(1000 / rate)) {
            if (send_map()) {
                last_map = now;
            }
        }
    }
}

/* ---------- public API ---------- */

static void publish_slot(result_slot_t *r, const daq_source_t *sources, int n_sources, int speaker_count)
{
    if (n_sources > DAQ_BLE_MAX_SOURCES) {
        n_sources = DAQ_BLE_MAX_SOURCES;
    }
    portENTER_CRITICAL(&s_lock);
    memcpy(r->sources, sources, n_sources * sizeof(daq_source_t));
    r->n_sources = n_sources;
    r->speaker_count = speaker_count;
    r->tick = xTaskGetTickCount();
    r->seen = true;
    portEXIT_CRITICAL(&s_lock);
}

void ble_publish_result(const daq_source_t *sources, int n_sources, int speaker_count)
{
    publish_slot(&s_tracked, sources, n_sources, speaker_count);
}

void ble_publish_raw(const daq_source_t *sources, int n_sources, int speaker_count)
{
    publish_slot(&s_raw, sources, n_sources, speaker_count);
}

void ble_publish_map(const uint8_t *map, size_t len)
{
    if (len != MAP_LEN) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    memcpy(s_map, map, MAP_LEN);
    s_map_fresh = true;
    portEXIT_CRITICAL(&s_lock);
}

esp_err_t ble_init(void)
{
    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "nimble_port_init");

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_gatts_count_cfg(s_svcs) != 0 || ble_gatts_add_svcs(s_svcs) != 0) {
        ESP_LOGE(TAG, "GATT table registration failed");
        return ESP_FAIL;
    }
    ble_att_set_preferred_mtu(247);

    nimble_port_freertos_init(host_task);
    BaseType_t ok = xTaskCreatePinnedToCore(ble_task, "ble", 4096, NULL, 4, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
