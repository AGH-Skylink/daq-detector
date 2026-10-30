#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "config.h"
#include "daq.h"

static const char *TAG = "stream";

static StreamBufferHandle_t s_ring;
static StaticStreamBuffer_t s_ring_struct;

void stream_set_enabled(bool enabled)
{
    if (enabled && !g_daq.streaming) {
        xStreamBufferReset(s_ring);   /* start from a clean packet boundary */
    }
    g_daq.streaming = enabled;
    ESP_LOGI(TAG, "USB stream %s", enabled ? "ON" : "OFF");
}

static void handle_host_commands(void)
{
    uint8_t cmd[8];
    int n = usb_serial_jtag_read_bytes(cmd, sizeof(cmd), 0);
    for (int i = 0; i < n; i++) {
        if (cmd[i] == 'S') {
            stream_set_enabled(true);
        } else if (cmd[i] == 'X') {
            stream_set_enabled(false);
        }
    }
}

static void stream_task(void *arg)
{
    static uint8_t chunk[DAQ_USB_CHUNK_BYTES];

    while (true) {
        handle_host_commands();
        size_t n = xStreamBufferReceive(s_ring, chunk, sizeof(chunk), pdMS_TO_TICKS(20));
        if (n == 0) {
            continue;
        }
        int sent = usb_serial_jtag_write_bytes(chunk, n, pdMS_TO_TICKS(DAQ_USB_WRITE_TIMEOUT_MS));
        if (sent > 0) {
            g_daq.usb_bytes_sent += sent;
        }
        /* A short write breaks packet alignment; the host resynchronises on the magic word */
    }
}

esp_err_t stream_init(void)
{
    uint8_t *ring_mem = heap_caps_malloc(DAQ_USB_RING_BYTES + 1, MALLOC_CAP_SPIRAM);
    if (!ring_mem) {
        return ESP_ERR_NO_MEM;
    }
    s_ring = xStreamBufferCreateStatic(DAQ_USB_RING_BYTES, 1, ring_mem, &s_ring_struct);

    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = DAQ_USB_TX_BUFFER_BYTES,
        .rx_buffer_size = 256,
    };
    ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&cfg), TAG, "usb_serial_jtag_driver_install");

    BaseType_t ok = xTaskCreatePinnedToCore(stream_task, "stream", 4096, NULL, 6, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

bool stream_push(const daq_packet_header_t *header, const void *data, size_t len)
{
    /* Single producer: header and payload go in together or not at all */
    if (xStreamBufferSpacesAvailable(s_ring) < sizeof(*header) + len) {
        return false;
    }
    xStreamBufferSend(s_ring, header, sizeof(*header), 0);
    xStreamBufferSend(s_ring, data, len, 0);
    return true;
}
