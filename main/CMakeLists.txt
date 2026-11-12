idf_component_register(SRCS "main.c" "capture.c" "storage.c" "stream.c" "ble.c" "system.c" "dsp.c" "localize.c" "track.c" "music.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_gpio
                                     esp_driver_i2c
                                     esp_driver_i2s
                                     esp_driver_spi
                                     esp_driver_sdspi
                                     esp_driver_usb_serial_jtag
                                     esp_timer
                                     fatfs
                                     sdmmc
                                     bt
                                     nvs_flash)
