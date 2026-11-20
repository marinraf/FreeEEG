idf_component_register(
    SRCS "tasks.c" "config_util.c"
    INCLUDE_DIRS "include"
    # Public headers (config_util.h, tasks.h) expose these:
    REQUIRES ads1299 eeg_core esp_ringbuf
    PRIV_REQUIRES eeg_net icm42670 ics43434 ws2812 sd_card esp_driver_gpio
)
