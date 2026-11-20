idf_component_register(
    SRCS "ics43434.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_i2s esp_driver_gpio
)
