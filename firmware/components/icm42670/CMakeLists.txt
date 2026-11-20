idf_component_register(
    SRCS "icm42670.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_i2c esp_driver_gpio
)
