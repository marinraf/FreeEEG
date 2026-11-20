idf_component_register(
    SRCS "sd_card.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_sdmmc esp_driver_gpio sdmmc fatfs
)
