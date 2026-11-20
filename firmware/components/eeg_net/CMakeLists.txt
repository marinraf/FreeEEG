# include/index.html.h is generated from index.html:
#   xxd -i -n webpage_html index.html > include/index.html.h
idf_component_register(
    SRCS "wifi_app_ap.c" "wifi_app_config.c" "wifi_app_data.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES eeg_core esp_wifi esp_netif esp_event esp_http_server lwip
)
