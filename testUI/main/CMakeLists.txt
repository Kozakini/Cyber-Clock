idf_component_register(
    SRCS
        "main.c"
        "epd_ui.c"
        "bme280_mqtt.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        esp_http_client
        json
        driver
        esp_timer
    PRIV_REQUIRES mqtt driver
)
