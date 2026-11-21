idf_component_register(
    SRCS
        "app_main.c"
        "heater_control.c"
        "max6675.c"
        "web_server.c"
        "wifi_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_event
        esp_http_server
        esp_netif
        esp_timer
        esp_wifi
        nvs_flash
        led_strip
    EMBED_TXTFILES "web/index.html"
)
