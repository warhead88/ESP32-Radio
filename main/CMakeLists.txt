idf_component_register(SRCS "main.c" "rda5807m.c" "web_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_server esp_wifi esp_event nvs_flash esp_netif driver esp_adc esp_driver_pcnt esp_driver_i2c esp_timer cjson)
