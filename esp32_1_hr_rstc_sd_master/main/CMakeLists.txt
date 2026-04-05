idf_component_register(SRCS "espnow_example_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_event esp_netif esp_wifi freertos log fatfs sdmmc driver esp_pm esp_adc)
