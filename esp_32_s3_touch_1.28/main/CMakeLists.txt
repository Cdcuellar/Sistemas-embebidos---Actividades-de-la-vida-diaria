idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
    driver
    esp_driver_i2c
    esp_driver_spi
    esp_driver_gpio
    esp_adc
    esp_pm
    esp_wifi
    nvs_flash
    esp_netif
    esp_event
    freertos
    log
    esp_timer
)

# Vincular la librería matemática estándar (necesaria para sinf/cosf/atan2f/sqrtf)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)
