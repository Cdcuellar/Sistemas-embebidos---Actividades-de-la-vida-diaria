idf_component_register(
    SRCS "main.c" "mpu6050.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_wifi esp_event esp_netif esp_pm nvs_flash
)
