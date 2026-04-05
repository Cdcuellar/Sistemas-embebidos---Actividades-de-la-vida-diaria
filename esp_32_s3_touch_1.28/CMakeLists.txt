# Las siguientes líneas son obligatorias para ESP-IDF (no cambiar orden)
cmake_minimum_required(VERSION 3.22)

# Fijar el target al ESP32-S3 (chip del wearable 1.28")
set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_32_s3_touch_1.28)
