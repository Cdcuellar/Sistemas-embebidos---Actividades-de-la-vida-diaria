#include <stdio.h>              // Incluye la biblioteca estándar de entrada/salida para funciones como printf
#include <string.h>             // Incluye la biblioteca de strings para funciones como manipulación de strings (aunque no se usa directamente aquí)
#include "esp_wifi.h"           // Incluye la biblioteca de WiFi de ESP-IDF para funciones relacionadas con WiFi
#include "esp_system.h"         // Incluye la biblioteca del sistema ESP para funciones del sistema
#include "nvs_flash.h"          // Incluye la biblioteca NVS (Non-Volatile Storage) para almacenamiento persistente
#include "esp_netif.h"          // Incluye la biblioteca de red de ESP-IDF para interfaces de red
#include "esp_event.h"          // Incluye la biblioteca de eventos de ESP-IDF para el loop de eventos

void app_main(void)             // Función principal del programa en ESP-IDF para ESP32, se ejecuta al iniciar la aplicación
{
    // Inicializar NVS (necesario para WiFi en ESP32)
    esp_err_t ret = nvs_flash_init();  // Inicializa el sistema NVS y guarda el resultado en 'ret'
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {  // Verifica si hay errores de NVS
        ESP_ERROR_CHECK(nvs_flash_erase());  // Borra el NVS si hay problemas
        ret = nvs_flash_init();  // Reintenta inicializar NVS
    }
    ESP_ERROR_CHECK(ret);  // Verifica si la inicialización de NVS fue exitosa

    // Inicializar la interfaz de red
    ESP_ERROR_CHECK(esp_netif_init());  // Inicializa la interfaz de red de ESP-IDF

    // Crear el loop de eventos por defecto
    ESP_ERROR_CHECK(esp_event_loop_create_default());  // Crea el loop de eventos por defecto para manejar eventos del sistema en ESP32

    // Crear la interfaz de red por defecto para WiFi STA
    esp_netif_create_default_wifi_sta();  // Crea la interfaz de red por defecto para el modo Station (cliente) de WiFi

    // Inicializar WiFi en modo station
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // Crea una configuración por defecto para WiFi
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));  // Inicializa WiFi con la configuración por defecto
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));  // Establece el modo WiFi como Station (cliente) en ESP32
    ESP_ERROR_CHECK(esp_wifi_start());  // Inicia el WiFi

    // Obtener la dirección MAC
    uint8_t mac[6];  // Declara un arreglo de 6 bytes para almacenar la dirección MAC del ESP32
    esp_wifi_get_mac(WIFI_IF_STA, mac);  // Obtiene la dirección MAC del interfaz Station del ESP32 y la guarda en 'mac'

    // Imprimir la dirección MAC en formato legible
    printf("Dirección MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",  // Imprime la dirección MAC en formato hexadecimal con dos dígitos por byte
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);  // Pasa cada byte de la MAC del ESP32 al printf

    // Aquí puedes agregar más código para ESP-NOW en ESP32
    // Por ejemplo, inicializar ESP-NOW con esp_now_init()
}
