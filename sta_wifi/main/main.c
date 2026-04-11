/**
 * @file main.c
 * @brief Cliente STA de ESP32 que envia solicitudes HTTP GET a un servidor PHP local.
 * @author Daniel
 */

#include <string.h>                 // strlen, strstr, strchr, strcmp, memcpy
#include <ctype.h>                  // isspace
#include <stdio.h>                  // snprintf

#include "freertos/FreeRTOS.h"      // API base de FreeRTOS
#include "freertos/task.h"          // vTaskDelay
#include "freertos/event_groups.h"  // grupos de eventos para bits de sincronizacion
#include "esp_event.h"              // bucle de eventos y manejadores
#include "esp_log.h"                // ESP_LOGI / ESP_LOGW / ESP_LOGE
#include "esp_system.h"             // definiciones del sistema
#include "nvs_flash.h"              // inicializacion NVS requerida por WiFi
#include "esp_netif.h"              // interfaces de red TCP/IP
#include "esp_wifi.h"               // API del driver WiFi
#include "esp_http_client.h"        // API del cliente HTTP
#include "esp_err.h"                // codigos y nombres de error

static const char *TAG = "STA_HTTP";            // etiqueta mostrada en los logs del monitor

/** @name Configuracion del proyecto */
/** @{ */
// *** CAMBIAR con tus credenciales WiFi ***
/** @brief SSID de la red WiFi a la que se conecta el ESP32. */
#define WIFI_SSID      "AQUI_TU_WIFI"            // SSID WiFi para conectar el ESP32
/** @brief Clave de la red WiFi. */
#define WIFI_PASSWORD  "AQUI_TU_CLAVE"           // clave WiFi

// *** CAMBIAR con la IPv4 actual de la PC (cmd -> ipconfig) ***
/** @brief Direccion IPv4 de la PC donde esta Apache/XAMPP. */
#define SERVER_HOST    "aqui la direccion ip"    // IP de la PC donde corre Apache/XAMPP
/** @brief Puerto HTTP del servidor local. */
#define SERVER_PORT    80                         // puerto HTTP de Apache
/** @brief Ruta base del proyecto dentro de htdocs. */
#define SERVER_PATH    "/sta/"                   // ruta de carpeta dentro de htdocs
/** @brief Mensaje enviado por la ESP32 al parametro mensaje. */
#define REQUEST_MESSAGE "hola mundo"             // mensaje que se envia al servidor

/** @brief Tiempo de espera entre solicitudes HTTP periodicas. */
#define HTTP_REQUEST_INTERVAL_MS 5000            // espera entre solicitudes en ms
/** @brief Tamano maximo del buffer usado para leer la respuesta HTTP. */
#define HTTP_BUFFER_SIZE         1024            // tamano del buffer de respuesta en bytes
/** @brief Longitud maxima del mensaje codificado para la URL. */
#define REQUEST_ENCODED_MAX_LEN  192             // maximo del mensaje codificado para URL
/** @} */

static EventGroupHandle_t s_wifi_event_group;    // handle del grupo de eventos para estado WiFi
#define WIFI_CONNECTED_BIT BIT0                  // bit activo cuando STA obtiene IP

/**
 * @brief Retorna true si el caracter no necesita codificacion URL.
 */
static bool is_url_unreserved(char c)
{
    return (c >= 'A' && c <= 'Z') ||
           (c >= 'a' && c <= 'z') ||
           (c >= '0' && c <= '9') ||
           c == '-' || c == '_' || c == '.' || c == '~';
}

/**
 * @brief Codifica texto para usarlo como valor de query (?mensaje=...).
 */
static void url_encode(const char *src, char *dst, size_t dst_size)
{
    size_t di = 0;

    if (dst_size == 0) {
        return;
    }

    for (size_t si = 0; src[si] != '\0' && di + 1 < dst_size; si++) {
        unsigned char ch = (unsigned char)src[si];

        if (is_url_unreserved((char)ch)) {
            dst[di++] = (char)ch;
        } else if (ch == ' ') {
            dst[di++] = '+';
        } else {
            if (di + 3 >= dst_size) {
                break;
            }

            int n = snprintf(&dst[di], dst_size - di, "%%%02X", ch);
            if (n != 3) {
                break;
            }
            di += 3;
        }
    }

    dst[di] = '\0';
}

/**
 * @brief Escanea APs visibles y los imprime en el monitor serial.
 */
static void log_visible_aps(void)
{
    wifi_scan_config_t scan_cfg = {               // estructura de configuracion de escaneo
        .ssid = NULL,                             // escanear todos los SSID
        .bssid = NULL,                            // cualquier BSSID
        .channel = 0,                             // todos los canales
        .show_hidden = true,                      // incluir SSID ocultos
    };                                            // fin de configuracion de escaneo

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true); // iniciar escaneo bloqueante
    if (err != ESP_OK) {                          // si falla el escaneo
        ESP_LOGW(TAG, "Failed to scan APs: %s", esp_err_to_name(err)); // mostrar razon
        return;                                   // salir de la funcion
    }                                             // fin verificacion de error

    uint16_t ap_count = 0;                        // total de AP detectados
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count)); // obtener total de AP

    wifi_ap_record_t ap_records[20] = {0};        // lista local con maximo 20 AP
    uint16_t to_read = (ap_count > 20) ? 20 : ap_count; // limitar al tamano del buffer
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&to_read, ap_records)); // leer info de AP

    bool found_target = false;                    // bandera: SSID objetivo detectado
    ESP_LOGI(TAG, "Visible APs: %u (showing %u)", ap_count, to_read); // linea resumen
    for (uint16_t i = 0; i < to_read; i++) {      // recorrer cada AP detectado
        ESP_LOGI(TAG, "[%u] SSID='%s' RSSI=%d CH=%u", i + 1, // imprimir detalles del AP
                 (char *)ap_records[i].ssid, ap_records[i].rssi, ap_records[i].primary);
        if (strcmp((char *)ap_records[i].ssid, WIFI_SSID) == 0) { // comparar con objetivo
            found_target = true;                  // marcar SSID encontrado
        }                                         // fin comparacion
    }                                             // fin for

    if (!found_target) {                          // si no se vio el SSID objetivo
        ESP_LOGW(TAG, "Target SSID '%s' not found in scan", WIFI_SSID); // advertencia
    }                                             // fin verificacion de objetivo
}                                                 // fin de funcion

/**
 * @brief Elimina espacios al inicio y al final de una cadena.
 * @param str Cadena C mutable.
 * @return Puntero al primer caracter no-espacio dentro del mismo buffer.
 */
static char *trim_whitespace(char *str)
{
    char *end;                                    // puntero usado para recortar al final

    while (*str != '\0' && isspace((unsigned char)*str)) { // mover inicio sobre espacios
        str++;                                     // siguiente caracter
    }                                              // fin while

    if (*str == '\0') {                           // si queda vacia tras trim-izquierda
        return str;                                // devolver puntero a cadena vacia
    }                                              // fin verificacion vacio

    end = str + strlen(str) - 1;                  // apuntar al ultimo caracter valido
    while (end > str && isspace((unsigned char)*end)) { // retroceder sobre espacios
        end--;                                     // caracter anterior
    }                                              // fin while

    end[1] = '\0';                                // cerrar cadena despues del ultimo no-espacio
    return str;                                    // devolver inicio recortado
}                                                  // fin de funcion

/**
 * @brief Extrae el valor de una clave JSON usando parseo simple de texto.
 * @param payload Texto del cuerpo de respuesta.
 * @param key Clave JSON a buscar, por ejemplo "\"valor\"".
 * @param output Buffer destino.
 * @param output_size Tamano del buffer destino.
 * @return true si se extrajo un valor, false en caso contrario.
 */
static bool extract_json_value(const char *payload, const char *key,
                               char *output, size_t output_size)
{
    const char *found = strstr(payload, key);     // localizar clave en payload
    if (found == NULL) {                           // clave no encontrada
        return false;                              // extraccion fallida
    }                                              // fin verificacion de clave

    const char *colon = strchr(found, ':');        // buscar separador ':'
    if (colon == NULL) {                           // par clave/valor mal formado
        return false;                              // extraccion fallida
    }                                              // fin verificacion de ':'

    colon++;                                       // mover despues de ':'
    while (*colon != '\0' && isspace((unsigned char)*colon)) { // saltar espacios
        colon++;                                   // siguiente caracter
    }                                              // fin trim-izquierda del valor

    if (*colon == '"') {                           // cadena JSON entre comillas
        colon++;                                   // saltar comilla de apertura
        const char *end_quote = strchr(colon, '"'); // buscar comilla de cierre
        if (end_quote == NULL) {                   // si no hay comilla de cierre
            return false;                          // cadena JSON mal formada
        }                                          // fin verificacion de comilla
        size_t length = end_quote - colon;         // longitud del valor en caracteres
        if (length >= output_size) {               // prevenir overflow
            length = output_size - 1;              // truncar de forma segura
        }                                          // fin ajuste de longitud
        memcpy(output, colon, length);             // copiar bytes del valor
        output[length] = '\0';                     // terminar cadena de salida
        return true;                               // extraccion exitosa
    }                                              // fin rama con comillas

    const char *end = colon;                       // inicio de valor sin comillas
    while (*end != '\0' && *end != ',' && *end != '}' && *end != '\n' && *end != '\r') { // tokens de parada
        end++;                                     // avanzar hasta token
    }                                              // fin while

    size_t length = end - colon;                   // longitud cruda del valor
    if (length >= output_size) {                   // prevenir overflow
        length = output_size - 1;                  // truncar de forma segura
    }                                              // fin ajuste de longitud
    memcpy(output, colon, length);                 // copiar valor crudo
    output[length] = '\0';                         // terminar cadena de salida
    trim_whitespace(output);                       // quitar espacios extra
    return true;                                   // extraccion exitosa
}                                                  // fin de funcion

/**
 * @brief Procesa la respuesta del servidor e imprime el valor extraido.
 * @param payload Texto del cuerpo HTTP recibido.
 */
static void procesar_respuesta(const char *payload)
{
    char valor[128] = {0};                         // buffer para "valor"

    if (extract_json_value(payload, "\"valor\"", valor, sizeof(valor))) { // intentar formato JSON
        ESP_LOGI(TAG, "Extracted value (valor): %s", valor); // imprimir valor
        return;                                    // detener si parseo JSON fue exitoso
    }                                              // fin intento JSON

    const char *dato_ptr = strstr(payload, "dato="); // intentar formato texto plano
    if (dato_ptr != NULL) {                        // si existe el token
        dato_ptr += strlen("dato=");               // saltar prefijo del token
        char dato[128] = {0};                      // buffer para valor plano
        const char *end = strchr(dato_ptr, '\n');  // buscar fin de linea
        if (end == NULL) {                         // si no hay salto de linea
            end = dato_ptr + strlen(dato_ptr);     // usar fin de cadena
        }                                          // fin verificacion de linea
        size_t length = end - dato_ptr;            // longitud del valor
        if (length >= sizeof(dato)) {              // prevenir overflow
            length = sizeof(dato) - 1;             // truncar de forma segura
        }                                          // fin ajuste de longitud
        memcpy(dato, dato_ptr, length);            // copiar valor
        dato[length] = '\0';                       // terminar cadena
        trim_whitespace(dato);                     // quitar espacios
        ESP_LOGI(TAG, "Extracted value (dato=): %s", dato); // imprimir valor
        return;                                    // detener despues de parsear
    }                                              // fin rama de texto plano

    ESP_LOGW(TAG, "No useful data pattern found in response."); // advertencia de fallback
}                                                  // fin de funcion

/**
 * @brief Maneja eventos de WiFi e IP.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;                                     // argumento no utilizado

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) { // driver WiFi iniciado
        ESP_LOGI(TAG, "WiFi STA started");        // log informativo
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) { // desconectado
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data; // convertir datos del evento
        esp_wifi_connect();                         // reconectar automaticamente
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // marcar como desconectado
        ESP_LOGW(TAG, "WiFi disconnected, retrying... reason=%u", disc->reason); // razon de desconexion
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { // obtuvo IPv4
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data; // convertir datos del evento IP
        ESP_LOGI(TAG, "Connected to WiFi. IP: " IPSTR, IP2STR(&event->ip_info.ip)); // imprimir IP
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // marcar como conectado
    }                                              // fin rama de eventos
}                                                  // fin de funcion

/**
 * @brief Inicializa ESP32 en modo STA e inicia el flujo de conexion.
 */
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();      // crear objeto de grupo de eventos

    ESP_ERROR_CHECK(esp_netif_init());             // inicializar interfaces de red
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // inicializar bucle de eventos
    esp_netif_create_default_wifi_sta();           // crear interfaz STA por defecto

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // valores por defecto de inicio WiFi
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));          // inicializar driver WiFi

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, // registrar eventos WiFi
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, // registrar eventos IP
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {                  // configuracion de conexion STA
        .sta = {
            .ssid = WIFI_SSID,                     // SSID objetivo
            .password = WIFI_PASSWORD,             // clave objetivo
            .scan_method = WIFI_ALL_CHANNEL_SCAN, // escanear todos los canales
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL, // preferir AP con mejor senal
            .threshold.authmode = WIFI_AUTH_OPEN, // no filtrar tipo de autenticacion
            .pmf_cfg = { .capable = true, .required = false }, // PMF opcional
        },
    };

    wifi_country_t country = {                     // configuracion de dominio de pais
        .cc = "01",                               // dominio global seguro
        .schan = 1,                                // canal inicial
        .nchan = 13,                               // cantidad de canales
        .policy = WIFI_COUNTRY_POLICY_MANUAL,     // politica manual fija
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country)); // aplicar configuracion de pais

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // establecer modo estacion
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // aplicar config STA
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // desactivar ahorro de energia para estabilidad
    ESP_ERROR_CHECK(esp_wifi_start());             // iniciar driver WiFi

    log_visible_aps();                             // escaneo diagnostico de AP
    ESP_ERROR_CHECK(esp_wifi_connect());           // lanzar primer intento de conexion

    ESP_LOGI(TAG, "WiFi initialization finished. Connecting to SSID: %s", WIFI_SSID); // log de inicio
}                                                  // fin de funcion

/**
 * @brief Verifica si el bit de conexion WiFi esta activo.
 * @return true si esta conectado y tiene IP, false en caso contrario.
 */
static bool is_wifi_connected(void)
{
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0; // prueba de bit
}                                                  // fin de funcion

/**
 * @brief Envia una solicitud HTTP GET y procesa el payload de respuesta.
 * @return ESP_OK en exito, codigo de error en otro caso.
 */
static esp_err_t send_http_request(void)
{
    char url[256];                                 // buffer de URL
    char encoded_message[REQUEST_ENCODED_MAX_LEN]; // mensaje codificado para URL

    url_encode(REQUEST_MESSAGE, encoded_message, sizeof(encoded_message)); // codificar query

    snprintf(url, sizeof(url), "http://%s:%d%s?mensaje=%s", // construir URL final con mensaje
             SERVER_HOST, SERVER_PORT, SERVER_PATH, encoded_message);
    ESP_LOGI(TAG, "GET URL: %s", url);            // imprimir URL

    esp_http_client_config_t config = {            // configuracion del cliente HTTP
        .url = url,                                // endpoint URL
        .method = HTTP_METHOD_GET,                 // solicitud GET
        .timeout_ms = 10000,                       // timeout en milisegundos
    };

    esp_http_client_handle_t client = esp_http_client_init(&config); // crear handle de cliente
    if (client == NULL) {                          // si falla reserva/inicializacion
        ESP_LOGE(TAG, "Failed to initialize HTTP client"); // reportar error
        return ESP_FAIL;                           // abortar llamada
    }                                              // fin verificacion de cliente

    esp_err_t err = esp_http_client_open(client, 0); // abrir conexion y enviar headers
    if (err == ESP_OK) {                           // si apertura fue exitosa
        int content_length = esp_http_client_fetch_headers(client); // obtener headers de respuesta
        int status_code = esp_http_client_get_status_code(client);  // leer estado HTTP
        ESP_LOGI(TAG, "HTTP status code: %d", status_code); // imprimir estado

        if (status_code == 200) {                  // procesar solo 200 OK
            char buffer[HTTP_BUFFER_SIZE + 1] = {0}; // buffer de payload + byte nulo
            int total_read = 0;                    // bytes acumulados leidos

            while (total_read < HTTP_BUFFER_SIZE) { // leer hasta llenar buffer o sin datos
                int r = esp_http_client_read(client, buffer + total_read, HTTP_BUFFER_SIZE - total_read); // leer bloque
                if (r <= 0) {                      // timeout/fin/error
                    break;                         // detener lectura
                }                                  // fin verificacion de bloque
                total_read += r;                   // sumar bytes leidos
            }                                      // fin bucle de lectura

            buffer[total_read] = '\0';             // forzar terminador nulo
            ESP_LOGI(TAG, "Received content (%d bytes): %s", total_read, buffer); // imprimir payload
            if (total_read > 0) {                  // si hay payload
                procesar_respuesta(buffer);         // parsear y loguear valor extraido
            } else {                               // payload vacio
                ESP_LOGW(TAG, "Empty response from server"); // advertencia
            }                                      // fin verificacion de payload

            (void)content_length;                  // silenciar warning de no usado
        } else {                                   // respuesta distinta de 200
            ESP_LOGW(TAG, "HTTP response not OK: %d", status_code); // advertencia
        }                                          // fin rama por estado
    } else {                                       // fallo apertura de conexion
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err)); // imprimir codigo de error
    }                                              // fin rama de apertura

    esp_http_client_cleanup(client);               // liberar recursos del cliente HTTP
    return err;                                    // devolver resultado de operacion
}                                                  // fin de funcion

/**
 * @brief Punto de entrada de la aplicacion.
 */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();              // inicializar flash NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { // NVS requiere borrado
        ESP_ERROR_CHECK(nvs_flash_erase());        // borrar NVS invalido
        ret = nvs_flash_init();                    // inicializar nuevamente
    }                                              // fin recuperacion NVS
    ESP_ERROR_CHECK(ret);                          // detener si NVS sigue fallando

    ESP_LOGI(TAG, "Starting ESP-IDF STA HTTP client"); // log de arranque

    wifi_init_sta();                               // inicializar modo WiFi STA

    while (true) {                                 // bucle infinito de la tarea
        if (is_wifi_connected()) {                 // si esta conectado a WiFi
            ESP_LOGI(TAG, "Sending HTTP request..."); // log de inicio de solicitud
            send_http_request();                   // ejecutar una solicitud HTTP GET
        } else {                                   // si esta desconectado
            ESP_LOGW(TAG, "Not connected to WiFi. Waiting for connection..."); // advertir desconexion
        }                                          // fin rama de conectividad

        vTaskDelay(pdMS_TO_TICKS(HTTP_REQUEST_INTERVAL_MS)); // esperar antes del siguiente ciclo
    }                                              // fin bucle principal
}                                                  // fin app_main
