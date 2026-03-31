/* ============================================================
   BIBLIOTECAS
   ============================================================ */

#include <stdio.h>           /* Entrada/salida estándar (printf, etc.)        */
#include <math.h>            /* Funciones matemáticas (atan2, sqrt, etc.)     */
#include <string.h>          /* Manipulación de memoria (memcpy, memset, etc.)*/
#include "freertos/FreeRTOS.h"  /* Núcleo de FreeRTOS: tipos y macros básicos */
#include "freertos/task.h"      /* Creación y control de tareas (xTaskCreate) */
#include "freertos/semphr.h"    /* Semáforos y mutexes (xSemaphoreCreateMutex)*/
#include "driver/i2c_master.h"  /* Driver I2C nuevo de ESP-IDF v5+            */
#include "esp_log.h"            /* Sistema de logs con niveles (ESP_LOGI, etc)*/
#include "esp_err.h"            /* Tipos de error y ESP_ERROR_CHECK            */
#include "esp_pm.h"             /* Gestión de energía (esp_pm_configure)       */
#include "esp_now.h"            /* Protocolo ESP-NOW para comunicación directa */
#include "esp_wifi.h"           /* Driver WiFi (necesario para usar ESP-NOW)   */
#include "esp_event.h"          /* Sistema de eventos (requerido por WiFi)     */
#include "esp_netif.h"          /* Interfaz de red (requerida por WiFi)        */
#include "nvs_flash.h"          /* Almacenamiento no volátil (necesario WiFi)  */
#include "mpu6050.h"            /* Driver propio del sensor MPU6050            */

/* ============================================================
   ETIQUETA DE LOG
   ============================================================ */

/* Prefijo que aparece en todos los mensajes del monitor serie */
static const char *TAG = "NODO2_MPU6050";

/* ============================================================
   CONFIGURACION DE HARDWARE - PINES I2C
   ============================================================ */

#define I2C_MASTER_SCL_IO        22       /* GPIO 22: reloj del bus I2C (SCL)       */
#define I2C_MASTER_SDA_IO        21       /* GPIO 21: datos del bus I2C (SDA)        */
#define I2C_MASTER_FREQ_HZ       100000   /* Velocidad I2C: 100 kHz (modo estándar)  */

/* ============================================================
   CONFIGURACION DEL SENSOR MPU6050
   ============================================================ */

#define MPU6050_ADDR             0x68     /* Dirección I2C con pin AD0 en GND        */
#define MPU6050_ADDR_ALT         0x69     /* Dirección I2C alternativa con AD0 en VCC*/

/* ============================================================
   CONFIGURACION DE TAREAS FREERTOS
   ============================================================ */

#define PERIODO_MUESTREO_MS      100      /* Cada cuántos ms lee el sensor (100 ms)  */
#define PERIODO_ENVIO_MS         500      /* Cada cuántos ms envía por ESP-NOW (500 ms)*/
#define REINTENTOS_DETECCION     3        /* Intentos para detectar el sensor al inicio*/
#define LOG_CADA_MUESTRAS        5        /* Imprime en consola cada 5 muestras leídas */

/* ============================================================
   CONFIGURACION DE RED ESP-NOW
   ============================================================ */

#define CANAL_WIFI_ESPNOW        1        /* Canal WiFi fijo para la comunicación ESP-NOW */

/* MAC del nodo maestro que recibirá los datos (6 bytes en hexadecimal) */
static const uint8_t MAC_MAESTRO[ESP_NOW_ETH_ALEN] = {0xC8, 0x2E, 0x18, 0x67, 0x2F, 0xC4};

/* ============================================================
   ESTRUCTURA DE DATOS COMPARTIDA ENTRE TAREAS
   ============================================================ */

/* Paquete de datos que se envía por ESP-NOW al maestro */
typedef struct {
    float mpu_x;   /* Aceleración en eje X en unidades g (±2g)  */
    float mpu_y;   /* Aceleración en eje Y en unidades g (±2g)  */
    float mpu_z;   /* Aceleración en eje Z en unidades g (±2g)  */
} datos_mpu_t;

/* ============================================================
   VARIABLES GLOBALES
   ============================================================ */

/* Buffer compartido: la tarea de muestreo escribe aquí, la de envío lee */
static datos_mpu_t datos_compartidos = {0};

/* Mutex binario: impide que dos tareas accedan a datos_compartidos al mismo tiempo */
static SemaphoreHandle_t mutex_datos = NULL;

/* Instancia del sensor MPU6050: contiene el handle I2C y los offsets de calibración */
static mpu6050_t sensor_mpu;

/* ============================================================
   CALLBACK DE CONFIRMACION ESP-NOW
   ============================================================ */

/*
 * El hardware llama a esta función automáticamente después de cada envío ESP-NOW.
 * Informa si el paquete llegó al maestro o si hubo fallo de transmisión.
 */
static void callback_envio_espnow(const esp_now_send_info_t *info_envio, esp_now_send_status_t estado)
{
    /* Protección: si el puntero es NULL el sistema está en estado inválido */
    if (info_envio == NULL) {
        return;
    }

    if (estado == ESP_NOW_SEND_SUCCESS) {
        /* El maestro acusó recibo del paquete enviado */
        ESP_LOGI(TAG, "ESP-NOW enviado OK a %02X:%02X:%02X:%02X:%02X:%02X",
                 info_envio->des_addr[0], info_envio->des_addr[1], info_envio->des_addr[2],
                 info_envio->des_addr[3], info_envio->des_addr[4], info_envio->des_addr[5]);
    } else {
        /* El maestro no respondió; puede estar apagado o fuera de rango */
        ESP_LOGW(TAG, "ESP-NOW fallo de envio a %02X:%02X:%02X:%02X:%02X:%02X",
                 info_envio->des_addr[0], info_envio->des_addr[1], info_envio->des_addr[2],
                 info_envio->des_addr[3], info_envio->des_addr[4], info_envio->des_addr[5]);
    }
}

/* ============================================================
   INICIALIZACION DE RED WIFI (BASE PARA ESP-NOW)
   ============================================================ */

/*
 * ESP-NOW necesita que el subsistema WiFi esté activo aunque no haya conexión a router.
 * Esta función arranca WiFi en modo estación (STA) y fija el canal acordado con el maestro.
 */
static esp_err_t iniciar_wifi_para_espnow(void)
{
    /* Inicializa la memoria NVS (requerida por el driver WiFi) */
    esp_err_t ret = nvs_flash_init();

    /* Si la partición NVS está llena o tiene una versión diferente, se borra y reinicia */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());   /* Borra toda la partición NVS */
        ret = nvs_flash_init();               /* Reinicia NVS desde cero */
    }
    ESP_ERROR_CHECK(ret);   /* Detiene ejecución si NVS sigue fallando */

    ESP_ERROR_CHECK(esp_netif_init());                   /* Arranca capa de red TCP/IP     */
    ESP_ERROR_CHECK(esp_event_loop_create_default());    /* Crea el bucle de eventos global */
    ESP_ERROR_CHECK(esp_wifi_init(                       /* Inicializa el driver WiFi con   */
        &(wifi_init_config_t)WIFI_INIT_CONFIG_DEFAULT()  /* configuración por defecto        */
    ));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));   /* Modo: estación (no access point) */
    ESP_ERROR_CHECK(esp_wifi_start());                   /* Enciende el hardware WiFi         */
    ESP_ERROR_CHECK(esp_wifi_set_channel(                /* Fija el canal al mismo que usa    */
        CANAL_WIFI_ESPNOW, WIFI_SECOND_CHAN_NONE         /* el maestro (canal 1, ancho 20MHz) */
    ));

    return ESP_OK;
}

/* ============================================================
   INICIALIZACION DE ESP-NOW
   ============================================================ */

/*
 * Registra el protocolo ESP-NOW sobre el WiFi ya iniciado,
 * asigna el callback de confirmación y añade el nodo maestro como destino conocido.
 */
static esp_err_t iniciar_espnow(void)
{
    ESP_ERROR_CHECK(esp_now_init());                          /* Arranca el stack ESP-NOW       */
    ESP_ERROR_CHECK(esp_now_register_send_cb(callback_envio_espnow)); /* Registra callback de envío */

    /* Estructura que describe al nodo destino (el maestro) */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, MAC_MAESTRO, ESP_NOW_ETH_ALEN);   /* Copia la MAC del maestro        */
    peer.channel  = CANAL_WIFI_ESPNOW;                        /* Canal WiFi (debe coincidir)     */
    peer.ifidx    = WIFI_IF_STA;                              /* Interfaz: modo estación         */
    peer.encrypt  = false;                                    /* Sin cifrado AES (más rápido)    */

    /* Si el peer ya existía de un arranque anterior, se elimina antes de re-añadirlo */
    if (esp_now_is_peer_exist(MAC_MAESTRO)) {
        ESP_ERROR_CHECK(esp_now_del_peer(MAC_MAESTRO));
    }
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));   /* Registra el maestro como destino válido */

    return ESP_OK;
}

/* ============================================================
   GESTION DE CONSUMO ENERGETICO
   ============================================================ */

/*
 * Configura el CPU a 80 MHz (suficiente para I2C + ESP-NOW) y
 * habilita Light Sleep automático cuando ambas tareas están en vTaskDelay.
 */
static void configurar_gestion_potencia(void)
{
    esp_pm_config_t config_pm = {
        .max_freq_mhz      = 80,    /* Frecuencia máxima de CPU: 80 MHz           */
        .min_freq_mhz      = 80,    /* Frecuencia mínima de CPU: 80 MHz (fija)    */
        .light_sleep_enable = true, /* Entra en Light Sleep entre vTaskDelays     */
    };

    esp_err_t ret = esp_pm_configure(&config_pm);   /* Aplica la configuración de energía */
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Power Management activo: CPU 80MHz, Light Sleep habilitado");
    } else {
        /* Puede fallar si el proyecto no tiene ESP_PM habilitado en sdkconfig */
        ESP_LOGW(TAG, "No se pudo habilitar Power Management: %s", esp_err_to_name(ret));
    }
}

/* ============================================================
   TAREA DE MUESTREO (cada 100 ms)
   ============================================================ */

/*
 * Lee los 6 ejes del MPU6050 cada 100 ms.
 * Convierte la aceleración a unidades g y actualiza la estructura compartida.
 * Cada 5 lecturas (cada 500 ms) imprime un resumen en el monitor serie.
 */
static void tarea_muestreo_mpu(void *param)
{
    (void)param;               /* Parámetro no usado; se suprime advertencia del compilador */
    int contador_logs = 0;     /* Cuenta muestras leídas para controlar frecuencia de logs */

    while (1) {
        /* Variables locales para recibir los datos crudos del sensor */
        int16_t ax = 0, ay = 0, az = 0;   /* Aceleración cruda en los 3 ejes (ADC 16 bits) */
        int16_t gx = 0, gy = 0, gz = 0;   /* Velocidad angular cruda en los 3 ejes          */

        /* Lee los 14 bytes del sensor por I2C; se pasa NULL en temperatura porque no se usa */
        esp_err_t ret = mpu6050_read_all_raw(&sensor_mpu, &ax, &ay, &az, NULL, &gx, &gy, &gz);

        if (ret == ESP_OK) {
            /* ------------------------------------------------
               CONVERSION A UNIDADES FISICAS
               ------------------------------------------------ */

            /* Rango ±2g → sensibilidad 16384 LSB/g → dividimos para obtener g */
            float ax_g = ax / 16384.0f;   /* Aceleración X en g */
            float ay_g = ay / 16384.0f;   /* Aceleración Y en g */
            float az_g = az / 16384.0f;   /* Aceleración Z en g */

            /* ------------------------------------------------
               CALCULO DE ANGULOS DE INCLINACION
               ------------------------------------------------ */

            /* Roll: rotación sobre el eje X (ladeo lateral), rango ±180° */
            float roll  = atan2f(ay_g, az_g) * (180.0f / (float)M_PI);

            /* Pitch: rotación sobre el eje Y (cabeceo adelante/atrás), rango ±90° */
            float pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * (180.0f / (float)M_PI);

            /* ------------------------------------------------
               ACTUALIZACION DE DATOS COMPARTIDOS (con mutex)
               ------------------------------------------------ */

            /* Toma el mutex: bloquea hasta 20 ms para no chocar con la tarea de envío */
            if (xSemaphoreTake(mutex_datos, pdMS_TO_TICKS(20)) == pdTRUE) {
                datos_compartidos.mpu_x = ax_g;   /* Escribe X en el buffer compartido */
                datos_compartidos.mpu_y = ay_g;   /* Escribe Y en el buffer compartido */
                datos_compartidos.mpu_z = az_g;   /* Escribe Z en el buffer compartido */
                xSemaphoreGive(mutex_datos);       /* Libera el mutex para que otros accedan */
            }

            /* ------------------------------------------------
               LOG EN MONITOR SERIE (cada 5 muestras = 500 ms)
               ------------------------------------------------ */

            contador_logs++;
            if ((contador_logs % LOG_CADA_MUESTRAS) == 0) {
                /* Rango ±250°/s → sensibilidad 131 LSB/(°/s) */
                float gx_dps = gx / 131.0f;   /* Velocidad angular X en grados/segundo */
                float gy_dps = gy / 131.0f;   /* Velocidad angular Y en grados/segundo */
                float gz_dps = gz / 131.0f;   /* Velocidad angular Z en grados/segundo */

                ESP_LOGI(TAG, "AX:%.3f AY:%.3f AZ:%.3f | GX:%.2f GY:%.2f GZ:%.2f | Roll:%.1f Pitch:%.1f",
                         ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, roll, pitch);
            }
        } else {
            /* Error de comunicación I2C; puede ser ruido o sensor desconectado */
            ESP_LOGE(TAG, "Error leyendo MPU6050: %s", esp_err_to_name(ret));
        }

        /* Cede CPU durante 100 ms; el PM puede poner el chip en Light Sleep aquí */
        vTaskDelay(pdMS_TO_TICKS(PERIODO_MUESTREO_MS));
    }
}

/* ============================================================
   TAREA DE TRANSMISION ESP-NOW (cada 500 ms)
   ============================================================ */

/*
 * Cada 500 ms copia el último dato disponible y lo envía al maestro por ESP-NOW.
 * El resultado real del envío (ACK o fallo) llega en callback_envio_espnow.
 */
static void tarea_transmision_espnow(void *param)
{
    (void)param;                        /* Parámetro no usado */
    datos_mpu_t copia_local = {0};      /* Copia local para no bloquear el mutex durante el envío */

    while (1) {
        /* ------------------------------------------------
           COPIA DE DATOS COMPARTIDOS (con mutex)
           ------------------------------------------------ */

        /* Toma el mutex: espera hasta 20 ms antes de rendirse */
        if (xSemaphoreTake(mutex_datos, pdMS_TO_TICKS(20)) == pdTRUE) {
            copia_local = datos_compartidos;   /* Copia atómica de la estructura completa */
            xSemaphoreGive(mutex_datos);       /* Libera el mutex inmediatamente           */
        }

        /* ------------------------------------------------
           ENVIO POR ESP-NOW
           ------------------------------------------------ */

        /* Envía sizeof(datos_mpu_t) = 12 bytes al maestro; confirmación llega por callback */
        esp_err_t ret = esp_now_send(MAC_MAESTRO, (const uint8_t *)&copia_local, sizeof(copia_local));
        if (ret != ESP_OK) {
            /* Error local de stack (ej. buffer lleno); diferente al fallo de ACK del callback */
            ESP_LOGW(TAG, "Error enviando ESP-NOW: %s", esp_err_to_name(ret));
        }

        /* Pausa de 500 ms; permite Light Sleep si la otra tarea también está dormida */
        vTaskDelay(pdMS_TO_TICKS(PERIODO_ENVIO_MS));
    }
}

/* ============================================================
   PUNTO DE ENTRADA PRINCIPAL
   ============================================================ */

/*
 * app_main() es el equivalente al main() de C estándar en ESP-IDF.
 * Se ejecuta una sola vez al arrancar; al final crea las tareas y retorna.
 */
void app_main(void)
{
    /* Establece nivel de log INFO para todos los módulos del sistema */
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Arranque Nodo Esclavo #2 MPU6050");

    /* ------------------------------------------------
       INICIALIZACION DEL BUS I2C
       ------------------------------------------------ */

    /* Configura el bus I2C en el puerto 0 con los pines y velocidad definidos arriba */
    i2c_master_bus_config_t config_bus = {
        .clk_source            = I2C_CLK_SRC_DEFAULT,      /* Fuente de reloj por defecto del chip    */
        .i2c_port              = I2C_NUM_0,                 /* Puerto I2C 0 (hay 2 en ESP32)           */
        .scl_io_num            = I2C_MASTER_SCL_IO,         /* Pin GPIO 22 para señal de reloj SCL     */
        .sda_io_num            = I2C_MASTER_SDA_IO,         /* Pin GPIO 21 para señal de datos SDA     */
        .glitch_ignore_cnt     = 7,                         /* Filtro de ruido: ignora pulsos <7 ciclos*/
        .flags.enable_internal_pullup = true,               /* Habilita pull-up interno del ESP32      */
    };
    i2c_master_bus_handle_t bus_i2c;                        /* Handle del bus I2C que usaremos después */
    ESP_ERROR_CHECK(i2c_new_master_bus(&config_bus, &bus_i2c));   /* Crea el bus; aborta si falla */

    /* ------------------------------------------------
       DETECCION DEL SENSOR MPU6050
       ------------------------------------------------ */

    esp_err_t ret = ESP_FAIL;   /* Variable de resultado; empieza en fallo hasta que detecte */

    /* Intenta inicializar el sensor hasta REINTENTOS_DETECCION veces */
    for (int intento = 1; intento <= REINTENTOS_DETECCION; intento++) {

        /* Primer intento: dirección 0x68 (AD0 conectado a GND, caso más común) */
        ret = mpu6050_init(&sensor_mpu, bus_i2c, MPU6050_ADDR, I2C_MASTER_FREQ_HZ);
        if (ret == ESP_OK) {
            break;   /* Sensor encontrado en 0x68, salimos del bucle */
        }

        /* 0x68 no respondió: avisa y prueba la dirección alternativa 0x69 (AD0 en VCC) */
        ESP_LOGW(TAG, "Intento %d/%d: 0x68 no responde (%s), probando 0x69",
                 intento, REINTENTOS_DETECCION, esp_err_to_name(ret));
        ret = mpu6050_init(&sensor_mpu, bus_i2c, MPU6050_ADDR_ALT, I2C_MASTER_FREQ_HZ);
        if (ret == ESP_OK) {
            break;   /* Sensor encontrado en 0x69, salimos del bucle */
        }

        vTaskDelay(pdMS_TO_TICKS(200));   /* Espera 200 ms antes del siguiente intento */
    }

    /* Si tras todos los intentos no se detectó el sensor, detiene el sistema */
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se detectó MPU6050. Revisa VCC/GND/SDA21/SCL22/AD0");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));   /* Bucle infinito: el sistema no puede continuar */
        }
    }

    /* ------------------------------------------------
       CALIBRACION DEL SENSOR (sensor en reposo)
       ------------------------------------------------ */

    /* Toma 100 muestras con el sensor quieto y calcula offsets para los 6 ejes */
    ESP_LOGI(TAG, "Calibrando MPU6050 en reposo...");
    ESP_ERROR_CHECK(mpu6050_calibrate(&sensor_mpu, 100));

    /* ------------------------------------------------
       CREACION DEL MUTEX DE DATOS
       ------------------------------------------------ */

    /* Crea el mutex binario que protegerá datos_compartidos entre las dos tareas */
    mutex_datos = xSemaphoreCreateMutex();
    if (mutex_datos == NULL) {
        /* Sin mutex no se puede garantizar consistencia de datos: detenemos el sistema */
        ESP_LOGE(TAG, "No se pudo crear mutex de datos");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    /* ------------------------------------------------
       INICIALIZACION DE RED Y ESP-NOW
       ------------------------------------------------ */

    ESP_ERROR_CHECK(iniciar_wifi_para_espnow());   /* Arrancar WiFi STA y fijar canal 1   */
    ESP_ERROR_CHECK(iniciar_espnow());             /* Iniciar ESP-NOW y registrar maestro */

    /* ------------------------------------------------
       CONFIGURACION DE CONSUMO ENERGETICO
       ------------------------------------------------ */

    configurar_gestion_potencia();   /* 80 MHz + Light Sleep automático entre tareas */

    /* ------------------------------------------------
       CREACION DE TAREAS FREERTOS
       ------------------------------------------------ */

    /* Tarea de muestreo: lee sensor cada 100 ms, prioridad 5 (más alta de las dos) */
    xTaskCreate(
        tarea_muestreo_mpu,          /* Función de la tarea                        */
        "tarea_muestreo_mpu",        /* Nombre visible en depuración               */
        4096,                        /* Tamaño de pila en bytes (4 KB)             */
        NULL,                        /* Parámetro de entrada (no se usa)           */
        5,                           /* Prioridad: 5 (mayor = más urgente)         */
        NULL                         /* Handle de tarea (no se usa aquí)           */
    );

    /* Tarea de transmisión: envía datos por ESP-NOW cada 500 ms, prioridad 4 */
    xTaskCreate(
        tarea_transmision_espnow,    /* Función de la tarea                        */
        "tarea_transmision_espnow",  /* Nombre visible en depuración               */
        4096,                        /* Tamaño de pila en bytes (4 KB)             */
        NULL,                        /* Parámetro de entrada (no se usa)           */
        4,                           /* Prioridad: 4 (menor que muestreo)          */
        NULL                         /* Handle de tarea (no se usa aquí)           */
    );

    /* app_main retorna: FreeRTOS sigue ejecutando las tareas creadas arriba */
}
