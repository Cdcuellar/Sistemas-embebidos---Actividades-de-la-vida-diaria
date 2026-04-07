/**
 * @file main.c
 * @brief Nodo esclavo #2 – sensor inercial MPU6050 con transmisión ESP-NOW.
 * @details Firmware del nodo esclavo que adquiere datos del sensor MPU6050
 *          (acelerómetro + giroscopio de 6 ejes) mediante I2C, calcula ángulos
 *          de inclinación (roll / pitch), clasifica estado de actividad
 *          (PARADO, ACOSTADO, CAMINANDO, CAIDA) y envía un paquete de 32 bytes
 *          cada 500 ms al nodo maestro usando el protocolo ESP-NOW sobre
 *          WiFi canal 1.
 *          Utiliza dos tareas FreeRTOS protegidas por mutex y gestión de energía
 *          (Light Sleep + CPU 80 MHz) para minimizar consumo.
 *
 *          Hardware:
 *          - ESP32 (ESP32-D0WD-V3)
 *          - MPU6050 conectado a SDA=GPIO21, SCL=GPIO22, AD0=GND
 *
 *          Protocolo:
 *          - ESP-NOW, canal 1, payload 32 bytes (8 floats), sin cifrado
 *
 * @author Carlos Daniel Cuellar Antury
 */

/* ============================================================
   BIBLIOTECAS
   ============================================================ */

#include <stdio.h>           /* Entrada/salida estándar (printf, etc.)        */
#include <math.h>            /* Funciones matemáticas (atan2, sqrt, etc.)     */
#include <string.h>          /* Manipulación de memoria (memcpy, memset, etc.)*/
#include <stdbool.h>         /* Tipo bool y valores true/false                */
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

/* Umbrales para clasificar postura/actividad (ajustables en campo) */
#define UMBRAL_G_IMPACTO_ALTO      1.55f  /* Pico de aceleración para impacto de caída   */
#define UMBRAL_G_IMPACTO_BAJO      0.55f  /* Casi ingravidez en caída libre breve        */
#define UMBRAL_MOV_DINAMICO        0.08f  /* Componente dinámica de |acc| para marcha    */
#define UMBRAL_GYRO_CAMINANDO      22.0f  /* Actividad angular mínima para caminar       */
#define UMBRAL_GYRO_QUIETO         10.0f  /* Actividad angular de reposo                 */
#define UMBRAL_ANGULO_PARADO       35.0f  /* Ángulo con referencia de pie para PARADO    */
#define UMBRAL_ANGULO_ACOSTADO     60.0f  /* Ángulo con referencia de pie para ACOSTADO  */
#define UMBRAL_GIRO_EXTREMO_CAIDA  180.0f /* Giro brusco compatible con caída            */

#define MUESTRAS_REF_POSTURA       18U    /* Muestras quietas para fijar referencia      */
#define MUESTRAS_CAMINANDO         3U     /* Confirmación temporal de CAMINANDO          */
#define MUESTRAS_REPOSO            4U     /* Confirmación temporal de postura quieta      */

#define VENTANA_CAIDA_MS         1500U    /* Tiempo para confirmar caída tras impacto    */
#define RETENCION_CAIDA_MS       4000U    /* Tiempo de retención del estado de caída     */

typedef enum {
    ESTADO_PARADO = 0,
    ESTADO_ACOSTADO,
    ESTADO_CAMINANDO,
    ESTADO_CAIDA
} estado_actividad_t;

/* ============================================================
   CONFIGURACION DE RED ESP-NOW
   ============================================================ */

#define CANAL_WIFI_ESPNOW        1        /* Canal WiFi fijo para la comunicación ESP-NOW */

/* MAC del nodo maestro que recibirá los datos (6 bytes en hexadecimal) */
static const uint8_t MAC_MAESTRO[ESP_NOW_ETH_ALEN] = {0xC8, 0x2E, 0x18, 0x67, 0x2F, 0xC4};

/* ============================================================
   ESTRUCTURA DE DATOS COMPARTIDA ENTRE TAREAS
   ============================================================ */

/**
 * @brief Paquete de telemetría enviado por ESP-NOW al maestro.
 * @details Estructura de 32 bytes exactos (8 floats × 4 bytes) que define
 *          el contrato de formato entre el nodo esclavo y el nodo maestro.
 *          El orden de los campos es significativo y no debe modificarse sin
 *          actualizar también el struct correspondiente en el maestro.
 */
typedef struct {
    float roll;    /**< Ángulo de inclinación lateral (rotación sobre eje X) en grados */
    float pitch;   /**< Ángulo de cabeceo adelante/atrás (rotación sobre eje Y) en grados */
    float acc_x;   /**< Aceleración en eje X en unidades g (rango ±2g) */
    float acc_y;   /**< Aceleración en eje Y en unidades g (rango ±2g) */
    float acc_z;   /**< Aceleración en eje Z en unidades g (rango ±2g) */
    float gyr_x;   /**< Velocidad angular en eje X en grados/segundo (rango ±250 °/s) */
    float gyr_y;   /**< Velocidad angular en eje Y en grados/segundo (rango ±250 °/s) */
    float gyr_z;   /**< Velocidad angular en eje Z en grados/segundo (rango ±250 °/s) */
} datos_mpu_t;

/* Verificación en tiempo de compilación: debe ser exactamente 32 bytes */
_Static_assert(sizeof(datos_mpu_t) == 32, "datos_mpu_t debe ser exactamente 32 bytes");

/* ============================================================
   VARIABLES GLOBALES
   ============================================================ */

/** @brief Buffer compartido entre tareas; la tarea de muestreo escribe y la de envío lee. */
static datos_mpu_t datos_compartidos = {0};

/** @brief Mutex binario que serializa el acceso a @ref datos_compartidos. */
static SemaphoreHandle_t mutex_datos = NULL;

/** @brief Instancia del driver MPU6050; almacena el handle I2C y los offsets de calibración. */
static mpu6050_t sensor_mpu;

/* ============================================================
   CLASIFICACION DE ACTIVIDAD (POSTURA / MARCHA / CAIDA)
   ============================================================ */

/**
 * @brief Limita un valor float entre dos extremos.
 */
static float limitar_float(float valor, float minimo, float maximo)
{
    if (valor < minimo) {
        return minimo;
    }
    if (valor > maximo) {
        return maximo;
    }
    return valor;
}

/**
 * @brief Convierte el enum de estado a texto legible para logs.
 */
static const char *texto_estado_actividad(estado_actividad_t estado)
{
    switch (estado) {
        case ESTADO_PARADO:
            return "PARADO";
        case ESTADO_ACOSTADO:
            return "ACOSTADO";
        case ESTADO_CAMINANDO:
            return "CAMINANDO";
        case ESTADO_CAIDA:
            return "CAIDA";
        default:
            return "PARADO";
    }
}

/**
 * @brief Clasificador heurístico para postura, caminata y caída.
 * @details Asume que el sensor está fijo al cuerpo (pecho/cintura) y combina:
 *          1) referencia de postura inicial en reposo (pie),
 *          2) aceleración dinámica + actividad angular para caminata,
 *          3) detección secuencial de caída (impacto/giro brusco + reposo),
 *          4) histéresis temporal para estabilidad de estado.
 *          Los umbrales están pensados para iniciar pruebas y pueden
 *          ajustarse con datos reales del usuario.
 */
static estado_actividad_t clasificar_actividad_mpu(float ax_g, float ay_g, float az_g,
                                                   float gx_dps, float gy_dps, float gz_dps)
{
    static bool impacto_detectado = false;
    static TickType_t tick_impacto = 0;
    static TickType_t tick_ultima_caida = 0;

    static bool referencia_lista = false;
    static uint32_t muestras_ref = 0;
    static float ref_x = 1.0f;
    static float ref_y = 0.0f;
    static float ref_z = 0.0f;
    static float suma_ref_x = 0.0f;
    static float suma_ref_y = 0.0f;
    static float suma_ref_z = 0.0f;

    static uint32_t contador_marcha = 0;
    static uint32_t contador_reposo = 0;

    static estado_actividad_t ultimo_estado_no_caida = ESTADO_PARADO;
    static float acc_mag_lp = 1.0f;

    const TickType_t ahora = xTaskGetTickCount();

    const float acc_mag = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
    const float gyro_mag = sqrtf((gx_dps * gx_dps) + (gy_dps * gy_dps) + (gz_dps * gz_dps));
    acc_mag_lp = (0.92f * acc_mag_lp) + (0.08f * acc_mag);
    const float acc_dinamica = fabsf(acc_mag - acc_mag_lp);

    const float inv_mag = 1.0f / fmaxf(acc_mag, 0.01f);
    const float ux = ax_g * inv_mag;
    const float uy = ay_g * inv_mag;
    const float uz = az_g * inv_mag;

    /* Auto-referencia: se fija en reposo al inicio y representa la postura de pie. */
    if (!referencia_lista) {
        if (gyro_mag < UMBRAL_GYRO_QUIETO && acc_dinamica < 0.12f) {
            suma_ref_x += ux;
            suma_ref_y += uy;
            suma_ref_z += uz;
            muestras_ref++;

            if (muestras_ref >= MUESTRAS_REF_POSTURA) {
                const float nx = suma_ref_x / (float)muestras_ref;
                const float ny = suma_ref_y / (float)muestras_ref;
                const float nz = suma_ref_z / (float)muestras_ref;
                const float norma = sqrtf((nx * nx) + (ny * ny) + (nz * nz));
                if (norma > 0.05f) {
                    ref_x = nx / norma;
                    ref_y = ny / norma;
                    ref_z = nz / norma;
                    referencia_lista = true;
                    ESP_LOGI(TAG, "Referencia de postura lista (pie): [%.2f %.2f %.2f]", ref_x, ref_y, ref_z);
                }
            }
        }

        return ultimo_estado_no_caida;
    }

    const float dot = limitar_float((ux * ref_x) + (uy * ref_y) + (uz * ref_z), -1.0f, 1.0f);
    const float angulo_ref_deg = acosf(fabsf(dot)) * (180.0f / (float)M_PI);

    if (acc_mag >= UMBRAL_G_IMPACTO_ALTO ||
        acc_mag <= UMBRAL_G_IMPACTO_BAJO ||
        gyro_mag >= UMBRAL_GIRO_EXTREMO_CAIDA) {
        impacto_detectado = true;
        tick_impacto = ahora;
    }

    if (impacto_detectado) {
        const uint32_t ms_desde_impacto = (uint32_t)pdTICKS_TO_MS(ahora - tick_impacto);

        if (ms_desde_impacto <= VENTANA_CAIDA_MS) {
            const bool quieto_postimpacto = (acc_mag > 0.75f && acc_mag < 1.25f && gyro_mag < 14.0f);
            const bool postura_colapso = (angulo_ref_deg > UMBRAL_ANGULO_ACOSTADO);
            if (quieto_postimpacto && postura_colapso) {
                tick_ultima_caida = ahora;
                impacto_detectado = false;
                return ESTADO_CAIDA;
            }
        } else {
            impacto_detectado = false;
        }
    }

    if ((uint32_t)pdTICKS_TO_MS(ahora - tick_ultima_caida) <= RETENCION_CAIDA_MS) {
        return ESTADO_CAIDA;
    }

    /* Marcha: requiere persistencia temporal para no marcar oscilaciones cortas. */
    /* Marcha real: requiere dinámica + giro, no solo sesgo de aceleración estática. */
    if (acc_dinamica > UMBRAL_MOV_DINAMICO && gyro_mag > UMBRAL_GYRO_CAMINANDO) {
        if (contador_marcha < (MUESTRAS_CAMINANDO + 2U)) {
            contador_marcha++;
        }
        contador_reposo = 0;
    } else {
        if (contador_marcha > 0U) {
            contador_marcha--;
        }
        if (contador_reposo < (MUESTRAS_REPOSO + 2U)) {
            contador_reposo++;
        }
    }

    if (contador_marcha >= MUESTRAS_CAMINANDO) {
        ultimo_estado_no_caida = ESTADO_CAMINANDO;
        return ESTADO_CAMINANDO;
    }

    if (contador_reposo >= MUESTRAS_REPOSO && gyro_mag < UMBRAL_GYRO_QUIETO) {
        if (angulo_ref_deg >= UMBRAL_ANGULO_ACOSTADO) {
            ultimo_estado_no_caida = ESTADO_ACOSTADO;
            return ESTADO_ACOSTADO;
        }
        if (angulo_ref_deg <= UMBRAL_ANGULO_PARADO) {
            ultimo_estado_no_caida = ESTADO_PARADO;
            return ESTADO_PARADO;
        }

        /* Zona intermedia: usa continuidad para evitar estados ambiguos. */
        return ultimo_estado_no_caida;
    }

    /* Si está en transición, conserva el último estado válido no-caída. */
    return ultimo_estado_no_caida;
}

/* ============================================================
   CALLBACK DE CONFIRMACION ESP-NOW
   ============================================================ */

/**
 * @brief Callback de confirmación de envío ESP-NOW.
 * @details El stack de ESP-NOW invoca esta función automáticamente tras cada
 *          intento de transmisión, tanto si el destinatario acusa recibo (ACK)
 *          como si la transmisión falla. Se registra con esp_now_register_send_cb().
 * @param info_envio Metadatos del envío (dirección MAC de destino). Puede ser NULL
 *                   si el sistema está en estado inválido; se maneja con guarda.
 * @param estado     Resultado: ESP_NOW_SEND_SUCCESS si el maestro recibió el paquete,
 *                   ESP_NOW_SEND_FAIL en caso contrario.
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

/**
 * @brief Inicializa el subsistema WiFi como base para ESP-NOW.
 * @details ESP-NOW requiere que el driver WiFi esté activo aunque no haya
 *          ninguna conexión a un router. Esta función:
 *          1. Inicializa (o borra y reinicia) la partición NVS.
 *          2. Arranca la capa de red y el bucle de eventos.
 *          3. Configura WiFi en modo estación (STA).
 *          4. Fija el canal WiFi al mismo que usa el nodo maestro.
 * @return ESP_OK si todo se inicializó correctamente.
 * @return Código de error ESP-IDF en caso de fallo en cualquier paso.
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

/**
 * @brief Inicializa el stack ESP-NOW y registra el nodo maestro como peer.
 * @details Debe llamarse después de @ref iniciar_wifi_para_espnow().
 *          Registra el callback de confirmación y añade la MAC del maestro
 *          como destino conocido. Si el peer ya existía de un arranque anterior
 *          lo elimina antes de re-añadirlo para evitar duplicados.
 * @return ESP_OK si ESP-NOW quedó listo para enviar.
 * @return Código de error ESP-IDF si alguno de los pasos falla.
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

/**
 * @brief Configura la gestión de energía del chip.
 * @details Fija la frecuencia del CPU a 80 MHz (suficiente para I2C a 100 kHz
 *          y ESP-NOW) y habilita Light Sleep automático. El chip entra en
 *          Light Sleep durante los vTaskDelay() de ambas tareas, reduciendo
 *          el consumo promedio sin afectar la temporización.
 *          Si el proyecto no tiene CONFIG_PM_ENABLE activo, la función avisa
 *          en el log pero no aborta la ejecución.
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

/**
 * @brief Tarea FreeRTOS de muestreo del sensor MPU6050.
 * @details Se ejecuta cada @ref PERIODO_MUESTREO_MS (100 ms). En cada ciclo:
 *          1. Lee los 6 ejes crudos por I2C con mpu6050_read_all_raw().
 *          2. Convierte aceleraciones a unidades g (÷16384) y velocidades
 *             angulares a °/s (÷131).
 *          3. Calcula roll y pitch mediante atan2f().
 *          4. Actualiza @ref datos_compartidos bajo @ref mutex_datos.
 *          5. Cada @ref LOG_CADA_MUESTRAS iteraciones imprime un resumen.
 *          Si la lectura I2C falla, registra el error y continúa (robustez).
 * @param param No utilizado (requerido por la firma de xTaskCreate). Pasa NULL.
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

            /* Cálculo de velocidades angulares en grados/segundo */
            /* Rango ±250°/s → sensibilidad 131 LSB/(°/s) */
            float gx_dps = gx / 131.0f;   /* Velocidad angular X en grados/segundo */
            float gy_dps = gy / 131.0f;   /* Velocidad angular Y en grados/segundo */
            float gz_dps = gz / 131.0f;   /* Velocidad angular Z en grados/segundo */

            estado_actividad_t estado_actual = clasificar_actividad_mpu(ax_g, ay_g, az_g,
                                                                         gx_dps, gy_dps, gz_dps);

            /* Toma el mutex: bloquea hasta 20 ms para no chocar con la tarea de envío */
            if (xSemaphoreTake(mutex_datos, pdMS_TO_TICKS(20)) == pdTRUE) {
                datos_compartidos.roll  = roll;      /* Ángulo Roll  */
                datos_compartidos.pitch = pitch;     /* Ángulo Pitch */
                datos_compartidos.acc_x = ax_g;      /* Aceleración X */
                datos_compartidos.acc_y = ay_g;      /* Aceleración Y */
                datos_compartidos.acc_z = az_g;      /* Aceleración Z */
                datos_compartidos.gyr_x = gx_dps;    /* Velocidad angular X */
                datos_compartidos.gyr_y = gy_dps;    /* Velocidad angular Y */
                datos_compartidos.gyr_z = gz_dps;    /* Velocidad angular Z */
                xSemaphoreGive(mutex_datos);         /* Libera el mutex para que otros accedan */
            }

            /* ------------------------------------------------
               LOG EN MONITOR SERIE (cada 5 muestras = 500 ms)
               ------------------------------------------------ */

            contador_logs++;
            if ((contador_logs % LOG_CADA_MUESTRAS) == 0) {
                ESP_LOGI(TAG, "Acc:%.3f,%.3f,%.3f g | Gyr:%.2f,%.2f,%.2f dps | Roll:%.1f Pitch:%.1f | Estado:%s",
                         ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, roll, pitch,
                         texto_estado_actividad(estado_actual));
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

/**
 * @brief Tarea FreeRTOS de transmisión ESP-NOW.
 * @details Se ejecuta cada @ref PERIODO_ENVIO_MS (500 ms). En cada ciclo:
 *          1. Copia @ref datos_compartidos a un buffer local bajo @ref mutex_datos
 *             (el mutex se libera antes del envío para no bloquear la tarea de muestreo).
 *          2. Llama a esp_now_send() con sizeof(datos_mpu_t) = 32 bytes.
 *          3. El resultado final del envío (ACK/NACK) se reporta en
 *             @ref callback_envio_espnow.
 *          Si esp_now_send() devuelve error local (buffer lleno, etc.) lo registra
 *          en el log pero continúa en el siguiente ciclo.
 * @param param No utilizado (requerido por la firma de xTaskCreate). Pasa NULL.
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

        /* Envía sizeof(datos_mpu_t) = 32 bytes al maestro; confirmación llega por callback */
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

/**
 * @brief Punto de entrada principal del firmware (equivalente a main()).
 * @details Se ejecuta una única vez al arrancar el chip. Realiza en orden:
 *          1. Inicialización del bus I2C (GPIO21/22, 100 kHz).
 *          2. Detección del MPU6050 (hasta @ref REINTENTOS_DETECCION intentos,
 *             prueba 0x68 y 0x69).
 *          3. Calibración del sensor con 100 muestras en reposo.
 *          4. Creación del mutex de protección de datos.
 *          5. Inicialización de WiFi + ESP-NOW con MAC del maestro.
 *          6. Configuración de gestión de energía.
 *          7. Creación y lanzamiento de las dos tareas FreeRTOS.
 *          Tras retornar, FreeRTOS toma el control y ejecuta las tareas
 *          de forma concurrente indefinidamente.
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
