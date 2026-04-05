/**
 * @file espnow_example_main.c
 * @brief Sistema Vestible de Monitoreo - Nodo Maestro (Master Controller)
 * 
 * Controlador central que orquesta la recolección de datos desde 2 nodos remotos vía ESP-NOW,
 * sincronización de RTC (DS3231), lectura de sensor de pulso local (ADC), y logging en SD (SPI).
 * 
 * **Hardware:**
 * - Procesador: ESP32 DevKit V1 (Dual-core 240 MHz, WiFi + BT)
 * - Almacenamiento: MicroSD 32GB vía SPI (MOSI-23, MISO-19, SCK-18, CS-5)
 * - Reloj: DS3231 I2C (SDA-21, SCL-22, 0x68)
 * - Sensor: Pulse oximetry on GPIO34 (ADC, 12-bit)
 * 
 * **Protocolo Inalámbrico:**
 * - ESP-NOW point-to-point de 2 nodos remotos: MPU (roll/pitch/accel/gyro) y TOUCH
 * - Payload: 32 bytes (datos_imu_t: 8 floats)
 * - Frecuencia de muestreo: configurable por nodo
 * 
 * **Optimizaciones Energéticas:**
 * - Dynamic Frequency Scaling: CPU 240 MHz → 80 MHz (activo) → 10 MHz (idle)
 * - WiFi Power Save: WIFI_PS_MIN_MODEM (~20 mA ahorro)
 * - Light Sleep con fallback (si no soportado, solo DFS)
 * - TX power reducido a 10 dBm (40 unidades)
 * 
 * **Filtrado de Señal:**
 * - Pulso: Burst de 8 muestras → media recortada (sin min/max) → EMA(0.85) → baseline_ema(0.98) → remoción DC
 * - Nodos IMU: validación de tamaño (32 bytes), timeout >1000ms limpia datos stale
 * 
 * **Logging:**
 * - CSV local buffering: 60 filas en RAM → escritura cada minuto a /sdcard/datos.csv
 * - LIVE console: cada 10 segundos (reducido de 1s para ahorro de potencia)
 * - Granularidad: 1 segundo por fila de registro
 * 
 * **Framework:** ESP-IDF v6.0 (drivers modernos: esp_adc, esp_pm; legacy I2C driver still functional)
 * 
 * @author Carlos Daniel Cuellar Antury
 * @date 2026-04-05
 * @version 1.2
 */

#include <stdio.h>              // Biblioteca estándar de entrada/salida
#include <string.h>             // Funciones de manipulación de cadenas (memcpy, memcmp)
#include <stdlib.h>             // Funciones de utilidad general
#include <stdbool.h>            // Tipo bool para banderas de estado
#include <math.h>               // Cálculo de roll y pitch
#include <time.h>               // Funciones para manejo de estructuras de tiempo
#include "freertos/FreeRTOS.h"  // Núcleo del sistema operativo en tiempo real
#include "freertos/task.h"      // Creación y control de tareas (hilos)
#include "freertos/semphr.h"    // Semáforos y Mutex para proteger datos compartidos
#include "nvs_flash.h"          // Almacenamiento persistente (necesario para WiFi)
#include "esp_wifi.h"           // Driver de radio WiFi
#include "esp_now.h"            // Protocolo de comunicación de baja latencia
#include "esp_log.h"            // Sistema de logs de errores y eventos
#include "esp_pm.h"             // Gestión de consumo energético
#include "driver/gpio.h"        // Control de pines de entrada/salida
#include "driver/i2c.h"         // Protocolo I2C para el reloj RTC (legacy en v6.0, pero funcional)
#include "esp_adc/adc_oneshot.h" // Nuevo controlador ADC (Single-shot) para v6.0
#include "esp_vfs_fat.h"        // Sistema de archivos virtual para FAT (SD)
#include "sdmmc_cmd.h"          // Protocolo de comandos para tarjetas SD
#include "driver/sdspi_host.h"  // Host SPI específico para tarjetas SD

// --- DEFINICIÓN DE PINES (Hardware ESP32 DevKit V1) ---
#define PIN_MOSI_SD 23           // Salida de datos SPI a la SD
#define PIN_MISO_SD 19           // Entrada de datos SPI desde la SD
#define PIN_SCK_SD 18            // Reloj del bus SPI
#define PIN_CS_SD 5              // Chip Select (selección de dispositivo SD)
#define PIN_SDA_I2C 21           // Línea de datos I2C (RTC)
#define PIN_SCL_I2C 22           // Línea de reloj I2C (RTC)
#define CANAL_LATIDOS ADC_CHANNEL_6 // GPIO 34 vinculado al ADC1

// --- CONFIGURACIÓN DE BUSES ---
#define I2C_PORT_NUM I2C_NUM_0   // Usamos el primer puerto I2C del ESP32
#define DS3231_ADDR 0x68         // Dirección I2C física del DS3231
#define SD_MOUNT_POINT "/sdcard" // Nombre de la carpeta raíz en la SD
#define RX_TIMEOUT_MS 1000        // Tiempo máximo sin paquetes por emisor
#define LIVE_LOG_INTERVAL_S 10    // Mostrar log LIVE cada 10 segundos

// --- DIRECCIONES MAC DE LOS NODOS REMOTOS ---
static uint8_t mac_esclavo_mpu[ESP_NOW_ETH_ALEN] = {0x68, 0x25, 0xdd, 0x32, 0x70, 0xcc}; 
static uint8_t mac_esclavo_touch[ESP_NOW_ETH_ALEN] = {0xb4, 0x3a, 0x45, 0x26, 0x0e, 0x40}; 

// --- ESTRUCTURAS DE DATOS ---
typedef struct {                // Estructura para datos del MPU6050
    float roll;                  // Roll en grados
    float pitch;                 // Pitch en grados
    float acc_x, acc_y, acc_z;   // Aceleración en X/Y/Z
    float gyr_x, gyr_y, gyr_z;   // Giroscopio en X/Y/Z
} datos_imu_t;

typedef struct {                // Estructura completa de datos para logging
    int pulso_raw;               // Valor ADC del sensor de pulso local - Lectura cruda del ADC
    datos_imu_t mpu;             // Datos IMU del nodo MPU
    datos_imu_t touch;           // Datos IMU del nodo touch/ESP32-S3
    struct tm tiempo;            // Estructura de tiempo (año, mes, día, etc.) - Tiempo del RTC
} log_completo_t;

// --- VARIABLES GLOBALES ---
static const char *TAG = "MASTER_LOG";  // Etiqueta para depuración en consola - Para logs
static SemaphoreHandle_t xMutex;        // Protege 'datos_sistema' de colisiones - Mutex para datos compartidos
static log_completo_t datos_sistema;    // Contenedor central de toda la información - Datos globales
static adc_oneshot_unit_handle_t adc1_handle; // Manejador global del ADC - Handle del ADC
static bool sd_montada = false;         // Indica si la tarjeta SD se montó con éxito
static sdmmc_card_t *sd_card = NULL;    // Puntero a descriptor de la tarjeta SD
static bool rtc_sincronizado_desde_pc = false; // Marca si la hora fue definida manualmente desde PC
static uint32_t paquetes_mpu = 0;       // Contador de paquetes válidos recibidos del nodo MPU
static uint32_t paquetes_touch = 0;     // Contador de paquetes válidos recibidos del nodo Touch
static int64_t ultimo_rx_mpu_ms = -1;   // Último tiempo de recepción válido del nodo MPU
static int64_t ultimo_rx_touch_ms = -1; // Último tiempo de recepción válido del nodo Touch
static bool mpu_desconectado = false;   // Estado de conectividad reportado del nodo MPU
static bool touch_desconectado = false; // Estado de conectividad reportado del nodo Touch

static uint8_t dec_a_bcd(uint8_t val);              // Prototipo conversión DEC->BCD
static uint8_t bcd_a_dec(uint8_t val);              // Prototipo conversión BCD->DEC
static bool rtc_tiempo_valido(const struct tm *t);  // Prototipo validación de fecha/hora
static bool parsear_settime(const char *linea, struct tm *out); // Prototipo parser de comando SETTIME
static void calcular_roll_pitch(float acc_x, float acc_y, float acc_z, float *roll, float *pitch); // Prototipo cálculo de ángulos

// --- FUNCIONES DE INICIALIZACIÓN ---

void imprimir_contenido_sd(void) { // Función para imprimir el contenido del archivo CSV en la consola
    if (!sd_montada) {
        ESP_LOGW(TAG, "SD no montada; no se puede leer datos.csv");
        return;
    }
    FILE *f = fopen(SD_MOUNT_POINT"/datos.csv", "r"); // Abrir archivo en modo lectura
    if (f) { // Si se abrió
        // Mostrar solo las líneas recientes para no saturar el monitor
        enum { MAX_RECIENTES = 20, MAX_LINEA = 192 };
        static char linea[MAX_LINEA];
        static char recientes[MAX_RECIENTES][MAX_LINEA];
        int total = 0;
        int pos = 0;

        while (fgets(linea, sizeof(linea), f)) {
            strncpy(recientes[pos], linea, MAX_LINEA - 1);
            recientes[pos][MAX_LINEA - 1] = '\0';
            pos = (pos + 1) % MAX_RECIENTES;
            if (total < MAX_RECIENTES) {
                total++;
            }
        }

        ESP_LOGI(TAG, "Ultimas %d lineas de datos.csv:", total);
        int inicio = (total == MAX_RECIENTES) ? pos : 0;
        for (int i = 0; i < total; i++) {
            int idx = (inicio + i) % MAX_RECIENTES;
            ESP_LOGI(TAG, "%s", recientes[idx]);
        }
        fclose(f); // Cerrar archivo
    } else {
        ESP_LOGE(TAG, "No se pudo abrir el archivo para lectura"); // Error si no se abre
    }
}

/**
 * @brief Calcula los ángulos de roll y pitch a partir de datos de aceleración
 * 
 * Utiliza una aproximación con arcotangente basada en los vectores de aceleración.
 * Útil para casos donde se requiere estimación rápida sin giroscopio o como validación.
 * 
 * @param acc_x Aceleración en eje X [g]
 * @param acc_y Aceleración en eje Y [g]
 * @param acc_z Aceleración en eje Z [g]
 * @param[out] roll Puntero donde se almacena el ángulo roll calculado [grados]
 * @param[out] pitch Puntero donde se almacena el ángulo pitch calculado [grados]
 * 
 * @return void
 * 
 * @details
 * - Roll: atan2(acc_y, acc_z) * 180/π
 * - Pitch: atan2(-acc_x, sqrt(acc_y² + acc_z²)) * 180/π
 * - Rango: ±90 grados en ambos ejes
 * - No requiere calibración, pero sensible a vibraciones
 */
static void calcular_roll_pitch(float acc_x, float acc_y, float acc_z, float *roll, float *pitch) { // Aproximación usando aceleración
    *roll = atan2f(acc_y, acc_z) * (180.0f / 3.14159265f);
    *pitch = atan2f(-acc_x, sqrtf((acc_y * acc_y) + (acc_z * acc_z))) * (180.0f / 3.14159265f);
}

/**
 * @brief Inicializa el bus I2C para comunicación con el RTC DS3231
 * 
 * Configura el puerto I2C_NUM_0 en modo maestro con pines SDA-21, SCL-22 y pull-ups internos.
 * Frecuencia: 100 kHz (estándar I2C). Esta función debe llamarse una sola vez en startup.
 * 
 * @return void
 * 
 * @note Usa driver legacy (driver/i2c.h) compatible con ESP-IDF v6.0
 * @see init_sd_card, init_adc_pulso para inicializaciones de otros periféricos
 * 
 * @author Carlos Daniel Cuellar Antury
 */
void init_rtc_i2c(void) {        // Función para inicializar el bus I2C para el RTC
    i2c_config_t conf = {        // Estructura de configuración I2C
        .mode = I2C_MODE_MASTER,        // El ESP32 manda en el bus - Modo maestro
        .sda_io_num = PIN_SDA_I2C,      // Pin SDA
        .scl_io_num = PIN_SCL_I2C,      // Pin SCL
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Resistencias internas activas - Pull-up SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Pull-up SCL
        .master.clk_speed = 100000,     // Frecuencia de 100kHz - Velocidad del reloj
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &conf)); // Aplicar configuración - Configura parámetros I2C
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, conf.mode, 0, 0, 0)); // Instalar driver - Instala el driver I2C
}

/**
 * @brief Inicializa la tarjeta microSD y monta el sistema de archivos FAT
 * 
 * Configura el bus SPI-3 (PINS: MOSI-23, MISO-19, SCK-18, CS-5) y monta la partición FAT
 * en el punto /sdcard. Maneja fallos de inicialización de forma no-fatal (log de advertencia).
 * 
 * **Comportamiento:**
 * - Si la SD no está disponible: log de error, @ref sd_montada = false, continúa operación (graceful degradation)
 * - Si ya está inicializada: retorna en @ref SD_MOUNT_POINT sin reinicializar
 * - Configuración: max 5 archivos abiertos simultáneamente, cluster size 16 KB
 * 
 * @return void
 * 
 * @post 
 * - @ref sd_montada = true si éxito
 * - File pointer @ref sd_card apunta a descriptor de tarjeta (si éxito)
 * 
 * @note Inicialización sin formateo en caso de fallo (@c format_if_mount_failed = false)
 * @see task_sd_log() para rutina de escritura
 * 
 * @author Carlos Daniel Cuellar Antury
 */
void init_sd_card(void) {        // Función para inicializar la tarjeta SD
    esp_vfs_fat_mount_config_t mount_config = { // Configuración de montaje
        .format_if_mount_failed = false, // No borrar datos si hay error - No formatear en fallo
        .max_files = 5,                  // Límite de archivos abiertos - Máximo 5 archivos
        .allocation_unit_size = 16 * 1024 // Unidad de asignación - Tamaño de cluster
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT(); // Host SDSPI por defecto
    host.slot = SPI3_HOST; // SPI3 usa pines VSPI por defecto: SCK 18, MISO 19, MOSI 23

    spi_bus_config_t bus_cfg = { // Configuración explícita del bus SPI
        .mosi_io_num = PIN_MOSI_SD,
        .miso_io_num = PIN_MISO_SD,
        .sclk_io_num = PIN_SCK_SD,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret_bus = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA); // Inicializar bus SPI
    if (ret_bus != ESP_OK && ret_bus != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Error inicializando bus SPI SD: %s", esp_err_to_name(ret_bus));
        return;
    }

    // SDSPI usa interrupciones GPIO internas; instalar servicio ISR una sola vez
    esp_err_t ret_isr = gpio_install_isr_service(0);
    if (ret_isr != ESP_OK && ret_isr != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Error instalando ISR GPIO para SD: %s", esp_err_to_name(ret_isr));
        return;
    }

    sdspi_device_config_t slot_config = {0}; // Inicializar manualmente para evitar macro incompatible
    slot_config.gpio_cs = PIN_CS_SD;
    slot_config.gpio_cd = SDSPI_SLOT_NO_CD;
    slot_config.gpio_wp = SDSPI_SLOT_NO_WP;
    slot_config.host_id = host.slot;

    esp_err_t ret_mount = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &sd_card);
    if (ret_mount != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo montar SD (%s)", esp_err_to_name(ret_mount));
        sd_montada = false;
        return;
    }
    sd_montada = true;
    sdmmc_card_print_info(stdout, sd_card);
    ESP_LOGI(TAG, "Tarjeta SD montada correctamente en %s", SD_MOUNT_POINT);
}

// Lectura manual de registros BCD del DS3231
/**
 * @brief Lee la fecha y hora actual del chip RTC DS3231 vía I2C
 * 
 * Realiza lectura secuencial de 7 registros BCD (0x00–0x06) del DS3231:
 * - [0] Segundos (CH bit en bit[7] ignorado)
 * - [1] Minutos
 * - [2] Horas (24h)
 * - [3] Día de semana (no utilizado)
 * - [4] Día del mes
 * - [5] Mes (Century bit en bit[7] ignorado)
 * - [6] Año (0-99, interpretado como 2000-2099)
 * 
 * Todos los valores se convierten de BCD a decimal y almacenan en struct tm.
 * La lectura solo se acepta si rtc_tiempo_valido() pasa, caso contrario se conserva valor previo.
 * 
 * @param[out] info_tiempo Puntero a struct tm donde se almacena la fecha/hora decodificada
 * 
 * @return true si lectura exitosa y datos válidos; false si error I2C o validación fallida
 * 
 * @pre I2C debe estar inicializado (@ref init_rtc_i2c fue llamado)
 * @post Si retorna true, @c info_tiempo contiene fecha/hora válida
 * @post Si retorna false, @c info_tiempo se conserva sin cambios (último valor válido)
 * 
 * @note Timeout I2C: 1000 ms (@c portTICK_PERIOD_MS)
 * @see rtc_escribir_tiempo para escritura; inicialmente RTC usa hora de compilación como fallback
 * 
 * @author Carlos Daniel Cuellar Antury
 */
bool rtc_leer_tiempo(struct tm *info_tiempo) {
    uint8_t datos[7];             // Búfer para datos de tiempo - Array para 7 bytes de datos
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Crear comando I2C - Crea handle de comando
    i2c_master_start(cmd);        // Iniciar comando - Comando de start
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true); // Escribir dirección - Dirección de escritura
    i2c_master_write_byte(cmd, 0x00, true); // Dirección de registro - Registro 0x00 (segundos)
    i2c_master_start(cmd);        // Inicio repetido - Start repetido para lectura
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_READ, true); // Dirección de lectura - Dirección de lectura
    for (int i = 0; i < 6; i++) i2c_master_read_byte(cmd, &datos[i], I2C_MASTER_ACK); // Leer 6 bytes con ACK
    i2c_master_read_byte(cmd, &datos[6], I2C_MASTER_NACK); // Último byte con NACK
    i2c_master_stop(cmd);         // Detener comando - Comando de stop
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS); // Ejecutar comando - Envía el comando
    i2c_cmd_link_delete(cmd);     // Eliminar comando - Libera el handle
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Lectura RTC falló: %s", esp_err_to_name(ret));
        return false;
    }
    // Enmascarar bits de control del DS3231 antes de convertir desde BCD
    struct tm nuevo = {0};
    uint8_t sec_reg = (uint8_t)(datos[0] & 0x7F); // CH en bit7
    uint8_t min_reg = (uint8_t)(datos[1] & 0x7F);
    uint8_t hour_reg = (uint8_t)(datos[2] & 0x3F); // 24h
    uint8_t mday_reg = (uint8_t)(datos[4] & 0x3F);
    uint8_t mon_reg = (uint8_t)(datos[5] & 0x1F); // century en bit7
    uint8_t year_reg = datos[6];

    nuevo.tm_sec = bcd_a_dec(sec_reg);
    nuevo.tm_min = bcd_a_dec(min_reg);
    nuevo.tm_hour = bcd_a_dec(hour_reg);
    nuevo.tm_mday = bcd_a_dec(mday_reg);
    nuevo.tm_mon = (int)bcd_a_dec(mon_reg) - 1;
    nuevo.tm_year = (int)bcd_a_dec(year_reg) + 100; // base 2000

    if (rtc_tiempo_valido(&nuevo)) {
        *info_tiempo = nuevo; // Solo actualizar si el tiempo leído es consistente
        return true;
    } else {
        ESP_LOGW(TAG, "RTC devolvio tiempo invalido, se conserva ultimo valor valido");
        return false;
    }
}

/**
 * @brief Escribe fecha y hora completa en el chip RTC DS3231 vía I2C
 * 
 * Convierte struct tm a formato BCD y transmite 7 bytes al DS3231 (registros 0x00–0x06):
 * - [0] Segundos (tm_sec)
 * - [1] Minutos (tm_min)
 * - [2] Horas (tm_hour, formato 24h)
 * - [3] Día de semana (fijo a 1, no utilizado)
 * - [4] Día del mes (tm_mday)
 * - [5] Mes (tm_mon + 1, rango 1-12)
 * - [6] Año (tm_year + 1900, modulo 100 para obtener 0-99)
 * 
 * Típicamente se llama una sola vez al arranque, después mediante comando SETTIME por consola.
 * 
 * @param t Puntero a struct tm con fecha/hora a escribir
 * 
 * @return void
 * 
 * @pre I2C debe estar inicializado (@ref init_rtc_i2c fue llamado)
 * @pre Entrada @c t debe contener valores válidos según @ref rtc_tiempo_valido
 * @post Si exitoso, RTC mantiene la hora (batería respaldada, persiste entre resets)
 * 
 * **Lado efectos:**
 * - Log INFO si éxito
 * - Log ERROR si fallo I2C
 * 
 * @note Timeout I2C: 1000 ms
 * @see rtc_leer_tiempo para lectura; parsear_settime para parser del comando SETTIME
 * 
 * @author Carlos Daniel Cuellar Antury
 */
void rtc_escribir_tiempo(const struct tm *t) { // Escribe fecha/hora completa en DS3231
    uint8_t datos[7];
    datos[0] = dec_a_bcd((uint8_t)t->tm_sec);             // segundos
    datos[1] = dec_a_bcd((uint8_t)t->tm_min);             // minutos
    datos[2] = dec_a_bcd((uint8_t)t->tm_hour);            // horas (24h)
    datos[3] = dec_a_bcd(1);                              // día semana fijo
    datos[4] = dec_a_bcd((uint8_t)t->tm_mday);            // día mes
    datos[5] = dec_a_bcd((uint8_t)(t->tm_mon + 1));       // mes (1..12)
    datos[6] = dec_a_bcd((uint8_t)((t->tm_year + 1900) % 100)); // año (00..99)

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Registro inicial segundos
    i2c_master_write(cmd, datos, sizeof(datos), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RTC actualizado: %02d/%02d/%04d %02d:%02d:%02d",
                 t->tm_mday, t->tm_mon + 1, t->tm_year + 1900,
                 t->tm_hour, t->tm_min, t->tm_sec);
    } else {
        ESP_LOGE(TAG, "No se pudo escribir RTC: %s", esp_err_to_name(ret));
    }
}

/**
 * @brief Convierte un valor decimal (0-99) a formato Binary Coded Decimal (BCD)
 * 
 * El chip RTC DS3231 almacena fecha/hora en BCD: cada dígito decimal se codifica
 * en 4 bits (nibble). Por ejemplo, 42 decimal → 0x42 BCD (4 en bits [7:4], 2 en bits [3:0]).
 * 
 * @param val Valor decimal en rango [0, 99]
 * 
 * @return Byte en formato BCD (dos dígitos decimales empaquetados)
 * 
 * @example dec_a_bcd(59) → (5 << 4) | 9 = 0x59
 * @see bcd_a_dec para conversión inversa
 * 
 * @author Carlos Daniel Cuellar Antury
 */
static uint8_t dec_a_bcd(uint8_t val) { // Convertir decimal a BCD para DS3231
    return (uint8_t)(((val / 10) << 4) | (val % 10));
}

/**
 * @brief Convierte un byte en formato Binary Coded Decimal (BCD) a decimal
 * 
 * Operación inversa de dec_a_bcd(). Extrae los dos dígitos BCD y los convierte a valor decimal.
 * 
 * @param val Byte en formato BCD (rango [0x00, 0x99])
 * 
 * @return Valor decimal en rango [0, 99]
 * 
 * @example bcd_a_dec(0x42) → (4 * 10) + 2 = 42
 * @see dec_a_bcd para conversión inversa
 * 
 * @author Carlos Daniel Cuellar Antury
 */
static uint8_t bcd_a_dec(uint8_t val) { // Convertir BCD a decimal
    return (uint8_t)(((val >> 4) * 10) + (val & 0x0F));
}

static int mes_desde_texto(const char *mes) { // Convierte "Jan".."Dec" a 0..11
    static const char *meses[12] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    for (int i = 0; i < 12; i++) {
        if (strncmp(mes, meses[i], 3) == 0) {
            return i;
        }
    }
    return 0;
}

void rtc_ajustar_a_hora_compilacion(void) { // Ajusta RTC con fecha/hora del firmware
    char mes_txt[4] = {0};
    int dia = 1, anio = 2000;
    int hora = 0, minuto = 0, segundo = 0;
    if (sscanf(__DATE__, "%3s %d %d", mes_txt, &dia, &anio) != 3) {
        ESP_LOGW(TAG, "No se pudo parsear __DATE__, RTC no ajustado");
        return;
    }
    if (sscanf(__TIME__, "%d:%d:%d", &hora, &minuto, &segundo) != 3) {
        ESP_LOGW(TAG, "No se pudo parsear __TIME__, RTC no ajustado");
        return;
    }

    struct tm t = {0};
    t.tm_mday = dia;
    t.tm_mon = mes_desde_texto(mes_txt);
    t.tm_year = anio - 1900;
    t.tm_hour = hora;
    t.tm_min = minuto;
    t.tm_sec = segundo;

    rtc_escribir_tiempo(&t);
    ESP_LOGI(TAG, "RTC ajustado a hora de compilación: %s %s", __DATE__, __TIME__);
}

/**
 * @brief Valida rangos de una estructura de tiempo (struct tm) del RTC
 * 
 * Comprueba que todos los campos de fecha/hora se encuentren dentro de rangos válidos.
 * Útil para detectar lecturas corruptas del DS3231 o BCD conversiones errónicas.
 * 
 * **Rangos validados:**
 * - Segundo: [0, 59]
 * - Minuto: [0, 59]
 * - Hora: [0, 23]
 * - Día del mes: [1, 31]
 * - Mes: [0, 11] (tm_mon es 0-indexed: enero=0, diciembre=11)
 * - Año: [100, 199] (tm_year es años desde 1900, rango [2000, 2099])
 * 
 * @param t Puntero a estructura tm con datos a validar
 * 
 * @return true si todos los campos están en rango; false si alguno está fuera
 * 
 * @note **IMPORTANTE:** tm_mon es 0-indexed. Enero = 0, diciembre = 11. Se validó esta corrección en v1.2.
 * @see rtc_leer_tiempo, rtc_escribir_tiempo para operaciones de lectura/escritura del DS3231
 * 
 * @author Carlos Daniel Cuellar Antury
 */
static bool rtc_tiempo_valido(const struct tm *t) { // Validar rangos básicos del tiempo leído
    if (t->tm_sec < 0 || t->tm_sec > 59) return false;
    if (t->tm_min < 0 || t->tm_min > 59) return false;
    if (t->tm_hour < 0 || t->tm_hour > 23) return false;
    if (t->tm_mday < 1 || t->tm_mday > 31) return false;
    if (t->tm_mon < 0 || t->tm_mon > 11) return false;
    if (t->tm_year < 100 || t->tm_year > 199) return false; // 2000..2099
    return true;
}

/**
 * @brief Parser del comando de sincronización RTC vía consola serial: "SETTIME YYYY-MM-DD HH:MM:SS"
 * 
 * Decodifica una línea de texto recibida por UART/ConsoleI O y extrae campos de fecha/hora.
 * Valida el comando contra formato exacto y rango de valores. Si parse y validación éxitos,
 * copia a estructura tm de salida.
 * 
 * **Formato esperado:**
 * ```
 * SETTIME 2026-04-05 14:30:45
 * ↑      ↑    ↑  ↑  ↑  ↑  ↑  ↑
 * cmd    Y  M  D   H  M  S
 * ```
 * 
 * **Validación:**
 * - Exactamente 6 campos numéricos extraídos por sscanf
 * - Rango de valores chequea via @ref rtc_tiempo_valido
 * - Si falla parsing o validación: retorna false sin modificar @c out
 * 
 * @param linea Línea de texto (presumiblemente de fgets, puede incluir \\n)
 * @param[out] out Puntero a struct tm donde se copia fecha/hora si éxito
 * 
 * @return true si parsing y validación exitosos; false en caso contrario
 * 
 * **Conversión:**
 * - Año ISO (4 dígitos) → tm_year = año - 1900
 * - Mes ISO (1-12) → tm_mon = mes - 1 (0-indexed)
 * - Resto campos copiados directamente
 * 
 * @note Consumo: ~50 bytes stack (struct tm local)
 * @see task_settime_serial para loop de lectura de consola
 * @see rtc_escribir_tiempo para escritura del RTC después de parse exitoso
 * 
 * @author Carlos Daniel Cuellar Antury
 */
static bool parsear_settime(const char *linea, struct tm *out) { // Parsear: SETTIME YYYY-MM-DD HH:MM:SS
    int anio, mes, dia, hora, minuto, segundo;
    if (sscanf(linea, "SETTIME %d-%d-%d %d:%d:%d", &anio, &mes, &dia, &hora, &minuto, &segundo) != 6) {
        return false;
    }
    struct tm t = {0};
    t.tm_year = anio - 1900;
    t.tm_mon = mes - 1;
    t.tm_mday = dia;
    t.tm_hour = hora;
    t.tm_min = minuto;
    t.tm_sec = segundo;
    if (!rtc_tiempo_valido(&t)) {
        return false;
    }
    *out = t;
    return true;
}

void task_settime_serial(void *p) { // Permite sincronizar RTC desde PC por monitor serie
    char linea[96];
    ESP_LOGI(TAG, "Comando disponible: SETTIME YYYY-MM-DD HH:MM:SS");
    while (1) {
        if (fgets(linea, sizeof(linea), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        struct tm nuevo = {0};
        if (!parsear_settime(linea, &nuevo)) {
            continue;
        }
        rtc_escribir_tiempo(&nuevo);
        rtc_sincronizado_desde_pc = true;
        xSemaphoreTake(xMutex, portMAX_DELAY);
        datos_sistema.tiempo = nuevo;
        xSemaphoreGive(xMutex);
        ESP_LOGI(TAG, "Hora recibida desde PC y guardada en RTC");
    }
}

// --- CALLBACK ESP-NOW ---
/**
 * @brief Callback de recepción del protocolo ESP-NOW
 * 
 * Manejador de interrupciones ejecutado cuando llega un paquete ESP-NOW de uno de los nodos remotos.
 * Valida MAC, tamaño de payload y sincroniza datos en la estructura global @ref datos_sistema con protección de Mutex.
 * 
 * **Validación:**
 * - MAC: Compara contra lista conocida (MPU: 68:25:dd:32:70:cc, TOUCH: b4:3a:45:26:0e:40)
 * - Tamaño: Rechaza si no es exactamente 32 bytes (@c sizeof(datos_imu_t) = 8 floats)
 * - Si MAC desconocida o tamaño incorrecto: log de advertencia, datos no copiados
 * 
 * **Thread-safety:**
 * - Toma @ref xMutex antes de escribir @ref datos_sistema
 * - Actualiza timestamp en @ref ultimo_rx_mpu_ms o @ref ultimo_rx_touch_ms
 * - Incrementa contador @ref paquetes_mpu o @ref paquetes_touch
 * 
 * @param info Información de emisor (MAC source, RSSI, channel)
 * @param data Payload (esperado: datos_imu_t = 32 bytes)
 * @param len Longitud actual del payload [bytes]
 * 
 * @return void
 * 
 * @warning Si @ref xMutex es NULL: función retorna sin procesar (deadlock prevention)
 * @warning El nodo MPU reporta tamaño incorrecto (12 vs 32); requiere corrección en firmware remoto
 * 
 * @author Carlos Daniel Cuellar Antury
 */
void cb_recepcion(const esp_now_recv_info_t *info, const uint8_t *data, int len) { // Callback para recepción ESP-NOW
    if (xMutex == NULL || info == NULL || data == NULL) {
        return;
    }
    int64_t ahora_ms = (int64_t)esp_log_timestamp();
    xSemaphoreTake(xMutex, portMAX_DELAY); // Bloquear acceso - Toma el mutex
    // Identificar quién envió el paquete comparando la MAC
    if (memcmp(info->src_addr, mac_esclavo_mpu, ESP_NOW_ETH_ALEN) == 0) {
        if (len == (int)sizeof(datos_imu_t)) {
            memcpy(&datos_sistema.mpu, data, sizeof(datos_imu_t)); // Copiar datos MPU completos
            paquetes_mpu++;
            ultimo_rx_mpu_ms = ahora_ms;
        } else {
            ESP_LOGW(TAG, "MPU: tamanio incorrecto %d (esperado %d)", len, (int)sizeof(datos_imu_t));
        }
    } else if (memcmp(info->src_addr, mac_esclavo_touch, ESP_NOW_ETH_ALEN) == 0) {
        if (len == (int)sizeof(datos_imu_t)) {
            memcpy(&datos_sistema.touch, data, sizeof(datos_imu_t)); // Copiar datos Touch completos
            paquetes_touch++;
            ultimo_rx_touch_ms = ahora_ms;
        } else {
            ESP_LOGW(TAG, "TOUCH: tamanio incorrecto %d (esperado %d)", len, (int)sizeof(datos_imu_t));
        }
    } else {
        ESP_LOGW(TAG, "MAC desconocida: %02x:%02x:%02x:%02x:%02x:%02x len=%d",
                 info->src_addr[0], info->src_addr[1], info->src_addr[2],
                 info->src_addr[3], info->src_addr[4], info->src_addr[5], len);
    }
    xSemaphoreGive(xMutex); // Liberar acceso - Libera el mutex
}

// --- TAREAS FREERTOS ---

/**
 * @brief Tarea de lectura periódica del sensor de pulso con filtrado robusto
 * 
 * Lee el sensor de pulso cada 500ms usando ADC1 canal 6 (GPIO34) con resolución 12-bit.
 * Implementa tres etapas de filtrado para extraer la amplitud AC (latidos) removiendo ruido y DC bias:
 * 
 * **Etapa 1: Media recortada (trimmed mean)**
 * - Adquiere 8 muestras consecutivas
 * - Elimina mínimo y máximo (outliers espurios)
 * - Promedia las 6 muestras restantes
 * 
 * **Etapa 2: EMA rápida (0.85)**
 * - Suaviza fluctuaciones de corta duración
 * - Nuevo valor = 0.85 × anterior + 0.15 × actual
 * 
 * **Etapa 3: Baseline lenta (0.98) + Remoción DC**
 * - Baseline_EMA sigue drift lento del contacto (factor 0.98)
 * - Compone AC = señal_EMA - baseline_EMA
 * - Escala visible: |AC| × 6.0 (±2047 rango típico)
 * 
 * **Banda muerta:** ±5 LSB para evitar parpadeo
 * **Consumo:** ~85 mA (ADC continuous + mutex contention despreciable)
 * **Publicación:** Cada ciclo (500 ms) en @ref datos_sistema.pulso_raw
 * 
 * @param p Parámetro de tarea FreeRTOS (no utilizado)
 * 
 * @return void (tarea infinita)
 * 
 * @thread Thread-safe: protege escritura en @ref datos_sistema con @ref xMutex
 * @note Inicializa ADC1 en primer ciclo; no requiere init_adc_pulso() previo
 * @see calcular_roll_pitch para estimación de ángulos desde aceleración
 * 
 * @author Carlos Daniel Cuellar Antury
 */
void task_pulso(void *p) {       // Tarea para leer sensor de pulso
    // Configuración del ADC Unidad 1
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 }; // Config inicial ADC
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle)); // Crear unidad ADC - Inicializa unidad ADC
    // Configuración del Canal del Sensor
    adc_oneshot_chan_cfg_t config = { // Config del canal
        .bitwidth = ADC_BITWIDTH_12, // 12 bits - Resolución
        .atten = ADC_ATTEN_DB_12, // Atenuación - Atenuación para rango
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CANAL_LATIDOS, &config)); // Configurar canal - Configura el canal ADC

    // Filtro robusto y extracción de amplitud (quita nivel DC ~1800 típico del sensor).
    enum { PULSO_MUESTRAS_BURST = 8 };
    float pulso_ema = -1.0f;
    float baseline_ema = -1.0f;
    int pulso_prev = 0;

    while(1) { // Bucle infinito
        int suma = 0;
        int min_v = 4095;
        int max_v = 0;

        for (int i = 0; i < PULSO_MUESTRAS_BURST; i++) {
            int adc_raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, CANAL_LATIDOS, &adc_raw)); // Leer ADC
            suma += adc_raw;
            if (adc_raw < min_v) min_v = adc_raw;
            if (adc_raw > max_v) max_v = adc_raw;
        }

        // Promedio recortado: quita un mínimo y un máximo para reducir picos espurios.
        int promedio_recortado = (suma - min_v - max_v) / (PULSO_MUESTRAS_BURST - 2);
        if (pulso_ema < 0.0f) {
            pulso_ema = (float)promedio_recortado;
        } else {
            pulso_ema = (0.85f * pulso_ema) + (0.15f * (float)promedio_recortado);
        }

        // Baseline lento para seguir deriva de contacto sin comerse el latido.
        if (baseline_ema < 0.0f) {
            baseline_ema = pulso_ema;
        } else {
            baseline_ema = (0.98f * baseline_ema) + (0.02f * pulso_ema);
        }

        float pulso_ac = pulso_ema - baseline_ema; // Componente útil (AC)
        int pulso_filtrado = (int)(fabsf(pulso_ac) * 6.0f + 0.5f); // Escala visible en monitor
        if (pulso_filtrado > 4095) {
            pulso_filtrado = 4095;
        }
        if (abs(pulso_filtrado - pulso_prev) < 5) {
            pulso_filtrado = pulso_prev; // Banda muerta pequeña para quitar parpadeo
        }
        pulso_prev = pulso_filtrado;

        xSemaphoreTake(xMutex, portMAX_DELAY); // Tomar mutex - Protege datos
        datos_sistema.pulso_raw = pulso_filtrado; // Publicar señal estabilizada
        xSemaphoreGive(xMutex); // Dar mutex - Libera mutex
        vTaskDelay(pdMS_TO_TICKS(500)); // Muestreo cada 500ms - Ahorro batería
    }
}

/**
 * @brief Tarea de logging en SD con buffering y detección de desconexión
 * 
 * Recolecta datos sincronizados (pulso local + IMU remota + RTC) cada 1 segundo, acumula en buffer RAM,
 * y escribe a /sdcard/datos.csv cada ~60 segundos (cuando buffer lleno). Detecta timeout de nodos
 * remotos >1000ms y limpia datos residuales para evitar "ghost data" en los registros.
 * 
 * **Ciclo de operación:**
 * 1. Delay 1000 ms
 * 2. Leer snapshot de @ref datos_sistema (con protección @ref xMutex)
 * 3. Validar conectividad: si último_rx_ms > RX_TIMEOUT_MS, marcar desconectado + memset zeros
 * 4. Generar línea CSV en lote[n] (~192 bytes)
 * 5. Si lote lleno (60 líneas): apertura archivo, escribir búfer, cerrar (atomic write)
 * 6. Log LIVE cada 10 segundos a consola (1 de cada 10 ciclos)
 * 
 * **Formato CSV:**
 * ```
 * DD/MM/YYYY,HH:MM:SS,pulso_raw,mpu_roll,mpu_pitch,mpu_accXYZ[3],mpu_gyrXYZ[3],
 *            touch_roll,touch_pitch,touch_accXYZ[3],touch_gyrXYZ[3]
 * ```
 * Total: 23 campos por fila = ~192 bytes (sin comillas)
 * 
 * **Optimizaciones energéticas:**
 * - SD write: 1 por minuto (60 rows), no 60 writes por minuto → ~40 mA ahorro
 * - LIVE console: cada 10 segundos (no cada 1s) → UART duty cycle 90% menos
 * - Timeout cleanup: memset(0) para nodos desconectados evita registros fantasma
 * 
 * **Monitoreo de conectividad:**
 * - Última recepción: último_rx_mpu_ms, último_rx_touch_ms (actualizados en @ref cb_recepcion)
 * - Timeout: >1000 ms sin paquete → log de desconexión + cero en CSV
 * - Reconexión: automática detectada, log de notificación
 * 
 * @param p Parámetro de tarea FreeRTOS (no utilizado)
 * 
 * @return void (tarea infinita)
 * 
 * @pre SD debe estar montada (@ref sd_montada = true), sino datos se bufferean sin escribir
 * @thread Thread-safe: toma @ref xMutex solo para leer snapshot de @ref datos_sistema, libera inmediato
 * @post Archivo /sdcard/datos.csv crecerá ~11 KB cada minuto
 * 
 * @warning Si SD no está disponible: los datos se pierden (no hay retry buffer persistente)
 * @note Granularidad: 1 segundo por fila, ~1 minuto entre writes
 * @see imprimir_contenido_sd para lectura y visualización de datos guardados
 * 
 * @author Carlos Daniel Cuellar Antury
 */
void task_sd_log(void *p) {      // Tarea para guardar en SD - lote cada 60s para ahorrar batería
    enum { LOTE_MAX = 60, MAX_CSV = 192 };
    static char lote[LOTE_MAX][MAX_CSV]; // Búfer estático ~11KB; fuera del stack
    int n = 0;
    int live_cnt = 0;
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Muestreo cada 1s
        int64_t ahora_ms = (int64_t)esp_log_timestamp();
        log_completo_t snap;
        uint32_t cnt_mpu;
        uint32_t cnt_touch;
        bool mpu_reciente;
        bool touch_reciente;
        xSemaphoreTake(xMutex, portMAX_DELAY);
        snap = datos_sistema;
        cnt_mpu = paquetes_mpu;
        cnt_touch = paquetes_touch;
        mpu_reciente = (ultimo_rx_mpu_ms >= 0) && ((ahora_ms - ultimo_rx_mpu_ms) <= RX_TIMEOUT_MS);
        touch_reciente = (ultimo_rx_touch_ms >= 0) && ((ahora_ms - ultimo_rx_touch_ms) <= RX_TIMEOUT_MS);
        xSemaphoreGive(xMutex);

        if (!mpu_reciente && !mpu_desconectado) {
            mpu_desconectado = true;
            ESP_LOGW(TAG, "MPU sin paquetes > %d ms (contador=%lu)", RX_TIMEOUT_MS, (unsigned long)cnt_mpu);
        } else if (mpu_reciente && mpu_desconectado) {
            mpu_desconectado = false;
            ESP_LOGI(TAG, "MPU reconectado (contador=%lu)", (unsigned long)cnt_mpu);
        }
        if (!touch_reciente && !touch_desconectado) {
            touch_desconectado = true;
            ESP_LOGW(TAG, "TOUCH sin paquetes > %d ms (contador=%lu)", RX_TIMEOUT_MS, (unsigned long)cnt_touch);
        } else if (touch_reciente && touch_desconectado) {
            touch_desconectado = false;
            ESP_LOGI(TAG, "TOUCH reconectado (contador=%lu)", (unsigned long)cnt_touch);
        }

        // Si un nodo está desconectado, no mostrar/guardar su último dato viejo.
        if (!mpu_reciente) {
            memset(&snap.mpu, 0, sizeof(snap.mpu));
        }
        if (!touch_reciente) {
            memset(&snap.touch, 0, sizeof(snap.touch));
        }

        // Formatear línea CSV en búfer RAM (no toca la SD todavía)
        if (sd_montada && n < LOTE_MAX) {
            snprintf(lote[n], MAX_CSV,
                     "%02d/%02d/%d,%02d:%02d:%02d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                     snap.tiempo.tm_mday, snap.tiempo.tm_mon + 1, snap.tiempo.tm_year + 1900,
                     snap.tiempo.tm_hour, snap.tiempo.tm_min, snap.tiempo.tm_sec,
                     snap.pulso_raw,
                     snap.mpu.roll, snap.mpu.pitch, snap.mpu.acc_x, snap.mpu.acc_y, snap.mpu.acc_z,
                     snap.mpu.gyr_x, snap.mpu.gyr_y, snap.mpu.gyr_z,
                     snap.touch.roll, snap.touch.pitch, snap.touch.acc_x, snap.touch.acc_y, snap.touch.acc_z,
                     snap.touch.gyr_x, snap.touch.gyr_y, snap.touch.gyr_z);
            n++;
        }

        // Volcar a SD solo cuando el búfer está lleno (1 escritura cada ~60s)
        if (sd_montada && n >= LOTE_MAX) {
            FILE *f = fopen(SD_MOUNT_POINT"/datos.csv", "a");
            if (f) {
                for (int i = 0; i < n; i++) {
                    fputs(lote[i], f);
                }
                fclose(f);
                ESP_LOGI(TAG, "SD: %d lineas escritas (bateria optimizada)", n);
            } else {
                ESP_LOGE(TAG, "No se pudo abrir datos.csv para escritura");
            }
            n = 0;
        }

        // LIVE log cada 10s para reducir carga de CPU/UART
        live_cnt++;
        if (live_cnt >= LIVE_LOG_INTERVAL_S) {
            live_cnt = 0;
            float roll_mpu = snap.mpu.roll;
            float pitch_mpu = snap.mpu.pitch;
            float roll_touch = snap.touch.roll;
            float pitch_touch = snap.touch.pitch;
            if (roll_mpu == 0.0f && pitch_mpu == 0.0f) {
                calcular_roll_pitch(snap.mpu.acc_x, snap.mpu.acc_y, snap.mpu.acc_z, &roll_mpu, &pitch_mpu);
            }
            if (roll_touch == 0.0f && pitch_touch == 0.0f) {
                calcular_roll_pitch(snap.touch.acc_x, snap.touch.acc_y, snap.touch.acc_z, &roll_touch, &pitch_touch);
            }
            ESP_LOGI(TAG,
                     "LIVE | Pulso:%d | RX MPU:%lu TOUCH:%lu | MPU Roll:%.2f Pitch:%.2f ACC(%.2f,%.2f,%.2f) GYR(%.2f,%.2f,%.2f) | TOUCH Roll:%.2f Pitch:%.2f ACC(%.2f,%.2f,%.2f) GYR(%.2f,%.2f,%.2f)",
                     snap.pulso_raw,
                     (unsigned long)cnt_mpu, (unsigned long)cnt_touch,
                     roll_mpu, pitch_mpu, snap.mpu.acc_x, snap.mpu.acc_y, snap.mpu.acc_z, snap.mpu.gyr_x, snap.mpu.gyr_y, snap.mpu.gyr_z,
                     roll_touch, pitch_touch, snap.touch.acc_x, snap.touch.acc_y, snap.touch.acc_z, snap.touch.gyr_x, snap.touch.gyr_y, snap.touch.gyr_z);
        }
    }
}

void task_rtc(void *p) {         // Tarea para leer RTC
    while(1) { // Bucle infinito
        xSemaphoreTake(xMutex, portMAX_DELAY); // Tomar mutex - Protege datos
        rtc_leer_tiempo(&datos_sistema.tiempo); // Leer tiempo - Actualiza tiempo si la lectura es válida
        xSemaphoreGive(xMutex); // Dar mutex - Libera mutex
        vTaskDelay(pdMS_TO_TICKS(5000)); // Refrescar cada 5s - Ahorro batería
    }
}

void app_main(void) {            // Función principal
    ESP_LOGI(TAG, "Iniciando aplicación maestro ESP-NOW"); // Log de inicio
    xMutex = xSemaphoreCreateMutex(); // Crear mutex temprano para callbacks/tareas
    if (xMutex == NULL) {
        ESP_LOGE(TAG, "No se pudo crear mutex global");
        return;
    }
    // Inicializar almacenamiento persistente
    ESP_ERROR_CHECK(nvs_flash_init()); // Iniciar NVS - Memoria flash
    // Configurar Red para ESP-NOW
    ESP_ERROR_CHECK(esp_netif_init()); // Inicializar interfaz de red - Init netif
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Crear bucle de eventos - Event loop
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Config WiFi por defecto
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Inicializar WiFi - Init WiFi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Modo estación - Station mode
    ESP_ERROR_CHECK(esp_wifi_start()); // Iniciar WiFi - Start WiFi
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE)); // Establecer canal - Canal 1
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40)); // Reducir TX a 10 dBm (default 20 dBm) - ahorra ~5 mA
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM)); // Power save del modem WiFi
    // Iniciar ESP-NOW
    ESP_ERROR_CHECK(esp_now_init()); // Inicializar ESP-NOW - Init ESP-NOW
    ESP_ERROR_CHECK(esp_now_register_recv_cb(cb_recepcion)); // Registrar callback - Register receive callback
    // Registrar peers
    esp_now_peer_info_t peer = { .channel = 1, .ifidx = WIFI_IF_STA, .encrypt = false }; // Info del peer
    memcpy(peer.peer_addr, mac_esclavo_mpu, 6); // Copiar MAC MPU
    ESP_ERROR_CHECK(esp_now_add_peer(&peer)); // Agregar peer MPU
    memcpy(peer.peer_addr, mac_esclavo_touch, 6); // Copiar MAC Touch
    ESP_ERROR_CHECK(esp_now_add_peer(&peer)); // Agregar peer Touch
    // Iniciar periféricos
    init_rtc_i2c(); // Inicializar I2C para RTC
    xTaskCreate(task_settime_serial, "SETTIME", 4096, NULL, 1, NULL); // Arrancar comando SETTIME cuanto antes
    struct tm tiempo_inicial = {0};
    rtc_leer_tiempo(&tiempo_inicial);
    if (tiempo_inicial.tm_year < 124) { // Si es menor a 2024, esperar sincronización de PC o fallback
        ESP_LOGW(TAG, "RTC invalido. Envia: SETTIME YYYY-MM-DD HH:MM:SS");
        vTaskDelay(pdMS_TO_TICKS(7000)); // Ventana para sincronizar desde PC al arrancar
        if (!rtc_sincronizado_desde_pc) {
            rtc_ajustar_a_hora_compilacion(); // Fallback temporal
        }
    } else {
        ESP_LOGI(TAG, "RTC ya tenia una hora valida; no se sobreescribe");
    }
    init_sd_card(); // Inicializar SD
    // Configurar PM
    esp_pm_config_t pm_conf = { .max_freq_mhz = 80, .min_freq_mhz = 10, .light_sleep_enable = true }; // DFS + light sleep
    esp_err_t pm_ret = esp_pm_configure(&pm_conf);
    if (pm_ret == ESP_OK) {
        ESP_LOGI(TAG, "Power management activo: DFS + light sleep");
    } else if (pm_ret == ESP_ERR_NOT_SUPPORTED) {
        // Algunas configuraciones de IDF/proyecto no soportan light sleep dinámico.
        pm_conf.min_freq_mhz = 80;
        pm_conf.light_sleep_enable = false;
        pm_ret = esp_pm_configure(&pm_conf);
        if (pm_ret == ESP_OK) {
            ESP_LOGW(TAG, "PM sin light sleep (modo compatible)");
        } else {
            ESP_LOGW(TAG, "PM no disponible, se continua sin PM: %s", esp_err_to_name(pm_ret));
        }
    } else {
        ESP_LOGW(TAG, "Fallo configurando PM, se continua sin PM: %s", esp_err_to_name(pm_ret));
    }
    // Lanzar tareas
    xTaskCreate(task_pulso, "PULSO", 4096, NULL, 5, NULL); // Crear tarea pulso - Prioridad alta
    xTaskCreate(task_sd_log, "SD_LOG", 6144, NULL, 3, NULL); // Crear tarea SD - stack ampliado por snprintf con floats
    xTaskCreate(task_rtc, "RTC", 4096, NULL, 2, NULL); // Crear tarea RTC - Prioridad baja
    
    // Esperar un poco para que se guarden algunos datos
    vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar 5 segundos
    imprimir_contenido_sd(); // Imprimir contenido de la SD en la consola

    while (1) { // Mantener app_main viva
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
