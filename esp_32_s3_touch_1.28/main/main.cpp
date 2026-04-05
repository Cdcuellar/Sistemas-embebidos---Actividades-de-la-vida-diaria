/**
 * @file main.cpp
 * @brief Monitor Touch - Nodo Slave.
 * @details Captura datos del IMU QMI8658, los muestra en la pantalla GC9A01
 * y los envia por ESP-NOW al receptor cada 50 ms (20 Hz).
 * @author Carlos Daniel Cuellar Antury
 */

#include <stdio.h>       // Librería estándar de C para funciones como snprintf (formatear texto en cadena)
#include <string.h>      // Librería para copiar y comparar bloques de memoria (memcpy, memset, etc.)
#include <math.h>        // Librería matemática: atan2f, sqrtf, M_PI (arcotangente, raíz cuadrada, pi)

#include "freertos/FreeRTOS.h" // Sistema operativo en tiempo real: gestiona tareas, tiempos y sincronización
#include "freertos/task.h"     // Funciones para crear y manejar tareas paralelas (hilos de ejecución)
#include "freertos/semphr.h"   // Semáforos/mutex: evita que dos tareas accedan a los mismos datos a la vez

#include "driver/i2c_master.h"   // Driver del bus I2C: comunicación con el IMU y el sensor táctil
#include "driver/spi_master.h"   // Driver del bus SPI: comunicación con la pantalla GC9A01
#include "driver/gpio.h"         // Control de pines GPIO: encender/apagar, leer estado de pines físicos
#include "esp_adc/adc_oneshot.h" // Conversor analógico-digital: mide el voltaje de la batería
#include "esp_log.h"             // Sistema de logs: imprime mensajes en el monitor serie (consola)
#include "esp_pm.h"              // Gestión de energía: controla la frecuencia del CPU para ahorrar batería
#include "esp_timer.h"           // Temporizador de alta precisión del ESP32 (resolución en microsegundos)

#include "nvs_flash.h"  // Almacenamiento no volátil: necesario para que WiFi/ESP-NOW guarden su configuración
#include "esp_event.h"  // Sistema de eventos del ESP-IDF: bucle interno de notificaciones del sistema
#include "esp_netif.h"  // Interfaz de red: capa requerida aunque no usemos internet en este proyecto
#include "esp_wifi.h"   // Driver WiFi: ESP-NOW usa el radio WiFi internamente aunque no haya red
#include "esp_now.h"    // ESP-NOW: protocolo de comunicación directa entre dos ESP32 sin necesidad de router

// Etiquetas de texto que aparecen al inicio de cada línea del monitor serie para identificar el origen
static const char *TAG_IMU  = "IMU";    // Mensajes del sensor de movimiento (acelerómetro/giroscopio)
static const char *TAG_MAIN = "MAIN";   // Mensajes generales del programa principal
static const char *TAG_NOW  = "ESPNOW"; // Mensajes del sistema de radio ESP-NOW

// --- Pines del bus I2C (comunicación con IMU y táctil) ---
#define PIN_I2C_SDA         GPIO_NUM_6  // Pin de datos I2C (SDA = Serial DAta)
#define PIN_I2C_SCL         GPIO_NUM_7  // Pin de reloj I2C (SCL = Serial CLock)
#define VELOCIDAD_I2C_HZ    400000      // Velocidad del bus I2C: 400.000 ciclos por segundo (modo rápido)

// --- Pines del bus SPI (comunicación con la pantalla GC9A01) ---
#define PIN_SPI_MOSI        11          // Pin de datos SPI saliente (Master Out Slave In)
#define PIN_SPI_SCLK        10          // Pin de reloj SPI
#define PIN_SPI_MISO        12          // Pin de datos SPI entrante (no se usa en pantalla, pero se declara)
#define PIN_SPI_CS          9           // Pin de selección de chip (activa la pantalla para recibir datos)
#define PIN_PANTALLA_DC     GPIO_NUM_8  // Pin Data/Command: indica si lo que se envía es un comando o un píxel
#define PIN_PANTALLA_RST    GPIO_NUM_14 // Pin de reset de la pantalla (reinicia el controlador de pantalla)
#define PIN_PANTALLA_BL     GPIO_NUM_2  // Pin de retroiluminación (BackLight: encender/apagar la luz)
#define PIN_TOUCH_RST       GPIO_NUM_13 // Pin de reset del sensor táctil CST816S
#define PIN_TOUCH_INT       GPIO_NUM_5  // Pin de interrupción del táctil (se activa cuando el dedo toca)
#define VELOCIDAD_SPI_HZ    40000000    // Velocidad del bus SPI: 40 millones de ciclos por segundo
#define HOST_SPI            SPI2_HOST   // Controlador SPI número 2 del ESP32 (hay SPI1, SPI2 y SPI3)

// --- Configuración de pantalla y comportamiento visual ---
#define TIEMPO_APAGADO_MS   10000       // Tiempo sin tocar antes de apagar la pantalla: 10.000 ms = 10 segundos
#define ANCHO_PANTALLA      240         // Resolución horizontal en píxeles
#define ALTO_PANTALLA       240         // Resolución vertical en píxeles
#define COLOR_NEGRO         0x0000      // Color negro en formato RGB565 (16 bits por píxel)
#define COLOR_BLANCO        0xFFFF      // Color blanco en formato RGB565

// --- Configuración de la batería ---
#define CANAL_BATERIA_ADC   ADC_CHANNEL_0 // Canal 0 del ADC para leer el voltaje de la batería
#define FACTOR_BATERIA      3.0f          // El divisor de voltaje externo divide la tensión real entre 3

// --- Registros del IMU QMI8658 (sensor de movimiento inercial) ---
#define DIRECCION_QMI8658    0x6B  // Dirección I2C del QMI8658 (su "número de casa" en el bus)
#define QMI8658_REG_ID       0x00  // Registro de identificación: permite verificar que el chip es el correcto
#define QMI8658_REG_CTRL1    0x02  // Registro de control 1: configuración general del chip (interfaz, endian)
#define QMI8658_REG_CTRL2    0x03  // Registro de control 2: configura el acelerómetro (rango y velocidad)
#define QMI8658_REG_CTRL3    0x04  // Registro de control 3: configura el giroscopio (rango y velocidad)
#define QMI8658_REG_CTRL7    0x08  // Registro de control 7: habilita acelerómetro y giroscopio
#define QMI8658_REG_AX_L     0x35  // Registro donde empiezan los datos crudos (byte bajo de aceleración X)
#define QMI8658_ID_VAL       0x05  // Valor esperado del registro de ID para confirmar que es QMI8658
#define QMI8658_ESCALA_ACCEL 4096.0f // Divisor para convertir dato crudo a g (±8g → 1g = 4096 LSB)
#define QMI8658_ESCALA_GYRO   64.0f  // Divisor para convertir dato crudo a °/s (±512°/s → 1°/s = 64 LSB)

// --- Registros del sensor táctil CST816S ---
#define DIRECCION_CST816S   0x15  // Dirección I2C del CST816S
#define CST816S_REG_GESTID  0x01  // Registro de gesto: indica qué tipo de toque se detectó
#define CST816S_GESTO_TAP   0x05  // Código de gesto "tap" (toque simple con el dedo)

// --- Parámetros del filtro complementario (combina acelerómetro y giroscopio) ---
#define FACTOR_COMP_ACCEL   0.02f // Peso del acelerómetro: 2% (corrige la deriva lenta del giroscopio)
#define FACTOR_COMP_GYRO    0.98f // Peso del giroscopio: 98% (responde rápido a movimientos bruscos)
#define PERIODO_MUESTREO_S  0.05f // Período de muestreo: 0,05 s = 50 ms = 20 lecturas por segundo

// Dirección MAC del receptor: identifica de forma única al otro ESP32 que recibirá los datos
// Es como el número de teléfono al que se llama — cada ESP32 tiene una MAC única grabada de fábrica
static const uint8_t MAC_DESTINO[6] = {0xC8, 0x2E, 0x18, 0x67, 0x2F, 0xC4};

/**
 * @brief Lectura cruda del IMU para uso interno del emisor.
 * @note Esta estructura no se transmite por ESP-NOW.
 */
typedef struct {
    float accel_x; // Aceleración en el eje X en unidades "g" (1g = 9,8 m/s²)
    float accel_y; // Aceleración en el eje Y
    float accel_z; // Aceleración en el eje Z
    float gyro_x;  // Velocidad angular en el eje X en grados por segundo
    float gyro_y;  // Velocidad angular en el eje Y
    float gyro_z;  // Velocidad angular en el eje Z
} datos_imu_t;

/**
 * @brief Angulos estimados por el filtro complementario.
 */
typedef struct {
    float alabeo;  // Ángulo de inclinación lateral (roll): cuánto se inclina de lado
    float cabeceo; // Ángulo hacia adelante/atrás (pitch): cuánto se inclina hacia adelante
} angulos_t;

/**
 * @brief Paquete transmitido por ESP-NOW al receptor.
 * @details Se declara packed para evitar relleno y mantener 32 bytes exactos.
 */
typedef struct __attribute__((packed)) {
    float roll;  // Ángulo de alabeo (inclinación lateral) en grados
    float pitch; // Ángulo de cabeceo (inclinación frontal) en grados
    float acc_x; // Aceleración eje X en g
    float acc_y; // Aceleración eje Y en g
    float acc_z; // Aceleración eje Z en g
    float gyr_x; // Velocidad angular eje X en °/s
    float gyr_y; // Velocidad angular eje Y en °/s
    float gyr_z; // Velocidad angular eje Z en °/s
} paquete_espnow_t;
// Verificación en tiempo de compilación: si el tamaño no es 32 bytes, el compilador da error
// Esto previene que un cambio accidental rompa la compatibilidad con el receptor
static_assert(sizeof(paquete_espnow_t) == 32, "paquete_espnow_t debe ser 32 bytes");

// --- Variables globales compartidas entre las dos tareas ---
// "static" significa que solo son visibles dentro de este archivo
static datos_imu_t       g_datos_imu  = {}; // Última lectura del IMU (aceleración y giro)
static angulos_t         g_angulos    = {}; // Últimos ángulos calculados (roll y pitch)
static paquete_espnow_t  g_paquete    = {}; // Último paquete preparado para enviar
static float             g_bateria_v  = 0.0f; // Voltaje actual de la batería en voltios
// El mutex (mutual exclusion) es un "candado": antes de leer/escribir las variables de arriba,
// una tarea toma el candado, hace su trabajo, y lo suelta para que la otra pueda entrar
static SemaphoreHandle_t g_mutex_datos = NULL;

// --- Manejadores ("handles") de los periféricos ---
// Un handle es un identificador que el sistema asigna a un dispositivo al inicializarlo
static i2c_master_bus_handle_t   g_bus_i2c     = NULL; // Bus I2C principal
static i2c_master_dev_handle_t   g_dev_imu     = NULL; // Dispositivo IMU QMI8658 en el bus I2C
static i2c_master_dev_handle_t   g_dev_touch   = NULL; // Dispositivo táctil CST816S en el bus I2C
static spi_device_handle_t       g_handle_spi  = NULL; // Pantalla GC9A01 en el bus SPI
static adc_oneshot_unit_handle_t g_adc_bateria = NULL; // ADC para leer el voltaje de la batería
// "volatile": esta variable puede cambiar en cualquier momento desde la interrupción del táctil
static volatile bool g_touch_evento = false; // true cuando se detectó un toque en la pantalla

// --- Buffers de memoria para enviar datos a la pantalla por SPI ---
// __attribute__((aligned(4))): alineado en 4 bytes para mayor velocidad con DMA
static uint8_t s_linea_spi[ANCHO_PANTALLA * 2] __attribute__((aligned(4))); // Una fila de píxeles (240 px × 2 bytes)
static uint8_t s_char_buf[5 * 2 * 7 * 2 * 2] __attribute__((aligned(4))); // Un carácter 5×7 a escala 2

// --- Declaraciones anticipadas (prototipos de funciones) ---
// El compilador necesita saber que estas funciones existen antes de que se usen más abajo.
// Es como un índice del documento: se listan aquí y se implementan en detalle más adelante.
static esp_err_t i2c_escribir_registro(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t valor);              // Escribe un byte en un registro I2C
static esp_err_t i2c_leer_registros(i2c_master_dev_handle_t dev, uint8_t reg_inicio, uint8_t *buffer, size_t longitud); // Lee N bytes desde un registro I2C
static esp_err_t i2c_inicializar_bus_y_dispositivos(void);   // Inicializa el bus I2C y registra IMU y táctil
static esp_err_t qmi8658_inicializar(void);                  // Configura el sensor IMU QMI8658
static esp_err_t imu_leer_datos(datos_imu_t *datos);         // Lee aceleración y giro del IMU
static void filtro_complementario(const datos_imu_t *medicion, angulos_t *angulos_previos, float dt); // Calcula roll y pitch
static void pantalla_enviar_comando(uint8_t comando);        // Envía un byte de comando a la pantalla por SPI
static void pantalla_enviar_dato(uint8_t dato);              // Envía un byte de dato (píxel) a la pantalla
static void pantalla_fijar_ventana(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1); // Define área de dibujo
static void pantalla_rellenar_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color); // Rellena rectángulo
static void pantalla_dibujar_caracter(int16_t x, int16_t y, char c, uint16_t color_fg, uint16_t color_bg, uint8_t escala); // Dibuja un carácter
static void pantalla_dibujar_texto(int16_t x, int16_t y, const char *texto, uint16_t color_fg, uint16_t color_bg, uint8_t escala); // Dibuja texto
static void pantalla_inicializar(void);       // Inicializa la pantalla GC9A01 (secuencia de arranque)
static void touch_inicializar(void);          // Inicializa el táctil y configura la interrupción GPIO
static uint8_t touch_leer_gesto(void);        // Lee qué tipo de gesto se detectó (tap, swipe, etc.)
static float bateria_leer_voltios(void);      // Lee el voltaje de la batería desde el ADC
static void espnow_inicializar(void);         // Inicializa WiFi y ESP-NOW y registra el receptor
static void tarea_captura(void *parametro);   // Tarea: captura IMU y envía por radio cada 50 ms
static void tarea_interfaz(void *parametro);  // Tarea: actualiza pantalla y maneja el táctil

// --- Rutina de interrupción del táctil ---
// IRAM_ATTR: esta función se guarda en RAM rápida porque se ejecuta durante una interrupción
// Una interrupción es como una señal de emergencia: el CPU para lo que hace y atiende esto primero
static void IRAM_ATTR touch_isr_handler(void *arg)
{
    (void)arg;             // El argumento no se usa; el (void) evita una advertencia del compilador
    g_touch_evento = true; // Marca la bandera: hubo un toque. La tarea de interfaz la leerá en el próximo ciclo
}

static const uint8_t fuente_5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x41,0x51,0x32},
    {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x26,0x49,0x49,0x49,0x32},
    {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x08,0x14,0x54,0x54,0x3C},
    {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x08,0x08,0x2A,0x1C,0x08},
};

// --- Escribe un byte en un registro de un dispositivo I2C ---
// dev: dispositivo destino | reg: número de registro | valor: dato a escribir
static esp_err_t i2c_escribir_registro(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t valor)
{
    uint8_t buffer[2] = {reg, valor}; // Paquete de 2 bytes: primero el registro, luego el valor
    // Envía los 2 bytes al dispositivo; espera máximo 100 ms antes de dar error por timeout
    return i2c_master_transmit(dev, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
}

// --- Lee N bytes consecutivos desde un registro de un dispositivo I2C ---
// Primero envía el número de registro (1 byte), luego recibe "longitud" bytes de respuesta
static esp_err_t i2c_leer_registros(i2c_master_dev_handle_t dev, uint8_t reg_inicio, uint8_t *buffer, size_t longitud)
{
    // transmit_receive: envía la dirección del registro y recibe la respuesta en una sola operación
    return i2c_master_transmit_receive(dev, &reg_inicio, 1, buffer, longitud, pdMS_TO_TICKS(100));
}

// --- Inicializa el bus I2C y registra el IMU y el táctil como dispositivos ---
static esp_err_t i2c_inicializar_bus_y_dispositivos(void)
{
    i2c_master_bus_config_t config_bus = {};              // Estructura de configuración del bus I2C
    config_bus.i2c_port = I2C_NUM_0;                     // Usar el controlador I2C número 0 del ESP32
    config_bus.sda_io_num = PIN_I2C_SDA;                 // Pin físico de datos (GPIO6)
    config_bus.scl_io_num = PIN_I2C_SCL;                 // Pin físico de reloj (GPIO7)
    config_bus.clk_source = I2C_CLK_SRC_DEFAULT;         // Fuente de reloj automática (el ESP elige la mejor)
    config_bus.glitch_ignore_cnt = 7;                    // Ignora pulsos de ruido menores a 7 ciclos (filtro)
    config_bus.intr_priority = 0;                        // Prioridad de interrupción automática
    config_bus.trans_queue_depth = 0;                    // Sin cola (modo síncrono: espera respuesta)
    config_bus.flags.enable_internal_pullup = true;      // Activa las resistencias pull-up internas del ESP32
    config_bus.flags.allow_pd = false;                   // No permitir que el bus entre en modo bajo consumo

    esp_err_t resultado = i2c_new_master_bus(&config_bus, &g_bus_i2c); // Crea el bus y guarda el handle
    if (resultado != ESP_OK) {
        return resultado; // Si falló la creación del bus, devuelve el error
    }

    // --- Registrar el IMU QMI8658 como dispositivo en el bus ---
    i2c_device_config_t config_dev_imu = {};
    config_dev_imu.dev_addr_length = I2C_ADDR_BIT_LEN_7; // El QMI8658 usa dirección de 7 bits (estándar I2C)
    config_dev_imu.device_address = DIRECCION_QMI8658;   // Dirección 0x6B
    config_dev_imu.scl_speed_hz = VELOCIDAD_I2C_HZ;      // Velocidad: 400 kHz
    config_dev_imu.scl_wait_us = 0;                      // Sin espera adicional entre transacciones
    resultado = i2c_master_bus_add_device(g_bus_i2c, &config_dev_imu, &g_dev_imu); // Registra el IMU
    if (resultado != ESP_OK) {
        return resultado; // Sin IMU no hay datos: error fatal
    }

    // --- Registrar el táctil CST816S como dispositivo en el bus ---
    i2c_device_config_t config_dev_touch = {};
    config_dev_touch.dev_addr_length = I2C_ADDR_BIT_LEN_7; // Dirección de 7 bits
    config_dev_touch.device_address = DIRECCION_CST816S;    // Dirección 0x15
    config_dev_touch.scl_speed_hz = VELOCIDAD_I2C_HZ;       // 400 kHz
    config_dev_touch.scl_wait_us = 0;                       // Sin espera extra
    resultado = i2c_master_bus_add_device(g_bus_i2c, &config_dev_touch, &g_dev_touch);
    if (resultado != ESP_OK) {
        // El táctil es opcional: si no responde, solo se advierte y el programa continúa sin él
        ESP_LOGW(TAG_MAIN, "Touch no registrado: %s", esp_err_to_name(resultado));
        g_dev_touch = NULL; // Marcar como no disponible para que el código lo ignore
    }

    return ESP_OK; // Todo bien: bus e IMU inicializados
}

// --- Inicializa el sensor IMU QMI8658 ---
// Verifica que el chip sea auténtico y configura el rango y velocidad de medición
static esp_err_t qmi8658_inicializar(void)
{
    uint8_t id = 0; // Variable donde se guardará el ID leído del chip
    esp_err_t resultado = i2c_leer_registros(g_dev_imu, QMI8658_REG_ID, &id, 1); // Lee el registro de ID
    if (resultado != ESP_OK) {
        return resultado; // Si no se pudo leer (cable desconectado, etc.), devuelve error
    }
    // El QMI8658 puede devolver 0x05 o 0x06 según la revisión del chip; ambos son válidos
    if (id != QMI8658_ID_VAL && id != 0x06) {
        ESP_LOGE(TAG_IMU, "ID inesperado QMI8658: 0x%02X", id); // Imprime el ID incorrecto recibido
        return ESP_ERR_NOT_FOUND; // Hay algo conectado pero no es el chip esperado
    }

    // CTRL1 = 0x60: configura interfaz SPI/I2C en modo 4 cables, little-endian (byte bajo primero)
    resultado = i2c_escribir_registro(g_dev_imu, QMI8658_REG_CTRL1, 0x60);
    if (resultado != ESP_OK) return resultado;
    // CTRL2 = 0x23: activa el acelerómetro, rango ±8g, velocidad de muestreo 500 Hz con antialiasing
    resultado = i2c_escribir_registro(g_dev_imu, QMI8658_REG_CTRL2, 0x23);
    if (resultado != ESP_OK) return resultado;
    // CTRL3 = 0x43: activa el giroscopio, rango ±512°/s, velocidad de muestreo 500 Hz
    resultado = i2c_escribir_registro(g_dev_imu, QMI8658_REG_CTRL3, 0x43);
    if (resultado != ESP_OK) return resultado;
    // CTRL7 = 0x03: habilita acelerómetro (bit 0) y giroscopio (bit 1) simultáneamente
    resultado = i2c_escribir_registro(g_dev_imu, QMI8658_REG_CTRL7, 0x03);
    if (resultado != ESP_OK) return resultado;

    ESP_LOGI(TAG_IMU, "QMI8658 detectado e inicializado"); // Confirmación en el monitor serie
    return ESP_OK;
}

// --- Lee los datos de aceleración y giro del IMU y los convierte a unidades físicas ---
static esp_err_t imu_leer_datos(datos_imu_t *datos)
{
    uint8_t buffer[12] = {0}; // 12 bytes: 2 bytes × 3 ejes de accel + 2 bytes × 3 ejes de giro
    // Lee 12 bytes consecutivos a partir del registro donde empieza la aceleración X
    esp_err_t resultado = i2c_leer_registros(g_dev_imu, QMI8658_REG_AX_L, buffer, sizeof(buffer));
    if (resultado != ESP_OK) {
        return resultado; // Si falla la lectura I2C, devuelve error sin procesar nada
    }

    // El chip envía cada valor como 2 bytes en formato little-endian (byte bajo primero, byte alto segundo)
    // Se combinan los 2 bytes en un entero de 16 bits con signo usando desplazamiento de bits
    int16_t bruto_ax = (int16_t)((uint16_t)buffer[1] << 8 | buffer[0]); // Aceleración X cruda
    int16_t bruto_ay = (int16_t)((uint16_t)buffer[3] << 8 | buffer[2]); // Aceleración Y cruda
    int16_t bruto_az = (int16_t)((uint16_t)buffer[5] << 8 | buffer[4]); // Aceleración Z cruda
    int16_t bruto_gx = (int16_t)((uint16_t)buffer[7] << 8 | buffer[6]); // Giro X crudo
    int16_t bruto_gy = (int16_t)((uint16_t)buffer[9] << 8 | buffer[8]); // Giro Y crudo
    int16_t bruto_gz = (int16_t)((uint16_t)buffer[11] << 8 | buffer[10]); // Giro Z crudo

    // Divide por la escala para obtener unidades físicas
    // Escala accel 4096: si el valor crudo es 4096, equivale a 1g (9,8 m/s²)
    datos->accel_x = (float)bruto_ax / QMI8658_ESCALA_ACCEL;
    datos->accel_y = (float)bruto_ay / QMI8658_ESCALA_ACCEL;
    datos->accel_z = (float)bruto_az / QMI8658_ESCALA_ACCEL;
    // Escala gyro 64: si el valor crudo es 64, equivale a 1 grado por segundo
    datos->gyro_x = (float)bruto_gx / QMI8658_ESCALA_GYRO;
    datos->gyro_y = (float)bruto_gy / QMI8658_ESCALA_GYRO;
    datos->gyro_z = (float)bruto_gz / QMI8658_ESCALA_GYRO;
    return ESP_OK;
}

// --- Filtro complementario: calcula roll y pitch combinando acelerómetro y giroscopio ---
// El acelerómetro es preciso a largo plazo pero ruidoso. El giroscopio es preciso a corto pero deriva.
// El filtro mezcla ambos: 98% giroscopio (respuesta rápida) + 2% acelerómetro (corrección de deriva).
// dt = intervalo de tiempo desde la última llamada en segundos (0,05 s en este proyecto)
static void filtro_complementario(const datos_imu_t *medicion, angulos_t *angulos_previos, float dt)
{
    // Ángulo de alabeo calculado solo con el acelerómetro:
    // atan2f devuelve el ángulo en radianes; se multiplica por 180/π para convertir a grados
    float alabeo_accel = atan2f(medicion->accel_y, medicion->accel_z) * (180.0f / (float)M_PI);

    // Ángulo de cabeceo calculado solo con el acelerómetro:
    // sqrtf calcula la raíz cuadrada del módulo del vector YZ (hipotenusa de los ejes Y y Z)
    float cabeceo_accel = atan2f(-medicion->accel_x,
                                 sqrtf(medicion->accel_y * medicion->accel_y + medicion->accel_z * medicion->accel_z)) * (180.0f / (float)M_PI);

    // Ángulo de alabeo estimado por el giroscopio:
    // ángulo anterior + velocidad_angular × tiempo (integración numérica simple)
    float alabeo_gyro  = angulos_previos->alabeo  + medicion->gyro_x * dt;
    float cabeceo_gyro = angulos_previos->cabeceo + medicion->gyro_y * dt;

    // Mezcla final: 98% del giroscopio + 2% del acelerómetro
    angulos_previos->alabeo  = FACTOR_COMP_GYRO * alabeo_gyro  + FACTOR_COMP_ACCEL * alabeo_accel;
    angulos_previos->cabeceo = FACTOR_COMP_GYRO * cabeceo_gyro + FACTOR_COMP_ACCEL * cabeceo_accel;
}

// --- Envía un byte de COMANDO a la pantalla por SPI ---
// Un comando le dice a la pantalla QUÉ hacer (ej: "establece área de dibujo", "enciéndete")
static void pantalla_enviar_comando(uint8_t comando)
{
    gpio_set_level(PIN_PANTALLA_DC, 0); // DC=0 indica que lo que sigue es un COMANDO (no píxeles)
    spi_transaction_t t = {};           // Estructura que describe la transacción SPI
    t.length = 8;                       // Longitud: 8 bits = 1 byte
    t.tx_buffer = &comando;             // Apunta al byte a enviar
    spi_device_polling_transmit(g_handle_spi, &t); // Envía por SPI en modo bloqueante
}

// --- Envía un byte de DATO a la pantalla por SPI ---
// Un dato es el contenido: colores de píxeles o parámetros de configuración
static void pantalla_enviar_dato(uint8_t dato)
{
    gpio_set_level(PIN_PANTALLA_DC, 1); // DC=1 indica que lo que sigue son DATOS
    spi_transaction_t t = {};           // Estructura de transacción SPI
    t.length = 8;                       // 8 bits = 1 byte
    t.tx_buffer = &dato;                // Apunta al byte a enviar
    spi_device_polling_transmit(g_handle_spi, &t); // Envía por SPI
}

// --- Define el área rectangular donde se pintarán los siguientes píxeles ---
// (x0,y0) = esquina superior izquierda, (x1,y1) = esquina inferior derecha
static void pantalla_fijar_ventana(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    pantalla_enviar_comando(0x2A);              // Comando "Column Address Set" (define columnas X)
    pantalla_enviar_dato((uint8_t)(x0 >> 8));   // Byte alto de X inicio
    pantalla_enviar_dato((uint8_t)(x0 & 0xFF)); // Byte bajo de X inicio
    pantalla_enviar_dato((uint8_t)(x1 >> 8));   // Byte alto de X fin
    pantalla_enviar_dato((uint8_t)(x1 & 0xFF)); // Byte bajo de X fin

    pantalla_enviar_comando(0x2B);              // Comando "Row Address Set" (define filas Y)
    pantalla_enviar_dato((uint8_t)(y0 >> 8));   // Byte alto de Y inicio
    pantalla_enviar_dato((uint8_t)(y0 & 0xFF)); // Byte bajo de Y inicio
    pantalla_enviar_dato((uint8_t)(y1 >> 8));   // Byte alto de Y fin
    pantalla_enviar_dato((uint8_t)(y1 & 0xFF)); // Byte bajo de Y fin

    pantalla_enviar_comando(0x2C); // Comando "Memory Write": a partir de aquí se esperan bytes de color
}

// --- Rellena un rectángulo de la pantalla con un color sólido ---
// Más eficiente que dibujar píxel a píxel: prepara una fila completa y la repite por cada fila del rectángulo
static void pantalla_rellenar_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    uint16_t ancho = x1 - x0 + 1;           // Número de columnas del rectángulo
    uint16_t alto  = y1 - y0 + 1;           // Número de filas del rectángulo
    uint8_t byte_alto = (uint8_t)(color >> 8);   // El color RGB565 son 2 bytes; byte más significativo
    uint8_t byte_bajo = (uint8_t)(color & 0xFF); // Byte menos significativo del color

    // Precalcula un buffer con los colores de una fila entera (todos del mismo color)
    for (uint16_t i = 0; i < ancho; i++) {
        s_linea_spi[i * 2]     = byte_alto; // Primer byte del píxel i
        s_linea_spi[i * 2 + 1] = byte_bajo; // Segundo byte del píxel i
    }

    pantalla_fijar_ventana(x0, y0, x1, y1); // Define el área del rectángulo en la pantalla
    gpio_set_level(PIN_PANTALLA_DC, 1);      // DC=1: lo que viene son datos de color (no comandos)

    spi_transaction_t t = {};               // Transacción SPI para enviar una fila completa
    t.length = (size_t)ancho * 16;          // Longitud en bits: ancho_px × 2 bytes × 8 bits = ancho × 16
    t.tx_buffer = s_linea_spi;              // Datos a enviar: la fila precalculada
    // Envía la misma fila de color tantas veces como filas tenga el rectángulo
    for (uint16_t fila = 0; fila < alto; fila++) {
        spi_device_polling_transmit(g_handle_spi, &t); // Envía una fila de píxeles por SPI
    }
}

// --- Dibuja un carácter ASCII en pantalla en la posición (x, y) ---
// color_fg: color del trazo | color_bg: color del fondo | escala: tamaño (1=normal, 2=doble)
static void pantalla_dibujar_caracter(int16_t x, int16_t y, char c, uint16_t color_fg, uint16_t color_bg, uint8_t escala)
{
    // Si el carácter está fuera del rango imprimible (ASCII 32–126), lo reemplaza con '?'
    if (c < 32 || c > 126) {
        c = '?';
    }
    // Obtiene el bitmap del carácter desde la tabla de fuente
    // La tabla empieza en espacio (ASCII 32), por eso se resta 32 al valor ASCII
    const uint8_t *columnas = fuente_5x7[(uint8_t)(c - 32)];
    uint16_t ancho = (uint16_t)(5 * escala); // Ancho en píxeles: 5 columnas × escala
    uint16_t alto  = (uint16_t)(7 * escala); // Alto en píxeles: 7 filas × escala
    uint16_t indice = 0;                     // Índice que recorre el buffer s_char_buf

    // Recorre cada fila de píxeles del carácter escalado
    for (uint8_t py = 0; py < 7 * escala; py++) {
        uint8_t fila = py / escala; // Fila real del bitmap (0-6): divide para aplicar el escalado
        // Recorre cada columna de píxeles del carácter escalado
        for (uint8_t px = 0; px < 5 * escala; px++) {
            uint8_t col = px / escala; // Columna real del bitmap (0-4)
            // Si el bit de la fuente está activo → usa color de trazo; si no → color de fondo
            uint16_t color = (columnas[col] & (1u << fila)) ? color_fg : color_bg;
            s_char_buf[indice++] = (uint8_t)(color >> 8);   // Byte alto del color del píxel
            s_char_buf[indice++] = (uint8_t)(color & 0xFF); // Byte bajo del color del píxel
        }
    }

    // Solo dibuja si el carácter cabe completamente dentro de los límites de la pantalla
    if (x >= 0 && y >= 0 && (uint16_t)x + ancho <= ANCHO_PANTALLA && (uint16_t)y + alto <= ALTO_PANTALLA) {
        pantalla_fijar_ventana((uint16_t)x, (uint16_t)y, (uint16_t)(x + ancho - 1), (uint16_t)(y + alto - 1));
        gpio_set_level(PIN_PANTALLA_DC, 1); // DC=1: modo datos (píxeles de color)
        spi_transaction_t t = {};
        t.length = (size_t)ancho * alto * 16; // Total de bits: ancho × alto × 16 bits/píxel
        t.tx_buffer = s_char_buf;             // Envía el bitmap completo del carácter
        spi_device_polling_transmit(g_handle_spi, &t);
    }
}

// --- Dibuja una cadena de texto en pantalla, carácter a carácter ---
// Avanza el cursor 6 píxeles × escala hacia la derecha por cada carácter (5 de glifo + 1 de espacio)
static void pantalla_dibujar_texto(int16_t x, int16_t y, const char *texto, uint16_t color_fg, uint16_t color_bg, uint8_t escala)
{
    int16_t cursor_x = x;  // Posición horizontal actual, empieza en x
    while (*texto) {       // Recorre la cadena hasta encontrar el carácter nulo '\0' (fin de texto)
        pantalla_dibujar_caracter(cursor_x, y, *texto, color_fg, color_bg, escala); // Dibuja el carácter actual
        cursor_x += (int16_t)(6 * escala); // Avanza el cursor al siguiente carácter
        texto++;  // Avanza al siguiente carácter de la cadena
    }
}

// --- Inicializa la pantalla GC9A01 ---
// Configura los pines, el bus SPI y envía la secuencia de comandos de arranque del fabricante
static void pantalla_inicializar(void)
{
    // Configura los pines de control de la pantalla como salidas digitales
    gpio_config_t cfg_gpio = {};
    cfg_gpio.pin_bit_mask = (1ULL << PIN_PANTALLA_DC) | (1ULL << PIN_PANTALLA_RST) | (1ULL << PIN_PANTALLA_BL);
    cfg_gpio.mode = GPIO_MODE_OUTPUT; // Los tres pines se configuran como salida
    gpio_config(&cfg_gpio);           // Aplica la configuración

    // Configura el bus SPI donde está conectada la pantalla
    spi_bus_config_t cfg_bus = {};
    cfg_bus.mosi_io_num = PIN_SPI_MOSI;   // Pin de datos salientes (envía píxeles a la pantalla)
    cfg_bus.miso_io_num = PIN_SPI_MISO;   // Pin de datos entrantes (no se usa con la pantalla)
    cfg_bus.sclk_io_num = PIN_SPI_SCLK;   // Pin de reloj SPI
    cfg_bus.quadwp_io_num = -1;           // -1 = no se usa (es para modo SPI de 4 bits)
    cfg_bus.quadhd_io_num = -1;           // -1 = no se usa
    cfg_bus.data4_io_num = -1;            // -1 = no se usa (modo SPI octal)
    cfg_bus.data5_io_num = -1;
    cfg_bus.data6_io_num = -1;
    cfg_bus.data7_io_num = -1;
    cfg_bus.max_transfer_sz = ANCHO_PANTALLA * ALTO_PANTALLA * 2; // Máximo: imagen completa (240×240×2 bytes)
    cfg_bus.data_io_default_level = false; // Nivel por defecto de los pines de datos: bajo
    cfg_bus.flags = 0;                     // Sin flags especiales
    cfg_bus.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO; // CPU para la interrupción: automático
    cfg_bus.intr_flags = 0;                // Sin flags de interrupción especiales
    spi_bus_initialize(HOST_SPI, &cfg_bus, SPI_DMA_CH_AUTO); // Inicializa el bus SPI con DMA automático

    // Configura la pantalla GC9A01 como dispositivo en el bus SPI
    spi_device_interface_config_t cfg_dev = {};
    cfg_dev.command_bits  = 0;                    // Sin bits de comando separados (se usan comandos estándar)
    cfg_dev.address_bits  = 0;                    // Sin bits de dirección separados
    cfg_dev.dummy_bits    = 0;                    // Sin bits de relleno entre comando y datos
    cfg_dev.mode          = 0;                    // Modo SPI 0 (CPOL=0, CPHA=0): reloj bajo en reposo
    cfg_dev.clock_source  = SPI_CLK_SRC_DEFAULT;  // Fuente de reloj automática
    cfg_dev.clock_speed_hz = VELOCIDAD_SPI_HZ;    // Velocidad: 40 MHz
    cfg_dev.duty_cycle_pos = 128;                 // Ciclo de trabajo del reloj: 50% (128/256)
    cfg_dev.cs_ena_pretrans  = 0;                 // Tiempo de CS antes de la transacción: 0
    cfg_dev.cs_ena_posttrans = 0;                 // Tiempo de CS después de la transacción: 0
    cfg_dev.input_delay_ns   = 0;                 // Retardo de entrada de datos: 0 ns
    cfg_dev.spics_io_num     = PIN_SPI_CS;        // Pin chip select de la pantalla
    cfg_dev.flags      = 0;                       // Sin flags especiales
    cfg_dev.queue_size = 7;                       // Cola de 7 transacciones en paralelo (para DMA)
    cfg_dev.pre_cb  = NULL;                       // Sin callback antes de la transacción
    cfg_dev.post_cb = NULL;                       // Sin callback después de la transacción
    spi_bus_add_device(HOST_SPI, &cfg_dev, &g_handle_spi); // Registra la pantalla en el bus y guarda el handle

    // Secuencia de encendido: apagar retroiluminación, resetear controlador, esperar
    gpio_set_level(PIN_PANTALLA_BL, 0);    // Apaga la retroiluminación durante la inicialización
    gpio_set_level(PIN_PANTALLA_RST, 0);   // Activa el reset (pone el controlador en estado inicial)
    vTaskDelay(pdMS_TO_TICKS(10));         // Espera 10 ms con el reset activo
    gpio_set_level(PIN_PANTALLA_RST, 1);   // Desactiva el reset (el controlador empieza a arrancar)
    vTaskDelay(pdMS_TO_TICKS(120));        // Espera 120 ms para que el firmware interno del GC9A01 arranque

    pantalla_enviar_comando(0xEF);
    pantalla_enviar_comando(0xEB); pantalla_enviar_dato(0x14);
    pantalla_enviar_comando(0xFE);
    pantalla_enviar_comando(0xEF);
    pantalla_enviar_comando(0xEB); pantalla_enviar_dato(0x14);
    pantalla_enviar_comando(0x84); pantalla_enviar_dato(0x40);
    pantalla_enviar_comando(0x85); pantalla_enviar_dato(0xFF);
    pantalla_enviar_comando(0x86); pantalla_enviar_dato(0xFF);
    pantalla_enviar_comando(0x87); pantalla_enviar_dato(0xFF);
    pantalla_enviar_comando(0x88); pantalla_enviar_dato(0x0A);
    pantalla_enviar_comando(0x89); pantalla_enviar_dato(0x21);
    pantalla_enviar_comando(0x8A); pantalla_enviar_dato(0x00);
    pantalla_enviar_comando(0x8B); pantalla_enviar_dato(0x80);
    pantalla_enviar_comando(0x8C); pantalla_enviar_dato(0x01);
    pantalla_enviar_comando(0x8D); pantalla_enviar_dato(0x01);
    pantalla_enviar_comando(0x8E); pantalla_enviar_dato(0xFF);
    pantalla_enviar_comando(0x8F); pantalla_enviar_dato(0xFF);
    pantalla_enviar_comando(0xB6); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00);
    pantalla_enviar_comando(0x36); pantalla_enviar_dato(0xD8);
    pantalla_enviar_comando(0x3A); pantalla_enviar_dato(0x05);
    pantalla_enviar_comando(0x90);
    pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x08);
    pantalla_enviar_comando(0xBD); pantalla_enviar_dato(0x06);
    pantalla_enviar_comando(0xBC); pantalla_enviar_dato(0x00);
    pantalla_enviar_comando(0xFF); pantalla_enviar_dato(0x60); pantalla_enviar_dato(0x01); pantalla_enviar_dato(0x04);
    pantalla_enviar_comando(0xC3); pantalla_enviar_dato(0x13);
    pantalla_enviar_comando(0xC4); pantalla_enviar_dato(0x13);
    pantalla_enviar_comando(0xC9); pantalla_enviar_dato(0x22);
    pantalla_enviar_comando(0xBE); pantalla_enviar_dato(0x11);
    pantalla_enviar_comando(0xE1); pantalla_enviar_dato(0x10); pantalla_enviar_dato(0x0E);
    pantalla_enviar_comando(0xDF); pantalla_enviar_dato(0x21); pantalla_enviar_dato(0x0C); pantalla_enviar_dato(0x02);
    pantalla_enviar_comando(0xF0); pantalla_enviar_dato(0x45); pantalla_enviar_dato(0x09); pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x26); pantalla_enviar_dato(0x2A);
    pantalla_enviar_comando(0xF1); pantalla_enviar_dato(0x43); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x72); pantalla_enviar_dato(0x36); pantalla_enviar_dato(0x37); pantalla_enviar_dato(0x6F);
    pantalla_enviar_comando(0xF2); pantalla_enviar_dato(0x45); pantalla_enviar_dato(0x09); pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x26); pantalla_enviar_dato(0x2A);
    pantalla_enviar_comando(0xF3); pantalla_enviar_dato(0x43); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x72); pantalla_enviar_dato(0x36); pantalla_enviar_dato(0x37); pantalla_enviar_dato(0x6F);
    pantalla_enviar_comando(0xED); pantalla_enviar_dato(0x1B); pantalla_enviar_dato(0x0B);
    pantalla_enviar_comando(0xAE); pantalla_enviar_dato(0x77);
    pantalla_enviar_comando(0xCD); pantalla_enviar_dato(0x63);
    pantalla_enviar_comando(0x70); pantalla_enviar_dato(0x07); pantalla_enviar_dato(0x07); pantalla_enviar_dato(0x04); pantalla_enviar_dato(0x0E); pantalla_enviar_dato(0x0F); pantalla_enviar_dato(0x09); pantalla_enviar_dato(0x07); pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x03);
    pantalla_enviar_comando(0xE8); pantalla_enviar_dato(0x34);
    pantalla_enviar_comando(0x62); pantalla_enviar_dato(0x18); pantalla_enviar_dato(0x0D); pantalla_enviar_dato(0x71); pantalla_enviar_dato(0xED); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x18); pantalla_enviar_dato(0x0F); pantalla_enviar_dato(0x71); pantalla_enviar_dato(0xEF); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x70);
    pantalla_enviar_comando(0x63); pantalla_enviar_dato(0x18); pantalla_enviar_dato(0x11); pantalla_enviar_dato(0x71); pantalla_enviar_dato(0xF1); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x18); pantalla_enviar_dato(0x13); pantalla_enviar_dato(0x71); pantalla_enviar_dato(0xF3); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x70);
    pantalla_enviar_comando(0x64); pantalla_enviar_dato(0x28); pantalla_enviar_dato(0x29); pantalla_enviar_dato(0xF1); pantalla_enviar_dato(0x01); pantalla_enviar_dato(0xF1); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x07);
    pantalla_enviar_comando(0x66); pantalla_enviar_dato(0x3C); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0xCD); pantalla_enviar_dato(0x67); pantalla_enviar_dato(0x45); pantalla_enviar_dato(0x45); pantalla_enviar_dato(0x10); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00);
    pantalla_enviar_comando(0x67); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x3C); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x01); pantalla_enviar_dato(0x54); pantalla_enviar_dato(0x10); pantalla_enviar_dato(0x32); pantalla_enviar_dato(0x98);
    pantalla_enviar_comando(0x74); pantalla_enviar_dato(0x10); pantalla_enviar_dato(0x85); pantalla_enviar_dato(0x80); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x4E); pantalla_enviar_dato(0x00);
    pantalla_enviar_comando(0x98); pantalla_enviar_dato(0x3E); pantalla_enviar_dato(0x07);
    pantalla_enviar_comando(0x35);
    pantalla_enviar_comando(0x35);          // Activa el "tearing effect" (sincronización de refresco)
    pantalla_enviar_comando(0x21);          // Invierte los colores (necesario para GC9A01)
    pantalla_enviar_comando(0x11);          // Sale del modo sleep (el panel de cristal líquido se activa)
    vTaskDelay(pdMS_TO_TICKS(120));         // Espera 120 ms para que el panel se estabilice
    pantalla_enviar_comando(0x29);          // Activa el display (muestra imagen; antes estaba en blanco)
    vTaskDelay(pdMS_TO_TICKS(20));          // Espera 20 ms de estabilización final

    gpio_set_level(PIN_PANTALLA_BL, 1); // Enciende la retroiluminación: la pantalla ya es visible
}
// --- Inicializa el sensor táctil CST816S ---
// Configura la interrupción (notificación automática al tocar) y realiza el ciclo de reset del chip
static void touch_inicializar(void)
{
    // Configura el pin INT como entrada con pull-up para detectar toques
    gpio_config_t cfg_touch_int = {};
    cfg_touch_int.pin_bit_mask = (1ULL << PIN_TOUCH_INT);  // Máscara: qué pin configurar (GPIO5)
    cfg_touch_int.mode = GPIO_MODE_INPUT;                   // El pin recibe señal (es entrada)
    cfg_touch_int.pull_up_en = GPIO_PULLUP_ENABLE;          // Activa resistencia pull-up interna
    cfg_touch_int.intr_type = GPIO_INTR_NEGEDGE;            // Interrupción en flanco de bajada (1→0 = toque detectado)
    gpio_config(&cfg_touch_int);                            // Aplica la configuración

    // Configura el pin RST como salida para poder resetear el chip táctil
    gpio_config_t cfg_touch_rst = {};
    cfg_touch_rst.pin_bit_mask = (1ULL << PIN_TOUCH_RST);  // Pin de reset (GPIO13)
    cfg_touch_rst.mode = GPIO_MODE_OUTPUT;                  // Es salida: nosotros controlamos el reset
    gpio_config(&cfg_touch_rst);

    // Ciclo de reset: alto → bajo → alto con pausas para que el chip arranque correctamente
    gpio_set_level(PIN_TOUCH_RST, 1);   // Estado normal (sin reset)
    vTaskDelay(pdMS_TO_TICKS(50));      // Espera 50 ms
    gpio_set_level(PIN_TOUCH_RST, 0);   // Activa el reset (el chip se reinicia)
    vTaskDelay(pdMS_TO_TICKS(5));       // Espera 5 ms (tiempo mínimo de reset según datasheet)
    gpio_set_level(PIN_TOUCH_RST, 1);   // Libera el reset (el chip arranca su firmware)
    vTaskDelay(pdMS_TO_TICKS(50));      // Espera 50 ms para que el chip inicialice completamente

    gpio_install_isr_service(0);                                   // Instala el servicio de interrupciones GPIO (solo se llama una vez en todo el programa)
    gpio_isr_handler_add(PIN_TOUCH_INT, touch_isr_handler, NULL); // Registra touch_isr_handler para GPIO5
}

// --- Lee el tipo de gesto detectado por el táctil CST816S ---
// Devuelve el código del gesto (0x05 = tap, etc.) o 0 si no hay gesto o hubo error
static uint8_t touch_leer_gesto(void)
{
    if (g_dev_touch == NULL) {
        return 0; // Si el táctil no se inicializó correctamente, devuelve "sin gesto"
    }
    uint8_t gesto = 0; // Variable donde se guardará el código de gesto leído
    // Lee 1 byte del registro GESTID del táctil por I2C
    if (i2c_leer_registros(g_dev_touch, CST816S_REG_GESTID, &gesto, 1) != ESP_OK) {
        return 0; // Si la lectura I2C falló, devuelve "sin gesto"
    }
    return gesto; // Devuelve el código de gesto detectado
}

// --- Lee el voltaje de la batería usando el conversor analógico-digital (ADC) ---
// El ADC mide el voltaje en el pin GPIO y lo devuelve en milivoltios (mV)
// Un divisor de voltaje externo reduce el voltaje real a un tercio (FACTOR_BATERIA = 3)
static float bateria_leer_voltios(void)
{
    int lectura_mv = 0; // El ADC devuelve el voltaje medido en el pin en milivoltios
    if (g_adc_bateria == NULL) {
        return 0.0f; // Si el ADC no se inicializó, devuelve cero
    }
    // Lee el voltaje del canal ADC configurado para la batería
    if (adc_oneshot_read(g_adc_bateria, CANAL_BATERIA_ADC, &lectura_mv) != ESP_OK) {
        return 0.0f; // Si la lectura falló, devuelve cero
    }
    // Conversión: mV → V (÷1000), luego compensa el divisor de voltaje (×3)
    // Ejemplo: si el ADC lee 1400 mV → (1400÷1000)×3 = 4,2 V (batería completamente cargada)
    return ((float)lectura_mv / 1000.0f) * FACTOR_BATERIA;
}

/**
 * @brief Callback de resultado de envio ESP-NOW.
 * @param info Informacion del intento de envio (no usada en esta implementacion).
 * @param estado Estado final del envio (exito o fallo).
 */
static void espnow_callback_envio(const esp_now_send_info_t *info, esp_now_send_status_t estado)
{
    (void)info; // Se ignora el parámetro "info"; el (void) evita una advertencia del compilador

    static uint32_t contador_ok = 0; // Conteo total de envíos exitosos (se conserva entre llamadas)
    if (estado == ESP_NOW_SEND_SUCCESS) {
        contador_ok++; // Incrementa el contador de éxitos
        // Solo imprime un log cada 20 éxitos (cada ~1 segundo a 50 ms/paquete)
        // Esto evita llenar el monitor con un mensaje cada 50 ms
        if ((contador_ok % 20U) == 0U) {
            ESP_LOGI(TAG_NOW, "Envio ESP-NOW exitoso (total=%lu)", (unsigned long)contador_ok);
        }
    } else {
        static uint32_t contador_fail = 0; // Conteo total de fallos
        contador_fail++; // Incrementa el contador de fallos
        // Igual que el éxito: solo imprime 1 log por segundo cuando el receptor está caído
        if ((contador_fail % 20U) == 0U) {
            ESP_LOGW(TAG_NOW, "Fallo ESP-NOW (total=%lu)", (unsigned long)contador_fail);
        }
    }
}

/**
 * @brief Inicializa WiFi en modo STA y habilita ESP-NOW.
 * @details Registra callback de envio y agrega el peer destino por MAC.
 */
static void espnow_inicializar(void)
{
    // Inicializa el almacenamiento no volátil (NVS), que WiFi necesita para guardar su configuración
    esp_err_t resultado = nvs_flash_init();
    // Si la partición NVS está llena o tiene versión diferente → la borra y reinicializa
    if (resultado == ESP_ERR_NVS_NO_FREE_PAGES || resultado == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Borra toda la NVS
        resultado = nvs_flash_init();        // Vuelve a inicializarla vacía
    }
    ESP_ERROR_CHECK(resultado); // Si sigue fallando, para con error fatal

    ESP_ERROR_CHECK(esp_netif_init());               // Inicializa la capa de red (requerido por WiFi)
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Crea el bucle de eventos del sistema

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT(); // Configuración WiFi por defecto
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));          // Inicializa el driver WiFi
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // Configura WiFi solo en RAM (no en flash)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));       // Modo Station (cliente, no punto de acceso)
    ESP_ERROR_CHECK(esp_wifi_start());                        // Enciende el radio WiFi
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));           // Desactiva ahorro de energía WiFi (latencia mínima)

    ESP_ERROR_CHECK(esp_now_init());                                  // Inicializa el protocolo ESP-NOW
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_callback_envio)); // Registra la función de notificación de envío

    // Registra el receptor como "peer" (destinatario conocido) para poder enviarle datos
    esp_now_peer_info_t peer = {};                             // Estructura con la información del receptor
    memcpy(peer.peer_addr, MAC_DESTINO, sizeof(MAC_DESTINO)); // Copia la dirección MAC del receptor
    peer.ifidx  = WIFI_IF_STA;  // Usa la interfaz WiFi en modo Station
    peer.channel = 0;            // Canal 0 = canal actual del WiFi (el ESP lo detecta automáticamente)
    peer.encrypt = false;        // Sin cifrado (más rápido; el mensaje viaja en claro)
    ESP_ERROR_CHECK(esp_now_add_peer(&peer)); // Añade el receptor a la lista de destinos conocidos
}

/**
 * @brief Tarea de captura IMU y transmision por ESP-NOW.
 * @param parametro Parametro de FreeRTOS (no usado).
 * @note Periodo fijo de 50 ms (20 Hz) usando vTaskDelayUntil.
 */
static void tarea_captura(void *parametro)
{
    (void)parametro;                            // Parámetro no usado
    datos_imu_t medicion = {};                  // Lectura del IMU de este ciclo
    angulos_t angulos_locales = {};             // Ángulos acumulados por el filtro
    TickType_t instante_anterior = xTaskGetTickCount(); // Marca de tiempo de inicio (para delay preciso)

    for (;;) { // Bucle infinito: la tarea nunca termina
        if (imu_leer_datos(&medicion) == ESP_OK) { // Lee el IMU; si hay error salta el ciclo

            // Actualiza el filtro complementario: combina acelerómetro y giroscopio → roll y pitch
            filtro_complementario(&medicion, &angulos_locales, PERIODO_MUESTREO_S);

            float bateria = bateria_leer_voltios(); // Lee voltaje de batería (solo para mostrar en pantalla)

            // Prepara el paquete de 32 bytes que se enviará por radio
            paquete_espnow_t paquete_local = {};
            paquete_local.roll  = angulos_locales.alabeo;  // Ángulo de inclinación lateral en grados
            paquete_local.pitch = angulos_locales.cabeceo; // Ángulo de inclinación frontal en grados
            paquete_local.acc_x = medicion.accel_x;        // Aceleración X en g
            paquete_local.acc_y = medicion.accel_y;        // Aceleración Y en g
            paquete_local.acc_z = medicion.accel_z;        // Aceleración Z en g
            paquete_local.gyr_x = medicion.gyro_x;         // Velocidad angular X en °/s
            paquete_local.gyr_y = medicion.gyro_y;         // Velocidad angular Y en °/s
            paquete_local.gyr_z = medicion.gyro_z;         // Velocidad angular Z en °/s

            // Toma el mutex (candado) para escribir en las variables globales compartidas
            // Espera máximo 10 ms; si no consigue el candado, salta la escritura global este ciclo
            if (xSemaphoreTake(g_mutex_datos, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_datos_imu = medicion;        // Actualiza los datos globales del IMU
                g_angulos   = angulos_locales; // Actualiza los ángulos globales
                g_paquete   = paquete_local;   // Actualiza el paquete global
                g_bateria_v = bateria;         // Actualiza el voltaje de batería global
                xSemaphoreGive(g_mutex_datos); // Libera el candado para que la tarea de interfaz pueda leer
            }

            // Envía el paquete por radio ESP-NOW al receptor (32 bytes)
            // Funciona SIEMPRE, sin importar si la pantalla está encendida o apagada
            esp_now_send(MAC_DESTINO, (const uint8_t *)&paquete_local, sizeof(paquete_local));
        }
        // Espera el tiempo restante hasta cumplir 50 ms desde el inicio del ciclo
        // vTaskDelayUntil compensa el tiempo que tardó la ejecución, manteniendo periodicidad exacta
        vTaskDelayUntil(&instante_anterior, pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Tarea de interfaz grafica y gestion tactil.
 * @param parametro Parametro de FreeRTOS (no usado).
 * @details Actualiza valores en pantalla y controla apagado por inactividad.
 */
static void tarea_interfaz(void *parametro)
{
    (void)parametro;                             // Parámetro no usado
    datos_imu_t datos_copia   = {};              // Copia local de los datos IMU
    angulos_t   angulos_copia = {};              // Copia local de los ángulos
    float       bateria_copia = 0.0f;            // Copia local del voltaje de batería
    TickType_t  instante_anterior = xTaskGetTickCount(); // Marca de tiempo para delay preciso
    bool        pantalla_encendida = true;       // Estado actual de la retroiluminación
    int64_t     ultimo_toque_ms = esp_timer_get_time() / 1000; // Tiempo del último toque en ms
    char        linea[24];                       // Buffer para formatear números como texto

    // ---- Dibuja la interfaz estática (se hace UNA SOLA VEZ al arrancar) ----
    pantalla_rellenar_rect(0, 0,   239, 47,  0xF410); // Franja roja oscura (encabezado)
    pantalla_rellenar_rect(0, 47,  239, 120, 0x4F30); // Franja verde oscura (roll, pitch, acc XY)
    pantalla_rellenar_rect(0, 120, 239, 195, 0xAD55); // Franja azul-gris (acc Z, giroscopios)
    pantalla_rellenar_rect(0, 195, 239, 239, 0x2595); // Franja morada (batería)

    // Etiquetas fijas: los valores numéricos se actualizarán en cada ciclo del bucle
    pantalla_dibujar_texto(70,  16, "HEALTH MONITOR", COLOR_BLANCO, 0xF410, 1); // Título
    pantalla_dibujar_texto(20,  52, "ROLL=",           COLOR_BLANCO, 0x4F30, 1);
    pantalla_dibujar_texto(20,  68, "PITCH=",          COLOR_BLANCO, 0x4F30, 1);
    pantalla_dibujar_texto(20,  90, "ACC_X =",         COLOR_BLANCO, 0x4F30, 1);
    pantalla_dibujar_texto(20, 106, "ACC_Y =",         COLOR_BLANCO, 0x4F30, 1);
    pantalla_dibujar_texto(20, 122, "ACC_Z =",         COLOR_BLANCO, 0xAD55, 1);
    pantalla_dibujar_texto(20, 152, "GYR_X =",         COLOR_BLANCO, 0xAD55, 1);
    pantalla_dibujar_texto(20, 168, "GYR_Y =",         COLOR_BLANCO, 0xAD55, 1);
    pantalla_dibujar_texto(20, 184, "GYR_Z =",         COLOR_BLANCO, 0xAD55, 1);
    pantalla_dibujar_texto(57, 208, "BAT(V)=",         COLOR_BLANCO, 0x2595, 2); // Escala 2 = doble tamaño

    for (;;) { // Bucle infinito: la tarea nunca termina

        // ---- Manejo del táctil ----
        if (g_touch_evento) {          // Si la interrupción del táctil activó la bandera...
            g_touch_evento = false;    // Limpia la bandera para el próximo toque
            uint8_t gesto = touch_leer_gesto(); // Lee qué tipo de gesto fue
            if (gesto == CST816S_GESTO_TAP || gesto == 0) { // Tap o gesto desconocido: activa pantalla
                ultimo_toque_ms = esp_timer_get_time() / 1000; // Actualiza el tiempo del último toque
                if (!pantalla_encendida) {                      // Si la pantalla estaba apagada...
                    gpio_set_level(PIN_PANTALLA_BL, 1);         // Enciende la retroiluminación
                    pantalla_encendida = true;
                }
            }
        }

        // ---- Timeout de apagado automático ----
        int64_t ahora_ms = esp_timer_get_time() / 1000; // Tiempo actual en ms
        // Si la pantalla está encendida y han pasado más de 10 s desde el último toque...
        if (pantalla_encendida && (ahora_ms - ultimo_toque_ms) > TIEMPO_APAGADO_MS) {
            gpio_set_level(PIN_PANTALLA_BL, 0); // Apaga la retroiluminación (ahorro de energía)
            pantalla_encendida = false;
        }

        // Si la pantalla está apagada no hay nada que dibujar: espera y vuelve al inicio
        if (!pantalla_encendida) {
            vTaskDelayUntil(&instante_anterior, pdMS_TO_TICKS(50));
            continue; // Salta el resto del ciclo
        }

        // ---- Lee los datos compartidos de forma segura con el mutex ----
        if (xSemaphoreTake(g_mutex_datos, pdMS_TO_TICKS(10)) == pdTRUE) {
            datos_copia   = g_datos_imu;  // Copia los datos IMU actuales
            angulos_copia = g_angulos;    // Copia los ángulos actuales
            bateria_copia = g_bateria_v;  // Copia el voltaje de batería
            xSemaphoreGive(g_mutex_datos); // Libera el candado
        }

        // ---- Borra las zonas de los valores antes de redibujar (evita artefactos visuales) ----
        pantalla_rellenar_rect(112, 52,  220, 60,  0x4F30); // Borra área del valor ROLL
        pantalla_rellenar_rect(112, 68,  220, 76,  0x4F30); // Borra área del valor PITCH
        pantalla_rellenar_rect(112, 90,  220, 98,  0x4F30); // Borra área del valor ACC_X
        pantalla_rellenar_rect(112, 106, 220, 114, 0x4F30); // Borra área del valor ACC_Y
        pantalla_rellenar_rect(112, 122, 220, 130, 0xAD55); // Borra área del valor ACC_Z
        pantalla_rellenar_rect(112, 152, 220, 160, 0xAD55); // Borra área del valor GYR_X
        pantalla_rellenar_rect(112, 168, 220, 176, 0xAD55); // Borra área del valor GYR_Y
        pantalla_rellenar_rect(112, 184, 220, 192, 0xAD55); // Borra área del valor GYR_Z
        pantalla_rellenar_rect(130, 200, 220, 214, 0x2595); // Borra área del valor BAT

        // ---- Dibuja los valores numéricos actualizados ----
        // snprintf formatea el número como texto; "%+6.1f" = con signo, 6 caracteres, 1 decimal
        snprintf(linea, sizeof(linea), "%+6.1f", (double)angulos_copia.alabeo);
        pantalla_dibujar_texto(112, 52, linea, COLOR_BLANCO, 0x4F30, 1);  // Muestra ROLL

        snprintf(linea, sizeof(linea), "%+6.1f", (double)angulos_copia.cabeceo);
        pantalla_dibujar_texto(112, 68, linea, COLOR_BLANCO, 0x4F30, 1);  // Muestra PITCH

        snprintf(linea, sizeof(linea), "%+1.3f", (double)datos_copia.accel_x);
        pantalla_dibujar_texto(112, 90, linea, COLOR_BLANCO, 0x4F30, 1);  // Muestra ACC_X

        snprintf(linea, sizeof(linea), "%+1.3f", (double)datos_copia.accel_y);
        pantalla_dibujar_texto(112, 106, linea, COLOR_BLANCO, 0x4F30, 1); // Muestra ACC_Y

        snprintf(linea, sizeof(linea), "%+1.3f", (double)datos_copia.accel_z);
        pantalla_dibujar_texto(112, 122, linea, COLOR_BLANCO, 0xAD55, 1); // Muestra ACC_Z

        snprintf(linea, sizeof(linea), "%+06.1f", (double)datos_copia.gyro_x);
        pantalla_dibujar_texto(112, 152, linea, COLOR_BLANCO, 0xAD55, 1); // Muestra GYR_X

        snprintf(linea, sizeof(linea), "%+06.1f", (double)datos_copia.gyro_y);
        pantalla_dibujar_texto(112, 168, linea, COLOR_BLANCO, 0xAD55, 1); // Muestra GYR_Y

        snprintf(linea, sizeof(linea), "%+06.1f", (double)datos_copia.gyro_z);
        pantalla_dibujar_texto(112, 184, linea, COLOR_BLANCO, 0xAD55, 1); // Muestra GYR_Z

        snprintf(linea, sizeof(linea), "%1.2f", (double)bateria_copia);   // Sin signo, 2 decimales
        pantalla_dibujar_texto(130, 200, linea, COLOR_BLANCO, 0x2595, 2); // Muestra voltaje batería (escala 2)

        vTaskDelayUntil(&instante_anterior, pdMS_TO_TICKS(50)); // Espera hasta completar 50 ms
    }
}
/**
 * @brief Punto de entrada del firmware en ESP-IDF.
 * @details Inicializa perifericos, energia, ESP-NOW y crea tareas de captura e interfaz.
 */
extern "C" void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "Iniciando monitor de salud ESP32-S3"); // Primer mensaje en el monitor serie

    // ---- Configuración de gestión de energía ----
    // Permite al CPU reducir su frecuencia a 40 MHz cuando no hay carga, ahorrando batería
    esp_pm_config_t config_pm = {};
    config_pm.max_freq_mhz = 80;          // CPU a máximo 80 MHz (suficiente para este proyecto)
    config_pm.min_freq_mhz = 40;          // CPU puede bajar a 40 MHz en reposo
    config_pm.light_sleep_enable = false; // No entrar en sleep profundo (interferiría con ESP-NOW)
    esp_err_t r_pm = esp_pm_configure(&config_pm); // Intenta aplicar la configuración de energía
    if (r_pm != ESP_OK) {
        // En algunos modos de compilación esta función no está disponible; no es error fatal
        ESP_LOGW(TAG_MAIN, "Power management no disponible: %s", esp_err_to_name(r_pm));
    }

    // ---- Crea el mutex para proteger las variables compartidas entre tareas ----
    g_mutex_datos = xSemaphoreCreateMutex(); // Crea el candado en el heap de FreeRTOS
    if (g_mutex_datos == NULL) {
        ESP_LOGE(TAG_MAIN, "No se pudo crear el mutex de datos"); // Error crítico: sin mutex no hay seguridad
        return; // Termina app_main; el watchdog reiniciará el ESP32
    }

    // ---- Inicializa los periféricos en orden ----
    ESP_ERROR_CHECK(i2c_inicializar_bus_y_dispositivos()); // Bus I2C + IMU + táctil (fallo = reinicio)
    ESP_ERROR_CHECK(qmi8658_inicializar());                // Configura el IMU (fallo = reinicio)
    touch_inicializar();                                   // Táctil e interrupción (no es fatal si falla)

    // ---- Inicializa el ADC para leer la batería ----
    adc_oneshot_unit_init_cfg_t adc_cfg = {};
    adc_cfg.unit_id = ADC_UNIT_1;                                     // Usar el ADC número 1 del ESP32
    adc_cfg.clk_src = static_cast<adc_oneshot_clk_src_t>(0);         // Fuente de reloj por defecto (cast necesario en C++)
    if (adc_oneshot_new_unit(&adc_cfg, &g_adc_bateria) == ESP_OK) {   // Si el ADC se inicializó bien...
        adc_oneshot_chan_cfg_t canal_cfg = {};
        canal_cfg.bitwidth = ADC_BITWIDTH_12;  // Resolución de 12 bits (valores 0–4095)
        canal_cfg.atten = ADC_ATTEN_DB_12;     // Atenuación 12 dB: amplía el rango medible hasta ~3,1 V
        adc_oneshot_config_channel(g_adc_bateria, CANAL_BATERIA_ADC, &canal_cfg); // Configura el canal
    }

    pantalla_inicializar();  // Inicializa la pantalla GC9A01 (secuencia completa de arranque)
    espnow_inicializar();    // Inicializa WiFi y ESP-NOW, registra al receptor como peer

    // ---- Crea las dos tareas paralelas que se ejecutan indefinidamente ----
    // xTaskCreatePinnedToCore(función, nombre, pila_bytes, argumento, prioridad, handle, núcleo)
    xTaskCreatePinnedToCore(tarea_captura,  "Tarea_Captura",  4096, NULL, 5, NULL, 0); // Núcleo 0, prio alta
    xTaskCreatePinnedToCore(tarea_interfaz, "Tarea_Interfaz", 6144, NULL, 3, NULL, 1); // Núcleo 1, prio media
    // app_main termina aquí; las tareas siguen ejecutándose indefinidamente en paralelo
}
