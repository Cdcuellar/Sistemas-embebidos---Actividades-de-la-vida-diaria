/*
 * =============================================================================
 *  Nivel de Burbuja Circular - ESP32-S3 con IMU y Pantalla GC9A01 1.28"
 * =============================================================================
 *  Descripción:
 *    Lee el IMU interno (QMI8658 o MPU6050) por I2C, calcula los ángulos de
 *    Alabeo (Roll) y Cabeceo (Pitch) con un Filtro Complementario, y dibuja
 *    un nivel de burbuja animado en la pantalla circular GC9A01.
 *
 *  Hardware objetivo: ESP32-S3 Wearable 1.28"
 *  Framework: ESP-IDF v6.0
 *  Frecuencia de CPU: 80 MHz (ahorro de energía con animación fluida)
 * =============================================================================
 */

/* -------------------------------------------------------------------------- */
/*  Inclusiones estándar y de ESP-IDF                                          */
/* -------------------------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"

/* -------------------------------------------------------------------------- */
/*  Etiquetas de log                                                            */
/* -------------------------------------------------------------------------- */
static const char *ETIQUETA_IMU     = "IMU";
static const char *ETIQUETA_DISP    = "DISPLAY";
static const char *ETIQUETA_MAIN    = "MAIN";

/* ============================================================================
 *  CONFIGURACIÓN DE PINES - Ajustar según el esquemático de tu placa
 * ============================================================================
 *
 *  Los componentes integrados (IMU y pantalla) están soldados en el PCB pero
 *  necesitan comunicarse con el ESP32-S3 mediante buses I2C y SPI. Esta
 *  comunicación se realiza a través de pistas de cobre que conectan los
 *  componentes a pines GPIO específicos del MCU.
 *
 *  El firmware debe conocer exactamente qué GPIO corresponde a cada línea
 *  del bus (SDA, SCL para I2C; MOSI, SCLK, CS, DC, RST, BL para SPI).
 *  Sin esta configuración, el sistema no puede comunicarse con los periféricos
 *  integrados aunque estén físicamente conectados.
 *
 *  Los valores definidos aquí corresponden a la configuración estándar del
 *  ESP32-S3 Touch LCD 1.28". Si tu placa tiene un esquemático diferente,
 *  verifica los pines reales y actualiza estos valores.
 *
 * ============================================================================ */

/* --- Bus I2C interno (IMU) --- */
#define PIN_I2C_SDA         6       /* GPIO 6: línea SDA del bus I2C          */
#define PIN_I2C_SCL         7       /* GPIO 7: línea SCL del bus I2C          */
#define VELOCIDAD_I2C_HZ    400000  /* 400 kHz (modo rápido, estándar)        */
#define PUERTO_I2C          I2C_NUM_0 /* Bus I2C 0 del ESP32-S3              */

/* --- Bus SPI de la pantalla GC9A01 --- */
#define PIN_SPI_MOSI        11      /* GPIO 11: MOSI (segun sketch Arduino)   */
#define PIN_SPI_SCLK        10      /* GPIO 10: SCLK (segun sketch Arduino)   */
#define PIN_SPI_MISO        12      /* GPIO 12: MISO compartido del bus SPI   */
#define PIN_SPI_CS          9       /* GPIO 9:  CS   (chip select pantalla)   */
#define PIN_PANTALLA_DC     8       /* GPIO 8:  D/C  (comando vs datos)       */
#define PIN_PANTALLA_RST    14      /* GPIO 14: RST  (reset hardware)         */
#define PIN_PANTALLA_BL     2       /* GPIO 2:  BL   (retroiluminación LED)   */
#define PIN_TOUCH_RST       13      /* GPIO 13: reset del CST816S             */
#define PIN_TOUCH_INT       5       /* GPIO 5:  IRQ del CST816S               */
#define VELOCIDAD_SPI_HZ    40000000 /* 40 MHz: velocidad máxima pantalla     */
#define HOST_SPI            SPI2_HOST /* Bus SPI 2 del ESP32-S3              */

#define TIEMPO_APAGADO_MS   10000   /* Timeout de backlight por inactividad    */

/* ADC batería (GPIO1 en el sketch Arduino) */
#define CANAL_BATERIA_ADC   ADC_CHANNEL_0
#define FACTOR_BATERIA      3.0f

/* ============================================================================
 *  PARÁMETROS DE LA PANTALLA GC9A01 1.28"
 * ============================================================================ */
#define ANCHO_PANTALLA      240     /* Píxeles de ancho                       */
#define ALTO_PANTALLA       240     /* Píxeles de alto                        */

/* ============================================================================
 *  PARÁMETROS DE COLORES (formato RGB565 big-endian)
 * ============================================================================ */
#define COLOR_NEGRO         0x0000  /* Fondo negro                            */
#define COLOR_BLANCO        0xFFFF  /* Texto blanco                           */

/* ============================================================================
 *  DIRECCIONES I2C DE LOS IMUs SOPORTADOS
 * ============================================================================ */
#define DIRECCION_QMI8658   0x6B    /* Dirección I2C confirmada del QMI8658   */
#define DIRECCION_MPU6050   0x68    /* Dirección MPU6050 (fallback inactivo)   */
#define SOLO_QMI8658        1       /* Esta placa solo tiene QMI8658          */

/* ============================================================================
 *  REGISTROS DEL QMI8658 (IMU preferido en wearables con GC9A01)
 * ============================================================================ */
#define QMI8658_REG_ID          0x00  /* Registro de identificación           */
#define QMI8658_REG_CTRL1       0x02  /* Control 1 (SPI/I2C, endian)         */
#define QMI8658_REG_CTRL2       0x03  /* Control 2 (config acelerómetro)     */
#define QMI8658_REG_CTRL3       0x04  /* Control 3 (config giroscopio)       */
#define QMI8658_REG_CTRL7       0x08  /* Control 7 (habilitar sensores)      */
#define QMI8658_REG_AX_L        0x35  /* Acelerómetro X bajo                 */
#define QMI8658_REG_AX_H        0x36  /* Acelerómetro X alto                 */
#define QMI8658_REG_AY_L        0x37  /* Acelerómetro Y bajo                 */
#define QMI8658_REG_AY_H        0x38  /* Acelerómetro Y alto                 */
#define QMI8658_REG_AZ_L        0x39  /* Acelerómetro Z bajo                 */
#define QMI8658_REG_AZ_H        0x3A  /* Acelerómetro Z alto                 */
#define QMI8658_REG_GX_L        0x3B  /* Giroscopio X bajo                   */
#define QMI8658_REG_GX_H        0x3C  /* Giroscopio X alto                   */
#define QMI8658_REG_GY_L        0x3D  /* Giroscopio Y bajo                   */
#define QMI8658_REG_GY_H        0x3E  /* Giroscopio Y alto                   */
#define QMI8658_REG_GZ_L        0x3F  /* Giroscopio Z bajo                   */
#define QMI8658_REG_GZ_H        0x40  /* Giroscopio Z alto                   */
#define QMI8658_ID_VAL          0x05  /* Valor esperado en el registro ID     */
#define INTERCAMBIAR_ACCEL_GYRO 0     /* 0: mapeo normal ACC/GYR */

/* ============================================================================
 *  REGISTROS DEL MPU6050 (IMU alternativo)
 * ============================================================================ */
#define MPU6050_REG_PWR_MGMT_1  0x6B  /* Gestión de energía 1               */
#define MPU6050_REG_CONFIG      0x1A  /* Configuración DLPF                  */
#define MPU6050_REG_ACCEL_CFG   0x1C  /* Configuración acelerómetro          */
#define MPU6050_REG_GYRO_CFG    0x1B  /* Configuración giroscopio            */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B /* Datos brutos acelerómetro          */
#define MPU6050_REG_WHO_AM_I    0x75  /* Registro de identidad               */
#define MPU6050_ID_VAL          0x68  /* Valor esperado en WHO_AM_I          */

/* ============================================================================
 *  PARÁMETROS DEL FILTRO COMPLEMENTARIO
 * ============================================================================ */
#define FACTOR_COMP_ACCEL   0.02f   /* Peso del acelerómetro (2%)            */
#define FACTOR_COMP_GYRO    0.98f   /* Peso del giroscopio   (98%)           */
#define PERIODO_IMU_SEG     0.020f  /* Período de muestreo IMU: 20 ms        */

/* ============================================================================
 *  ESCALAS DE CONVERSIÓN DE DATOS BRUTOS
 * ============================================================================ */
/* QMI8658: acelerómetro en +-8g -> 4096 LSB/g (como el sketch Arduino) */
#define QMI8658_ESCALA_ACCEL    4096.0f
/* QMI8658: giroscopio en +-512 dps -> 64 LSB/(deg/s) */
#define QMI8658_ESCALA_GYRO     64.0f
/* MPU6050: acelerómetro en +-2g -> 16384 LSB/g */
#define MPU6050_ESCALA_ACCEL    16384.0f
/* MPU6050: giroscopio en +-250 dps -> 131 LSB/(deg/s) */
#define MPU6050_ESCALA_GYRO     131.0f

/* ============================================================================
 *  TIPOS ENUMERADOS
 * ============================================================================ */
typedef enum {
	TIPO_IMU_QMI8658,       /* Sensor QMI8658 detectado                      */
	TIPO_IMU_MPU6050,       /* Sensor MPU6050 detectado                      */
	TIPO_IMU_DESCONOCIDO    /* No se detectó ningún IMU compatible            */
} tipo_imu_t;

/* ============================================================================
 *  ESTRUCTURAS DE DATOS
 * ============================================================================ */

/* Datos brutos del IMU en unidades de la física (g y deg/s) */
typedef struct {
	float accel_x;  /* Aceleración en X [g]                                  */
	float accel_y;  /* Aceleración en Y [g]                                  */
	float accel_z;  /* Aceleración en Z [g]                                  */
	float gyro_x;   /* Velocidad angular en X [deg/s]                        */
	float gyro_y;   /* Velocidad angular en Y [deg/s]                        */
	float gyro_z;   /* Velocidad angular en Z [deg/s]                        */
} datos_imu_t;

/* Ángulos de orientación calculados */
typedef struct {
	float alabeo;   /* Roll:  rotación sobre el eje X [deg] (-180 a +180)    */
	float cabeceo;  /* Pitch: rotación sobre el eje Y [deg] (-90  a  +90)    */
} angulos_t;

/* ============================================================================
 *  VARIABLES GLOBALES PROTEGIDAS POR MUTEX
 * ============================================================================ */
static datos_imu_t       g_datos_imu         = {0};  /* Última muestra IMU         */
static angulos_t         g_angulos           = {0};  /* Roll/Pitch estimados [deg] */
static SemaphoreHandle_t g_mutex_datos_imu   = NULL; /* Mutex de la muestra IMU    */

/* ============================================================================
 *  VARIABLES DEL IMU
 * ============================================================================ */
static tipo_imu_t              g_tipo_imu     = TIPO_IMU_DESCONOCIDO;
static i2c_master_bus_handle_t g_bus_i2c      = NULL;
static i2c_master_dev_handle_t g_dev_imu      = NULL;
static float                   g_escala_accel = QMI8658_ESCALA_ACCEL;
static float                   g_escala_gyro  = QMI8658_ESCALA_GYRO;

/* ============================================================================
 *  VARIABLES DE LA PANTALLA
 * ============================================================================ */
static spi_device_handle_t g_handle_spi = NULL; /* Handle del bus SPI      */
static adc_oneshot_unit_handle_t g_adc_bateria = NULL;
static volatile bool g_touch_evento = false;

/* Buffer de línea DMA-capable: evita configurar ventana pixel a pixel      */
/* Tamaño: fila más ancha posible (240 px × 2 bytes) + 1 caracter (10×14×2) */
static uint8_t s_linea_spi[240 * 2]           __attribute__((aligned(4)));
static uint8_t s_char_buf [5 * 2 * 7 * 2 * 2] __attribute__((aligned(4)));

/* ============================================================================
 *  PROTOTIPOS DE FUNCIONES
 * ============================================================================ */
static esp_err_t i2c_escribir_registro(uint8_t dir, uint8_t reg, uint8_t valor);
static esp_err_t i2c_leer_registros(uint8_t dir, uint8_t reg_inicio,
									uint8_t *buffer, size_t longitud);
static esp_err_t imu_detectar_e_inicializar(void);
static esp_err_t qmi8658_inicializar(void);
#if !SOLO_QMI8658
static esp_err_t mpu6050_inicializar(void);
#endif
static esp_err_t imu_leer_datos_brutos(datos_imu_t *datos);
static void      filtro_complementario(const datos_imu_t *medicion,
									   angulos_t *angulos_prev, float dt);
static void      pantalla_enviar_comando(uint8_t comando);
static void      pantalla_enviar_dato(uint8_t dato);
static void      pantalla_inicializar(void);
static void      pantalla_fijar_ventana(uint16_t x0, uint16_t y0,
										uint16_t x1, uint16_t y1);
static void      pantalla_rellenar_rect(uint16_t x0, uint16_t y0,
										uint16_t x1, uint16_t y1,
										uint16_t color);
static void      pantalla_dibujar_caracter(int16_t x, int16_t y, char c,
										   uint16_t color_fg, uint16_t color_bg,
										   uint8_t escala);
static void      pantalla_dibujar_texto(int16_t x, int16_t y, const char *texto,
										uint16_t color_fg, uint16_t color_bg,
										uint8_t escala);
static void      tarea_lectura_imu(void *parametro);
static void      tarea_display(void *parametro);
static void      touch_inicializar(void);
static float     bateria_leer_voltios(void);

static void IRAM_ATTR touch_isr_handler(void *arg)
{
	(void)arg;
	g_touch_evento = true;
}

/* ============================================================================
 *  Fuente de mapa de bits 5x7 (ASCII 32-126)
 *  Cada caracter ocupa 5 bytes (columnas); cada bit = 1 pixel (fila 0 arriba)
 * ============================================================================ */
static const uint8_t fuente_5x7[][5] = {
	{0x00,0x00,0x00,0x00,0x00}, /* espacio 32  */
	{0x00,0x00,0x5F,0x00,0x00}, /* !  33 */
	{0x00,0x07,0x00,0x07,0x00}, /* "  34 */
	{0x14,0x7F,0x14,0x7F,0x14}, /* #  35 */
	{0x24,0x2A,0x7F,0x2A,0x12}, /* $  36 */
	{0x23,0x13,0x08,0x64,0x62}, /* %  37 */
	{0x36,0x49,0x55,0x22,0x50}, /* &  38 */
	{0x00,0x05,0x03,0x00,0x00}, /* '  39 */
	{0x00,0x1C,0x22,0x41,0x00}, /* (  40 */
	{0x00,0x41,0x22,0x1C,0x00}, /* )  41 */
	{0x08,0x2A,0x1C,0x2A,0x08}, /* *  42 */
	{0x08,0x08,0x3E,0x08,0x08}, /* +  43 */
	{0x00,0x50,0x30,0x00,0x00}, /* ,  44 */
	{0x08,0x08,0x08,0x08,0x08}, /* -  45 */
	{0x00,0x60,0x60,0x00,0x00}, /* .  46 */
	{0x20,0x10,0x08,0x04,0x02}, /* /  47 */
	{0x3E,0x51,0x49,0x45,0x3E}, /* 0  48 */
	{0x00,0x42,0x7F,0x40,0x00}, /* 1  49 */
	{0x42,0x61,0x51,0x49,0x46}, /* 2  50 */
	{0x21,0x41,0x45,0x4B,0x31}, /* 3  51 */
	{0x18,0x14,0x12,0x7F,0x10}, /* 4  52 */
	{0x27,0x45,0x45,0x45,0x39}, /* 5  53 */
	{0x3C,0x4A,0x49,0x49,0x30}, /* 6  54 */
	{0x01,0x71,0x09,0x05,0x03}, /* 7  55 */
	{0x36,0x49,0x49,0x49,0x36}, /* 8  56 */
	{0x06,0x49,0x49,0x29,0x1E}, /* 9  57 */
	{0x00,0x36,0x36,0x00,0x00}, /* :  58 */
	{0x00,0x56,0x36,0x00,0x00}, /* ;  59 */
	{0x08,0x14,0x22,0x41,0x00}, /* <  60 */
	{0x14,0x14,0x14,0x14,0x14}, /* =  61 */
	{0x00,0x41,0x22,0x14,0x08}, /* >  62 */
	{0x02,0x01,0x51,0x09,0x06}, /* ?  63 */
	{0x32,0x49,0x79,0x41,0x3E}, /* @  64 */
	{0x7E,0x11,0x11,0x11,0x7E}, /* A  65 */
	{0x7F,0x49,0x49,0x49,0x36}, /* B  66 */
	{0x3E,0x41,0x41,0x41,0x22}, /* C  67 */
	{0x7F,0x41,0x41,0x22,0x1C}, /* D  68 */
	{0x7F,0x49,0x49,0x49,0x41}, /* E  69 */
	{0x7F,0x09,0x09,0x09,0x01}, /* F  70 */
	{0x3E,0x41,0x41,0x51,0x32}, /* G  71 */
	{0x7F,0x08,0x08,0x08,0x7F}, /* H  72 */
	{0x00,0x41,0x7F,0x41,0x00}, /* I  73 */
	{0x20,0x40,0x41,0x3F,0x01}, /* J  74 */
	{0x7F,0x08,0x14,0x22,0x41}, /* K  75 */
	{0x7F,0x40,0x40,0x40,0x40}, /* L  76 */
	{0x7F,0x02,0x04,0x02,0x7F}, /* M  77 */
	{0x7F,0x04,0x08,0x10,0x7F}, /* N  78 */
	{0x3E,0x41,0x41,0x41,0x3E}, /* O  79 */
	{0x7F,0x09,0x09,0x09,0x06}, /* P  80 */
	{0x3E,0x41,0x51,0x21,0x5E}, /* Q  81 */
	{0x7F,0x09,0x19,0x29,0x46}, /* R  82 */
	{0x26,0x49,0x49,0x49,0x32}, /* S  83 */
	{0x01,0x01,0x7F,0x01,0x01}, /* T  84 */
	{0x3F,0x40,0x40,0x40,0x3F}, /* U  85 */
	{0x1F,0x20,0x40,0x20,0x1F}, /* V  86 */
	{0x3F,0x40,0x38,0x40,0x3F}, /* W  87 */
	{0x63,0x14,0x08,0x14,0x63}, /* X  88 */
	{0x07,0x08,0x70,0x08,0x07}, /* Y  89 */
	{0x61,0x51,0x49,0x45,0x43}, /* Z  90 */
	{0x00,0x7F,0x41,0x41,0x00}, /* [  91 */
	{0x02,0x04,0x08,0x10,0x20}, /* \  92 */
	{0x00,0x41,0x41,0x7F,0x00}, /* ]  93 */
	{0x04,0x02,0x01,0x02,0x04}, /* ^  94 */
	{0x40,0x40,0x40,0x40,0x40}, /* _  95 */
	{0x00,0x01,0x02,0x04,0x00}, /* `  96 */
	{0x20,0x54,0x54,0x54,0x78}, /* a  97 */
	{0x7F,0x48,0x44,0x44,0x38}, /* b  98 */
	{0x38,0x44,0x44,0x44,0x20}, /* c  99 */
	{0x38,0x44,0x44,0x48,0x7F}, /* d 100 */
	{0x38,0x54,0x54,0x54,0x18}, /* e 101 */
	{0x08,0x7E,0x09,0x01,0x02}, /* f 102 */
	{0x08,0x14,0x54,0x54,0x3C}, /* g 103 */
	{0x7F,0x08,0x04,0x04,0x78}, /* h 104 */
	{0x00,0x44,0x7D,0x40,0x00}, /* i 105 */
	{0x20,0x40,0x44,0x3D,0x00}, /* j 106 */
	{0x7F,0x10,0x28,0x44,0x00}, /* k 107 */
	{0x00,0x41,0x7F,0x40,0x00}, /* l 108 */
	{0x7C,0x04,0x18,0x04,0x78}, /* m 109 */
	{0x7C,0x08,0x04,0x04,0x78}, /* n 110 */
	{0x38,0x44,0x44,0x44,0x38}, /* o 111 */
	{0x7C,0x14,0x14,0x14,0x08}, /* p 112 */
	{0x08,0x14,0x14,0x18,0x7C}, /* q 113 */
	{0x7C,0x08,0x04,0x04,0x08}, /* r 114 */
	{0x48,0x54,0x54,0x54,0x20}, /* s 115 */
	{0x04,0x3F,0x44,0x40,0x20}, /* t 116 */
	{0x3C,0x40,0x40,0x20,0x7C}, /* u 117 */
	{0x1C,0x20,0x40,0x20,0x1C}, /* v 118 */
	{0x3C,0x40,0x30,0x40,0x3C}, /* w 119 */
	{0x44,0x28,0x10,0x28,0x44}, /* x 120 */
	{0x0C,0x50,0x50,0x50,0x3C}, /* y 121 */
	{0x44,0x64,0x54,0x4C,0x44}, /* z 122 */
	{0x00,0x08,0x36,0x41,0x00}, /* { 123 */
	{0x00,0x00,0x7F,0x00,0x00}, /* | 124 */
	{0x00,0x41,0x36,0x08,0x00}, /* } 125 */
	{0x08,0x08,0x2A,0x1C,0x08}, /* ~ 126 */
};

/* ============================================================================
 *  FUNCIONES AUXILIARES DE I2C
 * ============================================================================ */

/**
 * @brief Escribe un byte en el registro de un dispositivo I2C.
 */
static esp_err_t i2c_escribir_registro(uint8_t dir, uint8_t reg, uint8_t valor)
{
	(void)dir; /* La dirección ya está en g_dev_imu                         */
	uint8_t buffer[2] = {reg, valor}; /* [registro, dato]                  */
	if (g_dev_imu == NULL) {
		return ESP_ERR_INVALID_STATE;
	}
	return i2c_master_transmit(g_dev_imu, buffer, 2, pdMS_TO_TICKS(100));
}

/**
 * @brief Lee N bytes consecutivos desde un registro de un dispositivo I2C.
 */
static esp_err_t i2c_leer_registros(uint8_t dir, uint8_t reg_inicio,
									uint8_t *buffer, size_t longitud)
{
	(void)dir; /* Usamos g_dev_imu directamente, con su dirección configurada */
	if (g_dev_imu == NULL) {
		return ESP_ERR_INVALID_STATE;
	}
	/* Write-then-read: enviar byte de registro, luego leer los datos       */
	return i2c_master_transmit_receive(g_dev_imu,
									   &reg_inicio, 1,
									   buffer, longitud,
									   pdMS_TO_TICKS(100));
}

/* ============================================================================
 *  INICIALIZACIÓN DEL IMU
 * ============================================================================ */

/**
 * @brief Detecta qué IMU está presente y lo inicializa.
 *
 * Intenta primero con QMI8658 (más común en relojes ESP32-S3 1.28").
 * Si falla, intenta MPU6050.
 */
static esp_err_t imu_detectar_e_inicializar(void)
{
	esp_err_t resultado;
	uint8_t   id = 0;
	#if !SOLO_QMI8658
	const uint8_t direcciones_mpu6050[] = {0x68, 0x69};
	#endif

	/* Configurar bus I2C con el nuevo driver maestro (ESP-IDF v5+)         */
	i2c_master_bus_config_t config_bus = {
		.i2c_port          = PUERTO_I2C,
		.sda_io_num        = PIN_I2C_SDA,
		.scl_io_num        = PIN_I2C_SCL,
		.clk_source        = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	resultado = i2c_new_master_bus(&config_bus, &g_bus_i2c);
	if (resultado != ESP_OK) {
		ESP_LOGE(ETIQUETA_IMU, "Error creando bus I2C: %s",
				 esp_err_to_name(resultado));
		return resultado;
	}

	/* --- Inicializar QMI8658 en 0x6B (dirección confirmada) --- */
	i2c_device_config_t config_dev_qmi = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address  = DIRECCION_QMI8658,
		.scl_speed_hz    = VELOCIDAD_I2C_HZ,
	};
	resultado = i2c_master_bus_add_device(g_bus_i2c, &config_dev_qmi, &g_dev_imu);
	if (resultado != ESP_OK) {
		ESP_LOGE(ETIQUETA_IMU, "Error registrando QMI8658 en bus I2C: %s",
				 esp_err_to_name(resultado));
		return resultado;
	}

	resultado = i2c_leer_registros(DIRECCION_QMI8658, QMI8658_REG_ID, &id, 1);
	if (resultado == ESP_OK && (id == QMI8658_ID_VAL || id == 0x06)) {
		ESP_LOGI(ETIQUETA_IMU, "QMI8658 detectado en 0x%02X (ID=0x%02X)",
				 DIRECCION_QMI8658, id);
		g_tipo_imu     = TIPO_IMU_QMI8658;
		g_escala_accel = QMI8658_ESCALA_ACCEL;
		g_escala_gyro  = QMI8658_ESCALA_GYRO;
		return qmi8658_inicializar();
	}

	ESP_LOGE(ETIQUETA_IMU,
			 "QMI8658 no responde en 0x6B (ID leido=0x%02X). Revisa pines SDA=%d SCL=%d.",
			 id, PIN_I2C_SDA, PIN_I2C_SCL);
	i2c_master_bus_rm_device(g_dev_imu);
	g_dev_imu = NULL;

	#if !SOLO_QMI8658
	/* --- Fallback MPU6050 (no aplica a esta placa) --- */
	#pragma message("SOLO_QMI8658=0: compilando soporte MPU6050 como fallback")
	/* --- Intentar con MPU6050 en direcciones posibles (0x68/0x69) --- */
	for (size_t i = 0; i < sizeof(direcciones_mpu6050); i++) {
		i2c_device_config_t config_dev_mpu = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address  = direcciones_mpu6050[i],
			.scl_speed_hz    = VELOCIDAD_I2C_HZ,
		};
		resultado = i2c_master_bus_add_device(g_bus_i2c, &config_dev_mpu, &g_dev_imu);
		if (resultado != ESP_OK) {
			continue;
		}

		resultado = i2c_leer_registros(direcciones_mpu6050[i], MPU6050_REG_WHO_AM_I, &id, 1);
		if (resultado == ESP_OK && (id == MPU6050_ID_VAL || id == 0x69)) {
			ESP_LOGI(ETIQUETA_IMU,
					 "MPU6050 detectado en 0x%02X (WHO_AM_I=0x%02X)",
					 direcciones_mpu6050[i], id);
			g_tipo_imu     = TIPO_IMU_MPU6050;
			g_escala_accel = MPU6050_ESCALA_ACCEL;
			g_escala_gyro  = MPU6050_ESCALA_GYRO;
			return mpu6050_inicializar();
		}

		ESP_LOGW(ETIQUETA_IMU,
				 "No es MPU6050 en 0x%02X (WHO_AM_I=0x%02X, err=%s)",
				 direcciones_mpu6050[i], id, esp_err_to_name(resultado));
		i2c_master_bus_rm_device(g_dev_imu);
		g_dev_imu = NULL;
	}

	ESP_LOGE(ETIQUETA_IMU,
			 "No se encontro IMU compatible. Revisa pines I2C y direccion del sensor.");
	return ESP_ERR_NOT_FOUND;
	#endif /* !SOLO_QMI8658 */

	return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Inicializa el QMI8658.
 *
 * Configura:
 *  - CTRL1: modo I2C, autoincremento habilitado, little-endian
 *  - CTRL2: acelerómetro +-4g a 250 Hz (modo low noise)
 *  - CTRL3: giroscopio +-512 dps a 250 Hz
 *  - CTRL7: habilita acelerómetro + giroscopio simultáneamente
 */
static esp_err_t qmi8658_inicializar(void)
{
	esp_err_t r;

	/* CTRL1: SPI 4 wire, autoincremento ON, little-endian                  */
	r = i2c_escribir_registro(DIRECCION_QMI8658, QMI8658_REG_CTRL1, 0x60);
	if (r != ESP_OK) return r;

	/* CTRL2: acelerómetro +-8g, ODR=1000 Hz, self-test desactivado         */
	r = i2c_escribir_registro(DIRECCION_QMI8658, QMI8658_REG_CTRL2, 0x23);
	if (r != ESP_OK) return r;

	/* CTRL3: giroscopio +-512 dps, ODR=1000 Hz, self-test desactivado      */
	r = i2c_escribir_registro(DIRECCION_QMI8658, QMI8658_REG_CTRL3, 0x43);
	if (r != ESP_OK) return r;

	/* CTRL7: habilitar acelerómetro (bit 0) + giroscopio (bit 1)           */
	r = i2c_escribir_registro(DIRECCION_QMI8658, QMI8658_REG_CTRL7, 0x03);
	if (r != ESP_OK) return r;

	ESP_LOGI(ETIQUETA_IMU, "QMI8658 inicializado: +-8g / +-512dps / 1000Hz");
	return ESP_OK;
}

#if !SOLO_QMI8658
/**
 * @brief Inicializa el MPU6050.
 *
 * Configura:
 *  - PWR_MGMT_1: sale de sleep, usa PLL del giroscopio X como reloj
 *  - CONFIG: filtro paso-bajo DLPF ~94 Hz
 *  - ACCEL_CONFIG: +-2g
 *  - GYRO_CONFIG:  +-250 dps
 */
static esp_err_t mpu6050_inicializar(void)
{
	esp_err_t r;

	/* Salir del modo sleep y usar PLL con referencia del giroscopio X      */
	r = i2c_escribir_registro(DIRECCION_MPU6050, MPU6050_REG_PWR_MGMT_1, 0x01);
	if (r != ESP_OK) return r;

	/* DLPF: ancho de banda ~94 Hz accel / 98 Hz gyro (reduce vibraciones) */
	r = i2c_escribir_registro(DIRECCION_MPU6050, MPU6050_REG_CONFIG, 0x02);
	if (r != ESP_OK) return r;

	/* Acelerómetro: rango +-2g (sin bits de escala adicionales)            */
	r = i2c_escribir_registro(DIRECCION_MPU6050, MPU6050_REG_ACCEL_CFG, 0x00);
	if (r != ESP_OK) return r;

	/* Giroscopio: rango +-250 dps                                          */
	r = i2c_escribir_registro(DIRECCION_MPU6050, MPU6050_REG_GYRO_CFG, 0x00);
	if (r != ESP_OK) return r;

	ESP_LOGI(ETIQUETA_IMU, "MPU6050 inicializado: +-2g / +-250dps");
	return ESP_OK;
}
#endif

/**
 * @brief Lee los seis ejes del IMU activo y devuelve valores en g y deg/s.
 */
static esp_err_t imu_leer_datos_brutos(datos_imu_t *datos)
{
	uint8_t  buffer[14]; /* Máximo: 14 bytes para MPU6050 (accel+temp+gyro) */
	esp_err_t r;
	int16_t bruto_ax, bruto_ay, bruto_az;
	int16_t bruto_gx, bruto_gy, bruto_gz;

	if (g_tipo_imu == TIPO_IMU_QMI8658) {
		/* QMI8658: 12 bytes continuos desde AX_L (little-endian)           */
		r = i2c_leer_registros(DIRECCION_QMI8658, QMI8658_REG_AX_L, buffer, 12);
		if (r != ESP_OK) return r;

		/* Combinar bytes low/high en entero con signo (little-endian)      */
		bruto_ax = (int16_t)((uint16_t)buffer[1]  << 8 | buffer[0]);
		bruto_ay = (int16_t)((uint16_t)buffer[3]  << 8 | buffer[2]);
		bruto_az = (int16_t)((uint16_t)buffer[5]  << 8 | buffer[4]);
		bruto_gx = (int16_t)((uint16_t)buffer[7]  << 8 | buffer[6]);
		bruto_gy = (int16_t)((uint16_t)buffer[9]  << 8 | buffer[8]);
		bruto_gz = (int16_t)((uint16_t)buffer[11] << 8 | buffer[10]);

	} else {
		/* MPU6050: 14 bytes desde ACCEL_XOUT_H (big-endian):
		   accel_x(2) accel_y(2) accel_z(2) temp(2) gyro_x(2) gyro_y(2) gyro_z(2) */
		r = i2c_leer_registros(DIRECCION_MPU6050, MPU6050_REG_ACCEL_XOUT_H,
							   buffer, 14);
		if (r != ESP_OK) return r;

		/* Combinar bytes high/low (big-endian del MPU6050)                 */
		bruto_ax = (int16_t)((uint16_t)buffer[0]  << 8 | buffer[1]);
		bruto_ay = (int16_t)((uint16_t)buffer[2]  << 8 | buffer[3]);
		bruto_az = (int16_t)((uint16_t)buffer[4]  << 8 | buffer[5]);
		/* buffer[6] y buffer[7] = temperatura (no se usa)                  */
		bruto_gx = (int16_t)((uint16_t)buffer[8]  << 8 | buffer[9]);
		bruto_gy = (int16_t)((uint16_t)buffer[10] << 8 | buffer[11]);
		bruto_gz = (int16_t)((uint16_t)buffer[12] << 8 | buffer[13]);
	}

	/* Convertir datos brutos a unidades físicas usando la escala configurada */
#if INTERCAMBIAR_ACCEL_GYRO
	datos->accel_x = (float)bruto_gx / g_escala_gyro;  /* [g]  (modo prueba) */
	datos->accel_y = (float)bruto_gy / g_escala_gyro;  /* [g]  (modo prueba) */
	datos->accel_z = (float)bruto_gz / g_escala_gyro;  /* [g]  (modo prueba) */
	datos->gyro_x  = (float)bruto_ax / g_escala_accel; /* [deg/s] (modo prueba) */
	datos->gyro_y  = (float)bruto_ay / g_escala_accel; /* [deg/s] (modo prueba) */
	datos->gyro_z  = (float)bruto_az / g_escala_accel; /* [deg/s] (modo prueba) */
#else
	datos->accel_x = (float)bruto_ax / g_escala_accel; /* [g]               */
	datos->accel_y = (float)bruto_ay / g_escala_accel; /* [g]               */
	datos->accel_z = (float)bruto_az / g_escala_accel; /* [g]               */
	datos->gyro_x  = (float)bruto_gx / g_escala_gyro;  /* [deg/s]           */
	datos->gyro_y  = (float)bruto_gy / g_escala_gyro;  /* [deg/s]           */
	datos->gyro_z  = (float)bruto_gz / g_escala_gyro;  /* [deg/s]           */
#endif

	return ESP_OK;
}

/* ============================================================================
 *  FILTRO COMPLEMENTARIO
 * ============================================================================ */

/**
 * @brief Aplica el Filtro Complementario para fusionar acelerómetro y giroscopio.
 *
 * TEORIA DEL FILTRO COMPLEMENTARIO:
 * ==================================
 * El acelerómetro calcula ángulos absolutos pero es muy ruidoso en movimiento.
 * El giroscopio integra velocidad angular y es preciso a corto plazo, pero
 * acumula deriva (drift) a lo largo del tiempo.
 *
 * La fusión complementaria combina ambos:
 *   angulo_nuevo = alfa * (angulo_prev + gyro * dt) + (1-alfa) * angulo_accel
 *
 * donde alfa = 0.98 da un 98% de confianza al giroscopio (instantáneo y suave)
 * y 2% al acelerómetro (corrección lenta de deriva).
 *
 * CALCULOS DE ANGULO DESDE EL ACELEROMETRO:
 * ==========================================
 * Roll  (alabeo)  = atan2(ay, az)          [rotacion lateral]
 * Pitch (cabeceo) = atan2(-ax, sqrt(ay^2+az^2))  [inclinacion frontal]
 *
 * @param medicion      Lecturas del IMU en unidades fisicas (g, deg/s).
 * @param angulos_prev  Angulos anteriores; se actualizan in-place.
 * @param dt            Intervalo de tiempo transcurrido [segundos].
 */
static void filtro_complementario(const datos_imu_t *medicion,
								  angulos_t *angulos_prev,
								  float dt)
{
	/* --- Paso 1: Calcular ángulos del acelerómetro --- */

	/* Alabeo (Roll): inclina lateralmente
	   Plano = 0 deg, derecha positivo, izquierda negativo               */
	float alabeo_accel = atan2f(medicion->accel_y, medicion->accel_z)
						 * (180.0f / (float)M_PI);

	/* Cabeceo (Pitch): inclina hacia adelante/atrás
	   Usamos sqrt(ay^2+az^2) como denominador para estabilidad cuando
	   el dispositivo está casi vertical                                  */
	float cabeceo_accel = atan2f(
		-medicion->accel_x,
		sqrtf(medicion->accel_y * medicion->accel_y +
			  medicion->accel_z * medicion->accel_z)
	) * (180.0f / (float)M_PI);

	/* --- Paso 2: Predicción por integración del giroscopio --- */
	/* angulo += velocidad_angular * tiempo => acumulacion de giro        */
	float alabeo_gyro  = angulos_prev->alabeo  + medicion->gyro_x * dt;
	float cabeceo_gyro = angulos_prev->cabeceo + medicion->gyro_y * dt;

	/* --- Paso 3: Fusión complementaria --- */
	/* 98% giroscopio (preciso a corto) + 2% acelerómetro (corrige deriva) */
	angulos_prev->alabeo  = FACTOR_COMP_GYRO * alabeo_gyro
						  + FACTOR_COMP_ACCEL * alabeo_accel;
	angulos_prev->cabeceo = FACTOR_COMP_GYRO * cabeceo_gyro
						  + FACTOR_COMP_ACCEL * cabeceo_accel;
}

/* ============================================================================
 *  FUNCIONES DE LA PANTALLA GC9A01
 * ============================================================================ */

/**
 * @brief Envía un byte de COMANDO a la pantalla (pin D/C = bajo).
 */
static void pantalla_enviar_comando(uint8_t comando)
{
	gpio_set_level(PIN_PANTALLA_DC, 0); /* D/C=0 indica COMANDO al GC9A01    */
	spi_transaction_t t = {
		.length    = 8,
		.tx_buffer = &comando,
	};
	spi_device_polling_transmit(g_handle_spi, &t);
}

/**
 * @brief Envía un byte de DATO a la pantalla (pin D/C = alto).
 */
static void pantalla_enviar_dato(uint8_t dato)
{
	gpio_set_level(PIN_PANTALLA_DC, 1); /* D/C=1 indica DATO al GC9A01       */
	spi_transaction_t t = {
		.length    = 8,
		.tx_buffer = &dato,
	};
	spi_device_polling_transmit(g_handle_spi, &t);
}

/**
 * @brief Inicializa la pantalla GC9A01 1.28" circular.
 *
 * Secuencia: reset hardware + comandos de configuración del controlador.
 */
static void pantalla_inicializar(void)
{
	/* Configurar pines de control como salida digital                      */
	gpio_config_t cfg_gpio = {
		.pin_bit_mask = (1ULL << PIN_PANTALLA_DC)  |
						(1ULL << PIN_PANTALLA_RST) |
						(1ULL << PIN_PANTALLA_BL),
		.mode         = GPIO_MODE_OUTPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&cfg_gpio);

	/* Inicializar el bus SPI para la pantalla                              */
	spi_bus_config_t cfg_bus = {
		.mosi_io_num     = PIN_SPI_MOSI,
		.miso_io_num     = PIN_SPI_MISO,
		.sclk_io_num     = PIN_SPI_SCLK,
		.quadwp_io_num   = -1,
		.quadhd_io_num   = -1,
		.max_transfer_sz = ANCHO_PANTALLA * ALTO_PANTALLA * 2,
	};
	spi_bus_initialize(HOST_SPI, &cfg_bus, SPI_DMA_CH_AUTO);

	/* Añadir el dispositivo pantalla al bus SPI                            */
	spi_device_interface_config_t cfg_dev = {
		.clock_speed_hz = VELOCIDAD_SPI_HZ,
		.mode           = 0,            /* CPOL=0, CPHA=0                   */
		.spics_io_num   = PIN_SPI_CS,
		.queue_size     = 7,
		.pre_cb         = NULL,
	};
	spi_bus_add_device(HOST_SPI, &cfg_dev, &g_handle_spi);

	/* Reset hardware de la pantalla (pulso bajo mínimo 10 ms)             */
	gpio_set_level(PIN_PANTALLA_BL,  0); /* Backlight apagado durante init  */
	gpio_set_level(PIN_PANTALLA_RST, 0); /* RST activo (bajo)               */
	vTaskDelay(pdMS_TO_TICKS(10));
	gpio_set_level(PIN_PANTALLA_RST, 1); /* RST liberado                    */
	vTaskDelay(pdMS_TO_TICKS(120));      /* GC9A01 necesita 120 ms después  */

	/* ---- Secuencia de inicialización del GC9A01 ---- */
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

	/* MADCTL: rotación 180° sin espejo (mantiene mapeo del panel) + BGR    */
	pantalla_enviar_comando(0x36); pantalla_enviar_dato(0xD8);

	/* Formato de pixel: 16 bits (RGB565)                                   */
	pantalla_enviar_comando(0x3A); pantalla_enviar_dato(0x05);

	pantalla_enviar_comando(0x90);
	pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x08);
	pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x08);
	pantalla_enviar_comando(0xBD); pantalla_enviar_dato(0x06);
	pantalla_enviar_comando(0xBC); pantalla_enviar_dato(0x00);
	pantalla_enviar_comando(0xFF);
	pantalla_enviar_dato(0x60); pantalla_enviar_dato(0x01); pantalla_enviar_dato(0x04);
	pantalla_enviar_comando(0xC3); pantalla_enviar_dato(0x13);
	pantalla_enviar_comando(0xC4); pantalla_enviar_dato(0x13);
	pantalla_enviar_comando(0xC9); pantalla_enviar_dato(0x22);
	pantalla_enviar_comando(0xBE); pantalla_enviar_dato(0x11);
	pantalla_enviar_comando(0xE1); pantalla_enviar_dato(0x10); pantalla_enviar_dato(0x0E);
	pantalla_enviar_comando(0xDF);
	pantalla_enviar_dato(0x21); pantalla_enviar_dato(0x0C); pantalla_enviar_dato(0x02);
	pantalla_enviar_comando(0xF0);
	pantalla_enviar_dato(0x45); pantalla_enviar_dato(0x09); pantalla_enviar_dato(0x08);
	pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x26); pantalla_enviar_dato(0x2A);
	pantalla_enviar_comando(0xF1);
	pantalla_enviar_dato(0x43); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x72);
	pantalla_enviar_dato(0x36); pantalla_enviar_dato(0x37); pantalla_enviar_dato(0x6F);
	pantalla_enviar_comando(0xF2);
	pantalla_enviar_dato(0x45); pantalla_enviar_dato(0x09); pantalla_enviar_dato(0x08);
	pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x26); pantalla_enviar_dato(0x2A);
	pantalla_enviar_comando(0xF3);
	pantalla_enviar_dato(0x43); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x72);
	pantalla_enviar_dato(0x36); pantalla_enviar_dato(0x37); pantalla_enviar_dato(0x6F);
	pantalla_enviar_comando(0xED); pantalla_enviar_dato(0x1B); pantalla_enviar_dato(0x0B);
	pantalla_enviar_comando(0xAE); pantalla_enviar_dato(0x77);
	pantalla_enviar_comando(0xCD); pantalla_enviar_dato(0x63);
	pantalla_enviar_comando(0x70);
	pantalla_enviar_dato(0x07); pantalla_enviar_dato(0x07); pantalla_enviar_dato(0x04);
	pantalla_enviar_dato(0x0E); pantalla_enviar_dato(0x0F); pantalla_enviar_dato(0x09);
	pantalla_enviar_dato(0x07); pantalla_enviar_dato(0x08); pantalla_enviar_dato(0x03);
	pantalla_enviar_comando(0xE8); pantalla_enviar_dato(0x34);
	pantalla_enviar_comando(0x62);
	pantalla_enviar_dato(0x18); pantalla_enviar_dato(0x0D); pantalla_enviar_dato(0x71);
	pantalla_enviar_dato(0xED); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x70);
	pantalla_enviar_dato(0x18); pantalla_enviar_dato(0x0F); pantalla_enviar_dato(0x71);
	pantalla_enviar_dato(0xEF); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x70);
	pantalla_enviar_comando(0x63);
	pantalla_enviar_dato(0x18); pantalla_enviar_dato(0x11); pantalla_enviar_dato(0x71);
	pantalla_enviar_dato(0xF1); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x70);
	pantalla_enviar_dato(0x18); pantalla_enviar_dato(0x13); pantalla_enviar_dato(0x71);
	pantalla_enviar_dato(0xF3); pantalla_enviar_dato(0x70); pantalla_enviar_dato(0x70);
	pantalla_enviar_comando(0x64);
	pantalla_enviar_dato(0x28); pantalla_enviar_dato(0x29); pantalla_enviar_dato(0xF1);
	pantalla_enviar_dato(0x01); pantalla_enviar_dato(0xF1); pantalla_enviar_dato(0x00);
	pantalla_enviar_dato(0x07);
	pantalla_enviar_comando(0x66);
	pantalla_enviar_dato(0x3C); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0xCD);
	pantalla_enviar_dato(0x67); pantalla_enviar_dato(0x45); pantalla_enviar_dato(0x45);
	pantalla_enviar_dato(0x10); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00);
	pantalla_enviar_dato(0x00);
	pantalla_enviar_comando(0x67);
	pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x3C); pantalla_enviar_dato(0x00);
	pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x01);
	pantalla_enviar_dato(0x54); pantalla_enviar_dato(0x10); pantalla_enviar_dato(0x32);
	pantalla_enviar_dato(0x98);
	pantalla_enviar_comando(0x74);
	pantalla_enviar_dato(0x10); pantalla_enviar_dato(0x85); pantalla_enviar_dato(0x80);
	pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x00); pantalla_enviar_dato(0x4E);
	pantalla_enviar_dato(0x00);
	pantalla_enviar_comando(0x98); pantalla_enviar_dato(0x3E); pantalla_enviar_dato(0x07);
	pantalla_enviar_comando(0x35); /* Tearing effect activo                  */
	pantalla_enviar_comando(0x21); /* Inversión de display (mejora colores)  */
	pantalla_enviar_comando(0x11); /* Sleep Out                              */
	vTaskDelay(pdMS_TO_TICKS(120));
	pantalla_enviar_comando(0x29); /* Display ON                             */
	vTaskDelay(pdMS_TO_TICKS(20));

	/* Encender la retroiluminación                                         */
	gpio_set_level(PIN_PANTALLA_BL, 1);
	ESP_LOGI(ETIQUETA_DISP, "GC9A01 1.28\" inicializada y encendida");
}

/**
 * @brief Define la ventana de escritura activa en la pantalla.
 *
 * Todos los píxeles enviados después se escriben dentro de este rectángulo.
 * Comando CASET (0x2A) para columnas y RASET (0x2B) para filas.
 */
static void pantalla_fijar_ventana(uint16_t x0, uint16_t y0,
								   uint16_t x1, uint16_t y1)
{
	pantalla_enviar_comando(0x2A);                    /* CASET: fijar columnas */
	pantalla_enviar_dato((uint8_t)(x0 >> 8));
	pantalla_enviar_dato((uint8_t)(x0 & 0xFF));
	pantalla_enviar_dato((uint8_t)(x1 >> 8));
	pantalla_enviar_dato((uint8_t)(x1 & 0xFF));

	pantalla_enviar_comando(0x2B);                    /* RASET: fijar filas    */
	pantalla_enviar_dato((uint8_t)(y0 >> 8));
	pantalla_enviar_dato((uint8_t)(y0 & 0xFF));
	pantalla_enviar_dato((uint8_t)(y1 >> 8));
	pantalla_enviar_dato((uint8_t)(y1 & 0xFF));

	pantalla_enviar_comando(0x2C);                    /* RAMWR: inicio escritura */
}

/**
 * @brief Rellena un rectángulo con un color sólido RGB565.
 *
 * Optimización: pre-rellena una fila en s_linea_spi y la envía en una sola
 * transacción SPI (en vez de 2 txns por pixel). Reducción de ~300x en txns.
 */
static void pantalla_rellenar_rect(uint16_t x0, uint16_t y0,
							   uint16_t x1, uint16_t y1,
							   uint16_t color)
{
	uint16_t ancho = x1 - x0 + 1;
	uint16_t alto  = y1 - y0 + 1;
	uint8_t  bh    = (uint8_t)(color >> 8);
	uint8_t  bl    = (uint8_t)(color & 0xFF);

	/* Rellenar buffer de línea con el color (big-endian RGB565)             */
	for (uint16_t i = 0; i < ancho; i++) {
		s_linea_spi[i * 2]     = bh;
		s_linea_spi[i * 2 + 1] = bl;
	}

	pantalla_fijar_ventana(x0, y0, x1, y1);
	gpio_set_level(PIN_PANTALLA_DC, 1);

	/* Una transacción por fila: ancho×16 bits en una sola llamada SPI       */
	spi_transaction_t t = {
		.length    = (size_t)ancho * 16,
		.tx_buffer = s_linea_spi,
	};
	for (uint16_t fila = 0; fila < alto; fila++) {
		spi_device_polling_transmit(g_handle_spi, &t);
	}
}

/* ============================================================================
 *  FUENTE DE TEXTO 5x7
 * ============================================================================ */

/**
 * @brief Dibuja un carácter ASCII con la fuente de mapa de bits 5x7.
 *
 * Optimización: construye el mapa de bits completo del carácter en s_char_buf
 * y lo envía en una sola transacción SPI (vs. 140 llamadas a pintar_pixel).
 * Los píxeles se generan en orden fila-mayor (row-major) para el GC9A01.
 */
static void pantalla_dibujar_caracter(int16_t x, int16_t y, char c,
								  uint16_t color_fg, uint16_t color_bg,
								  uint8_t escala)
{
	if (c < 32 || c > 126) c = '?';
	const uint8_t *columnas = fuente_5x7[(uint8_t)(c - 32)];
	uint16_t ancho = (uint16_t)(5 * escala);
	uint16_t alto  = (uint16_t)(7 * escala);

	/* Construir buffer en orden fila-mayor (como espera el GC9A01)          */
	uint16_t idx = 0;
	for (uint8_t py = 0; py < 7 * escala; py++) {
		uint8_t fila = py / escala;
		for (uint8_t px = 0; px < 5 * escala; px++) {
			uint8_t  col   = px / escala;
			uint16_t color = (columnas[col] & (1u << fila)) ? color_fg : color_bg;
			s_char_buf[idx++] = (uint8_t)(color >> 8);
			s_char_buf[idx++] = (uint8_t)(color & 0xFF);
		}
	}

	/* Fijar ventana del carácter y enviar todos los píxeles de una vez      */
	if (x >= 0 && y >= 0 &&
		(uint16_t)x + ancho <= ANCHO_PANTALLA &&
		(uint16_t)y + alto  <= ALTO_PANTALLA) {
		pantalla_fijar_ventana((uint16_t)x, (uint16_t)y,
							  (uint16_t)(x + ancho - 1),
							  (uint16_t)(y + alto  - 1));
		gpio_set_level(PIN_PANTALLA_DC, 1);
		spi_transaction_t t = {
			.length    = (size_t)ancho * alto * 16,
			.tx_buffer = s_char_buf,
		};
		spi_device_polling_transmit(g_handle_spi, &t);
	}
}

/**
 * @brief Dibuja una cadena de texto en la pantalla.
 *
 * Los caracteres se espacian 6*escala pixeles (5 de carácter + 1 de margen).
 */
static void pantalla_dibujar_texto(int16_t x, int16_t y, const char *texto,
								   uint16_t color_fg, uint16_t color_bg,
								   uint8_t escala)
{
	int16_t cursor_x = x;
	while (*texto) {
		pantalla_dibujar_caracter(cursor_x, y, *texto, color_fg, color_bg, escala);
		cursor_x += (int16_t)(6 * escala); /* Avanzar: 5 px carácter + 1 px espacio */
		texto++;
	}
}

static void touch_inicializar(void)
{
	gpio_config_t cfg_touch_int = {
		.pin_bit_mask = (1ULL << PIN_TOUCH_INT),
		.mode         = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_NEGEDGE,
	};
	gpio_config(&cfg_touch_int);

	gpio_config_t cfg_touch_rst = {
		.pin_bit_mask = (1ULL << PIN_TOUCH_RST),
		.mode         = GPIO_MODE_OUTPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&cfg_touch_rst);

	gpio_set_level(PIN_TOUCH_RST, 1);
	vTaskDelay(pdMS_TO_TICKS(50));
	gpio_set_level(PIN_TOUCH_RST, 0);
	vTaskDelay(pdMS_TO_TICKS(5));
	gpio_set_level(PIN_TOUCH_RST, 1);
	vTaskDelay(pdMS_TO_TICKS(50));

	gpio_install_isr_service(0);
	gpio_isr_handler_add(PIN_TOUCH_INT, touch_isr_handler, NULL);
	ESP_LOGI(ETIQUETA_MAIN, "Touch CST816S inicializado (IRQ GPIO %d)", PIN_TOUCH_INT);
}

static float bateria_leer_voltios(void)
{
	int lectura_mv = 0;
	if (g_adc_bateria == NULL) {
		return 0.0f;
	}
	if (adc_oneshot_read(g_adc_bateria, CANAL_BATERIA_ADC, &lectura_mv) != ESP_OK) {
		return 0.0f;
	}
	return ((float)lectura_mv / 1000.0f) * FACTOR_BATERIA;
}

/* ============================================================================
 *  TAREAS FREERTOS
 * ============================================================================ */

/**
 * @brief Tarea_Lectura_IMU: periodo 20 ms (50 Hz).
 *
 * Lee el IMU, aplica el Filtro Complementario y actualiza g_angulos
 * bajo protección del mutex para evitar condiciones de carrera con
 * la tarea de display que lee esos mismos valores.
 *
 * Prioridad: 5 (alta, para cumplir el ciclo de 20 ms con determinismo).
 * Core: 0 (dedicado a procesamiento de sensores).
 */
static void tarea_lectura_imu(void *parametro)
{
	(void)parametro;

	datos_imu_t medicion = {0};                   /* Lectura física del IMU */
	angulos_t   angulos_locales = {0.0f, 0.0f};   /* Ángulos estimados      */
	TickType_t  instante_anterior = xTaskGetTickCount();

	ESP_LOGI(ETIQUETA_IMU, "Tarea_Lectura_IMU iniciada (20ms / 50Hz)");

	for (;;) {
		/* Leer datos brutos del sensor IMU                                 */
		esp_err_t resultado = imu_leer_datos_brutos(&medicion);

		if (resultado == ESP_OK) {
			/* Estimar ángulos reales en grados con filtro complementario */
			filtro_complementario(&medicion, &angulos_locales, PERIODO_IMU_SEG);

			/* Publicar última muestra IMU para la tarea de pantalla         */
			if (xSemaphoreTake(g_mutex_datos_imu, pdMS_TO_TICKS(5)) == pdTRUE) {
				g_datos_imu = medicion;
				g_angulos   = angulos_locales;
				xSemaphoreGive(g_mutex_datos_imu);
			}
		} else {
			ESP_LOGW(ETIQUETA_IMU, "Error de lectura IMU: %s",
					 esp_err_to_name(resultado));
		}

		/* Esperar hasta completar el período de 20 ms exactos              */
		/* vTaskDelayUntil compensa el tiempo que tomó la tarea en ejecutarse */
		vTaskDelayUntil(&instante_anterior, pdMS_TO_TICKS(20));
	}
}

/**
 * @brief Tarea_Display: periodo 50 ms (20 FPS).
 *
 * Lee una copia de los ángulos protegida por mutex y actualiza la
 * interfaz gráfica del nivel de burbuja. Corre en el núcleo 1 para
 * no competir con la tarea IMU del núcleo 0.
 *
 * Prioridad: 3 (menor que la tarea IMU para no bloquear la lectura).
 * Core: 1 (dedicado al renderizado gráfico).
 */
static void tarea_display(void *parametro)
{
	(void)parametro;

	datos_imu_t datos_copia = {0};                 /* Copia local de IMU     */
	angulos_t angulos_copia = {0};                 /* Copia local de ángulos */
	char linea[24];
	bool pantalla_encendida = true;
	int64_t ultimo_toque_ms = esp_timer_get_time() / 1000;
	TickType_t instante_anterior = xTaskGetTickCount();

	/* Fondo tipo dashboard, similar al sketch Arduino                      */
	pantalla_rellenar_rect(0,   0, 239,  47, 0xF410);
	pantalla_rellenar_rect(0,  47, 239, 120, 0x4F30);
	pantalla_rellenar_rect(0, 120, 239, 195, 0xAD55);
	pantalla_rellenar_rect(0, 195, 239, 239, 0x2595);

	pantalla_dibujar_texto(82,  16, "IMU TELEMETRY", COLOR_BLANCO, 0xF410, 1);
	pantalla_dibujar_texto(20,  52, "ROLL=",        COLOR_BLANCO, 0x4F30, 1);
	pantalla_dibujar_texto(20,  68, "PITCH=",       COLOR_BLANCO, 0x4F30, 1);
	pantalla_dibujar_texto(20,  90, "ACC_X =",      COLOR_BLANCO, 0x4F30, 1);
	pantalla_dibujar_texto(20, 106, "ACC_Y =",      COLOR_BLANCO, 0x4F30, 1);
	pantalla_dibujar_texto(20, 122, "ACC_Z =",      COLOR_BLANCO, 0xAD55, 1);
	pantalla_dibujar_texto(20, 152, "GYR_X =",      COLOR_BLANCO, 0xAD55, 1);
	pantalla_dibujar_texto(20, 168, "GYR_Y =",      COLOR_BLANCO, 0xAD55, 1);
	pantalla_dibujar_texto(20, 184, "GYR_Z =",      COLOR_BLANCO, 0xAD55, 1);
	pantalla_dibujar_texto(57, 208, "BAT(V)=",      COLOR_BLANCO, 0x2595, 2);

	ESP_LOGI(ETIQUETA_DISP, "Tarea_Display iniciada (50ms / 20FPS)");

	for (;;) {
		int64_t ahora_ms = esp_timer_get_time() / 1000;

		if (g_touch_evento) {
			g_touch_evento = false;
			ultimo_toque_ms = ahora_ms;
			if (!pantalla_encendida) {
				gpio_set_level(PIN_PANTALLA_BL, 1);
				pantalla_encendida = true;
				ESP_LOGI(ETIQUETA_DISP, "Pantalla encendida por toque");
			}
		}

		if (pantalla_encendida && ((ahora_ms - ultimo_toque_ms) > TIEMPO_APAGADO_MS)) {
			gpio_set_level(PIN_PANTALLA_BL, 0);
			pantalla_encendida = false;
			ESP_LOGI(ETIQUETA_DISP, "Pantalla apagada por inactividad");
		}

		if (!pantalla_encendida) {
			vTaskDelayUntil(&instante_anterior, pdMS_TO_TICKS(50));
			continue;
		}

		/* Tomar copia de la muestra IMU de forma segura                    */
		if (xSemaphoreTake(g_mutex_datos_imu, pdMS_TO_TICKS(10)) == pdTRUE) {
			datos_copia = g_datos_imu;
			angulos_copia = g_angulos;
			xSemaphoreGive(g_mutex_datos_imu);
		}

		/* Limpiar solo celdas de valores (mismo ancho en todas las filas)  */
		pantalla_rellenar_rect(112,  52, 220,  60, 0x4F30); /* ROLL  */
		pantalla_rellenar_rect(112,  68, 220,  76, 0x4F30); /* PITCH */
		pantalla_rellenar_rect(112,  90, 220,  98, 0x4F30); /* ACC_X */
		pantalla_rellenar_rect(112, 106, 220, 114, 0x4F30); /* ACC_Y */
		pantalla_rellenar_rect(112, 122, 220, 130, 0xAD55); /* ACC_Z */
		pantalla_rellenar_rect(112, 152, 220, 160, 0xAD55); /* GYR_X */
		pantalla_rellenar_rect(112, 168, 220, 176, 0xAD55); /* GYR_Y */
		pantalla_rellenar_rect(112, 184, 220, 192, 0xAD55); /* GYR_Z */
		pantalla_rellenar_rect(130, 200, 220, 214, 0x2595); /* BAT   */

		snprintf(linea, sizeof(linea), "%+6.1f", (double)angulos_copia.alabeo);
		pantalla_dibujar_texto(112, 52, linea, COLOR_BLANCO, 0x4F30, 1);
		snprintf(linea, sizeof(linea), "%+6.1f", (double)angulos_copia.cabeceo);
		pantalla_dibujar_texto(112, 68, linea, COLOR_BLANCO, 0x4F30, 1);

		snprintf(linea, sizeof(linea), "%+1.3f", (double)datos_copia.accel_x);
		pantalla_dibujar_texto(112, 90, linea, COLOR_BLANCO, 0x4F30, 1);
		snprintf(linea, sizeof(linea), "%+1.3f", (double)datos_copia.accel_y);
		pantalla_dibujar_texto(112, 106, linea, COLOR_BLANCO, 0x4F30, 1);
		snprintf(linea, sizeof(linea), "%+1.3f", (double)datos_copia.accel_z);
		pantalla_dibujar_texto(112, 122, linea, COLOR_BLANCO, 0xAD55, 1);

		snprintf(linea, sizeof(linea), "%+06.1f", (double)datos_copia.gyro_x);
		pantalla_dibujar_texto(112, 152, linea, COLOR_BLANCO, 0xAD55, 1);
		snprintf(linea, sizeof(linea), "%+06.1f", (double)datos_copia.gyro_y);
		pantalla_dibujar_texto(112, 168, linea, COLOR_BLANCO, 0xAD55, 1);
		snprintf(linea, sizeof(linea), "%+06.1f", (double)datos_copia.gyro_z);
		pantalla_dibujar_texto(112, 184, linea, COLOR_BLANCO, 0xAD55, 1);

		snprintf(linea, sizeof(linea), "%1.2f", (double)bateria_leer_voltios());
		pantalla_dibujar_texto(130, 200, linea, COLOR_BLANCO, 0x2595, 2);

		/* Período fijo de 50 ms (20 FPS)                                   */
		vTaskDelayUntil(&instante_anterior, pdMS_TO_TICKS(50));
	}
}

/* ============================================================================
 *  PUNTO DE ENTRADA PRINCIPAL
 * ============================================================================ */

/**
 * @brief Punto de entrada de la aplicación ESP-IDF.
 *
 * Orden de inicialización:
 *  1. CPU a 80 MHz (balance potencia/rendimiento para animación fluida)
 *  2. Crear el Mutex de protección de ángulos
 *  3. Detectar e inicializar el IMU por I2C
 *  4. Inicializar la pantalla GC9A01 por SPI
 *  5. Lanzar Tarea_Lectura_IMU y Tarea_Display
 */
void app_main(void)
{
	ESP_LOGI(ETIQUETA_MAIN, "=== Nivel de Burbuja ESP32-S3 1.28\" ===");
	ESP_LOGI(ETIQUETA_MAIN, "Iniciando sistema...");

	/* ------------------------------------------------------------------ */
	/*  1. Configurar la CPU a 80 MHz para ahorro de energía               */
	/*     La animación a 20 FPS es fluida incluso a 80 MHz               */
	/* ------------------------------------------------------------------ */
	esp_pm_config_t config_pm = {
		.max_freq_mhz       = 80,
		.min_freq_mhz       = 40,
		.light_sleep_enable = false, /* Desactivar: latencia incompatible con SPI */
	};
	esp_err_t r_pm = esp_pm_configure(&config_pm);
	if (r_pm != ESP_OK) {
		ESP_LOGW(ETIQUETA_MAIN, "Gestion de potencia no disponible: %s",
				 esp_err_to_name(r_pm));
	} else {
		ESP_LOGI(ETIQUETA_MAIN, "CPU configurada a 80 MHz");
	}

	/* ------------------------------------------------------------------ */
	/*  2. Crear el Mutex para proteger la última muestra IMU              */
	/* ------------------------------------------------------------------ */
	g_mutex_datos_imu = xSemaphoreCreateMutex();
	if (g_mutex_datos_imu == NULL) {
		ESP_LOGE(ETIQUETA_MAIN, "Fallo al crear el mutex. Sistema detenido.");
		return;
	}
	ESP_LOGI(ETIQUETA_MAIN, "Mutex de datos IMU creado");

	/* ------------------------------------------------------------------ */
	/*  3. Detectar e inicializar el IMU (QMI8658 o MPU6050)               */
	/* ------------------------------------------------------------------ */
	esp_err_t r_imu = imu_detectar_e_inicializar();
	if (r_imu != ESP_OK) {
		/* Continuar con la pantalla aunque no haya IMU (mostrará 0.0°)   */
		ESP_LOGW(ETIQUETA_MAIN, "IMU no encontrado. La pantalla mostrara 0.0 deg");
	}

	/* ------------------------------------------------------------------ */
	/*  4. Inicializar la pantalla circular GC9A01 por SPI                 */
	/* ------------------------------------------------------------------ */
	pantalla_inicializar();
	ESP_LOGI(ETIQUETA_MAIN, "Pantalla GC9A01 lista");

	/* ------------------------------------------------------------------ */
	/*  4.1 Inicializar touch (CST816S) y ADC de batería                    */
	/* ------------------------------------------------------------------ */
	touch_inicializar();

	adc_oneshot_unit_init_cfg_t adc_cfg = {
		.unit_id = ADC_UNIT_1,
	};
	if (adc_oneshot_new_unit(&adc_cfg, &g_adc_bateria) == ESP_OK) {
		adc_oneshot_chan_cfg_t canal_cfg = {
			.bitwidth = ADC_BITWIDTH_12,
			.atten    = ADC_ATTEN_DB_12,
		};
		adc_oneshot_config_channel(g_adc_bateria, CANAL_BATERIA_ADC, &canal_cfg);
		ESP_LOGI(ETIQUETA_MAIN, "ADC bateria inicializado (GPIO1)");
	} else {
		ESP_LOGW(ETIQUETA_MAIN, "No se pudo inicializar ADC de bateria");
	}

	/* ------------------------------------------------------------------ */
	/*  5. Lanzar las tareas FreeRTOS                                       */
	/* ------------------------------------------------------------------ */

	/* Tarea_Lectura_IMU: 50 Hz, prioridad alta, anclada al core 0          */
	xTaskCreatePinnedToCore(
		tarea_lectura_imu,      /* Función de la tarea                      */
		"Tarea_Lectura_IMU",    /* Nombre para el depurador                 */
		4096,                   /* Stack: 4 KB (filtro + datos IMU)         */
		NULL,                   /* Parámetro de entrada (no se usa)         */
		5,                      /* Prioridad alta                           */
		NULL,                   /* Handle de tarea (no necesario)           */
		0                       /* Core 0: sensores                         */
	);

	/* Tarea_Display: 20 FPS, prioridad media, anclada al core 1            */
	xTaskCreatePinnedToCore(
		tarea_display,          /* Función de la tarea                      */
		"Tarea_Display",        /* Nombre para el depurador                 */
		6144,                   /* Stack: 6 KB (dibujo circular + texto)    */
		NULL,                   /* Parámetro de entrada (no se usa)         */
		3,                      /* Prioridad media                          */
		NULL,                   /* Handle de tarea (no necesario)           */
		1                       /* Core 1: pantalla                         */
	);

	ESP_LOGI(ETIQUETA_MAIN, "Tareas creadas. FreeRTOS toma el control.");
	/* app_main retorna aquí; el scheduler de FreeRTOS gestiona las tareas  */
}
