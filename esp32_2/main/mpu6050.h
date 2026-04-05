/**
 * @file mpu6050.h
 * @brief Driver público para el sensor inercial MPU6050 (acelerómetro + giroscopio 6 ejes).
 * @details Expone la API del driver del MPU6050 para su uso desde el código de aplicación.
 *          El driver se comunica mediante el nuevo I2C master driver de ESP-IDF v5+
 *          (driver/i2c_master.h) y soporta ambas direcciones del chip (0x68 y 0x69).
 *
 *          Rangos configurados internamente:
 *          - Acelerómetro: ±2g → sensibilidad 16384 LSB/g
 *          - Giroscopio:   ±250 °/s → sensibilidad 131 LSB/(°/s)
 *
 */

/* ============================================================
   GUARDIA DE INCLUSION
   ============================================================ */

/* Evita que este archivo se incluya más de una vez en la misma unidad de compilación */
#pragma once

/* ============================================================
   BIBLIOTECAS
   ============================================================ */

#include <stdint.h>              /* Tipos enteros de tamaño fijo: int16_t, uint32_t, etc. */
#include "esp_err.h"             /* Tipo esp_err_t y códigos de error ESP-IDF             */
#include "driver/i2c_master.h"  /* Tipo i2c_master_dev_handle_t y i2c_master_bus_handle_t */

/* ============================================================
   ESTRUCTURA DEL SENSOR
   ============================================================ */

/**
 * @brief Estructura de control del driver MPU6050.
 * @details Agrupa el handle I2C del dispositivo y los seis offsets de calibración
 *          (uno por eje de acelerómetro y giroscopio). Debe inicializarse con
 *          mpu6050_init() antes de llamar a cualquier otra función del driver.
 *          Declarar una instancia por cada sensor físico conectado al bus.
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle;  /**< Handle del dispositivo en el bus I2C I2C. */
    int16_t offset_ax;                   /**< Offset de calibración del acelerómetro X. */
    int16_t offset_ay;                   /**< Offset de calibración del acelerómetro Y. */
    int16_t offset_az;                   /**< Offset de calibración del acelerómetro Z. */
    int16_t offset_gx;                   /**< Offset de calibración del giroscopio X.   */
    int16_t offset_gy;                   /**< Offset de calibración del giroscopio Y.   */
    int16_t offset_gz;                   /**< Offset de calibración del giroscopio Z.   */
} mpu6050_t;

/* ============================================================
   DECLARACIONES DE FUNCIONES PUBLICAS
   ============================================================ */

/**
 * @brief Inicializa el sensor MPU6050 en el bus I2C.
 * @details Pasos internos:
 *          1. Registra el dispositivo en el bus con la dirección y velocidad indicadas.
 *          2. Lee el registro WHO_AM_I para confirmar que el chip es un MPU6050
 *             (acepta 0x68, 0x69 y 0x72 para la variante MPU6050C).
 *          3. Resetea el chip y lo despierta del modo sleep.
 *          4. Configura rangos: acelerómetro ±2g, giroscopio ±250 °/s.
 *          5. Inicializa los offsets de calibración a cero.
 * @param sensor       Puntero a la instancia @ref mpu6050_t a inicializar.
 * @param bus          Handle del bus I2C obtenido de i2c_new_master_bus().
 * @param i2c_addr     Dirección del sensor: 0x68 (AD0=GND) o 0x69 (AD0=VCC).
 * @param scl_speed_hz Velocidad del bus en Hz (usar 100000 para 100 kHz).
 * @return ESP_OK               Sensor detectado y configurado correctamente.
 * @return ESP_ERR_INVALID_ARG  Algún puntero de entrada es NULL.
 * @return ESP_ERR_INVALID_RESPONSE WHO_AM_I no coincide con un MPU6050 conocido.
 * @return Otro código de error si hay fallo de comunicación I2C.
 */
esp_err_t mpu6050_init(mpu6050_t *sensor, i2c_master_bus_handle_t bus, uint8_t i2c_addr, uint32_t scl_speed_hz);

/**
 * @brief Calcula y almacena los offsets de calibración para los 6 ejes.
 * @details El sensor debe estar completamente quieto y horizontal durante
 *          esta función. Toma @p num_muestras lecturas, promedia y calcula
 *          los offsets para que la salida sea cero en reposo (o ~1g en Z).
 *          Los offsets quedan guardados en @p sensor->offset_* y se aplican
 *          automáticamente en mpu6050_read_all_raw().
 * @param sensor       Instancia ya inicializada con mpu6050_init().
 * @param num_muestras Número de lecturas a promediar (recomendado: 100).
 * @return ESP_OK               Calibración completada sin errores.
 * @return ESP_ERR_INVALID_ARG  @p sensor es NULL o @p num_muestras es cero.
 * @return Otro código de error si hay fallo de comunicación I2C.
 */
esp_err_t mpu6050_calibrate(mpu6050_t *sensor, uint16_t num_muestras);

/**
 * @brief Lee únicamente los 3 ejes del acelerómetro en valores crudos.
 * @details Lee 6 bytes a partir del registro ACCEL_XOUT_H en una sola
 *          transacción I2C. Los offsets de calibración quedan aplicados.
 *          Para convertir: valor_g = valor_crudo / 16384.0f  (rango ±2g).
 * @param sensor Instancia inicializada con mpu6050_init().
 * @param ax     Lectura cruda del eje X (acelerométrico).
 * @param ay     Lectura cruda del eje Y (acelerométrico).
 * @param az     Lectura cruda del eje Z (acelerométrico).
 * @return ESP_OK o código de error I2C.
 */
esp_err_t mpu6050_read_accel_raw(mpu6050_t *sensor, int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief Lee los 14 bytes completos del sensor en una sola transacción I2C.
 * @details Devuelve los 6 ejes (acelerómetro + giroscopio) y la temperatura
 *          del die. Los offsets de calibración ya están aplicados al retornar.
 *
 *          Conversiones:
 *          - Aceleración (g)        = valor / 16384.0f    (rango ±2g)
 *          - Velocidad angular (°/s) = valor / 131.0f      (rango ±250 °/s)
 *          - Temperatura (°C)       = (valor / 340.0f) + 36.53f
 *
 * @param sensor Instancia inicializada con mpu6050_init().
 * @param ax     Aceleración cruda eje X (offset aplicado).
 * @param ay     Aceleración cruda eje Y (offset aplicado).
 * @param az     Aceleración cruda eje Z (offset aplicado).
 * @param temp   Temperatura cruda del die. Puede ser NULL si no se necesita.
 * @param gx     Velocidad angular cruda eje X (offset aplicado).
 * @param gy     Velocidad angular cruda eje Y (offset aplicado).
 * @param gz     Velocidad angular cruda eje Z (offset aplicado).
 * @return ESP_OK o código de error I2C.
 */
esp_err_t mpu6050_read_all_raw(
    mpu6050_t *sensor,
    int16_t *ax,
    int16_t *ay,
    int16_t *az,
    int16_t *temp,
    int16_t *gx,
    int16_t *gy,
    int16_t *gz
);

