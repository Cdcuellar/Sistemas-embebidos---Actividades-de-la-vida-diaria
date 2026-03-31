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

/*
 * mpu6050_t agrupa todo lo que el driver necesita para manejar un sensor MPU6050:
 * - El handle I2C para comunicarse con el chip
 * - Los 6 offsets de calibración (uno por cada eje de acelerómetro y giroscopio)
 *
 * Se crea una instancia por cada sensor conectado al bus.
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle;  /* Handle del dispositivo en el bus I2C            */
    int16_t offset_ax;                   /* Corrección del error de cero del acelerómetro X */
    int16_t offset_ay;                   /* Corrección del error de cero del acelerómetro Y */
    int16_t offset_az;                   /* Corrección del error de cero del acelerómetro Z */
    int16_t offset_gx;                   /* Corrección del bias del giroscopio en eje X     */
    int16_t offset_gy;                   /* Corrección del bias del giroscopio en eje Y     */
    int16_t offset_gz;                   /* Corrección del bias del giroscopio en eje Z     */
} mpu6050_t;

/* ============================================================
   DECLARACIONES DE FUNCIONES PUBLICAS
   ============================================================ */

/*
 * mpu6050_init - Inicializa el sensor en el bus I2C.
 *
 * Pasos internos:
 *   1. Registra el dispositivo en el bus con la dirección y velocidad indicadas
 *   2. Lee el registro WHO_AM_I para confirmar que el chip es un MPU6050
 *   3. Resetea el chip y lo despierta del modo sleep
 *   4. Configura rangos: acelerómetro ±2g, giroscopio ±250 °/s
 *
 * Parámetros:
 *   sensor       - puntero a la instancia mpu6050_t a inicializar
 *   bus          - handle del bus I2C obtenido de i2c_new_master_bus()
 *   i2c_addr     - dirección del sensor: 0x68 (AD0=GND) o 0x69 (AD0=VCC)
 *   scl_speed_hz - velocidad del bus en Hz (se usa 100000 = 100 kHz)
 *
 * Retorna:
 *   ESP_OK               si el sensor fue detectado y configurado correctamente
 *   ESP_ERR_INVALID_ARG  si algún puntero es NULL
 *   ESP_ERR_INVALID_RESPONSE si el WHO_AM_I no coincide con un MPU6050 conocido
 *   Otro código de error si hay fallo de comunicación I2C
 */
esp_err_t mpu6050_init(mpu6050_t *sensor, i2c_master_bus_handle_t bus, uint8_t i2c_addr, uint32_t scl_speed_hz);

/*
 * mpu6050_calibrate - Calcula y guarda los offsets de los 6 ejes.
 *
 * El sensor debe estar completamente quieto durante esta función.
 * Toma 'num_muestras' lecturas, promedia los valores y calcula los offsets
 * necesarios para que la salida sea cero (en reposo) o 1g en el eje de gravedad.
 *
 * Parámetros:
 *   sensor       - instancia ya inicializada con mpu6050_init()
 *   num_muestras - cantidad de lecturas a promediar (se recomienda 100)
 *
 * Retorna:
 *   ESP_OK               si la calibración se completó sin errores
 *   ESP_ERR_INVALID_ARG  si sensor es NULL o num_muestras es cero
 *   Otro código de error si hay fallo de comunicación I2C durante la calibración
 */
esp_err_t mpu6050_calibrate(mpu6050_t *sensor, uint16_t num_muestras);

/*
 * mpu6050_read_accel_raw - Lee solo los 3 ejes del acelerómetro.
 *
 * Lee 6 bytes del registro ACCEL_XOUT_H. Los valores son enteros crudos de 16 bits.
 * Para convertir a g: valor_g = valor_crudo / 16384.0f  (rango ±2g)
 *
 * Parámetros:
 *   sensor - instancia inicializada
 *   ax     - puntero donde se guarda la lectura del eje X
 *   ay     - puntero donde se guarda la lectura del eje Y
 *   az     - puntero donde se guarda la lectura del eje Z
 *
 * Retorna:
 *   ESP_OK o código de error I2C
 */
esp_err_t mpu6050_read_accel_raw(mpu6050_t *sensor, int16_t *ax, int16_t *ay, int16_t *az);

/*
 * mpu6050_read_all_raw - Lee los 14 bytes completos del sensor en una sola transacción.
 *
 * Devuelve los 6 ejes (acelerómetro + giroscopio) y la temperatura del die.
 * Los offsets de calibración ya están aplicados al retornar.
 *
 * Conversiones:
 *   Aceleración (g)       = valor / 16384.0f      (rango ±2g)
 *   Velocidad angular(°/s)= valor / 131.0f        (rango ±250 °/s)
 *   Temperatura (°C)      = (valor / 340.0f) + 36.53f
 *
 * Parámetros:
 *   sensor - instancia inicializada
 *   ax, ay, az   - acelerómetro en los 3 ejes (con offset aplicado)
 *   temp         - temperatura cruda del die (puede ser NULL si no se necesita)
 *   gx, gy, gz   - giroscopio en los 3 ejes (con offset aplicado)
 *
 * Retorna:
 *   ESP_OK o código de error I2C
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

