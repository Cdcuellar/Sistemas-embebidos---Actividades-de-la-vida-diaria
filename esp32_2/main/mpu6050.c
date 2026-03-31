/* ============================================================
   BIBLIOTECAS
   ============================================================ */

#include "mpu6050.h"          /* Tipos y declaraciones propias del driver    */

#include <stddef.h>           /* NULL, size_t                                */
#include <stdbool.h>          /* Tipo bool y valores true/false              */
#include "freertos/FreeRTOS.h"/* Núcleo de FreeRTOS                          */
#include "freertos/task.h"    /* vTaskDelay para pausas dentro del driver    */
#include "esp_log.h"          /* ESP_LOGI / ESP_LOGE / ESP_LOGW              */

/* ============================================================
   ETIQUETA DE LOG Y CONSTANTES INTERNAS
   ============================================================ */

/* Prefijo que aparece en todos los mensajes del monitor serie de este driver */
static const char *MPU_TAG = "MPU6050";

/* Tiempo máximo que esperamos respuesta del sensor en cada operación I2C (ms) */
#define MPU6050_I2C_TIMEOUT_MS   100

/* Número de reintentos para lecturas/escrituras I2C si hay error transitorio */
#define MPU6050_I2C_RETRIES      5

/* Número de reintentos específico para leer el registro WHO_AM_I al inicializar */
#define MPU6050_WHOAMI_RETRIES   20

/* ============================================================
   MAPA DE REGISTROS DEL MPU6050 (hoja de datos)
   ============================================================ */

#define MPU6050_REG_WHO_AM_I      0x75   /* Identificador del chip: debe devolver 0x68 o 0x69 */
#define MPU6050_REG_PWR_MGMT_1   0x6B   /* Control de energía y reset del dispositivo         */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B   /* Primer byte de los 14 bytes de datos (accel+temp+gyro)*/
#define MPU6050_WHOAMI_ADDR0     0x68   /* Valor esperado en WHO_AM_I con AD0=GND              */
#define MPU6050_WHOAMI_ADDR1     0x69   /* Valor esperado en WHO_AM_I con AD0=VCC              */

/* ============================================================
   FUNCIONES INTERNAS DE COMUNICACION I2C
   ============================================================ */

/*
 * Escribe un byte en un registro del sensor.
 * Parámetros:
 *   sensor  - instancia del driver con el handle I2C
 *   reg     - dirección del registro destino (ej: 0x6B para PWR_MGMT_1)
 *   value   - valor a escribir en ese registro
 */
static esp_err_t mpu6050_escribir_reg(mpu6050_t *sensor, uint8_t reg, uint8_t valor)
{
    /* El protocolo I2C para escritura: [dirección_registro, dato] en un solo frame */
    uint8_t buf[2] = {reg, valor};
    return i2c_master_transmit(sensor->dev_handle, buf, sizeof(buf), MPU6050_I2C_TIMEOUT_MS);
}

/*
 * Lee 'len' bytes consecutivos del sensor a partir del registro 'reg'.
 * Implementa reintentos automáticos si hay errores transitorios en el bus.
 * Parámetros:
 *   sensor  - instancia del driver con el handle I2C
 *   reg     - registro de inicio de lectura
 *   salida  - buffer donde se guardan los bytes leídos
 *   len     - cantidad de bytes a leer
 */
static esp_err_t mpu6050_leer_regs(mpu6050_t *sensor, uint8_t reg, uint8_t *salida, size_t len)
{
    esp_err_t ret = ESP_FAIL;   /* Resultado del intento actual; empieza en fallo */

    for (int intento = 0; intento < MPU6050_I2C_RETRIES; intento++) {
        /* Envía el registro de inicio y lee la respuesta en la misma transacción */
        ret = i2c_master_transmit_receive(sensor->dev_handle, &reg, 1, salida, len, MPU6050_I2C_TIMEOUT_MS);
        if (ret == ESP_OK) {
            return ESP_OK;   /* Lectura exitosa; no hace falta más intentos */
        }

        vTaskDelay(pdMS_TO_TICKS(5));   /* Pequeña pausa antes de reintentar */
    }

    return ret;   /* Devuelve el último error si todos los intentos fallaron */
}

/* ============================================================
   FUNCION AUXILIAR
   ============================================================ */

/*
 * Valor absoluto para enteros de 32 bits.
 * La librería estándar abs() no garantiza comportamiento correcto
 * con int32_t en todos los compiladores; por eso se define aquí.
 */
static int32_t mpu6050_abs_i32(int32_t valor)
{
    return valor < 0 ? -valor : valor;
}

/* ============================================================
   INICIALIZACION DEL SENSOR
   ============================================================ */

/*
 * Conecta el sensor al bus I2C, verifica la identidad del chip (WHO_AM_I),
 * lo resetea, lo despierta del modo sleep y configura rangos de medición.
 *
 * Parámetros:
 *   sensor       - instancia a inicializar (struct que luego se pasa a las demás funciones)
 *   bus          - handle del bus I2C creado previamente con i2c_new_master_bus()
 *   i2c_addr     - dirección del sensor en el bus (0x68 o 0x69)
 *   scl_speed_hz - velocidad del bus en Hz (usamos 100000 = 100 kHz)
 */
esp_err_t mpu6050_init(mpu6050_t *sensor, i2c_master_bus_handle_t bus, uint8_t i2c_addr, uint32_t scl_speed_hz)
{
    /* Validar que los punteros de entrada no sean NULL */
    if (sensor == NULL || bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Asegurar que el handle empieza limpio (evita fugas si se llama más de una vez) */
    sensor->dev_handle = NULL;

    /* Configura los parámetros del dispositivo I2C */
    i2c_device_config_t cfg_dispositivo = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,   /* Dirección de 7 bits (estándar I2C)       */
        .device_address  = i2c_addr,              /* 0x68 o 0x69 según el pin AD0 del sensor  */
        .scl_speed_hz    = scl_speed_hz,          /* Velocidad de reloj acordada (100 kHz)    */
    };

    /* Añade el dispositivo al bus I2C y obtiene su handle de comunicación */
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg_dispositivo, &sensor->dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(MPU_TAG, "bus_add_device fallo (addr=0x%02X): %s", i2c_addr, esp_err_to_name(ret));
        return ret;
    }

    /* ------------------------------------------------
       ESPERA DE ARRANQUE DEL SENSOR
       ------------------------------------------------ */

    /* El MPU6050 necesita ~30 ms después de energizarse antes de responder por I2C */
    vTaskDelay(pdMS_TO_TICKS(50));   /* Usamos 50 ms para mayor seguridad */

    /* ------------------------------------------------
       VERIFICACION DE IDENTIDAD (WHO_AM_I)
       ------------------------------------------------ */

    uint8_t whoami  = 0;       /* Almacenará el identificador devuelto por el sensor */
    bool lectura_ok = false;   /* Bandera: true si se consiguió leer un valor válido  */

    /* Intenta leer WHO_AM_I hasta MPU6050_WHOAMI_RETRIES veces */
    for (int intento = 0; intento < MPU6050_WHOAMI_RETRIES; intento++) {
        ret = mpu6050_leer_regs(sensor, MPU6050_REG_WHO_AM_I, &whoami, 1);
        if (ret == ESP_OK) {
            lectura_ok = true;
            break;   /* Lectura exitosa; salimos del bucle de reintentos */
        }

        vTaskDelay(pdMS_TO_TICKS(20));   /* Espera 20 ms entre intentos fallidos */
    }

    /* Si no se pudo leer WHO_AM_I ni con los reintentos, el sensor no responde */
    if (!lectura_ok) {
        ESP_LOGE(MPU_TAG, "Lectura WHO_AM_I fallo (addr=0x%02X): %s", i2c_addr, esp_err_to_name(ret));
        i2c_master_bus_rm_device(sensor->dev_handle);   /* Libera el handle del bus */
        sensor->dev_handle = NULL;                       /* Marca como no inicializado */
        return ret;
    }

    /* Muestra el valor leído para diagnóstico en el monitor serie */
    ESP_LOGI(MPU_TAG, "WHO_AM_I = 0x%02X (addr=0x%02X)", whoami, i2c_addr);

    /* Verifica que el chip responde con un identificador conocido */
    if (whoami != 0x68 && whoami != 0x69 && whoami != 0x72) {
        /* 0x72 es la variante MPU6050C; se acepta también para compatibilidad */
        ESP_LOGE(MPU_TAG, "WHO_AM_I no coincide (esperado 0x68/0x69/0x72, recibido 0x%02X)", whoami);
        i2c_master_bus_rm_device(sensor->dev_handle);   /* Libera el handle del bus */
        sensor->dev_handle = NULL;
        return ESP_ERR_INVALID_RESPONSE;                 /* Error: chip incorrecto o dañado */
    }

    /* ------------------------------------------------
       SECUENCIA DE RESET Y CONFIGURACION
       ------------------------------------------------ */

    /* Bit 7 (DEVICE_RESET) de PWR_MGMT_1: reinicia todos los registros a sus valores por defecto */
    ret = mpu6050_escribir_reg(sensor, MPU6050_REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100));   /* El sensor tarda ~100 ms en completar el reset */

    /* Coloca 0x00 en PWR_MGMT_1: despierta el sensor (sale del sleep) con oscilador interno */
    ret = mpu6050_escribir_reg(sensor, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));    /* Espera estabilización del oscilador interno */

    /* Registro 0x1B (GYRO_CONFIG): bits 4:3 = 00 → rango ±250 °/s, sensibilidad 131 LSB/(°/s) */
    ret = mpu6050_escribir_reg(sensor, 0x1B, 0x00);
    if (ret != ESP_OK) return ret;

    /* Registro 0x1C (ACCEL_CONFIG): bits 4:3 = 00 → rango ±2g, sensibilidad 16384 LSB/g */
    ret = mpu6050_escribir_reg(sensor, 0x1C, 0x00);

    /* ------------------------------------------------
       INICIALIZACION DE OFFSETS DE CALIBRACION EN CERO
       ------------------------------------------------ */

    /* Se ponen a cero hasta que mpu6050_calibrate() calcule los valores reales */
    sensor->offset_ax = 0;   /* Offset eje X del acelerómetro */
    sensor->offset_ay = 0;   /* Offset eje Y del acelerómetro */
    sensor->offset_az = 0;   /* Offset eje Z del acelerómetro */
    sensor->offset_gx = 0;   /* Offset eje X del giroscopio   */
    sensor->offset_gy = 0;   /* Offset eje Y del giroscopio   */
    sensor->offset_gz = 0;   /* Offset eje Z del giroscopio   */

    return ret;   /* ESP_OK si todo fue bien */
}

/* ============================================================
   CALIBRACION DEL SENSOR
   ============================================================ */

/*
 * Calcula los offsets de los 6 ejes mientras el sensor está completamente quieto.
 * Los offsets se guardan en sensor->offset_* y se aplican automáticamente en cada lectura.
 *
 * Parámetros:
 *   sensor       - instancia ya inicializada del driver
 *   num_muestras - cuántas lecturas se promedian (usamos 100)
 *
 * IMPORTANTE: el sensor debe estar inmóvil y horizontal durante esta función.
 */
esp_err_t mpu6050_calibrate(mpu6050_t *sensor, uint16_t num_muestras)
{
    /* Validaciones de entrada */
    if (sensor == NULL || num_muestras == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Reinicia los offsets a cero antes de tomar las muestras de referencia */
    sensor->offset_ax = 0;
    sensor->offset_ay = 0;
    sensor->offset_az = 0;
    sensor->offset_gx = 0;
    sensor->offset_gy = 0;
    sensor->offset_gz = 0;

    ESP_LOGI(MPU_TAG, "Iniciando calibración con %d muestras (MANTÉN EL SENSOR INMÓVIL)...", num_muestras);
    vTaskDelay(pdMS_TO_TICKS(1000));   /* 1 segundo para que el usuario suelte el sensor */

    /* ------------------------------------------------
       RECOLECCION DE MUESTRAS
       ------------------------------------------------ */

    /* Acumuladores de 32 bits para sumar todas las muestras sin desbordamiento */
    int32_t suma_ax = 0, suma_ay = 0, suma_az = 0;   /* Suma de lecturas del acelerómetro */
    int32_t suma_gx = 0, suma_gy = 0, suma_gz = 0;   /* Suma de lecturas del giroscopio   */

    uint16_t muestra            = 0;   /* Contador de muestras válidas recolectadas    */
    uint16_t fallos_transitorios = 0;  /* Contador de errores I2C seguidos en calibración */

    while (muestra < num_muestras) {
        /* Variables temporales para cada lectura cruda */
        int16_t ax, ay, az, temp, gx, gy, gz;
        esp_err_t ret = mpu6050_read_all_raw(sensor, &ax, &ay, &az, &temp, &gx, &gy, &gz);

        if (ret != ESP_OK) {
            /* Error de lectura: se cuenta pero no corta inmediatamente */
            fallos_transitorios++;
            if (fallos_transitorios >= MPU6050_I2C_RETRIES) {
                /* Demasiados errores seguidos: el sensor tuvo un problema real */
                ESP_LOGE(MPU_TAG, "Error leyendo datos en calibración: %s", esp_err_to_name(ret));
                return ret;
            }

            ESP_LOGW(MPU_TAG, "Lectura transitoria fallida en calibración (%u/%u): %s",
                     fallos_transitorios, MPU6050_I2C_RETRIES, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(20));   /* Espera antes de reintentar la lectura */
            continue;                         /* No incrementa 'muestra', repite el intento */
        }

        fallos_transitorios = 0;   /* Lectura exitosa: reinicia el contador de fallos */

        /* Acumula los valores en los sumadores de 32 bits */
        suma_ax += ax;
        suma_ay += ay;
        suma_az += az;
        suma_gx += gx;
        suma_gy += gy;
        suma_gz += gz;

        muestra++;   /* Cuenta esta muestra como válida */

        /* Progreso en consola cada 10 muestras para confirmar que va bien */
        if ((muestra % 10) == 0) {
            ESP_LOGI(MPU_TAG, "Calibración: %d/%d muestras", muestra, num_muestras);
        }

        vTaskDelay(pdMS_TO_TICKS(10));   /* 10 ms entre muestras = 100 Hz de muestreo */
    }

    /* ------------------------------------------------
       CALCULO DE PROMEDIOS
       ------------------------------------------------ */

    /* Divide la suma total entre el número de muestras para obtener el valor promedio */
    int16_t prom_ax = (int16_t)(suma_ax / num_muestras);   /* Promedio acelerómetro X */
    int16_t prom_ay = (int16_t)(suma_ay / num_muestras);   /* Promedio acelerómetro Y */
    int16_t prom_az = (int16_t)(suma_az / num_muestras);   /* Promedio acelerómetro Z */
    int16_t prom_gx = (int16_t)(suma_gx / num_muestras);   /* Promedio giroscopio X   */
    int16_t prom_gy = (int16_t)(suma_gy / num_muestras);   /* Promedio giroscopio Y   */
    int16_t prom_gz = (int16_t)(suma_gz / num_muestras);   /* Promedio giroscopio Z   */

    /* ------------------------------------------------
       DETECCION DEL EJE DE GRAVEDAD
       ------------------------------------------------ */

    /* El acelerómetro mide la gravedad (≈ 9.8 m/s²) cuando está quieto.
     * 1g equivale a 16384 cuentas en rango ±2g.
     * El eje con mayor valor absoluto es el que apunta hacia arriba o hacia abajo. */

    int16_t objetivo_ax = 0;   /* Valor esperado de gravedad en X (0 si no es el eje dominante) */
    int16_t objetivo_ay = 0;   /* Valor esperado de gravedad en Y */
    int16_t objetivo_az = 0;   /* Valor esperado de gravedad en Z */

    /* Calcula el valor absoluto de cada promedio para comparar magnitudes */
    int32_t abs_ax = mpu6050_abs_i32(prom_ax);
    int32_t abs_ay = mpu6050_abs_i32(prom_ay);
    int32_t abs_az = mpu6050_abs_i32(prom_az);

    if (abs_ax >= abs_ay && abs_ax >= abs_az) {
        /* El eje X tiene la mayor aceleración: la gravedad apunta en X */
        objetivo_ax = prom_ax >= 0 ? 16384 : -16384;   /* +1g o -1g según la orientación */
    } else if (abs_ay >= abs_ax && abs_ay >= abs_az) {
        /* El eje Y tiene la mayor aceleración: la gravedad apunta en Y */
        objetivo_ay = prom_ay >= 0 ? 16384 : -16384;
    } else {
        /* El eje Z tiene la mayor aceleración (posición normal boca arriba): gravedad en Z */
        objetivo_az = prom_az >= 0 ? 16384 : -16384;
    }

    /* ------------------------------------------------
       CALCULO DE OFFSETS
       ------------------------------------------------ */

    /* Offset = valor_esperado - valor_promedio_medido
     * Al aplicarse en cada lectura: valor_corregido = valor_crudo + offset ≈ valor_real */
    sensor->offset_ax = objetivo_ax - prom_ax;   /* Corrección para acelerómetro X */
    sensor->offset_ay = objetivo_ay - prom_ay;   /* Corrección para acelerómetro Y */
    sensor->offset_az = objetivo_az - prom_az;   /* Corrección para acelerómetro Z */
    sensor->offset_gx = -prom_gx;                /* Para giroscopio X: el reposo ideal es 0 */
    sensor->offset_gy = -prom_gy;                /* Para giroscopio Y */
    sensor->offset_gz = -prom_gz;                /* Para giroscopio Z */

    /* Muestra resultados en el monitor serie para verificar */
    ESP_LOGI(MPU_TAG, "Calibración completada");
    ESP_LOGI(MPU_TAG, "Promedios en reposo - AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d",
             prom_ax, prom_ay, prom_az, prom_gx, prom_gy, prom_gz);
    ESP_LOGI(MPU_TAG, "Offsets - AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d",
             sensor->offset_ax, sensor->offset_ay, sensor->offset_az,
             sensor->offset_gx, sensor->offset_gy, sensor->offset_gz);

    return ESP_OK;
}

/* ============================================================
   LECTURA DE ACELEROMETRO (solo 3 ejes)
   ============================================================ */

/*
 * Lee únicamente los 3 ejes del acelerómetro (6 bytes).
 * Útil si no se necesita giroscopio ni temperatura para ahorrar transferencias I2C.
 */
esp_err_t mpu6050_read_accel_raw(mpu6050_t *sensor, int16_t *ax, int16_t *ay, int16_t *az)
{
    /* Validar que ningún puntero sea NULL */
    if (sensor == NULL || ax == NULL || ay == NULL || az == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[6] = {0};   /* Buffer para los 6 bytes: 2 bytes por cada eje X, Y, Z */

    /* Lee 6 bytes a partir del registro ACCEL_XOUT_H (0x3B) */
    esp_err_t ret = mpu6050_leer_regs(sensor, MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data));
    if (ret != ESP_OK) {
        return ret;
    }

    /* Combina los dos bytes de cada eje (big-endian: byte alto primero) */
    *ax = (int16_t)((data[0] << 8) | data[1]);   /* Bytes 0-1: ACCEL_XOUT_H/L */
    *ay = (int16_t)((data[2] << 8) | data[3]);   /* Bytes 2-3: ACCEL_YOUT_H/L */
    *az = (int16_t)((data[4] << 8) | data[5]);   /* Bytes 4-5: ACCEL_ZOUT_H/L */

    return ESP_OK;
}

/* ============================================================
   LECTURA COMPLETA: ACELEROMETRO + TEMPERATURA + GIROSCOPIO
   ============================================================ */

/*
 * Lee los 14 bytes contiguos del sensor (accel + temp + gyro) en una sola transacción I2C.
 * Aplica los offsets de calibración antes de devolver los valores.
 *
 * Mapa de los 14 bytes desde el registro 0x3B:
 *   [0-1]  ACCEL_X   [2-3]  ACCEL_Y   [4-5]  ACCEL_Z
 *   [6-7]  TEMP
 *   [8-9]  GYRO_X    [10-11] GYRO_Y   [12-13] GYRO_Z
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
)
{
    /* Valida todos los punteros de entrada (temp puede ser NULL, los demás no) */
    if (sensor == NULL || ax == NULL || ay == NULL || az == NULL ||
        gx == NULL || gy == NULL || gz == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[14] = {0};   /* Buffer para los 14 bytes del bloque de datos del sensor */

    /* Lee los 14 bytes de una sola vez comenzando en ACCEL_XOUT_H */
    esp_err_t ret = mpu6050_leer_regs(sensor, MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data));
    if (ret != ESP_OK) {
        return ret;
    }

    /* ------------------------------------------------
       DECODIFICACION DE BYTES (big-endian → int16_t)
       ------------------------------------------------ */

    *ax = (int16_t)((data[0]  << 8) | data[1]);    /* Aceleración X cruda */
    *ay = (int16_t)((data[2]  << 8) | data[3]);    /* Aceleración Y cruda */
    *az = (int16_t)((data[4]  << 8) | data[5]);    /* Aceleración Z cruda */

    /* Temperatura es opcional; solo se escribe si el puntero no es NULL */
    if (temp != NULL) {
        *temp = (int16_t)((data[6] << 8) | data[7]);   /* Temperatura cruda del die */
    }

    *gx = (int16_t)((data[8]  << 8) | data[9]);    /* Velocidad angular X cruda */
    *gy = (int16_t)((data[10] << 8) | data[11]);   /* Velocidad angular Y cruda */
    *gz = (int16_t)((data[12] << 8) | data[13]);   /* Velocidad angular Z cruda */

    /* ------------------------------------------------
       APLICACION DE OFFSETS DE CALIBRACION
       ------------------------------------------------ */

    /* Suma los offsets calculados en mpu6050_calibrate() para centrar las lecturas */
    *ax += sensor->offset_ax;   /* Corrige el error de cero del acelerómetro en X */
    *ay += sensor->offset_ay;   /* Corrige el error de cero del acelerómetro en Y */
    *az += sensor->offset_az;   /* Corrige el error de cero del acelerómetro en Z */
    *gx += sensor->offset_gx;   /* Corrige el bias del giroscopio en X            */
    *gy += sensor->offset_gy;   /* Corrige el bias del giroscopio en Y            */
    *gz += sensor->offset_gz;   /* Corrige el bias del giroscopio en Z            */

    return ESP_OK;
}
