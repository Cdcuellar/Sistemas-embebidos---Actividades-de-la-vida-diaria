===========================================================================
NOTAS DE DEPURACION DE HARDWARE - PROYECTO ACTIVIDADES DE LA VIDA DIARIA
===========================================================================

PROYECTO: Sistema de Monitoreo Vestible (Nodo 2 - Movimiento)
DESARROLLADOR: Carlos Daniel Cuellar Antury (Ingenieria Electronica y Telecomunicaciones - Unicauca)
HERRAMIENTA: Identificador de ID Interno (WHO_AM_I)

---------------------------------------------------------------------------
DESCRIPCION DE LA HERRAMIENTA
---------------------------------------------------------------------------
Esta carpeta contiene un codigo fuente esencial para realizar un diagnostico 
fisico del sensor MPU6050. Su funcion principal es leer el registro 0x75 
(WHO_AM_I) para verificar la identidad real del silicio, lo cual es critico 
cuando el sensor no es original o utiliza una revision de hardware distinta.

INSTRUCCIONES DE EJECUCION:
1. Abrir el archivo .ino en Arduino IDE.
2. Seleccionar la placa: ESP32 DevKit V1.
3. Conectar el sensor a los pines GPIO 21 (SDA) y GPIO 22 (SCL).
4. Cargar el codigo y abrir el Monitor Serie a 115200 baudios.

LIBRERIAS NECESARIAS:
Para compilar este codigo no se requieren librerias externas complejas, 
solo se utiliza la libreria nativa de comunicacion I2C:
- Wire.h (Viene integrada por defecto en el nucleo de ESP32 para Arduino).

---------------------------------------------------------------------------
FUNCIONALIDAD DE LA HERRAMIENTA: ID INTEENO (0x72)
---------------------------------------------------------------------------
Para la identificacion del dispsitivo, desde idf, se requiere la identificacion
interna del dipositivo, este codigo lo que hae es encontrar dicha identificacion

---------------------------------------------------------------------------
CONFIGURACION FISICA
---------------------------------------------------------------------------
- SDA: GPIO 21
- SCL: GPIO 22
- Velocidad Serial: 115200 baudios
===========================================================================