========================================================
  ESP32 - Cliente HTTP en modo STA | Proyecto sta_wifi
========================================================
 Autor    : Daniel

DESCRIPCIÓN
-----------
El ESP32 se conecta a una red WiFi y cada 5 segundos envía
una solicitud HTTP GET a un servidor Apache (XAMPP) en la
misma red. La respuesta del servidor se muestra en el
monitor serial de ESP-IDF.

URL base del servidor:
  http://<SERVER_HOST>:80/sta/

La ESP32 agrega el mensaje en la solicitud asi:
  http://<SERVER_HOST>:80/sta/?mensaje=<texto_codificado>


CONFIGURACIÓN ANTES DE FLASHEAR
--------------------------------
1. WIFI_SSID y WIFI_PASSWORD (main.c)
   - Cambia los valores por los de tu red WiFi.

2. SERVER_HOST (main.c)
   - Debe ser la IPv4 de la PC donde corre XAMPP.
   - Para obtenerla en Windows: abrir CMD -> escribir ipconfig
     y copiar el valor de "Dirección IPv4".
  - IP de esta PC: aqui la direccion ip
   - Si usas OTRA PC: repite el paso y actualiza SERVER_HOST.


LÍNEAS EXACTAS A CAMBIAR EN MAIN.C
-----------------------------------
Archivo: main/main.c

- Línea 29: WIFI_SSID
- Línea 31: WIFI_PASSWORD
- Línea 35: SERVER_HOST
- Línea 39: SERVER_PATH
- Línea 41: REQUEST_MESSAGE
- Línea 44: HTTP_REQUEST_INTERVAL_MS



CONFIGURAR XAMPP
----------------
1. Abre XAMPP Control Panel e inicia Apache.
2. Crea el archivo: C:\xampp\htdocs\sta\index.php
   Contenido mínimo del archivo:

     <?php
       $msg = $_GET['mensaje'] ?? 'sin mensaje';
       echo json_encode(["valor" => "Recibido: $msg"]);
     ?>

3. Prueba desde el navegador de la misma PC:
  http://localhost/sta/
   Esa es la ruta base del proyecto.

4. Prueba desde otro dispositivo en la red:
  http://aqui la direccion ip/sta/
   Esa es la ruta base vista desde la red.
   Si no responde, revisa el firewall (ver sección siguiente).


REQUISITO DE BANDA WIFI (MUY IMPORTANTE)
-----------------------------------------
El ESP32 clásico (ESP32-WROOM, ESP32-D0WD) SOLO soporta 2.4 GHz.
NO se conectará a redes de 5 GHz (reason=201 en el monitor).

Cómo verificar si tu red es 2.4 GHz desde Windows:
  1. Conéctate a la red con la PC.
  2. Abre PowerShell y ejecuta:
       netsh wlan show interfaces
  3. Busca la línea "Canal":
       - Canal 1-13  → 2.4 GHz (compatible con ESP32)
       - Canal 36+   → 5 GHz   (NO compatible, ESP32 no conectará)

Si usas hotspot de celular (Android/Redmi):
  - Ajustes → Punto de acceso portátil → Configurar punto de acceso.
  - Busca "Banda AP" o "Frecuencia" y selecciona 2.4 GHz.
  - Si no aparece esa opción:
      · Busca "Modo compatibilidad" y actívalo.
      · Desactiva "Wi-Fi 6" o "AX" si aparece.
      · Cambia el nombre del hotspot a algo simple sin espacios ni
        caracteres especiales (ej: Redmi24) y la clave a algo corto.
      · Si el celular no permite elegir banda, usa otro celular o
        crea un hotspot desde Windows (ver más abajo).

Crear hotspot 2.4 GHz desde Windows:
  1. Ajustes → Red e Internet → Zona de conexión móvil.
  2. Actívala y elige banda 2.4 GHz (en Windows 10/11 aparece
     al editar las propiedades de la zona de conexión).
  3. O bien: Panel de control → Centro de redes → Configurar
     nueva red → Red ad hoc (solo si el adaptador lo permite).


FIREWALL DE WINDOWS (si el ESP32 no llega al servidor)
-------------------------------------------------------
Abre PowerShell como Administrador y ejecuta:

  New-NetFirewallRule -DisplayName "XAMPP HTTP" `
    -Direction Inbound -Protocol TCP -LocalPort 80 -Action Allow


FORMATO DE RESPUESTA SOPORTADO
-------------------------------
El código en main.c acepta dos formatos de respuesta:

  1. JSON con clave "valor":
       {"valor": "algo"}

  2. Texto plano con formato clave=valor:
       dato=algo

Si el servidor responde en cualquier otro formato, el ESP32
mostrará una advertencia en el monitor serial.


INTERVALO DE SOLICITUDES
-------------------------
Por defecto: 5000 ms (5 segundos).
Para cambiarlo, modifica en main.c:
  #define HTTP_REQUEST_INTERVAL_MS 5000


CAMBIAR MENSAJE ENVIADO
-----------------------
En esta versión SI se envía parámetro mensaje desde la ESP32.
Para cambiarlo, edita en main.c:
  #define REQUEST_MESSAGE "hola mundo"

El firmware codifica ese texto para URL y arma la solicitud:
  /sta/?mensaje=<tu_mensaje>


MONITOR SERIAL
--------------
Velocidad: 115200 baud
Comando idf.py: idf.py monitor
Los logs mostrarán la URL, el código HTTP y el dato recibido.


EXPLICACION DE MAIN.C
---------------------
El archivo main.c ahora esta comentado linea por linea.

Resumen por funcion:
  1. log_visible_aps()
    - Escanea redes visibles y muestra SSID, RSSI y canal.
    - Sirve para diagnosticar reason=201 (AP no encontrado).

  2. trim_whitespace()
    - Quita espacios al inicio y al final de una cadena.

  3. extract_json_value()
    - Busca una clave JSON dentro del payload y extrae su valor.

  4. procesar_respuesta()
    - Procesa el body HTTP.
    - Intenta formato JSON (clave "valor") y formato plano (dato=...).

  5. wifi_event_handler()
    - Maneja eventos WiFi e IP.
    - Reconecta automaticamente y marca estado de conexion.

  6. wifi_init_sta()
    - Inicializa red, eventos y driver WiFi en modo STA.
    - Aplica SSID/password y lanza conexion.

  7. is_wifi_connected()
    - Consulta el bit de estado para saber si ya hay IP.

  8. send_http_request()
    - Construye URL con query: /sta/?mensaje=...
    - Envia GET
    - Lee body HTTP en bucle
    - Procesa respuesta.

  9. app_main()
    - Punto de entrada.
    - Inicializa NVS y WiFi.
    - Ejecuta solicitudes periodicas cada 5 segundos.


NOTA FINAL
----------
El archivo index.php debe colocarse en:
  C:\xampp\htdocs\sta\index.php
En caso de usar otra direccion se debera modificar el llamado 
en el codigo linea 39
