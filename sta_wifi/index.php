<?php
/**
 * --------------------------------------------------------------------------
 * Archivo  : index.php
 * Autor    : Daniel
 * Proyecto : sta_wifi
 * Ruta     : C:\xampp\htdocs\sta\index.php
 * --------------------------------------------------------------------------
 *
 * PROPOSITO
 * Este script recibe solicitudes HTTP GET desde una ESP32.
 *
 * Comportamiento esperado:
 * 1) Si llega /sta/?mensaje=algo
 *    - Guarda "algo" como ultimo mensaje recibido.
 *    - Registra la peticion en un archivo de log.
 *    - Responde JSON: {"valor":"Recibido: algo"}
 *
 * 2) Si llega /sta/ (sin parametro mensaje)
 *    - Lee el ultimo mensaje guardado.
 *    - Responde JSON con ese ultimo valor.
 *    - Si nunca llego ninguno, responde "sin mensaje".
 *
 * De esta forma, al abrir /sta/ siempre puedes ver el ultimo mensaje que
 * envio la ESP32.
 */

/**
 * Indicamos que la respuesta sera JSON y en UTF-8.
 * Esto ayuda a navegadores, clientes HTTP y a la ESP32.
 */
header('Content-Type: application/json; charset=utf-8');

/**
 * Archivo donde se guarda el ultimo mensaje recibido.
 * Se crea automaticamente si no existe.
 */
$storage_file = __DIR__ . '/ultimo_mensaje.txt';

/**
 * Archivo de log para historial de solicitudes.
 * Se agrega una linea por cada mensaje recibido.
 */
$log_file = __DIR__ . '/requests.log';

/**
 * Leer parametro GET "mensaje" si existe.
 * Ejemplo de URL:
 *   /sta/?mensaje=hola%20mundo
 *
 * Si no existe, dejamos cadena vacia.
 */
$incoming = isset($_GET['mensaje']) ? trim((string)$_GET['mensaje']) : '';

/**
 * Si llega un mensaje:
 * - Lo limpiamos de etiquetas HTML por seguridad basica.
 * - Lo guardamos como "ultimo mensaje".
 * - Lo registramos en log con fecha e IP origen.
 */
if ($incoming !== '') {
    $incoming = strip_tags($incoming);

    // Guardar ultimo mensaje (sobrescribe el anterior)
    file_put_contents($storage_file, $incoming);

    // Construir linea de log
    $line = date('Y-m-d H:i:s')
          . ' | IP: ' . ($_SERVER['REMOTE_ADDR'] ?? 'unknown')
          . ' | mensaje: ' . $incoming . PHP_EOL;

    // Agregar al final del archivo de log
    file_put_contents($log_file, $line, FILE_APPEND);
}

/**
 * Valor por defecto de salida si no hay mensajes previos.
 */
$current_message = 'sin mensaje';

/**
 * Si existe archivo con ultimo mensaje, leerlo.
 */
if (file_exists($storage_file)) {
    $saved = trim((string)file_get_contents($storage_file));
    if ($saved !== '') {
        $current_message = $saved;
    }
}

/**
 * Respuesta JSON final.
 * IMPORTANTE:
 * Tu firmware ESP32 busca la clave "valor", por eso se mantiene ese nombre.
 */
echo json_encode(
    ['valor' => 'Recibido: ' . $current_message],
    JSON_UNESCAPED_UNICODE
);
?>