# Codigos_embed_ESP32.
Este es una agrupación de códigos para 2 ESP32 que conformarán parte del sistema SENA Crowd Inteligence. Se contendrá la información del programa y todo lo requerido con conexiones.

## Configuración local

Cada proyecto (`ESP32_1` y `ESP32_2`) lee las variables `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_HOST`, `MQTT_USER`, `MQTT_PASSWORD` y `MQTT_CA` desde su propio archivo `.env` al compilar. El script `load_env.py` genera el encabezado temporal usado por el firmware dentro de `.pio`; no subas los archivos `.env` ni los binarios al repositorio.
