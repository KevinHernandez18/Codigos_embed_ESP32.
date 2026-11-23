// ======================================================
// ESP32 PLACA 1 — SENA EMERGENCIAS
// BLOQUE 1: LIBRERIAS Y CONFIGURACION
// ======================================================

// Firebase
// Supawaze
// 1

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHTesp.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <math.h>

#include "secrets.h"

// ================= WIFI =================

// Colombia UTC-5
constexpr long DESFASE_UTC_SEGUNDOS = -5L * 3600L;

constexpr uint32_t REINTENTO_WIFI_MS = 20000;
constexpr uint32_t PAUSA_REINTENTO_WIFI_MS = 1000;

// ================= MQTT EMQX =================

constexpr uint16_t MQTT_PORT = 8883;

#include <esp_system.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ======================================================
// CONFIGURACION
// ======================================================

constexpr int SDA_PIN = 33;
constexpr int SCL_PIN = 32;

constexpr int OLED_SDA_PIN = 13;
constexpr int OLED_SCL_PIN = 14;

constexpr int TOUCH_PIN = 34;
constexpr int TOUCH_ACTIVO = HIGH;

constexpr int DHT_PIN = 17;

constexpr int TRIG_PIN = 25;
constexpr int ECHO_PIN = 35;

const char* TOPIC_METRICAS =
    "sena/casanare/metricas";

const char* TOPIC_INCENDIO_REMOTO =
    "sena/casanare/placa2/eventos/incendio";

const char* TOPIC_COMANDOS_P2 =
    "sena/casanare/placa2/comandos/alarma";

const char* TOPIC_CONEXION =
    "sena/casanare/placa1/estado/conexion";

// Objetivo de muestreo: 100 Hz.
constexpr uint32_t MUESTREO_MS = 10;
constexpr uint32_t VENTANA_MS = 1500;

constexpr float GRAVEDAD = 9.80665f;
constexpr float UMBRAL_ACTIVIDAD = 0.50f;
constexpr float UMBRAL_RMS = 0.70f;
constexpr float ACTIVIDAD_SOSTENIDA = 60.0f;
constexpr float UMBRAL_IMPACTO = 6.0f;
constexpr float CRESTA_IMPACTO = 3.0f;

constexpr float TEMP_INCENDIO = 40.0f;
constexpr float TEMP_REARME = 37.0f;

// ======================================================
// DISPOSITIVOS
// ======================================================

LiquidCrystal_I2C lcd27(0x27, 16, 2);
LiquidCrystal_I2C lcd3F(0x3F, 16, 2);
LiquidCrystal_I2C* lcd = nullptr;

TwoWire busOLED(1);

Adafruit_SSD1306 oled(
    128, 64, &busOLED, -1, 400000UL, 400000UL
);

DHTesp dht;

bool oledOK = false;
uint32_t ultimoIntentoPantallas = 0;
uint32_t ultimaPantalla = 0;

char cacheLCD[2][17] = {};

// ======================================================
// EVENTOS Y ESTADO
// ======================================================

enum Evento : uint8_t {
  NINGUNO = 0,
  INCENDIO = 1,
  SISMO = 2,
  IMPACTO = 3,
  SIMULACRO = 4,
  EMERGENCIA = 5
};

struct EstadoEvento {
  bool detectando = false;
  bool retenido = false;
  char id[64] = {};
};

EstadoEvento eventos[6];

bool incendioRemotoDetectando = false;
bool incendioRemotoRetenido = false;
char idIncendioRemoto[64] = {};

char sesion[32] = {};
uint32_t secuenciaJSON = 0;
uint32_t numeroEvento = 0;

bool forzarMetrica = false;
bool metricasRapidasSuspendidas = false;

char avisoTitulo[32] = {};
char avisoTexto[160] = {};
uint32_t inicioAviso = 0;
bool avisoActivo = false;

std::atomic<bool> redWiFi(false);
std::atomic<bool> redMQTT(false);

// ======================================================
// DATOS DE SENSORES
// ======================================================

bool dhtValido = false;
bool humedadValida = false;
float temperaturaAmbiente = NAN;
float humedadAmbiente = NAN;
uint32_t fallosDHT = 0;
uint32_t ultimoDHT = 0;

uint8_t direccionMPU = 0;
uint8_t identidadMPU = 0;

bool mpuOK = false;
bool filtroListo = false;
bool estadisticaValida = false;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float temperaturaMPU = NAN;

float gravedadX = 0, gravedadY = 0, gravedadZ = 0;

float picoVentana = 0;
float sumaCuadrados = 0;

float ultimoPico = 0;
float ultimoRMS = 0;
float ultimaActividad = 0;
float ultimoFactorCresta = 0;
float frecuenciaMuestreo = 0;

uint32_t muestras = 0;
uint32_t muestrasActivas = 0;
uint32_t inicioVentana = 0;
uint32_t inicioFiltro = 0;
uint32_t ultimaMuestra = 0;
uint32_t ultimoIntentoMPU = 0;

bool ventanaConHueco = false;
bool saturacionVentana = false;
bool ultimaSaturacion = false;

uint8_t ventanasSismo = 0;
uint8_t ventanasCalmaSismo = 0;
uint8_t ventanasCalmaImpacto = 0;

// ======================================================
// TOUCH E IR
// ======================================================

struct EntradaDigital {
  int pin;
  int activo;
  int crudo;
  int estable;
  uint32_t cambio;
};

EntradaDigital touch = {
  TOUCH_PIN, HIGH, LOW, LOW, 0
};

uint8_t toques = 0;
uint32_t primerToque = 0;
bool gestoDuranteSimulacro = false;
bool gestoDuranteEmergencia = false;

// ======================================================
// ULTRASONICO
// ======================================================

portMUX_TYPE muxEcho = portMUX_INITIALIZER_UNLOCKED;

volatile bool ecoArmado = false;
volatile bool ecoSubio = false;
volatile bool ecoListo = false;
volatile uint32_t inicioEcoUs = 0;
volatile uint32_t anchoEcoUs = 0;

bool hcPendiente = false;
bool distanciaValida = false;
float distanciaCm = NAN;
const char* estadoHC = "esperando";

uint32_t inicioDisparoUs = 0;
uint32_t ultimoHC = 0;

// ======================================================
// COLAS MQTT
// ======================================================

struct Paquete {
  char topic[96];
  char json[1800];
};

struct IncendioRecibido {
  bool estado;
  char id[64];

  bool suspenderMetricas;
  bool soloSimulacro;
  bool soloEmergencia;
  bool falsaAlarma;
};

QueueHandle_t colaEventos = nullptr;
QueueHandle_t colaMetricas = nullptr;
QueueHandle_t colaRemotos = nullptr;

// ======================================================
// UTILIDADES
// ======================================================

const char* nombreEvento(Evento evento) {
  switch (evento) {
    case INCENDIO: return "incendio";
    case SISMO: return "sismo";
    case IMPACTO: return "impacto";
    case SIMULACRO: return "simulacro";
    case EMERGENCIA: return "emergencia";
    default: return "ninguno";
  }
}

bool hayAlertaReal() {
  return eventos[INCENDIO].retenido ||
         eventos[SISMO].retenido ||
         eventos[IMPACTO].retenido ||
         eventos[EMERGENCIA].retenido ||
         incendioRemotoRetenido;
}

bool hayAlerta() {
  return hayAlertaReal() || eventos[SIMULACRO].retenido;
}

void mostrarAviso(const char* titulo, const char* texto) {
  snprintf(avisoTitulo, sizeof(avisoTitulo), "%s", titulo);
  snprintf(avisoTexto, sizeof(avisoTexto), "%s", texto);

  inicioAviso = millis();
  avisoActivo = true;
}

void cabecera(JsonDocument& doc, const char* tipo) {
  doc["version"] = 1;
  doc["placa"] = "placa1";
  doc["sala"] = "riesgo_bajo";
  doc["sesion"] = sesion;
  doc["secuencia"] = ++secuenciaJSON;
  doc["tipo"] = tipo;
  doc["tiempo_activo_ms"] = millis();

  const time_t fecha = time(nullptr);

  if (fecha > 1700000000) {
    struct tm utc;
    gmtime_r(&fecha, &utc);

    char texto[25];
    strftime(
      texto, sizeof(texto),
      "%Y-%m-%dT%H:%M:%SZ", &utc
    );

    doc["fecha"] = texto;
  } else {
    doc["fecha"] = nullptr;
  }
}

bool encolar(
    const char* topic,
    JsonDocument& doc,
    bool esMetrica = false) {

  Paquete paquete = {};

  if (doc.overflowed() ||
      measureJson(doc) >= sizeof(paquete.json)) {
    Serial.println("[JSON] Mensaje demasiado grande.");
    return false;
  }

  snprintf(paquete.topic, sizeof(paquete.topic), "%s", topic);
  serializeJson(doc, paquete.json, sizeof(paquete.json));

  if (esMetrica) {
    if (!colaMetricas) return false;

    return xQueueOverwrite(
      colaMetricas, &paquete
    ) == pdPASS;
  }

  if (!colaEventos ||
      xQueueSend(colaEventos, &paquete, 0) != pdPASS) {
    Serial.println("[MQTT] Cola llena: mensaje no guardado.");
    return false;
  }

  Serial.printf(
    "[MQTT COLA] %s | %s\n",
    paquete.topic, paquete.json
  );

  return true;
}

// ======================================================
// RECEPCION Y TAREA DE RED
// ======================================================

void recibirMQTT(
    char* topic,
    byte* payload,
    unsigned int longitud) {

  if (!colaRemotos || longitud == 0 || longitud > 1600) {
    return;
  }

  const bool esComando =
      strcmp(topic, TOPIC_COMANDOS_P2) == 0;

  const bool esIncendio =
      strcmp(topic, TOPIC_INCENDIO_REMOTO) == 0;

  if (!esComando && !esIncendio) {
    return;
  }

  JsonDocument doc;

  if (deserializeJson(doc, payload, longitud)) {
    Serial.println("[MQTT RX] JSON invalido.");
    return;
  }

  IncendioRecibido recibido = {};

  if (esComando) {
    if (!doc["action"].is<bool>()) {
      Serial.println("[MQTT RX] action debe ser booleano.");
      return;
    }

    // Esta suscripción solo procesa el apagado.
    if (doc["action"].as<bool>()) {
      return;
    }

    const char* alcance = doc["alcance"] | "todas";

    if (strcmp(alcance, "todas") != 0 &&
    strcmp(alcance, "simulacro") != 0 &&
    strcmp(alcance, "emergencia") != 0) {

      Serial.println("[MQTT RX] Alcance no reconocido.");
      return;
    }

    recibido.suspenderMetricas = true;

    recibido.soloSimulacro =
        strcmp(alcance, "simulacro") == 0;

    recibido.soloEmergencia =
        strcmp(alcance, "emergencia") == 0;

    const char* motivo = doc["motivo"] | "";

    recibido.falsaAlarma =
        recibido.soloSimulacro &&
        strcmp(motivo, "falsa_alarma") == 0;
        
  } else {
    const char* evento = doc["evento"] | "";
    const char* placa = doc["placa"] | "placa2";
    const char* id = doc["evento_id"] | "";

    if (strcmp(evento, "incendio") != 0 ||
        strcmp(placa, "placa2") != 0 ||
        !doc["estado"].is<bool>() ||
        strlen(id) >= sizeof(recibido.id)) {

      Serial.println("[MQTT RX] Incendio remoto invalido.");
      return;
    }

    recibido.estado = doc["estado"].as<bool>();

    snprintf(
      recibido.id,
      sizeof(recibido.id),
      "%s",
      id
    );
  }

  if (xQueueSend(colaRemotos, &recibido, 0) != pdPASS) {
    Serial.println("[MQTT RX] Cola de recepcion llena.");
  }
}

void tareaRed(void*) {
  WiFiClientSecure tls;
  PubSubClient cliente(tls);

  tls.setCACert(MQTT_CA);
  tls.setHandshakeTimeout(5);

  cliente.setServer(MQTT_HOST, MQTT_PORT);
  cliente.setCallback(recibirMQTT);
  cliente.setKeepAlive(30);
  cliente.setSocketTimeout(3);

  if (!cliente.setBufferSize(2048)) {
    Serial.println("[MQTT] No hay memoria para el buffer.");
    vTaskDelete(nullptr);
    return;
  }

  char clientId[48];
  const uint64_t mac = ESP.getEfuseMac();

  snprintf(
    clientId, sizeof(clientId),
    "placa1-%04X%08lX",
    (unsigned int)((mac >> 32) & 0xFFFF),
    (unsigned long)(mac & 0xFFFFFFFF)
  );

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t ultimoWiFi = millis();
  uint32_t ultimoMQTT = 0;
  bool primerIntento = true;
  bool wifiPrevio = false;

  Paquete pendiente = {};
  bool hayPendiente = false;

  for (;;) {
    const bool conectado = WiFi.status() == WL_CONNECTED;
    redWiFi.store(conectado);

    if (!conectado) {
      redMQTT.store(false);
      tls.stop();

      if (wifiPrevio) {
        ultimoWiFi = millis();
        Serial.println("[WiFi] Conexion perdida.");
      }

      wifiPrevio = false;
      primerIntento = true;

      if (millis() - ultimoWiFi >= 20000) {
        WiFi.disconnect(false, false);
        vTaskDelay(pdMS_TO_TICKS(1000));
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        ultimoWiFi = millis();
        Serial.println("[WiFi] Reintentando.");
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (!wifiPrevio) {
      wifiPrevio = true;

      configTime(
        -5L * 3600L, 0,
        "pool.ntp.org", "time.nist.gov"
      );

      Serial.println("[WiFi] Conectado. NTP solicitado.");
    }

    if (time(nullptr) <= 1700000000) {
      redMQTT.store(false);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (!cliente.connected() &&
        (primerIntento || millis() - ultimoMQTT >= 10000)) {

      primerIntento = false;
      tls.stop();

      bool conectadoMQTT = cliente.connect(
        clientId,
        MQTT_USER,
        MQTT_PASSWORD,
        TOPIC_CONEXION,
        1,
        true,
        "{\"online\":false}"
      );

      if (conectadoMQTT) {
        const bool online = cliente.publish(
          TOPIC_CONEXION, "{\"online\":true}", true
        );

        const bool suscrito = cliente.subscribe(
          TOPIC_INCENDIO_REMOTO, 1
        );

        const bool suscritoComandos = cliente.subscribe(
          TOPIC_COMANDOS_P2, 1
        );

        if (!online || !suscrito || !suscritoComandos) {
          tls.stop();
        } else {
          Serial.println("[MQTT] Conectado y suscripcion solicitada.");
        }
      } else {
        Serial.printf("[MQTT] Error=%d\n", cliente.state());
      }

      ultimoMQTT = millis();
    }

    const bool operativo =
        cliente.connected() && cliente.loop();

    redMQTT.store(operativo);

    if (operativo) {
      // Mantiene el mensaje pendiente si falla el envio.
      if (!hayPendiente) {
        hayPendiente =
            xQueueReceive(colaEventos, &pendiente, 0) == pdPASS;
      }

      if (hayPendiente) {
        if (cliente.publish(
              pendiente.topic, pendiente.json, false)) {
          hayPendiente = false;
        } else {
          tls.stop();
        }

      } else {
        Paquete metrica = {};

        if (xQueueReceive(colaMetricas, &metrica, 0) == pdPASS) {
          if (!cliente.publish(
                metrica.topic, metrica.json, false)) {

            // Si hay una mas reciente en la cola, se conserva esa.
            xQueueSend(colaMetricas, &metrica, 0);
            tls.stop();
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======================================================
// PUBLICACION Y CICLO DE EVENTOS
// ======================================================

void publicarEvento(Evento evento, bool estado) {
  JsonDocument doc;
  cabecera(doc, "evento");

  doc["evento_id"] = eventos[evento].id;
  doc["evento"] = nombreEvento(evento);
  doc["estado"] = estado;

  if (evento == INCENDIO) {
    doc["evidencia"]["origen"] = "temperatura_alta";

    if (dhtValido)
      doc["evidencia"]["temperatura_c"] = temperaturaAmbiente;
    else
      doc["evidencia"]["temperatura_c"] = nullptr;

    doc["evidencia"]["umbral_c"] = TEMP_INCENDIO;
    doc["evidencia"]["rearme_inferior_a_c"] = TEMP_REARME;

  } else if (evento == SISMO || evento == IMPACTO) {
    doc["evidencia"]["pico_m_s2"] = ultimoPico;
    doc["evidencia"]["rms_m_s2"] = ultimoRMS;
    doc["evidencia"]["actividad_porcentaje"] = ultimaActividad;
    doc["evidencia"]["factor_cresta"] = ultimoFactorCresta;
    doc["evidencia"]["ventana_valida"] = estadisticaValida;

  } else {
    doc["evidencia"]["origen"] = "touch_o_control_local";
  }

  char topic[96];

  snprintf(
    topic, sizeof(topic),
    "sena/casanare/placa1/eventos/%s",
    nombreEvento(evento)
  );

  encolar(topic, doc);
  forzarMetrica = true;
}

void comandoApagado(bool soloSimulacro, bool falsaAlarma) {
  JsonDocument doc;

  doc["action"] = false;
  doc["alcance"] = soloSimulacro ? "simulacro" : "todas";

  if (falsaAlarma) {
    doc["motivo"] = "falsa_alarma";
  }

  encolar(TOPIC_COMANDOS_P2, doc);
}

void finalizarDeteccion(Evento evento) {
  if (!eventos[evento].detectando) return;

  eventos[evento].detectando = false;
  publicarEvento(evento, false);

  // La alerta visual sigue retenida.
}

void cerrarSimulacro(bool falsaAlarma) {
  if (!eventos[SIMULACRO].retenido) return;

  finalizarDeteccion(SIMULACRO);
  eventos[SIMULACRO].retenido = false;

  comandoApagado(true, falsaAlarma);

  mostrarAviso(
    falsaAlarma ? "Falsa alarma" : "Alerta finalizada",
    falsaAlarma
        ? "Simulacro cancelado."
        : "Siga las indicaciones del responsable."
  );

  forzarMetrica = true;
}

void iniciarEvento(Evento evento) {
  if (evento == NINGUNO) return;

  if (evento == SIMULACRO && hayAlertaReal()) {
    Serial.println("[SIMULACRO] Ignorado: alerta real retenida.");
    return;
  }

  if (eventos[evento].detectando) return;

  // Una emergencia real sustituye al simulacro.
  if (evento != SIMULACRO && eventos[SIMULACRO].retenido) {
    cerrarSimulacro(false);
  }

  EstadoEvento& e = eventos[evento];

  snprintf(
    e.id, sizeof(e.id),
    "%s-%lu",
    sesion, (unsigned long)++numeroEvento
  );

  e.detectando = true;
  e.retenido = true;
  metricasRapidasSuspendidas = false;
  avisoActivo = false;

  publicarEvento(evento, true);

  Serial.printf(
    "[EVENTO] %s | %s\n",
    nombreEvento(evento), e.id
  );
}

void procesarRemotos() {
  if (!colaRemotos) return;

  IncendioRecibido recibido = {};

  while (xQueueReceive(colaRemotos, &recibido, 0) == pdPASS) {

    // ==========================================
    // COMANDOS DE CIERRE
    // ==========================================

    if (recibido.suspenderMetricas) {
      bool mostrarCierre = false;
      const char* titulo = "Alerta finalizada";
      const char* mensaje =
          "Siga las indicaciones del responsable.";

      if (recibido.soloEmergencia) {
        // Cerrar exclusivamente la emergencia manual.
        mostrarCierre = eventos[EMERGENCIA].retenido;

        finalizarDeteccion(EMERGENCIA);
        eventos[EMERGENCIA].retenido = false;

        titulo = "Alarma manual";
        mensaje = "finalizada";

      } else if (recibido.soloSimulacro) {
        // Cerrar exclusivamente el simulacro.
        // Las demás alertas permanecen intactas.
        mostrarCierre = eventos[SIMULACRO].retenido;

        finalizarDeteccion(SIMULACRO);
        eventos[SIMULACRO].retenido = false;

        if (recibido.falsaAlarma) {
          titulo = "Falsa alarma";
          mensaje = "Simulacro cancelado.";
        }

      } else {
        // Cierre general: action:false sin alcance específico.
        mostrarCierre = hayAlerta();

        finalizarDeteccion(SIMULACRO);
        finalizarDeteccion(EMERGENCIA);

        for (int i = 1; i <= 5; i++) {
          eventos[i].retenido = false;
        }

        incendioRemotoRetenido = false;

        // Conservamos los estados físicos de detección.
        // Cerrar la pantalla no significa que la temperatura
        // haya bajado o que el movimiento haya terminado.
      }

      toques = 0;
      gestoDuranteSimulacro = false;
      gestoDuranteEmergencia = false;

      // Si el cierre específico deja otra alerta activa,
      // conservar su envío rápido y su mensaje.
      if (!hayAlerta()) {
        metricasRapidasSuspendidas = true;
        forzarMetrica = false;

        if (colaMetricas) {
          xQueueReset(colaMetricas);
        }

        if (mostrarCierre) {
          mostrarAviso(titulo, mensaje);
        }

        Serial.println(
          "[CIERRE] Sin alertas retenidas. "
          "Retorno a pantalla normal tras el aviso."
        );

      } else {
        Serial.println(
          "[CIERRE] Evento cerrado; otra alerta sigue vigente."
        );
      }

      // No republicar el comando recibido.
      continue;
    }

    // ==========================================
    // INCENDIO REMOTO DE LA PLACA 2
    // ==========================================

    if (recibido.estado) {
      // Ignorar el mismo evento incluso si su alerta
      // visual ya fue cerrada.
      if (recibido.id[0] != '\0' &&
          strcmp(recibido.id, idIncendioRemoto) == 0) {
        continue;
      }

      if (eventos[SIMULACRO].retenido) {
        cerrarSimulacro(false);
      }

      snprintf(
        idIncendioRemoto,
        sizeof(idIncendioRemoto),
        "%s",
        recibido.id
      );

      incendioRemotoDetectando = true;
      incendioRemotoRetenido = true;
      metricasRapidasSuspendidas = false;
      avisoActivo = false;

    } else {
      // Un cierre antiguo no cambia una detección nueva.
      if (recibido.id[0] != '\0' &&
          idIncendioRemoto[0] != '\0' &&
          strcmp(recibido.id, idIncendioRemoto) != 0) {
        continue;
      }

      incendioRemotoDetectando = false;

      // estado:false termina la detección,
      // pero no sustituye al comando de cierre visual.
    }

    forzarMetrica = true;
  }
}

void cerrarAlertas() {
  if (eventos[INCENDIO].detectando ||
      eventos[SISMO].detectando ||
      eventos[IMPACTO].detectando ||
      incendioRemotoDetectando) {

    Serial.println(
      "[CIERRE] Aun existe una deteccion activa. No se apaga."
    );
    return;
  }

  if (eventos[SIMULACRO].retenido) {
    finalizarDeteccion(SIMULACRO);
  }

  finalizarDeteccion(EMERGENCIA);

  for (int i = 1; i <= 5; i++) {
    eventos[i].retenido = false;
  }

  incendioRemotoRetenido = false;
  toques = 0;

  comandoApagado(false, false);

  mostrarAviso(
    "Alerta finalizada",
    "Siga las indicaciones del responsable."
  );

  forzarMetrica = true;
}

// ======================================================
// MPU: REGISTROS Y MUESTREO
// ======================================================

bool respondeI2C(TwoWire& bus, uint8_t direccion) {
  bus.beginTransmission(direccion);
  return bus.endTransmission() == 0;
}

bool escribirMPU(uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(direccionMPU);
  Wire.write(registro);
  Wire.write(valor);
  return Wire.endTransmission() == 0;
}

bool leerMPU(uint8_t registro, uint8_t* datos, size_t cantidad) {
  Wire.beginTransmission(direccionMPU);
  Wire.write(registro);

  if (Wire.endTransmission(false) != 0) return false;

  size_t recibidos = Wire.requestFrom(
    (uint8_t)direccionMPU, cantidad, true
  );

  if (recibidos != cantidad) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (size_t i = 0; i < cantidad; i++) {
    datos[i] = Wire.read();
  }

  return true;
}

int16_t valor16(const uint8_t* p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void reiniciarVentana(uint32_t ahora) {
  picoVentana = 0;
  sumaCuadrados = 0;
  muestras = 0;
  muestrasActivas = 0;
  ventanaConHueco = false;
  saturacionVentana = false;
  inicioVentana = ahora;
}

void iniciarMPU() {
  ultimoIntentoMPU = millis();

  for (uint8_t direccion : {uint8_t(0x68), uint8_t(0x69)}) {
    direccionMPU = direccion;

    uint8_t id = 0;

    if (!leerMPU(0x75, &id, 1)) continue;

    Serial.printf(
      "[MPU] Direccion=0x%02X WHO_AM_I=0x%02X\n",
      direccion, id
    );

    // MPU6050 / MPU6500 / MPU6515.
    if (id != 0x68 && id != 0x70 && id != 0x74) {
      Serial.println("[MPU] Identidad no soportada.");
      continue;
    }

    identidadMPU = id;

    bool ok =
        escribirMPU(0x6B, 0x01) &&  // Despertar, reloj PLL.
        escribirMPU(0x6C, 0x00) &&  // Habilitar ejes.
        escribirMPU(0x1A, 0x03) &&  // Filtro.
        escribirMPU(0x19, 9) &&     // 1 kHz / 10 = 100 Hz.
        escribirMPU(0x1B, 0x00) &&  // Giroscopio +/-250 grados/s.
        escribirMPU(0x1C, 0x10);    // Acelerometro +/-8 g.

    if (ok && id != 0x68) {
      ok = escribirMPU(0x1D, 0x03);
    }

    if (!ok) continue;

    mpuOK = true;
    filtroListo = false;
    estadisticaValida = false;
    inicioFiltro = millis();
    ultimaMuestra = millis();

    ventanasSismo = 0;
    ventanasCalmaSismo = 0;
    ventanasCalmaImpacto = 0;

    reiniciarVentana(millis());

    Serial.println("[MPU] Inicializado. Mantener quieto 2 segundos.");
    return;
  }

  mpuOK = false;
  Serial.println("[MPU] Sin respuesta valida. Se reintentara.");
}

void analizarVentana(uint32_t ahora) {
  const uint32_t duracion = ahora - inicioVentana;

  if (duracion < VENTANA_MS) return;

  frecuenciaMuestreo =
      duracion ? muestras * 1000.0f / duracion : 0;

  // No clasificar ventanas con baja cobertura o grandes pausas.
  estadisticaValida =
      muestras >= 100 &&
      frecuenciaMuestreo >= 70.0f &&
      !ventanaConHueco;

  ultimoPico = picoVentana;
  ultimoRMS = muestras ? sqrtf(sumaCuadrados / muestras) : 0;
  ultimaActividad =
      muestras ? 100.0f * muestrasActivas / muestras : 0;

  ultimoFactorCresta =
      ultimoRMS > 0.001f ? ultimoPico / ultimoRMS : 0;

  ultimaSaturacion = saturacionVentana;

  if (estadisticaValida) {
    const bool impacto =
        ultimaSaturacion ||
        (ultimoPico >= UMBRAL_IMPACTO &&
         ultimoFactorCresta >= CRESTA_IMPACTO);

    const bool sostenida =
        !impacto &&
        ultimoRMS >= UMBRAL_RMS &&
        ultimaActividad >= ACTIVIDAD_SOSTENIDA;

    if (impacto) {
      iniciarEvento(IMPACTO);
      ventanasCalmaImpacto = 0;
    } else if (eventos[IMPACTO].detectando) {
      if (++ventanasCalmaImpacto >= 2) {
        finalizarDeteccion(IMPACTO);
      }
    }

    if (sostenida) {
      ventanasCalmaSismo = 0;

      if (ventanasSismo < 2) ventanasSismo++;

      if (ventanasSismo >= 2) {
        iniciarEvento(SISMO);
      }
    } else {
      ventanasSismo = 0;

      if (eventos[SISMO].detectando &&
          ++ventanasCalmaSismo >= 2) {
        finalizarDeteccion(SISMO);
      }
    }

  } else {
    ventanasSismo = 0;
    ventanasCalmaSismo = 0;
    ventanasCalmaImpacto = 0;

    // Perder datos no equivale a que el riesgo haya terminado.
  }

  Serial.printf(
    "[MPU] pico=%.2f RMS=%.2f actividad=%.1f%% "
    "cresta=%.2f Hz=%.1f ventana=%s\n",
    ultimoPico, ultimoRMS, ultimaActividad,
    ultimoFactorCresta, frecuenciaMuestreo,
    estadisticaValida ? "valida" : "incompleta"
  );

  reiniciarVentana(ahora);
}

void actualizarMPU() {
  uint32_t ahora = millis();

  if (!mpuOK) {
    if (ahora - ultimoIntentoMPU >= 3000) iniciarMPU();
    return;
  }

  if (ahora - ultimaMuestra < MUESTREO_MS) return;

  const uint32_t intervalo = ahora - ultimaMuestra;
  ultimaMuestra = ahora;

  uint8_t datos[14];

  if (!leerMPU(0x3B, datos, sizeof(datos))) {
    mpuOK = false;
    filtroListo = false;
    estadisticaValida = false;
    ultimoIntentoMPU = ahora;
    Serial.println("[MPU] Lectura fallida.");
    return;
  }

  const int16_t rx = valor16(datos);
  const int16_t ry = valor16(datos + 2);
  const int16_t rz = valor16(datos + 4);
  const int16_t rt = valor16(datos + 6);

  ax = rx * GRAVEDAD / 4096.0f;
  ay = ry * GRAVEDAD / 4096.0f;
  az = rz * GRAVEDAD / 4096.0f;

  gx = valor16(datos + 8) / 131.0f;
  gy = valor16(datos + 10) / 131.0f;
  gz = valor16(datos + 12) / 131.0f;

  temperaturaMPU = identidadMPU == 0x68
      ? rt / 340.0f + 36.53f
      : rt / 333.87f + 21.0f;

  if (!filtroListo) {
    gravedadX = ax;
    gravedadY = ay;
    gravedadZ = az;
    filtroListo = true;
  }

  const float dt = intervalo / 1000.0f;
  const float alpha = dt / (0.50f + dt);

  gravedadX += alpha * (ax - gravedadX);
  gravedadY += alpha * (ay - gravedadY);
  gravedadZ += alpha * (az - gravedadZ);

  if (ahora - inicioFiltro < 2000) {
    reiniciarVentana(ahora);
    return;
  }

  const float dx = ax - gravedadX;
  const float dy = ay - gravedadY;
  const float dz = az - gravedadZ;

  const float movimiento = sqrtf(dx * dx + dy * dy + dz * dz);

  if (intervalo > 100) ventanaConHueco = true;

  if (abs((int)rx) > 32000 ||
      abs((int)ry) > 32000 ||
      abs((int)rz) > 32000) {
    saturacionVentana = true;
  }

  if (movimiento > picoVentana) picoVentana = movimiento;

  sumaCuadrados += movimiento * movimiento;
  muestras++;

  if (movimiento >= UMBRAL_ACTIVIDAD) muestrasActivas++;

  analizarVentana(ahora);
}

// ======================================================
// ULTRASONICO CON INTERRUPCION
// ======================================================

void IRAM_ATTR interrupcionEcho() {
  const uint32_t ahora = micros();
  const int nivel = digitalRead(ECHO_PIN);

  portENTER_CRITICAL_ISR(&muxEcho);

  if (ecoArmado) {
    if (nivel == HIGH) {
      inicioEcoUs = ahora;
      ecoSubio = true;

    } else if (ecoSubio) {
      anchoEcoUs = ahora - inicioEcoUs;
      ecoListo = true;
      ecoArmado = false;
    }
  }

  portEXIT_CRITICAL_ISR(&muxEcho);
}

void actualizarHC() {
  if (hcPendiente) {
    bool listo;
    uint32_t ancho;

    portENTER_CRITICAL(&muxEcho);
    listo = ecoListo;
    ancho = anchoEcoUs;

    if (listo) ecoListo = false;

    portEXIT_CRITICAL(&muxEcho);

    if (listo) {
      hcPendiente = false;

      const float distancia = ancho * 0.0343f / 2.0f;
      distanciaValida = distancia >= 2 && distancia <= 400;
      distanciaCm = distanciaValida ? distancia : NAN;
      estadoHC = distanciaValida ? "ok" : "fuera_de_rango";
      return;
    }

    if ((uint32_t)(micros() - inicioDisparoUs) >= 30000) {
      portENTER_CRITICAL(&muxEcho);
      ecoArmado = false;
      ecoSubio = false;
      ecoListo = false;
      portEXIT_CRITICAL(&muxEcho);

      hcPendiente = false;
      distanciaValida = false;
      distanciaCm = NAN;
      estadoHC = "sin_eco";
    }

    return;
  }

  if (millis() - ultimoHC < 500) return;
  ultimoHC = millis();

  if (digitalRead(ECHO_PIN) == HIGH) {
    distanciaValida = false;
    distanciaCm = NAN;
    estadoHC = "echo_high";
    return;
  }

  portENTER_CRITICAL(&muxEcho);
  ecoArmado = true;
  ecoSubio = false;
  ecoListo = false;
  portEXIT_CRITICAL(&muxEcho);

  hcPendiente = true;
  inicioDisparoUs = micros();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

// ======================================================
// DHT11
// ======================================================

void actualizarDHT() {
  const uint32_t ahora = millis();

  // Mantener 2.5 segundos entre lecturas.
  // No leer mientras se espera el ECHO del ultrasónico.
  if (ahora - ultimoDHT < 2500 || hcPendiente) {
    return;
  }

  ultimoDHT = ahora;

  Serial.println("[DHT11] Iniciando lectura...");

  const uint32_t inicioLectura = micros();

  const TempAndHumidity datos = dht.getTempAndHumidity();
  const DHTesp::DHT_ERROR_t estado = dht.getStatus();

  const uint32_t duracionUs = micros() - inicioLectura;

  const bool lecturaCorrecta =
      estado == DHTesp::ERROR_NONE &&
      isfinite(datos.temperature) &&
      isfinite(datos.humidity);

  if (!lecturaCorrecta) {
    dhtValido = false;
    humedadValida = false;

    temperaturaAmbiente = NAN;
    humedadAmbiente = NAN;

    if (fallosDHT < 1000000) {
      fallosDHT++;
    }

    Serial.printf(
      "[DHT11] Error=%s | Duracion=%lu us | Fallos=%lu\n",
      dht.getStatusString(),
      (unsigned long)duracionUs,
      (unsigned long)fallosDHT
    );

    // Una lectura fallida no cancela una detección anterior.
    return;
  }

  temperaturaAmbiente = datos.temperature;
  humedadAmbiente = datos.humidity;

  dhtValido = true;
  humedadValida = true;
  fallosDHT = 0;

  Serial.printf(
    "[DHT11] Temperatura=%.1f C | Humedad=%.1f %% | "
    "Duracion=%lu us\n",
    temperaturaAmbiente,
    humedadAmbiente,
    (unsigned long)duracionUs
  );

  if (temperaturaAmbiente >= TEMP_INCENDIO) {
    iniciarEvento(INCENDIO);

  } else if (temperaturaAmbiente < TEMP_REARME) {
    finalizarDeteccion(INCENDIO);
  }
}

// ======================================================
// ENTRADAS DIGITALES Y GESTOS
// ======================================================

void iniciarEntrada(EntradaDigital& entrada) {
  pinMode(entrada.pin, INPUT);
  entrada.crudo = digitalRead(entrada.pin);
  entrada.estable = entrada.crudo;
  entrada.cambio = millis();
}

bool nuevaActivacion(EntradaDigital& entrada) {
  const int lectura = digitalRead(entrada.pin);
  const uint32_t ahora = millis();

  if (lectura != entrada.crudo) {
    entrada.crudo = lectura;
    entrada.cambio = ahora;
  }

  if (ahora - entrada.cambio < 30 ||
      lectura == entrada.estable) {
    return false;
  }

  entrada.estable = lectura;
  return lectura == entrada.activo;
}

void cerrarEmergenciaManual() {
  if (!eventos[EMERGENCIA].retenido) return;

  finalizarDeteccion(EMERGENCIA);
  eventos[EMERGENCIA].retenido = false;

  JsonDocument doc;
  doc["action"] = false;
  doc["alcance"] = "emergencia";

  encolar(TOPIC_COMANDOS_P2, doc);

  // Si queda otro incidente, conserva su envío rápido.
  if (!hayAlerta()) {
    metricasRapidasSuspendidas = true;
    forzarMetrica = false;

    if (colaMetricas) {
      xQueueReset(colaMetricas);
    }
  }

  mostrarAviso(
    "Alarma manual",
    "finalizada"
  );

  Serial.println("[EMERGENCIA] Alarma manual finalizada.");
}

void resolverGestos(uint32_t ahora) {
  if (!toques) return;

  const uint32_t transcurrido = ahora - primerToque;

  if (gestoDuranteEmergencia || gestoDuranteSimulacro) {
    // Esperar permite distinguir dos toques de tres.
    if (transcurrido >= 2000) {
      if (toques == 2) {
        if (gestoDuranteEmergencia) {
          cerrarEmergenciaManual();

        } else if (eventos[SIMULACRO].retenido &&
                   !hayAlertaReal()) {
          cerrarSimulacro(true);
        }
      }

      toques = 0;
      gestoDuranteEmergencia = false;
      gestoDuranteSimulacro = false;
    }

    return;
  }

  if (toques >= 2 && transcurrido >= 2000) {
    toques = 0;

  } else if (toques == 1 && transcurrido >= 3000) {
    toques = 0;
    iniciarEvento(SIMULACRO);
  }
}

void actualizarTouch() {
  const uint32_t ahora = millis();

  resolverGestos(ahora);

  if (!nuevaActivacion(touch)) return;

  Serial.println("[TOUCH] Toque detectado en G34.");

  if (!toques) {
    toques = 1;
    primerToque = ahora;

    // La emergencia manual tiene prioridad sobre el simulacro.
    gestoDuranteEmergencia = eventos[EMERGENCIA].retenido;

    gestoDuranteSimulacro =
        !gestoDuranteEmergencia &&
        eventos[SIMULACRO].retenido;

    if (!gestoDuranteEmergencia &&
        !gestoDuranteSimulacro &&
        !hayAlertaReal()) {
      avisoActivo = false;
    }

    return;
  }

  const uint32_t transcurrido = ahora - primerToque;

  // Segundo toque entre los segundos 2 y 3:
  // cancelar una solicitud nueva de simulacro.
  if (!gestoDuranteEmergencia &&
      !gestoDuranteSimulacro &&
      toques == 1 &&
      transcurrido >= 2000) {

    toques = 0;

    mostrarAviso(
      "Solicitud cancelada",
      "El simulacro no se ha iniciado."
    );

    return;
  }

  toques++;

  if (toques >= 3 && transcurrido < 2000) {
    toques = 0;
    gestoDuranteEmergencia = false;
    gestoDuranteSimulacro = false;

    iniciarEvento(EMERGENCIA);
    return;
  }

  if (toques == 2 &&
      !gestoDuranteEmergencia &&
      !gestoDuranteSimulacro) {

    mostrarAviso(
      "Solicitud cancelada",
      "El simulacro no se ha iniciado."
    );
  }
}

// ======================================================
// METRICAS
// ======================================================

void publicarMetricas() {
  JsonDocument doc;
  cabecera(doc, "metricas");

  doc["temperatura"]["valida"] = dhtValido;
  doc["temperatura"]["fallos_consecutivos"] = fallosDHT;

  doc["temperatura"]["estado_sensor"] =
      dhtValido ? "ok" :
      (fallosDHT >= 3 ? "fallo" : "sin_datos");

  doc["temperatura"]["deteccion_incendio_activa"] =
      eventos[INCENDIO].detectando;

  if (dhtValido)
    doc["temperatura"]["valor_c"] = temperaturaAmbiente;
  else
    doc["temperatura"]["valor_c"] = nullptr;

  if (humedadValida)
    doc["temperatura"]["humedad_porcentaje"] = humedadAmbiente;
  else
    doc["temperatura"]["humedad_porcentaje"] = nullptr;

  JsonObject imu = doc["acelerometro"].to<JsonObject>();

  imu["valido"] = mpuOK && filtroListo;
  imu["identidad"] = identidadMPU;
  imu["ventana_valida"] = estadisticaValida;
  imu["frecuencia_muestreo_hz"] = frecuenciaMuestreo;

  if (mpuOK && filtroListo) {
    imu["aceleracion_m_s2"]["x"] = ax;
    imu["aceleracion_m_s2"]["y"] = ay;
    imu["aceleracion_m_s2"]["z"] = az;

    imu["giro_grados_s"]["x"] = gx;
    imu["giro_grados_s"]["y"] = gy;
    imu["giro_grados_s"]["z"] = gz;

    imu["temperatura_interna_c"] = temperaturaMPU;
  } else {
    imu["aceleracion_m_s2"] = nullptr;
    imu["giro_grados_s"] = nullptr;
    imu["temperatura_interna_c"] = nullptr;
  }

  if (estadisticaValida) {
    imu["pico_m_s2"] = ultimoPico;
    imu["rms_m_s2"] = ultimoRMS;
    imu["actividad_porcentaje"] = ultimaActividad;
    imu["factor_cresta"] = ultimoFactorCresta;
    imu["saturacion"] = ultimaSaturacion;
  } else {
    imu["pico_m_s2"] = nullptr;
    imu["rms_m_s2"] = nullptr;
    imu["actividad_porcentaje"] = nullptr;
    imu["factor_cresta"] = nullptr;
  }

  imu["condicion"] =
      !mpuOK ? "sin_datos" :
      eventos[IMPACTO].detectando ? "impacto" :
      eventos[SISMO].detectando ? "posible_sismo" :
      !estadisticaValida ? "observacion" : "sin_evento";

  doc["ultrasonico"]["valido"] = distanciaValida;
  doc["ultrasonico"]["estado"] = estadoHC;

  if (distanciaValida)
    doc["ultrasonico"]["distancia_cm"] = distanciaCm;
  else
    doc["ultrasonico"]["distancia_cm"] = nullptr;

  doc["touch"]["activo"] = touch.estable == HIGH;
  doc["alerta_retenida"] = hayAlerta();
  doc["incendio_remoto_activo"] = incendioRemotoDetectando;

  for (int i = 1; i <= 5; i++) {
    doc["eventos"][nombreEvento((Evento)i)] =
        eventos[i].detectando;
  }

  encolar(TOPIC_METRICAS, doc, true);
}

void atenderMetricas() {
  static int64_t ultimoMinuto = -1;
  static uint32_t ultimaMetrica = 0;

  const uint32_t ahora = millis();
  const time_t epoch = time(nullptr);

  const bool horaDisponible = epoch > 1700000000;
  bool cambioMinuto = false;

  if (horaDisponible) {
    const int64_t minuto = (int64_t)epoch / 60;

    if (ultimoMinuto < 0) {
      ultimoMinuto = minuto;

    } else if (minuto != ultimoMinuto) {
      ultimoMinuto = minuto;
      cambioMinuto = true;
    }
  } else {
    ultimoMinuto = -1;
  }

  const bool envioRapido =
      hayAlerta() && !metricasRapidasSuspendidas;

  bool publicar =
      forzarMetrica && !metricasRapidasSuspendidas;

  // Consumir la solicitud; un nuevo evento generará otra.
  forzarMetrica = false;

  if (envioRapido) {
    publicar |= ahora - ultimaMetrica >= 1000;

  } else if (horaDisponible) {
    publicar |= cambioMinuto;

  } else {
    publicar |= ahora - ultimaMetrica >= 60000;
  }

  if (publicar) {
    ultimaMetrica = ahora;
    publicarMetricas();
  }
}

// ======================================================
// PANTALLAS
// ======================================================

void iniciarPantallas() {
  ultimoIntentoPantallas = millis();

  if (!lcd) {
    if (respondeI2C(Wire, 0x27)) lcd = &lcd27;
    else if (respondeI2C(Wire, 0x3F)) lcd = &lcd3F;

    if (lcd) {
      lcd->init();
      lcd->backlight();

      // Caracter personalizado: ó.
      uint8_t oAcentuada[8] = {
        0b00010, 0b00100, 0b00000, 0b01110,
        0b10001, 0b10001, 0b01110, 0b00000
      };

      lcd->createChar(1, oAcentuada);
      memset(cacheLCD, 0, sizeof(cacheLCD));
    }
  }

  if (!oledOK) {
    uint8_t direccion = 0;

    if (respondeI2C(busOLED, 0x3C)) direccion = 0x3C;
    else if (respondeI2C(busOLED, 0x3D)) direccion = 0x3D;

    if (direccion) {
      oledOK = oled.begin(
        SSD1306_SWITCHCAPVCC, direccion, false, false
      );

      if (oledOK) {
        oled.setTextColor(SSD1306_WHITE);
        oled.setTextWrap(false);
        oled.cp437(true);
      }
    }
  }
}

String acentosOLED(const char* texto) {
  String resultado;

  while (*texto) {
    uint8_t c = (uint8_t)*texto++;

    if (c == 0xC3 && *texto) {
      uint8_t siguiente = (uint8_t)*texto++;

      switch (siguiente) {
        case 0xA1: resultado += (char)0xA0; break;
        case 0xA9: resultado += (char)0x82; break;
        case 0xAD: resultado += (char)0xA1; break;
        case 0xB3: resultado += (char)0xA2; break;
        case 0xBA: resultado += (char)0xA3; break;
        case 0xB1: resultado += (char)0xA4; break;
        default: resultado += '?'; break;
      }
    } else {
      resultado += (char)c;
    }
  }

  return resultado;
}

void centradoOLED(const char* texto, int y) {
  String convertido = acentosOLED(texto);

  int16_t x1, y1;
  uint16_t ancho, alto;

  oled.getTextBounds(
    convertido.c_str(), 0, y,
    &x1, &y1, &ancho, &alto
  );

  oled.setCursor((128 - (int)ancho) / 2, y);
  oled.print(convertido);
}

void textoMovilOLED(const char* texto, int y) {
  const String convertido = acentosOLED(texto);
  const int ancho = convertido.length() * 6;

  if (ancho <= 128) {
    oled.setCursor((128 - ancho) / 2, y);
    oled.print(convertido);
    return;
  }

  const int ciclo = ancho + 36;
  const int desplazamiento = (millis() / 120) % ciclo;

  oled.setCursor(-desplazamiento, y);
  oled.print(convertido);

  oled.setCursor(ciclo - desplazamiento, y);
  oled.print(convertido);
}

void mensajeActual(const char*& titulo, const char*& texto) {
  titulo = nullptr;
  texto = nullptr;

  if (eventos[INCENDIO].retenido) {
    titulo = eventos[INCENDIO].detectando
        ? "Alerta" : "Alerta vigente";

    texto = eventos[INCENDIO].detectando
        ? "Posible riesgo de incendio en esta sala. Evacúe inmediatamente la zona."
        : "Detección finalizada. Espere indicaciones.";

  } else if (incendioRemotoRetenido) {
    titulo = incendioRemotoDetectando
        ? "Alerta" : "Alerta vigente";

    texto = incendioRemotoDetectando
        ? "Riesgo de incendio en sala de alto riesgo. Evacúe de inmediato la zona."
        : "Detección finalizada. Espere indicaciones.";

  } else if (eventos[EMERGENCIA].retenido) {
    titulo = "Emergencia manual";
    texto = "Evacúe la zona de inmediato, mantenga la calma.";

  } else if (eventos[SISMO].retenido) {
    titulo = eventos[SISMO].detectando
        ? "Alerta" : "Alerta vigente";

    texto = eventos[SISMO].detectando
        ? "Posible sismo detectado. Evacúe con calma y siga el protocolo."
        : "Detección finalizada. Espere indicaciones.";

  } else if (eventos[IMPACTO].retenido) {
    titulo = eventos[IMPACTO].detectando
        ? "Alerta" : "Alerta vigente";

    texto = eventos[IMPACTO].detectando
        ? "Impacto de alta energía detectado. Mantenga la calma y siga el protocolo."
        : "Detección finalizada. Espere indicaciones.";

  } else if (eventos[SIMULACRO].retenido) {
    titulo = "Simulacro";
    texto = "Evacúe la zona con calma.";
  }
}

void filaLCD(uint8_t fila, const char* texto) {
  if (!lcd) return;

  char salida[17];
  memset(salida, ' ', 16);
  salida[16] = '\0';

  const size_t longitud = strlen(texto);
  memcpy(salida, texto, longitud < 16 ? longitud : 16);

  if (memcmp(salida, cacheLCD[fila], 16) != 0) {
    lcd->setCursor(0, fila);
    lcd->print(salida);

    memcpy(cacheLCD[fila], salida, sizeof(salida));
  }
}

void actualizarPantallas() {
  const uint32_t ahora = millis();

  if (ahora - ultimaPantalla < 250) return;
  ultimaPantalla = ahora;

  if ((!lcd || !oledOK) &&
      ahora - ultimoIntentoPantallas >= 5000) {
    iniciarPantallas();
  }

  if (avisoActivo && ahora - inicioAviso >= 5000) {
    avisoActivo = false;
  }

  const time_t epoch = time(nullptr);
  char hora[16] = "--:-- --";

  if (epoch > 1700000000) {
    struct tm local;
    localtime_r(&epoch, &local);
    strftime(hora, sizeof(hora), "%I:%M %p", &local);
  }

  // LCD: bienvenida fija arriba.
  // Abajo: texto desplazable de 7 columnas + hora de 8.
  filaLCD(0, "Bienvenido");

  const bool corazon = (ahora / 6000) % 2;

  // \1 es el caracter ó definido con createChar().
  const char* leyenda = corazon
      ? "Coraz\1n llanero   "
      : "Sena Casanare   ";

  char fila[17];
  memset(fila, ' ', 16);
  fila[16] = '\0';

  const size_t longitud = strlen(leyenda);
  const size_t paso = (ahora % 6000) / 350;

  for (size_t i = 0; i < 7; i++) {
    fila[i] = leyenda[(paso + i) % longitud];
  }

  memcpy(fila + 8, hora, 8);
  filaLCD(1, fila);

  if (!oledOK) return;

  oled.clearDisplay();
  oled.setTextSize(1);

  const char* titulo;
  const char* texto;
  mensajeActual(titulo, texto);

  char cuenta[96];

  if (!titulo && toques == 1 && !gestoDuranteSimulacro) {
    const uint32_t transcurrido = ahora - primerToque;
    const uint32_t restante =
        transcurrido < 3000 ? (3000 - transcurrido + 999) / 1000 : 0;

    titulo = "Confirmar simulacro";

    snprintf(
      cuenta, sizeof(cuenta),
      "Inicia en %lu s. Toque otra vez para cancelar.",
      (unsigned long)restante
    );

    texto = cuenta;

  } else if (!titulo && avisoActivo) {
    titulo = avisoTitulo;
    texto = avisoTexto;
  }

  if (titulo) {
    centradoOLED(titulo, 5);
    oled.drawFastHLine(0, 20, 128, SSD1306_WHITE);
    textoMovilOLED(texto, 33);

  } else {
    centradoOLED(
      (ahora / 5000) % 2 ? "Sala de bajo riesgo" : "Bienvenido",
      5
    );

    oled.drawFastHLine(0, 21, 128, SSD1306_WHITE);

    char datos[32];

    if (dhtValido)
      snprintf(datos, sizeof(datos), "%s  %.1f C", hora, temperaturaAmbiente);
    else
      snprintf(datos, sizeof(datos), "%s  --.- C", hora);

    centradoOLED(datos, 32);

    // Rotar avisos tecnicos para que uno no oculte a los demas.
    const char* avisos[5];
    uint8_t cantidad = 0;

    if (!redWiFi.load())
      avisos[cantidad++] = "Sin Wi-Fi. Reintentando...";
    else if (!redMQTT.load())
      avisos[cantidad++] = "Sin broker. Reintentando...";

    if (epoch <= 1700000000)
      avisos[cantidad++] = "Sincronizando hora...";

    if (!dhtValido)
      avisos[cantidad++] = "Temperatura no disponible.";

    if (!mpuOK)
      avisos[cantidad++] = "Acelerómetro sin datos.";

    if (cantidad) {
      textoMovilOLED(avisos[(ahora / 4000) % cantidad], 53);
    }
  }

  oled.display();
}

// ======================================================
// CONTROL LOCAL
// ======================================================

void atenderSerial() {
  for (int i = 0; i < 16 && Serial.available(); i++) {
    char c = Serial.read();

    if (c == 'S' || c == 's') {
      cerrarAlertas();

    } else if (c == 'F' || c == 'f') {
      // Finalizacion autorizada de un simulacro.
      if (!hayAlertaReal()) cerrarSimulacro(false);

    } else if (c == '?') {
      Serial.println(
        "S: cerrar alertas sin detecciones activas\n"
        "F: finalizar simulacro\n"
        "Touch: 1 toque -> esperar 3 s\n"
        "3 toques <2 s -> emergencia\n"
        "2 toques <2 s durante simulacro -> falsa alarma"
      );
    }
  }
}

// Ultrasónico lecturas

void reportarUltrasonico() {
  static uint32_t ultimoReporte = 0;
  const uint32_t ahora = millis();

  if (ahora - ultimoReporte < 1000) {
    return;
  }

  ultimoReporte = ahora;

  if (distanciaValida) {
    Serial.printf(
      "[HC-SR04] Distancia: %.1f cm | Estado: %s\n",
      distanciaCm,
      estadoHC
    );
  } else {
    Serial.printf(
      "[HC-SR04] Sin lectura valida | Estado: %s | ECHO: %s\n",
      estadoHC,
      digitalRead(ECHO_PIN) == HIGH ? "HIGH" : "LOW"
    );
  }
}

// ======================================================
// SETUP Y LOOP
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\nPLACA 1 - SALA DE BAJO RIESGO");

  snprintf(
    sesion, sizeof(sesion),
    "p1-%08lX",
    (unsigned long)esp_random()
  );

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);

  busOLED.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  busOLED.setClock(400000);
  busOLED.setTimeOut(20);

  iniciarEntrada(touch);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ECHO_PIN),
    interrupcionEcho,
    CHANGE
  );

  dht.setup(DHT_PIN, DHTesp::DHT11);
  ultimoDHT = millis();

  iniciarPantallas();
  iniciarMPU();

  colaEventos = xQueueCreate(16, sizeof(Paquete));
  colaMetricas = xQueueCreate(1, sizeof(Paquete));
  colaRemotos = xQueueCreate(8, sizeof(IncendioRecibido));

  if (colaEventos && colaMetricas && colaRemotos) {
    if (xTaskCreate(
          tareaRed,
          "red_placa1",
          12288,
          nullptr,
          1,
          nullptr
        ) != pdPASS) {
      Serial.println("[RED] No se pudo crear la tarea.");
    }
  } else {
    Serial.println("[RED] No hay memoria para las colas.");
  }

  Serial.println("Sistema iniciado. Envia ? para ver controles.");
}

void loop() {
  procesarRemotos();
  atenderSerial();

  actualizarTouch();

  actualizarMPU();

  // Evita iniciar una medicion HC durante la lectura DHT.
  actualizarHC();
  reportarUltrasonico();
  actualizarDHT();

  atenderMetricas();
  actualizarPantallas();

  delay(1);
}