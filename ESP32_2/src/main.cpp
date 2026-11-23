#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ======================================================
#include "secrets.h"

constexpr uint16_t MQTT_PORT = 8883;

// IDENTIDAD Y TEMAS
// ======================================================

const char* PLACA = "placa2";
const char* SALA = "riesgo_alto";

const char* TOPIC_METRICAS =
    "sena/casanare/metricas";

const char* TOPIC_INCENDIO_LOCAL =
    "sena/casanare/placa2/eventos/incendio";

const char* TOPIC_EVENTOS_REMOTOS =
    "sena/casanare/placa1/eventos/+";

const char* TOPIC_COMANDOS =
    "sena/casanare/placa2/comandos/alarma";

const char* TOPIC_ESTADO_CONEXION =
    "sena/casanare/placa2/estado/conexion";

// ======================================================
// PINES
// ======================================================

constexpr uint8_t DHT_PIN = 17;
constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint8_t ALARMA_PIN = 16;

// Índices: 0 = LED 1; 1 = LED 2; 2 = LED 3.
constexpr uint8_t LED_VERDES[] = {18, 19, 23};

// ======================================================
// SENSORES IR — PLACA 2
// ======================================================

constexpr uint8_t IR_ENTRADA_PIN = 27;
constexpr uint8_t IR_SALIDA_PIN = 26;

// Los módulos utilizados detectan con OUT en LOW.
constexpr uint8_t IR_NIVEL_ACTIVO = LOW;
constexpr uint32_t IR_ANTIRREBOTE_MS = 30;

struct EstadoIR {
  uint8_t pin;
  int lecturaCruda;
  int lecturaEstable;
  uint32_t ultimoCambio;
  uint32_t detecciones;
};

EstadoIR irEntrada = {
  IR_ENTRADA_PIN, HIGH, HIGH, 0, 0
};

EstadoIR irSalida = {
  IR_SALIDA_PIN, HIGH, HIGH, 0, 0
};

// ======================================================
// TIEMPOS Y UMBRALES
// ======================================================

constexpr uint32_t DHT_INTERVALO_MS = 2500;
constexpr uint32_t METRICAS_INTERVALO_NORMAL_MS = 60000;
constexpr uint32_t METRICAS_INTERVALO_INCENDIO_MS = 1000;
constexpr uint32_t LED_INTERVALO_MS = 1000;
constexpr uint32_t TITULO_INTERVALO_MS = 5000;
constexpr uint32_t OLED_INTERVALO_MS = 250;

constexpr float UMBRAL_INCENDIO_C = 40.0f;
constexpr float UMBRAL_REARME_C = 37.0f;

constexpr long UTC_OFFSET = -5L * 3600L;

// ======================================================
// DISPOSITIVOS Y ESTADO LOCAL
// ======================================================

DHT dht(DHT_PIN, DHT11);

constexpr uint8_t MAX_FALLOS_DHT = 3;

uint8_t fallosConsecutivosDht = 0;
bool falloSensorActivo = false;

Adafruit_SSD1306 oled(
    128, 64, &Wire, -1, 100000, 100000
);

enum class Incidente : uint8_t {
  Ninguno,
  Incendio,
  Sismo,
  Impacto,
  Simulacro,
  Emergencia
};

enum class Origen : uint8_t {
  Local,
  Placa1
};

enum class AvisoOLED {
  Ninguno,
  FalsaAlarma,
  AlertaFinalizada,
  ManualFinalizada
};

AvisoOLED avisoOLED = AvisoOLED::Ninguno;

uint32_t inicioAvisoOLED = 0;
constexpr uint32_t DURACION_AVISO_OLED_MS = 5000;

// Nuevas variables.

int64_t ultimoMinutoMetricas = -1;
bool incendioAnteriorMetricas = false;
bool metricasRapidasSuspendidas = false;

// Fin nuevas variables.

Incidente incidenteActual = Incidente::Ninguno;
Origen origenActual = Origen::Local;

bool alarmaActiva = false;
bool salidaAlarma = false;
bool deteccionTemperaturaActiva = false;

// Control independiente del sonido y del aviso visual.
// alarmaActiva seguirá controlando el aviso de la OLED.
bool sonidoHabilitado = false;

constexpr uint32_t DURACION_SONIDO_SIMULACRO_MS = 30000;
uint32_t inicioSonidoSimulacro = 0;

bool oledDisponible = false;
bool mostrarSala = false;

bool temperaturaValida = false;
bool lecturaIntentada = false;
float temperatura = NAN;

// Comienza por LED 3.
uint8_t verdeActual = 2;

uint32_t ultimoDht = 0;
uint32_t ultimaMetrica = 0;
uint32_t ultimoLed = 0;
uint32_t ultimoPatron = 0;
uint32_t ultimoTitulo = 0;
uint32_t ultimoDibujo = 0;
uint32_t ultimoIntentoOled = 0;

char sesion[32] = {};
char eventoLocalId[64] = {};
uint32_t secuencia = 0;
uint32_t numeroEvento = 0;

// ======================================================
// COLAS ENTRE loop() Y LA TAREA DE RED
// ======================================================

struct MensajeMQTT {
  char json[1024];
};

struct OrdenAlarma {
  Incidente incidente;
  Origen origen;
  bool apagar;

  // Si es true, la orden solo puede cancelar un simulacro.
  bool soloSimulacro;

  // Permite mostrar el mensaje "Falsa alarma".
  bool falsaAlarma;

  bool soloEmergencia;
};

QueueHandle_t colaMetricas = nullptr;
QueueHandle_t colaEventos = nullptr;
QueueHandle_t colaOrdenes = nullptr;

// ======================================================
// ALARMAS Y LED
// ======================================================

void aplicarVerdes();

const char* nombreIncidente(Incidente incidente) {
  switch (incidente) {
    case Incidente::Incendio:   return "incendio";
    case Incidente::Sismo:      return "sismo";
    case Incidente::Impacto:    return "impacto";
    case Incidente::Simulacro:  return "simulacro";
    case Incidente::Emergencia: return "emergencia";
    default:                   return "ninguno";
  }
}

Incidente interpretarIncidente(const char* nombre) {
  if (!nombre) return Incidente::Ninguno;

  if (strcmp(nombre, "incendio") == 0)
    return Incidente::Incendio;

  if (strcmp(nombre, "sismo") == 0)
    return Incidente::Sismo;

  if (strcmp(nombre, "impacto") == 0)
    return Incidente::Impacto;

  if (strcmp(nombre, "simulacro") == 0)
    return Incidente::Simulacro;

  if (strcmp(nombre, "emergencia") == 0)
    return Incidente::Emergencia;

  return Incidente::Ninguno;
}

void activarAlarma(Incidente incidente, Origen origen) {
  if (incidente == Incidente::Ninguno) return;

  // Un simulacro no sustituye una alerta real.
  if (incidente == Incidente::Simulacro &&
      alarmaActiva &&
      incidenteActual != Incidente::Simulacro) {
    Serial.println("[ALARMA] Simulacro ignorado: existe una alerta real activa.");
    return;
  }

  if (alarmaActiva &&
      incidenteActual == incidente &&
      origenActual == origen) {
    return;
  }

  // La alerta real más reciente sustituye a la anterior.
  incidenteActual = incidente;
  origenActual = origen;
  alarmaActiva = true;

  avisoOLED = AvisoOLED::Ninguno;

  sonidoHabilitado = true;
  salidaAlarma = true;
  const uint32_t ahora = millis();
  ultimoPatron = ahora;
  ultimoLed = ahora;
  inicioSonidoSimulacro = ahora;

  digitalWrite(ALARMA_PIN, HIGH);
  verdeActual = 2;  // Comenzar por LED 3.
  aplicarVerdes();

  Serial.printf(
    "[ALARMA] %s | Origen: %s\n",
    nombreIncidente(incidente),
    origen == Origen::Placa1 ? "placa1" : "local"
  );

  if (incidente == Incidente::Simulacro) {
    Serial.println(
      "[SIMULACRO] Buzzer y LED rojos activos durante 30 segundos."
    );
    Serial.println(
      "[SIMULACRO] La OLED permanecera en alerta hasta recibir apagado."
    );
  }
}

void apagarAlarma() {
  metricasRapidasSuspendidas = true;

  // Eliminar la métrica pendiente, si todavía no se transmitió.
  if (colaMetricas) {
    xQueueReset(colaMetricas);
  }

  Serial.println(
    "[METRICAS] Apagado recibido. Regreso al envio por minuto."
  );

  alarmaActiva = false;
  sonidoHabilitado = false;
  salidaAlarma = false;
  incidenteActual = Incidente::Ninguno;
  origenActual = Origen::Local;
  inicioSonidoSimulacro = 0;

  digitalWrite(ALARMA_PIN, LOW);

  verdeActual = 2;
  ultimoLed = millis();
  aplicarVerdes();

  mostrarSala = false;
  ultimoTitulo = millis();

  // AQUÍ VAN LAS DOS LÍNEAS:
  avisoOLED = AvisoOLED::AlertaFinalizada;
  inicioAvisoOLED = millis();

  Serial.println(
    "[ALARMA] Apagado completo. Mostrando alerta finalizada."
  );
}

void aplicarVerdes() {
  for (uint8_t i = 0; i < 3; i++) {
    // Sin alerta, todos los verdes permanecen apagados.
    bool encender = alarmaActiva && (i == verdeActual);

    // Mientras suena la alarma, siguen sus pulsos.
    if (sonidoHabilitado) {
      encender = encender && salidaAlarma;
    }

    digitalWrite(
      LED_VERDES[i],
      encender ? HIGH : LOW
    );
  }
}

void mostrarAyuda() {
  Serial.println("\nPRUEBAS LOCALES; NO PUBLICAN INCIDENTES:");
  Serial.println("1: incendio local");
  Serial.println("2: sismo");
  Serial.println("3: impacto");
  Serial.println("4 o A: simulacro");
  Serial.println("5: incendio de la otra sala");
  Serial.println("S: apagar | ?: ayuda");
}

void atenderSerial() {
  for (uint8_t i = 0; i < 32 && Serial.available(); i++) {
    const char c = Serial.read();

    switch (c) {
      case '1':
        activarAlarma(Incidente::Incendio, Origen::Local);
        break;

      case '2':
        activarAlarma(Incidente::Sismo, Origen::Local);
        break;

      case '3':
        activarAlarma(Incidente::Impacto, Origen::Local);
        break;

      case '4':
      case 'A':
      case 'a':
        activarAlarma(Incidente::Simulacro, Origen::Local);
        break;

      case '5':
        activarAlarma(Incidente::Incendio, Origen::Placa1);
        break;

      case 'S':
      case 's':
        apagarAlarma();
        break;

      case '?':
        mostrarAyuda();
        break;
    }
  }
}

void procesarOrdenes() {
  if (!colaOrdenes) return;

  OrdenAlarma orden = {};

  for (uint8_t i = 0; i < 8; i++) {
    if (xQueueReceive(colaOrdenes, &orden, 0) != pdPASS) {
      break;
    }

    if (!orden.apagar) {
      activarAlarma(orden.incidente, orden.origen);
      continue;
    }

    if (orden.soloSimulacro) {
      // Una cancelación de simulacro no apaga un incendio,
      // sismo, impacto o emergencia manual.
      if (!alarmaActiva ||
          incidenteActual != Incidente::Simulacro) {
        Serial.println(
          "[ALARMA] Cancelacion ignorada: no hay simulacro activo."
        );
        continue;
      }
    }

    if (orden.soloEmergencia &&
        (!alarmaActiva ||
        incidenteActual != Incidente::Emergencia)) {
      continue;
    }

    apagarAlarma();

    if (orden.soloEmergencia) {
      avisoOLED = AvisoOLED::ManualFinalizada;
      inicioAvisoOLED = millis();

      Serial.println(
        "[EMERGENCIA] Alarma manual finalizada."
      );

    } else if (orden.falsaAlarma) {
      avisoOLED = AvisoOLED::FalsaAlarma;
      inicioAvisoOLED = millis();

      Serial.println(
        "[SIMULACRO] Cancelado por falsa alarma."
      );
    }
  }
}

// ======================================================
// PUBLICACIONES JSON
// ======================================================

void agregarCabecera(JsonDocument& doc, const char* tipo) {
  doc["version"] = 1;
  doc["placa"] = PLACA;
  doc["sala"] = SALA;
  doc["sesion"] = sesion;
  doc["secuencia"] = ++secuencia;
  doc["tipo"] = tipo;
  doc["tiempo_activo_ms"] = millis();

  const time_t ahora = time(nullptr);

  if (ahora > 1700000000) {
    struct tm utc;
    gmtime_r(&ahora, &utc);

    char fecha[25];
    strftime(
      fecha, sizeof(fecha),
      "%Y-%m-%dT%H:%M:%SZ", &utc
    );

    doc["fecha"] = fecha;
  } else {
    doc["fecha"] = nullptr;
  }
}

void encolarJSON(JsonDocument& doc, bool esEvento) {
  MensajeMQTT mensaje = {};

  if (doc.overflowed() ||
      measureJson(doc) >= sizeof(mensaje.json)) {
    Serial.println("[JSON] Error de memoria/tamano.");
    return;
  }

  serializeJson(doc, mensaje.json, sizeof(mensaje.json));
  Serial.printf("[JSON] %s\n", mensaje.json);

  if (esEvento) {
    if (!colaEventos ||
        xQueueSend(colaEventos, &mensaje, 0) != pdPASS) {
      Serial.println("[MQTT] Evento no guardado: cola llena/no disponible.");
    }
  } else if (colaMetricas) {
    // Si no hay conexión, conserva solo la métrica más reciente.
    xQueueOverwrite(colaMetricas, &mensaje);
  }
}

void prepararMetrica() {
  JsonDocument doc;
  agregarCabecera(doc, "metricas");

  // Temperatura.
  doc["temperatura"]["valida"] = temperaturaValida;

  doc["temperatura"]["estado_sensor"] =
      !lecturaIntentada ? "esperando" :
      temperaturaValida ? "ok" :
      falloSensorActivo ? "fallo" : "sin_datos";

  doc["temperatura"]["fallos_consecutivos"] =
      fallosConsecutivosDht;

  doc["temperatura"]["deteccion_incendio_activa"] =
      deteccionTemperaturaActiva;

  if (temperaturaValida) {
    doc["temperatura"]["valor_c"] = temperatura;
  } else {
    doc["temperatura"]["valor_c"] = nullptr;
  }

  // Infrarrojos.
  doc["ir"]["entrada"] =
      irEntrada.lecturaEstable == IR_NIVEL_ACTIVO;

  doc["ir"]["salida"] =
      irSalida.lecturaEstable == IR_NIVEL_ACTIVO;

  doc["ir"]["detecciones_entrada"] =
      irEntrada.detecciones;

  doc["ir"]["detecciones_salida"] =
      irSalida.detecciones;

  encolarJSON(doc, false);
}

void prepararEventoIncendio(bool estado) {
  JsonDocument doc;
  agregarCabecera(doc, "evento");

  doc["evento_id"] = eventoLocalId;
  doc["evento"] = "incendio";
  doc["estado"] = estado;

  doc["evidencia"]["origen"] = "temperatura_alta";
  doc["evidencia"]["temperatura_c"] = temperatura;
  doc["evidencia"]["umbral_c"] = UMBRAL_INCENDIO_C;
  doc["evidencia"]["rearme_inferior_a_c"] = UMBRAL_REARME_C;

  encolarJSON(doc, true);
}

// ======================================================
// DHT11
// ======================================================


void leerTemperatura() {
  lecturaIntentada = true;

  Serial.println("[DHT11] Leyendo...");
  temperatura = dht.readTemperature();
  temperaturaValida = isfinite(temperatura);

  // ==============================
  // DETECCION DE FALLOS
  // ==============================

  if (!temperaturaValida) {

    if (fallosConsecutivosDht < MAX_FALLOS_DHT) {
      fallosConsecutivosDht++;
    }

    Serial.printf(
      "[DHT11] Lectura invalida (%u/%u)\n",
      fallosConsecutivosDht,
      MAX_FALLOS_DHT
    );

    if (fallosConsecutivosDht >= MAX_FALLOS_DHT &&
        !falloSensorActivo) {

      falloSensorActivo = true;

      Serial.println(
        "[DHT11] ALERTA: fallo del sensor detectado."
      );
    }

    return;
  }

  // ==============================
  // RECUPERACION DEL SENSOR
  // ==============================

  fallosConsecutivosDht = 0;

  if (falloSensorActivo) {
    falloSensorActivo = false;

    Serial.println(
      "[DHT11] Sensor recuperado correctamente."
    );
  }

  Serial.printf("[DHT11] %.1f C\n", temperatura);

  // ==============================
  // DETECCION DE INCENDIO
  // ==============================

  if (!deteccionTemperaturaActiva &&
    temperatura >= UMBRAL_INCENDIO_C) {

    deteccionTemperaturaActiva = true;
    metricasRapidasSuspendidas = false;

    snprintf(
      eventoLocalId,
      sizeof(eventoLocalId),
      "%s-%lu",
      sesion,
      (unsigned long)++numeroEvento
    );

    activarAlarma(
      Incidente::Incendio,
      Origen::Local
    );

    prepararEventoIncendio(true);

  } else if (deteccionTemperaturaActiva &&
             temperatura < UMBRAL_REARME_C) {

    deteccionTemperaturaActiva = false;

    prepararEventoIncendio(false);

    Serial.println(
      "[DETECCION] Rearmada. "
      "La alarma requiere orden de apagado."
    );
  }
}


// ======================================================
// OLED
// ======================================================

bool respondeI2C(uint8_t direccion) {
  Wire.beginTransmission(direccion);
  return Wire.endTransmission() == 0;
}

void iniciarOLED() {
  ultimoIntentoOled = millis();

  uint8_t direccion = 0;
  if (respondeI2C(0x3C)) direccion = 0x3C;
  else if (respondeI2C(0x3D)) direccion = 0x3D;

  if (!direccion) {
    oledDisponible = false;
    Serial.println("[OLED] Sin respuesta en 0x3C/0x3D.");
    return;
  }

  oledDisponible = oled.begin(
    SSD1306_SWITCHCAPVCC,
    direccion,
    false,
    false
  );

  if (!oledDisponible) {
    Serial.println("[OLED] Fallo de inicializacion.");
    return;
  }

  oled.setTextColor(SSD1306_WHITE);
  oled.setTextWrap(false);
  oled.cp437(true);
  oled.clearDisplay();
  oled.display();

  Serial.printf("[OLED] Inicializada en 0x%02X.\n", direccion);
}

void textoCentrado(const char* texto, int16_t y) {
  int16_t x1, y1;
  uint16_t ancho, alto;

  oled.getTextBounds(
    texto, 0, y,
    &x1, &y1, &ancho, &alto
  );

  oled.setCursor((128 - (int16_t)ancho) / 2, y);
  oled.print(texto);
}

String convertirAcentos(const char* texto) {
  String salida;
  salida.reserve(strlen(texto));

  for (size_t i = 0; texto[i] != '\0'; i++) {
    const uint8_t c = (uint8_t)texto[i];

    if (c == 0xC3 && texto[i + 1] != '\0') {
      const uint8_t siguiente = (uint8_t)texto[++i];

      switch (siguiente) {
        case 0xA1: salida += (char)0xA0; break; // á
        case 0xA9: salida += (char)0x82; break; // é
        case 0xAD: salida += (char)0xA1; break; // í
        case 0xB3: salida += (char)0xA2; break; // ó
        case 0xBA: salida += (char)0xA3; break; // ú
        case 0xB1: salida += (char)0xA4; break; // ñ
        default: salida += '?'; break;
      }
    } else {
      salida += (char)c;
    }
  }

  return salida;
}

// Los mensajes configurados caben en cinco líneas de 21 caracteres.
void dibujarParrafo(const char* texto) {
  const String convertido = convertirAcentos(texto);
  String linea;

  int inicio = 0;
  const int longitud = convertido.length();
  int16_t y = 20;

  oled.setTextSize(1);

  while (inicio < longitud) {
    int fin = convertido.indexOf(' ', inicio);
    if (fin < 0) fin = longitud;

    const String palabra = convertido.substring(inicio, fin);

    const int necesario =
        linea.length() +
        (linea.length() ? 1 : 0) +
        palabra.length();

    if (necesario > 21 && linea.length()) {
      oled.setCursor(0, y);
      oled.print(linea);
      linea = "";
      y += 9;
    }

    if (linea.length()) linea += ' ';
    linea += palabra;
    inicio = fin + 1;
  }

  if (linea.length()) {
    oled.setCursor(0, y);
    oled.print(linea);
  }
}

void dibujarOLED() {
  if (!oledDisponible) return;

  const uint32_t ahoraMs = millis();

  if (avisoOLED != AvisoOLED::Ninguno &&
      ahoraMs - inicioAvisoOLED >= DURACION_AVISO_OLED_MS) {
    avisoOLED = AvisoOLED::Ninguno;

    mostrarSala = false;
    ultimoTitulo = ahoraMs;
  }

  oled.clearDisplay();

  if (alarmaActiva) {
    // Las alertas tienen prioridad sobre los avisos temporales.
    if (incidenteActual == Incidente::Emergencia) {
      oled.setTextSize(1);
      textoCentrado("Emergencia manual", 4);

    } else {
      oled.setTextSize(2);

      textoCentrado(
        incidenteActual == Incidente::Simulacro
            ? "Simulacro"
            : "Alerta",
        0
      );
    }

    switch (incidenteActual) {
      case Incidente::Incendio:
        if (origenActual == Origen::Placa1) {
          dibujarParrafo(
            "Riesgo de incendio en sala de bajo riesgo, "
            "evacúe de inmediato la zona."
          );
        } else {
          dibujarParrafo(
            "Posible riesgo de incendio, "
            "evacúe inmediatamente la zona."
          );
        }
        break;

      case Incidente::Sismo:
        dibujarParrafo(
          "Posible sismo detectado. "
          "Evacúe con calma y siga el protocolo."
        );
        break;

      case Incidente::Impacto:
        dibujarParrafo(
          "Impacto de alta energía detectado. "
          "Mantenga la calma y siga el protocolo."
        );
        break;

      case Incidente::Simulacro:
        dibujarParrafo(
          "Evacúe la zona con calma."
        );
        break;

      case Incidente::Emergencia:
        dibujarParrafo(
          "Evacúe la zona de inmediato, "
          "mantenga la calma."
        );
        break;

      default:
        break;
    }

  } else if (avisoOLED != AvisoOLED::Ninguno) {
    oled.setTextSize(1);

    if (avisoOLED == AvisoOLED::FalsaAlarma) {
      textoCentrado("Falsa alarma", 4);
      dibujarParrafo("Simulacro cancelado.");

    } else if (avisoOLED == AvisoOLED::ManualFinalizada) {
      textoCentrado("Alarma manual", 4);
      dibujarParrafo("finalizada.");

    } else {
      textoCentrado("Alerta finalizada", 4);
      dibujarParrafo(
        "Siga las indicaciones del responsable."
      );
    }

  } else {
    oled.setTextSize(1);

    textoCentrado(
      mostrarSala
          ? "Sala de alto riesgo"
          : "Bienvenido",
      12
    );

    oled.drawFastHLine(
      0, 28, 128, SSD1306_WHITE
    );

    char hora[16] = "--:-- --";

    const time_t ahora = time(nullptr);

    if (ahora > 1700000000) {
      struct tm local;
      localtime_r(&ahora, &local);

      strftime(
        hora,
        sizeof(hora),
        "%I:%M %p",
        &local
      );
    }

    char linea[32];

    if (temperaturaValida) {
      snprintf(
        linea,
        sizeof(linea),
        "%s  %.1f C",
        hora,
        temperatura
      );

    } else {
      snprintf(
        linea,
        sizeof(linea),
        "%s  --.- C",
        hora
      );
    }

    textoCentrado(linea, 39);

    if (!temperaturaValida) {
      textoCentrado(
        lecturaIntentada
            ? "DHT11 sin datos"
            : "Esperando DHT11",
        54
      );

    } else if (WiFi.status() != WL_CONNECTED) {
      textoCentrado("Sin Wi-Fi", 54);

    } else if (ahora <= 1700000000) {
      textoCentrado("Sincronizando hora", 54);
    }
  }

  oled.display();
}

// ======================================================
// RECEPCION MQTT
// ======================================================

// Historial limitado de eventos remotos procesados.
// Solo lo utiliza la tarea de red.
char idsRecibidos[16][64] = {};
uint8_t siguienteId = 0;

bool eventoYaRecibido(const char* id) {
  if (id[0] == '\0') return false;

  for (uint8_t i = 0; i < 16; i++) {
    if (strcmp(idsRecibidos[i], id) == 0) return true;
  }

  return false;
}

void revisarTiempoSimulacro(uint32_t ahora) {
  // El límite de tiempo solo se aplica al simulacro.
  if (!alarmaActiva ||
      incidenteActual != Incidente::Simulacro ||
      !sonidoHabilitado) {
    return;
  }

  if (ahora - inicioSonidoSimulacro <
      DURACION_SONIDO_SIMULACRO_MS) {
    return;
  }

  // Silencia G16 sin borrar el incidente ni la alerta visual.
  sonidoHabilitado = false;
  salidaAlarma = false;

  digitalWrite(ALARMA_PIN, LOW);

  // Los verdes vuelven a su secuencia independiente.
  verdeActual = 2;
  ultimoLed = ahora;
  aplicarVerdes();

  Serial.println(
    "[SIMULACRO] Tiempo cumplido: buzzer y LED rojos apagados."
  );
  Serial.println(
    "[SIMULACRO] OLED activa. Esperando action:false desde MQTT o S."
  );
}

void atenderPatron(uint32_t ahora) {
  if (!alarmaActiva || !sonidoHabilitado) return;

  uint32_t tiempoEncendido = 500;
  uint32_t tiempoApagado = 500;

  switch (incidenteActual) {
    case Incidente::Incendio:
      tiempoEncendido = 150;
      tiempoApagado = 150;
      break;

    case Incidente::Sismo:
      tiempoEncendido = 400;
      tiempoApagado = 400;
      break;

    case Incidente::Impacto:
      // Conserva la cadencia de tu último archivo.
      tiempoEncendido = 100;
      tiempoApagado = 100;
      break;

    case Incidente::Simulacro:
      tiempoEncendido = 700;
      tiempoApagado = 700;
      break;

    case Incidente::Emergencia:
      tiempoEncendido = 250;
      tiempoApagado = 250;
      break;

    default:
      return;
  }

  const uint32_t intervalo =
      salidaAlarma ? tiempoEncendido : tiempoApagado;

  if (ahora - ultimoPatron >= intervalo) {
    ultimoPatron = ahora;
    salidaAlarma = !salidaAlarma;

    digitalWrite(
      ALARMA_PIN,
      salidaAlarma ? HIGH : LOW
    );

    // Durante el sonido, conserva los verdes
    // sincronizados con cada nuevo pulso.
    if (salidaAlarma) {
      verdeActual = (verdeActual + 2) % 3;
    }

    aplicarVerdes();
  }
}



void recordarEvento(const char* id) {
  if (id[0] == '\0') return;

  snprintf(
    idsRecibidos[siguienteId],
    sizeof(idsRecibidos[siguienteId]),
    "%s",
    id
  );

  siguienteId = (siguienteId + 1) % 16;
}

void recibirMQTT(
    char* topic,
    byte* payload,
    unsigned int longitud) {

  if (!colaOrdenes || longitud == 0 || longitud > 600) {
    Serial.println("[MQTT RX] Mensaje no procesable.");
    return;
  }

  JsonDocument doc;

  const DeserializationError error =
      deserializeJson(doc, payload, longitud);

  if (error) {
    Serial.printf(
      "[MQTT RX] JSON invalido: %s\n",
      error.c_str()
    );
    return;
  }

  OrdenAlarma orden = {};
  orden.incidente = Incidente::Ninguno;
  orden.origen = Origen::Local;

  const bool esComando =
      strcmp(topic, TOPIC_COMANDOS) == 0;

  const char* id = "";

  if (esComando) {
    if (!doc["action"].is<bool>()) {
      Serial.println(
        "[MQTT RX] action debe ser booleano."
      );
      return;
    }

    const bool activar = doc["action"].as<bool>();

    if (activar) {
      const char* evento = doc["evento"] | "";

      orden.incidente = interpretarIncidente(evento);

      if (orden.incidente == Incidente::Ninguno) {
        Serial.println(
          "[MQTT RX] Evento de comando no reconocido."
        );
        return;
      }

      orden.origen = Origen::Local;

    } else {
      orden.apagar = true;

      const char* alcance = doc["alcance"] | "todas";
      const char* motivo = doc["motivo"] | "";

      if (strcmp(alcance, "simulacro") == 0) {
        orden.soloSimulacro = true;

      } else if (strcmp(alcance, "emergencia") == 0) {
        orden.soloEmergencia = true;

      } else if (strcmp(alcance, "todas") != 0) {
        Serial.println("[MQTT RX] Alcance no reconocido.");
        return;
      }

      orden.falsaAlarma =
          strcmp(motivo, "falsa_alarma") == 0;

      // Evita etiquetar un apagado general como
      // falsa alarma de un simulacro.
      if (orden.falsaAlarma && !orden.soloSimulacro) {
        Serial.println(
          "[MQTT RX] falsa_alarma requiere alcance simulacro."
        );
        return;
      }
    }

  } else {
    if (!doc["estado"].is<bool>()) {
      Serial.println(
        "[MQTT RX] estado debe ser booleano."
      );
      return;
    }

    const char* evento = doc["evento"] | "";

    orden.incidente = interpretarIncidente(evento);

    if (orden.incidente == Incidente::Ninguno) {
      Serial.println(
        "[MQTT RX] Evento no reconocido."
      );
      return;
    }

    char topicEsperado[100];

    snprintf(
      topicEsperado,
      sizeof(topicEsperado),
      "sena/casanare/placa1/eventos/%s",
      evento
    );

    if (strcmp(topic, topicEsperado) != 0) {
      Serial.println(
        "[MQTT RX] Tema y evento no coinciden."
      );
      return;
    }

    if (!doc["estado"].as<bool>()) {
      Serial.printf(
        "[MQTT RX] Finalizo deteccion de %s; "
        "se conserva la alerta hasta recibir apagado.\n",
        evento
      );

      return;
    }

    orden.origen = Origen::Placa1;

    id = doc["evento_id"] | "";

    if (strlen(id) >= sizeof(idsRecibidos[0])) {
      Serial.println(
        "[MQTT RX] evento_id demasiado largo."
      );
      return;
    }

    if (eventoYaRecibido(id)) {
      Serial.println(
        "[MQTT RX] Evento duplicado ignorado."
      );
      return;
    }
  }

  if (xQueueSend(colaOrdenes, &orden, 0) != pdPASS) {
    Serial.println(
      "[MQTT RX] Cola llena; orden descartada."
    );
    return;
  }

  if (!esComando) {
    recordarEvento(id);
  }

  Serial.printf(
    "[MQTT RX] Orden aceptada desde %s\n",
    topic
  );
}

// ======================================================
// TAREA DE RED
// ======================================================

void tareaRed(void*) {
  WiFiClientSecure tls;
  PubSubClient mqtt(tls);

  tls.setCACert(MQTT_CA);
  tls.setHandshakeTimeout(5);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(recibirMQTT);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(3);

  bool mqttHabilitado =
      MQTT_USER[0] != '\0' &&
      MQTT_PASSWORD[0] != '\0';

  if (!mqtt.setBufferSize(1280)) {
    mqttHabilitado = false;
    Serial.println("[MQTT] Sin memoria para el buffer.");
  }

  if (!mqttHabilitado) {
    Serial.println("[MQTT] Completa credenciales o revisa memoria.");
  }

  if (WIFI_SSID[0] == '\0' || WIFI_PASSWORD[0] == '\0') {
    Serial.println("[WiFi] Completa credenciales. Prueba local disponible.");
    vTaskDelete(nullptr);
    return;
  }

  const uint64_t mac = ESP.getEfuseMac();
  char clientId[40];

  snprintf(
    clientId, sizeof(clientId),
    "esp32-placa2-%04X%08lX",
    (unsigned int)((mac >> 32) & 0xFFFF),
    (unsigned long)(mac & 0xFFFFFFFF)
  );

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t inicioWiFi = millis();
  uint32_t ultimoIntentoMQTT = 0;

  bool wifiAnterior = false;
  bool primerIntentoMQTT = true;
  bool esperandoHora = false;

  bool eventoPendiente = false;
  MensajeMQTT evento = {};

  Serial.println("[WiFi] Iniciando conexion.");

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiAnterior) {
        Serial.println("[WiFi] Conexion perdida.");
        inicioWiFi = millis();
      }

      wifiAnterior = false;
      primerIntentoMQTT = true;
      tls.stop();

      if (millis() - inicioWiFi >= 30000) {
        Serial.println("[WiFi] Reintentando conexion.");

        if (WiFi.disconnect(false, false)) {
          vTaskDelay(pdMS_TO_TICKS(1000));
          WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }

        inicioWiFi = millis();
      }

      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (!wifiAnterior) {
      wifiAnterior = true;

      Serial.print("[WiFi] Conectado. IP: ");
      Serial.println(WiFi.localIP());

      configTime(
        UTC_OFFSET,
        0,
        "pool.ntp.org",
        "time.nist.gov"
      );

      esperandoHora = true;
      Serial.println("[RELOJ] Esperando sincronizacion NTP.");
    }

    // La hora válida es necesaria para validar el certificado.
    if (time(nullptr) <= 1700000000) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (esperandoHora) {
      esperandoHora = false;
      Serial.println("[RELOJ] Hora disponible.");
    }

    if (!mqttHabilitado) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (!mqtt.connected() &&
        (primerIntentoMQTT ||
         millis() - ultimoIntentoMQTT >= 10000)) {
      primerIntentoMQTT = false;
      tls.stop();

      Serial.printf(
        "[MQTT] Conectando a %s:%u\n",
        MQTT_HOST,
        MQTT_PORT
      );


      if (mqtt.connect(clientId, MQTT_USER, MQTT_PASSWORD, TOPIC_ESTADO_CONEXION, 1, true, "{\"online\":false}")) {

        Serial.printf("[MQTT] Conectado como %s\n", clientId);

        // =====================================
        // PUBLICAR ESTADO ONLINE
        // =====================================

        if (mqtt.publish(
          TOPIC_ESTADO_CONEXION,
          "{\"online\":true}",
          true
        )) {

          Serial.println("[MQTT] Estado online publicado.");

        } else {

          Serial.println("[MQTT] Error al publicar estado online.");
        }

        // =====================================
        // SUSCRIPCIONES MQTT
        // =====================================

        const bool eventos =
            mqtt.subscribe(TOPIC_EVENTOS_REMOTOS, 0);

        const bool comandos =
            mqtt.subscribe(TOPIC_COMANDOS, 0);

        if (eventos && comandos) {

          Serial.println("[MQTT] Suscripciones solicitadas.");

        } else {

          Serial.println("[MQTT] Fallo al solicitar suscripciones.");
          tls.stop();
        }

      } else {

        Serial.printf(
          "[MQTT] Fallo de conexion. Estado=%d\n",
          mqtt.state()
        );
      }


      ultimoIntentoMQTT = millis();
    }

    if (mqtt.connected() && mqtt.loop()) {
      // Los eventos tienen prioridad sobre las métricas.
      if (!eventoPendiente) {
        eventoPendiente =
            xQueueReceive(colaEventos, &evento, 0) == pdPASS;
      }

      if (eventoPendiente) {
        if (mqtt.publish(
              TOPIC_INCENDIO_LOCAL,
              evento.json,
              false
            )) {
          eventoPendiente = false;
          Serial.println("[MQTT] Evento enviado por el cliente.");
        } else {
          tls.stop();
        }

      } else {
        MensajeMQTT metrica = {};

        if (xQueueReceive(colaMetricas, &metrica, 0) == pdPASS) {
          if (mqtt.publish(
                TOPIC_METRICAS,
                metrica.json,
                false
              )) {
            Serial.println("[MQTT] Metrica enviada por el cliente.");
          } else {
            // Reinsertar si no llegó una métrica más reciente.
            xQueueSend(colaMetricas, &metrica, 0);
            tls.stop();
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void iniciarIR(EstadoIR& sensor) {
  pinMode(sensor.pin, INPUT);

  // Tomar el estado inicial sin contarlo como una detección.
  const int lectura = digitalRead(sensor.pin);

  sensor.lecturaCruda = lectura;
  sensor.lecturaEstable = lectura;
  sensor.ultimoCambio = millis();
  sensor.detecciones = 0;
}

bool actualizarIR(EstadoIR& sensor) {
  const uint32_t ahora = millis();
  const int lectura = digitalRead(sensor.pin);

  if (lectura != sensor.lecturaCruda) {
    sensor.lecturaCruda = lectura;
    sensor.ultimoCambio = ahora;
  }

  // Esperar a que la lectura permanezca estable.
  if (ahora - sensor.ultimoCambio < IR_ANTIRREBOTE_MS) {
    return false;
  }

  if (sensor.lecturaEstable == sensor.lecturaCruda) {
    return false;
  }

  sensor.lecturaEstable = sensor.lecturaCruda;

  // Contar únicamente una nueva transición hacia detección.
  if (sensor.lecturaEstable == IR_NIVEL_ACTIVO) {
    sensor.detecciones++;
    return true;
  }

  return false;
}

void atenderSensoresIR() {
  if (actualizarIR(irEntrada)) {
    Serial.printf(
      "[IR] ENTRADA detectada | G27 | Total: %lu\n",
      (unsigned long)irEntrada.detecciones
    );
  }

  if (actualizarIR(irSalida)) {
    Serial.printf(
      "[IR] SALIDA detectada | G26 | Total: %lu\n",
      (unsigned long)irSalida.detecciones
    );
  }
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  digitalWrite(ALARMA_PIN, LOW);
  pinMode(ALARMA_PIN, OUTPUT);

  for (uint8_t pin : LED_VERDES) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }

  Serial.begin(115200);
  delay(300);

  iniciarIR(irEntrada);
  iniciarIR(irSalida);

  Serial.println("[IR] Entrada=G27 | Salida=G26 | Activos en LOW");

  Serial.println("\nESP32 2 - SALA DE ALTO RIESGO");
  Serial.println("DHT11=G17 | OLED SDA=G21 SCL=G22");
  Serial.println("Verdes: G23 -> G19 -> G18");
  Serial.println("Alarma: G16 mediante transistor");
  Serial.println("Trigger >=40 C | Rearme <37 C");
  mostrarAyuda();

  snprintf(
    sesion,
    sizeof(sesion),
    "p2-%08lX",
    (unsigned long)esp_random()
  );

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  dht.begin();
  iniciarOLED();
  aplicarVerdes();

  colaMetricas = xQueueCreate(1, sizeof(MensajeMQTT));
  colaEventos = xQueueCreate(8, sizeof(MensajeMQTT));
  colaOrdenes = xQueueCreate(8, sizeof(OrdenAlarma));

  if (colaMetricas && colaEventos && colaOrdenes) {
    if (xTaskCreate(
          tareaRed,
          "red",
          12288,
          nullptr,
          1,
          nullptr
        ) != pdPASS) {
      Serial.println("[RED] No se pudo crear la tarea.");
    }
  } else {
    Serial.println("[RED] Sin memoria para las colas.");
  }

  const uint32_t ahora = millis();
  ultimoDht = ahora;
  ultimaMetrica = ahora;
  ultimoLed = ahora;
  ultimoTitulo = ahora;
}

// ======================================================
// LOOP
// ======================================================


void loop() {
  atenderSensoresIR();

  atenderSerial();
  procesarOrdenes();

  const uint32_t ahora = millis();

  // Temporizador del simulacro y patrón del buzzer.
  revisarTiempoSimulacro(ahora);
  atenderPatron(ahora);

  // Verdes: solo funcionan mientras exista una alerta.
  // Después de silenciar el simulacro, siguen 3 -> 2 -> 1.
  if (alarmaActiva &&
      !sonidoHabilitado &&
      ahora - ultimoLed >= LED_INTERVALO_MS) {

    ultimoLed = ahora;
    verdeActual = (verdeActual + 2) % 3;
    aplicarVerdes();
  }

  // Leer el DHT11 cada 2.5 segundos.
  if (ahora - ultimoDht >= DHT_INTERVALO_MS) {
    ultimoDht = ahora;
    leerTemperatura();
  }

  // ==========================================
  // PUBLICACION DE METRICAS
  // ==========================================

  const time_t ahoraEpoch = time(nullptr);
  const bool horaDisponible = ahoraEpoch > 1700000000;

  bool cambioDeMinuto = false;

  if (horaDisponible) {
    const int64_t minutoActual =
        static_cast<int64_t>(ahoraEpoch) / 60;

    if (ultimoMinutoMetricas < 0) {
      // Al sincronizar por primera vez, esperar
      // al siguiente cambio de minuto.
      ultimoMinutoMetricas = minutoActual;

    } else if (minutoActual != ultimoMinutoMetricas) {
      ultimoMinutoMetricas = minutoActual;
      cambioDeMinuto = true;
    }

  } else {
    ultimoMinutoMetricas = -1;
  }

  const bool inicioIncendio =
      deteccionTemperaturaActiva &&
      !incendioAnteriorMetricas;

  incendioAnteriorMetricas = deteccionTemperaturaActiva;

  if (deteccionTemperaturaActiva &&
    !metricasRapidasSuspendidas) {
    // Métrica inmediata al comenzar el incendio;
    // después, una por segundo.
    if (inicioIncendio ||
        ahora - ultimaMetrica >=
            METRICAS_INTERVALO_INCENDIO_MS) {

      ultimaMetrica = ahora;
      prepararMetrica();
    }

  } else if (horaDisponible) {
    // En condiciones normales:
    // 08:05:00, 08:06:00, 08:07:00...
    if (cambioDeMinuto) {
      ultimaMetrica = ahora;
      prepararMetrica();
    }

  } else {
    // Sin hora sincronizada, publicar cada 60 segundos.
    if (ahora - ultimaMetrica >=
        METRICAS_INTERVALO_NORMAL_MS) {

      ultimaMetrica = ahora;
      prepararMetrica();
    }
  }

  // ==========================================
  // PANTALLA OLED
  // ==========================================

  // Alternar bienvenida y nombre de la sala.
  if (!alarmaActiva &&
      ahora - ultimoTitulo >= TITULO_INTERVALO_MS) {

    ultimoTitulo = ahora;
    mostrarSala = !mostrarSala;
  }

  // Reintentar inicialización si no está disponible.
  if (!oledDisponible &&
      ahora - ultimoIntentoOled >= 5000) {

    iniciarOLED();
  }

  // Actualizar la pantalla.
  if (oledDisponible &&
      ahora - ultimoDibujo >= OLED_INTERVALO_MS) {

    ultimoDibujo = ahora;
    dibujarOLED();
  }

  delay(1);
}