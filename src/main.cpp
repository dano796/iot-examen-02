// Examen 2 - IoT - End Devices
// Daniel Ortiz Aristizabal - ID 000186841
//
// T-Beam SX1278 + HDC1080 (temperatura) + GPS NEO-6M -> colector LoRa
//
// Máquina de estados:
//
//     CHIRP  ->  PRUNNING  ->  BUNDLING  ->  TX  -+
//       ^                                         |
//       +-----------------------------------------+
//
//   CHIRP     proceso de toma de medidas: lee N muestras del HDC1080
//             respetando el tiempo de recuperacion del sensor entre cada una
//   PRUNNING  proceso de consolidacion de una medida: descarta las muestras
//             atípicas y promedia las que sobreviven. Consolida tambien la
//             posición del GPS.
//   BUNDLING  proceso de conformación de los datos: arma las tramas JSON
//   TX        transmite el paquete
//
// Espera con smartDelay(), que mientras cuenta el tiempo sigue alimentando el 
// parser NMEA. A 9600 baud el UART del GPS se desborda si se deja de leer. Es
// por esto que no se usa delay().
//

#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <ClosedCube_HDC1080.h>
#include "LoRaBoards.h"

#ifndef CONFIG_RADIO_FREQ
#define CONFIG_RADIO_FREQ           915.0
#endif
#ifndef CONFIG_RADIO_OUTPUT_POWER
#define CONFIG_RADIO_OUTPUT_POWER   17
#endif
#ifndef CONFIG_RADIO_BW
#define CONFIG_RADIO_BW             125.0
#endif

#define NODE_ID                     "186841"

#define CICLO_MS                    10000

// --- CHIRP ---
// prunning de 10 muestras, 300 ms de espera entre c/u
// costo total: 10 * 300 ms = 3 s de los 10 del ciclo
#define CHIRP_MUESTRAS              10
#define CHIRP_RECUPERACION_MS       300

// --- PRUNNING ---
// se descartan las muestras que se alejen más de esto (en desviaciones
// estándar) del promedio del grupo
#define PRUNE_SIGMA                 2.0f

#define PRUNNING_MIN_MUESTRAS       3

#define GPS_FIX_MAX_EDAD_MS         10000

// HDC1080: direccion I2C fija de fábrica
#define HDC1080_ADDR                0x40

// dejar en 1 si se prueba contra un receptor propio que
// exija CRC de payload
#define USAR_CRC                    1

// dejar en 1 para ver la trama NMEA cruda
#define DEBUG_NMEA                  0


#if !defined(USING_SX1276) && !defined(USING_SX1278)
#error "LoRa example is only allowed to run SX1276/78. For other RF models, please run examples/RadioLibExamples"
#endif

#ifndef HAS_GPS
#error "Esta placa no tiene GPS habilitado en utilities.h (falta HAS_GPS)"
#endif


enum Estado {
    ESTADO_CHIRP,
    ESTADO_PRUNNING,
    ESTADO_BUNDLING,
    ESTADO_TX,
};

static Estado   estado = ESTADO_CHIRP;
static uint32_t cicloInicio = 0;    // marca de tiempo del arranque del ciclo

TinyGPSPlus         gps;
ClosedCube_HDC1080  hdc1080;

// --- CHIRP. muestras crudas del sensor ---
static bool     sensorOk = false;
static float    muestras[CHIRP_MUESTRAS];
static uint8_t  nMuestras = 0;      // número de muestras que se lograron leer sin error

// --- PRUNNING. resultado consolidado del ciclo ---
static float    temperatura = 0.0f; // último valor consolidado válido
static bool     hayTemp = false;    // ya hubo al menos una consolidación
static uint8_t  sobrevivieron = 0;  // número de muestras que pasaron el prunning

static double   latitud = 0.0;      // último fix válido
static double   longitud = 0.0;
static bool     hayFix = false;

// --- BUNDLING. trama lista para TX ---
// 400 bytes: la trama sin metadata ronda los 122. Se deja margen para cuando
// se agreguen los campos de cifrado (~120) e integridad (32) de la bonificación
static char     payload[400];
static int      contador = 0;       // solo para la traza serie


// smartDelay
static void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do {
        while (SerialGPS.available()) {
            char c = SerialGPS.read();
            gps.encode(c);
#if DEBUG_NMEA
            Serial.write(c);
#endif
        }
    } while (millis() - start < ms);
}


// --------------------------------------------
// Estado 1: CHIRP. proceso de toma de medidas
// --------------------------------------------
// toma CHIRP_MUESTRAS lecturas de temperatura, separadas por el tiempo de
// recuperacion del sensor
static void doChirp()
{
    nMuestras = 0;

    for (uint8_t i = 0; i < CHIRP_MUESTRAS; i++) {

        // tiempo de recuperacion del sensor entre medida y medida
        smartDelay(CHIRP_RECUPERACION_MS);

        if (!sensorOk) {
            continue;
        }

        float t = hdc1080.readTemperature();

        if (isnan(t) || t < -40.0f || t > 124.0f) {
            Serial.printf("CHIRP........ muestra %u descartada (%.1f C fuera de rango)\n",
                          (unsigned)(i + 1), t);
            continue;
        }

        muestras[nMuestras++] = t;
    }

    Serial.printf("CHIRP........ %u/%u muestras validas\n",
                  (unsigned)nMuestras, (unsigned)CHIRP_MUESTRAS);
}


// -----------------------------------------------------------
// Estado 2: PRUNNING. proceso de consolidación de una medida
// -----------------------------------------------------------
// promedia el arreglo descartando los valores que se alejan más de
// PRUNE_SIGMA desviaciones estándar
static float promedioPodado(const float *buf, uint8_t n, uint8_t *kept)
{
    float sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += buf[i];
    }
    float media = sum / n;

    float var = 0;
    for (uint8_t i = 0; i < n; i++) {
        var += (buf[i] - media) * (buf[i] - media);
    }
    float sd = sqrtf(var / n);

    // segunda pasada: promedia solo las muestras dentro del margen
    float sumOk = 0;
    uint8_t nOk = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (fabsf(buf[i] - media) <= PRUNE_SIGMA * sd) {
            sumOk += buf[i];
            nOk++;
        }
    }
    *kept = nOk;
    return nOk ? (sumOk / nOk) : media;
}

static void doPrunning()
{
    // --- temperatura ---
    if (nMuestras >= PRUNNING_MIN_MUESTRAS) {
        temperatura = promedioPodado(muestras, nMuestras, &sobrevivieron);
        hayTemp = true;
        Serial.printf("PRUNNING..... temp %.2f C (%u/%u muestras conservadas)\n",
                      temperatura, (unsigned)sobrevivieron, (unsigned)nMuestras);
    } else {
        sobrevivieron = 0;
        // menos de PRUNNING_MIN_MUESTRAS no da para consolidar. Como el campo de
        // temperatura es obligatorio, se retransmite el último valor bueno; si
        // nunca hubo uno muestra 0.0 y queda el aviso en la traza serie
        Serial.printf("PRUNNING..... solo %u muestras validas, minimo %u (%s). Se reenvia %.2f C\n",
                      (unsigned)nMuestras, (unsigned)PRUNNING_MIN_MUESTRAS,
                      sensorOk ? "sensor mudo" : "HDC1080 ausente", temperatura);
    }

    // --- posición ---
    // el fix se acumula durante todo el ciclo gracias a smartDelay(). Solo se
    // toma si es válido y reciente; si no, se conserva el último
    if (gps.location.isValid() && gps.location.age() < GPS_FIX_MAX_EDAD_MS) {
        latitud  = gps.location.lat();
        longitud = gps.location.lng();
        hayFix = true;
        Serial.printf("PRUNNING..... fix %.6f, %.6f (%u sats)\n",
                      latitud, longitud,
                      (unsigned)(gps.satellites.isValid() ? gps.satellites.value() : 0));
    } else {
        Serial.printf("PRUNNING..... sin fix reciente (%u sats a la vista), se reenvia %.6f, %.6f\n",
                      (unsigned)(gps.satellites.isValid() ? gps.satellites.value() : 0),
                      latitud, longitud);
    }
}


// ---------------------------------------------------------
// Estado 3: BUNDLING. proceso de conformacion de los datos
// ---------------------------------------------------------
// Arma la trama JSON de la forma:
//
//   {"id": "186841","lat": 6.245259, "lon": -75.596510, "temperatura" : 22.4,
//    "metadata":{"campoCifrado":"","campoIntegridad":""}}
//
static void doBundling()
{
    snprintf(payload, sizeof(payload),
             "{\"id\": \"" NODE_ID "\",\"lat\": %.6f, \"lon\": %.6f, "
             "\"temperatura\" : %.1f, "
             "\"metadata\":{\"campoCifrado\":\"\",\"campoIntegridad\":\"\"}}",
             latitud,
             longitud,
             temperatura);
}


// --------------------------------------
// Estado 4: TX. transmición del paquete
// --------------------------------------
static void doTx()
{
    LoRa.beginPacket();
    LoRa.print(payload);
    int ok = LoRa.endPacket();

    Serial.println("---------- TX ----------");
    Serial.println(payload);
    Serial.printf("Payload...... %u bytes | TX %s\n",
                  (unsigned)strlen(payload), ok ? "OK" : "ERROR");
    Serial.printf("Salud........ fix: %s | sensor: %s | prunning: %u/%u muestras\n",
                  hayFix ? "valido"
                         : (latitud == 0.0 && longitud == 0.0 ? "AUSENTE (mandando 0,0)"
                                                              : "vencido (posicion reenviada)"),
                  !sensorOk ? "AUSENTE" : (hayTemp ? "ok" : "sin consolidar"),
                  (unsigned)sobrevivieron, (unsigned)CHIRP_MUESTRAS);
    Serial.println("------------------------");

    contador++;
}


void setup()
{
    // arranca SerialGPS (Serial1) en GPS_RX_PIN/GPS_TX_PIN a GPS_BAUD_RATE
    // y hace Wire.begin(I2C_SDA=21, I2C_SCL=22) mas un scan del bus
    setupBoards(true);

    smartDelay(1500);

    // ping al bus antes de usar el sensor: si no esta conectado, el bus
    // devuelve 0xFFFF y la libreria lo traduce a 125 C, que parece válido
    Wire.beginTransmission(HDC1080_ADDR);
    sensorOk = (Wire.endTransmission() == 0);
    if (sensorOk) {
        hdc1080.begin(HDC1080_ADDR);
        Serial.println("HDC1080 detectado en 0x40");
    } else {
        Serial.println("WARN: HDC1080 no responde en 0x40");
    }
    smartDelay(1500);

#ifdef  RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif

    Serial.println("LoRa Sender - Examen 2 IoT - ID " NODE_ID);
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(CONFIG_RADIO_FREQ * 1000000)) {
        Serial.println("Starting LoRa failed!!");
        while (1);
    }

    LoRa.setTxPower(CONFIG_RADIO_OUTPUT_POWER);
    LoRa.setSignalBandwidth(CONFIG_RADIO_BW * 1000);
    LoRa.setSpreadingFactor(10);
    LoRa.setPreambleLength(16);
    LoRa.setSyncWord(0xAB);
#if USAR_CRC
    LoRa.enableCrc();
#else
    LoRa.disableCrc();
#endif
    LoRa.disableInvertIQ();
    LoRa.setCodingRate4(7);

    cicloInicio = millis();
}

void loop()
{
    switch (estado) {

    case ESTADO_CHIRP:
        cicloInicio = millis();
        Serial.printf("\n===== ciclo %d =====\n", contador);
        doChirp();
        estado = ESTADO_PRUNNING;
        break;

    case ESTADO_PRUNNING:
        doPrunning();
        estado = ESTADO_BUNDLING;
        break;

    case ESTADO_BUNDLING:
        doBundling();
        estado = ESTADO_TX;
        break;

    case ESTADO_TX:
        doTx();

        {
            uint32_t transcurrido = millis() - cicloInicio;
            if (transcurrido < CICLO_MS) {
                smartDelay(CICLO_MS - transcurrido);
            }
        }

        estado = ESTADO_CHIRP;
        break;
    }
}
