/*******************************************************
        #### RuralWaterTankMonitor-ESP32 WIFI/SINRICPRO ####

  Regras solicitadas (LED + Deep Sleep):
  - DIA (07:00:00 até 19:00:59): SEM deep sleep. LED de nível sempre ACESO.
  - NOITE (19:01:00 até 06:59:59): COM deep sleep.
      * LED fica APAGADO e acende por 3 minutos a cada hora cheia (HH:00:00 até HH:02:59).
      * Acorda exatamente no próximo marco necessário (fim da janela ON ou próxima hora cheia).

  Capacidades NO SINRIC (conforme imagem):
  - Range (Nível da Água)         -> instance: "nivel_agua"
  - Range (Quantidade em Litros)  -> instance: "litros_uteis"
  - Alcance (Bateria)             -> instance: "bateria_pct"
  - Power                         -> liga/desliga (pausa medições/envio e apaga LEDs)

  Conversão NOVA: HC-SR04 (cm) -> LITROS ÚTEIS (Fortlev 1000 L)
  - Boia a 8 cm abaixo da borda (máximo útil)
  - Saída a 12 cm da base (mínimo útil = 0 L úteis)
  - Modelo linear (volume proporcional à altura). Aproximação.

  Autor: Lilian de Ávila Costa
  Versão: 1.4.1
  Revisão: 27/12/2025
 ******************************************************/

#include <Arduino.h>
#include <WiFi.h>

#include <SinricPro.h>
#include <SinricProDevice.h>
#include <Capabilities/RangeController.h>
#include <Capabilities/PushNotification.h>
#include <Capabilities/PowerStateController.h>

#include <time.h>

#include "secrets_nivel.h"   // APP_KEY, APP_SECRET, DEVICE_ID, WIFI_SSID_1..4, WIFI_PASS_1..4

#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#ifdef ESP32
  #include "esp_bt.h"
#endif

// ====================================================
// TIPOS + PROTÓTIPOS
// ====================================================
struct Medicao {
  float distCm;
  int   nivelPct;     // % baseado em LITROS ÚTEIS (0..100)
  int   litrosUteis;  // litros úteis (0..max útil)
  float vbat;
  int   batPct;       // -1 se indefinido
};

Medicao realizarMedicao();
bool enviarSinric(const Medicao &m);
bool garantirTempo(bool wifiOK, bool &ntpOK);

// ====================================================
// CONFIG PRODUÇÃO
// ====================================================
static const uint32_t WIFI_TIMEOUT_MS    = 10000;
static const uint32_t SINRIC_TIMEOUT_MS  = 15000;
static const uint32_t DAY_CYCLE_MS       = 10UL * 60UL * 1000UL; // 10 min

// NTP / Horário (UTC-3)
static const char* NTP_SERVER          = "pool.ntp.org";
static const long  GMT_OFFSET_SEC      = -3 * 3600;
static const int   DAYLIGHT_OFFSET_SEC = 0;

// Janela ON do LED à noite: 3 min após hora cheia
static const int NOITE_ON_MIN = 3; // (HH:00:00 até HH:02:59)

// Regra de janela DIA (sem deep sleep)
static const int DIA_INICIO_H = 7;
static const int DIA_INICIO_M = 0;
static const int DIA_FIM_H    = 19;
static const int DIA_FIM_M    = 0; // 19:00 ainda é dia; 19:01+ é noite

// Persistência em deep sleep
RTC_DATA_ATTR time_t ultimoNTP = 0;
RTC_DATA_ATTR int bootCount = 0;

// POWER (Sinric) - persistente
RTC_DATA_ATTR bool g_powerOn = true;

// ====================================================
// TANQUE (Fortlev 1000 L) - parâmetros do seu cenário
// ====================================================
static const int TANK_CAP_L              = 1000; // capacidade nominal
static const int BOIA_ABAIXO_BORDA_CM    = 8;    // máximo útil (do topo para baixo)
static const int SAIDA_ACIMA_BASE_CM     = 12;   // mínimo útil (altura da saída)

// ====================================================
// WIFI (até 4 redes)
// ====================================================
struct WiFiConfig { const char* ssid; const char* pass; };
WiFiConfig wifiList[] = {
  {WIFI_SSID_1, WIFI_PASS_1},
  {WIFI_SSID_2, WIFI_PASS_2},
  {WIFI_SSID_3, WIFI_PASS_3},
  {WIFI_SSID_4, WIFI_PASS_4}
};
const int WIFI_COUNT = sizeof(wifiList) / sizeof(wifiList[0]);

// ====================================================
// HARDWARE
// ====================================================
const int PIN_TRIG = 5;
const int PIN_ECHO = 18; // HC-SR04 ECHO -> divisor 5V->3.3V no ESP32

// LEDs (RTC GPIOs)
const int LED_AZUL    = 32;
const int LED_AMARELO = 26;
const int LED_VERDE   = 27;
const int LED_VERM    = 33;

const int PIN_BAT_ADC = 34; // ADC1

// GPIO -> resistor -> LED -> GND (ativo em HIGH)
#define LED_ACTIVE_LOW 0
inline void ledWrite(int pin, bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

// ====================================================
// CALIBRAÇÃO (persistente em deep sleep)
// ====================================================
// g_alturaCheioCm: distância sensor->água no MÁXIMO ÚTIL (boia) ~ 8 cm
// g_alturaVazioCm: distância sensor->água quando água está na ALTURA DA SAÍDA (0 L úteis)
// Obs: Como o Portal (imagem) não tem sliders de calibração, mantemos esses valores locais.
RTC_DATA_ATTR int g_alturaCheioCm = 8;
RTC_DATA_ATTR int g_alturaVazioCm = 74;

const int DIST_MIN_CM = 3;
const int DIST_MAX_CM = 200;

// ====================================================
// BATERIA 1S (divisor 220k / 100k)
// ====================================================
const float R1 = 220000.0f;
const float R2 = 100000.0f;
const float FATOR_DIV = R2 / (R1 + R2);
const float VREF      = 3.30f;
const int   ADC_MAX   = 4095;

const float VBAT_MIN  = 3.20f;
const float VBAT_MAX  = 4.20f;

bool bateria_conectada = false;

// Alertas
const int LIM_CRITICO_NIVEL = 5;   // % útil
const int LIM_CRITICO_BAT   = 15;  // %

RTC_DATA_ATTR bool notificadoNivelCritico = false;
RTC_DATA_ATTR bool notificadoBatCritica   = false;

// ====================================================
// SINRIC PRO (conforme imagem: 3 Ranges + Power)
// ====================================================
class WaterLevelIndicator
: public SinricProDevice
, public RangeController<WaterLevelIndicator>
, public PushNotification<WaterLevelIndicator>
, public PowerStateController<WaterLevelIndicator> {

  friend class RangeController<WaterLevelIndicator>;
  friend class PushNotification<WaterLevelIndicator>;
  friend class PowerStateController<WaterLevelIndicator>;

public:
  WaterLevelIndicator(const String &deviceId) : SinricProDevice(deviceId, "WaterLevelIndicator") {}
};

WaterLevelIndicator &tank = SinricPro[DEVICE_ID];

// Instances DEVEM bater com o Portal (Configurar -> Instance)
const char* DP_NIVEL   = "nivel_agua";    // Range (Nível da Água)
const char* DP_LITROS  = "litros_uteis";  // Range (Quantidade em Litros)
const char* DP_BATERIA = "bateria_pct";   // Alcance (Bateria)

// ====================================================
// UTIL: economia
// ====================================================
void desligarWiFiBtLogoNoBoot() {
  WiFi.mode(WIFI_OFF);
  WiFi.persistent(false);
#ifdef ESP32
  btStop();
#endif
}

void desligarRadios() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
#ifdef ESP32
  btStop();
#endif
}

// ====================================================
// LED helpers
// ====================================================
void apagarTodosLEDs() {
  ledWrite(LED_AZUL,    false);
  ledWrite(LED_VERDE,   false);
  ledWrite(LED_AMARELO, false);
  ledWrite(LED_VERM,    false);
}

const char* aplicarLEDNivel(int nivel) {
  apagarTodosLEDs();
  if      (nivel >= 75) { ledWrite(LED_AZUL,    true); return "    🔵 AZUL (>=75%)"; }
  else if (nivel >= 50) { ledWrite(LED_VERDE,   true); return "  🟢 VERDE (50-74%)"; }
  else if (nivel >= 25) { ledWrite(LED_AMARELO, true); return "🟡 AMARELO (25-49%)"; }
  else                  { ledWrite(LED_VERM,    true); return " 🔴 VERMELHO (<25%)"; }
}

// ====================================================
// RTC HOLD LEDs
// ====================================================
void liberarHoldLEDsNoBoot() {
  gpio_deep_sleep_hold_dis();

  rtc_gpio_hold_dis((gpio_num_t)LED_AZUL);
  rtc_gpio_hold_dis((gpio_num_t)LED_VERDE);
  rtc_gpio_hold_dis((gpio_num_t)LED_AMARELO);
  rtc_gpio_hold_dis((gpio_num_t)LED_VERM);

  rtc_gpio_deinit((gpio_num_t)LED_AZUL);
  rtc_gpio_deinit((gpio_num_t)LED_VERDE);
  rtc_gpio_deinit((gpio_num_t)LED_AMARELO);
  rtc_gpio_deinit((gpio_num_t)LED_VERM);
}

void aplicarHoldLEDsOffAntesDeDormir() {
  rtc_gpio_init((gpio_num_t)LED_AZUL);
  rtc_gpio_init((gpio_num_t)LED_VERDE);
  rtc_gpio_init((gpio_num_t)LED_AMARELO);
  rtc_gpio_init((gpio_num_t)LED_VERM);

  rtc_gpio_set_direction((gpio_num_t)LED_AZUL,    RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction((gpio_num_t)LED_VERDE,   RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction((gpio_num_t)LED_AMARELO, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction((gpio_num_t)LED_VERM,    RTC_GPIO_MODE_OUTPUT_ONLY);

#if LED_ACTIVE_LOW
  rtc_gpio_set_level((gpio_num_t)LED_AZUL,    1);
  rtc_gpio_set_level((gpio_num_t)LED_VERDE,   1);
  rtc_gpio_set_level((gpio_num_t)LED_AMARELO, 1);
  rtc_gpio_set_level((gpio_num_t)LED_VERM,    1);
#else
  rtc_gpio_set_level((gpio_num_t)LED_AZUL,    0);
  rtc_gpio_set_level((gpio_num_t)LED_VERDE,   0);
  rtc_gpio_set_level((gpio_num_t)LED_AMARELO, 0);
  rtc_gpio_set_level((gpio_num_t)LED_VERM,    0);
#endif

  rtc_gpio_hold_en((gpio_num_t)LED_AZUL);
  rtc_gpio_hold_en((gpio_num_t)LED_VERDE);
  rtc_gpio_hold_en((gpio_num_t)LED_AMARELO);
  rtc_gpio_hold_en((gpio_num_t)LED_VERM);

  gpio_deep_sleep_hold_en();
}

void aplicarHoldLEDsEspelhadoAntesDeDormir() {
  rtc_gpio_init((gpio_num_t)LED_AZUL);
  rtc_gpio_init((gpio_num_t)LED_VERDE);
  rtc_gpio_init((gpio_num_t)LED_AMARELO);
  rtc_gpio_init((gpio_num_t)LED_VERM);

  rtc_gpio_set_direction((gpio_num_t)LED_AZUL,    RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction((gpio_num_t)LED_VERDE,   RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction((gpio_num_t)LED_AMARELO, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_direction((gpio_num_t)LED_VERM,    RTC_GPIO_MODE_OUTPUT_ONLY);

  rtc_gpio_set_level((gpio_num_t)LED_AZUL,    gpio_get_level((gpio_num_t)LED_AZUL));
  rtc_gpio_set_level((gpio_num_t)LED_VERDE,   gpio_get_level((gpio_num_t)LED_VERDE));
  rtc_gpio_set_level((gpio_num_t)LED_AMARELO, gpio_get_level((gpio_num_t)LED_AMARELO));
  rtc_gpio_set_level((gpio_num_t)LED_VERM,    gpio_get_level((gpio_num_t)LED_VERM));

  rtc_gpio_hold_en((gpio_num_t)LED_AZUL);
  rtc_gpio_hold_en((gpio_num_t)LED_VERDE);
  rtc_gpio_hold_en((gpio_num_t)LED_AMARELO);
  rtc_gpio_hold_en((gpio_num_t)LED_VERM);

  gpio_deep_sleep_hold_en();
}

// ====================================================
// SENSOR HC-SR04 (mediana 7)
// ====================================================
float medirDistanciaMedianaCm() {
  const int N = 7;
  float v[N];

  for (int i = 0; i < N; i++) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);
    float d  = (dur > 0) ? (dur / 29.0f) / 2.0f : NAN;
    v[i] = d;
    delay(8);
  }

  int m = 0;
  for (int i = 0; i < N; i++) {
    if (isnan(v[i])) continue;
    if (v[i] < DIST_MIN_CM || v[i] > DIST_MAX_CM) continue;
    v[m++] = v[i];
  }
  if (m == 0) return NAN;

  for (int i = 0; i < m - 1; i++) {
    for (int j = i + 1; j < m; j++) {
      if (v[j] < v[i]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
    }
  }
  return v[m / 2];
}

// ====================================================
// CONVERSÃO CM -> LITROS ÚTEIS (Fortlev 1000 L) - linear
// ====================================================
static inline float alturaInternaEstimCm() {
  // Interpretação: quando água está na altura da saída (12cm da base) o sensor lê g_alturaVazioCm
  // Logo: altura interna aproximada H = g_alturaVazioCm + 12
  return (float)g_alturaVazioCm + (float)SAIDA_ACIMA_BASE_CM;
}

static inline float distanciaToAlturaAguaCm(float distCm) {
  float H = alturaInternaEstimCm();
  float hAgua = H - distCm; // base->água
  if (hAgua < 0) hAgua = 0;
  if (hAgua > H) hAgua = H;
  return hAgua;
}

static inline float alturaAguaToLitrosTotais(float hAguaCm) {
  float H = alturaInternaEstimCm();
  if (H <= 1.0f) return 0.0f;
  return (hAguaCm / H) * (float)TANK_CAP_L;
}

static inline int distanciaToLitrosUteis(float distCm) {
  if (isnan(distCm)) return 0;

  float H = alturaInternaEstimCm();
  if (H <= 1.0f) return 0;

  float hAgua = distanciaToAlturaAguaCm(distCm);

  // Limite superior útil (boia 8cm abaixo do topo)
  float hMaxUtil = H - (float)BOIA_ABAIXO_BORDA_CM;
  if (hAgua > hMaxUtil) hAgua = hMaxUtil;

  // Volume morto abaixo da saída (12cm da base)
  float hMinUtil = (float)SAIDA_ACIMA_BASE_CM;
  float litrosMinUtil = (hMinUtil / H) * (float)TANK_CAP_L;

  float litros = alturaAguaToLitrosTotais(hAgua) - litrosMinUtil;
  if (litros < 0) litros = 0;

  float litrosMaxUtil = ((hMaxUtil / H) * (float)TANK_CAP_L) - litrosMinUtil;
  if (litros > litrosMaxUtil) litros = litrosMaxUtil;

  return (int)lroundf(litros);
}

static inline int distanciaToNivelPctUteis(float distCm) {
  float H = alturaInternaEstimCm();
  if (H <= 1.0f) return 0;

  float hMaxUtil = H - (float)BOIA_ABAIXO_BORDA_CM;
  float hMinUtil = (float)SAIDA_ACIMA_BASE_CM;

  float litrosMaxUtil = ((hMaxUtil - hMinUtil) / H) * (float)TANK_CAP_L;
  if (litrosMaxUtil <= 1.0f) return 0;

  int litrosUteis = distanciaToLitrosUteis(distCm);
  float pct = ((float)litrosUteis / litrosMaxUtil) * 100.0f;
  return constrain((int)lroundf(pct), 0, 100);
}

// ====================================================
// Bateria
// ====================================================
float lerBateriaV() {
  const int N = 10;
  long soma = 0;
  for (int i = 0; i < N; i++) { soma += analogRead(PIN_BAT_ADC); delay(2); }

  float adc  = (float)soma / N;
  float vadc = (adc / ADC_MAX) * VREF;
  float vbat = vadc / FATOR_DIV;

  if (adc < 100) {
    bateria_conectada = false;
    return 0.0f;
  }
  bateria_conectada = true;
  return vbat;
}

int bateriaPct(float vbat) {
  if (!bateria_conectada) return -1;
  float pct = (vbat - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100.0f;
  return constrain((int)lroundf(pct), 0, 100);
}

// ====================================================
// WiFi / NTP / Relógio / Janela DIA-NOITE
// ====================================================
bool conectarWiFiMulti(uint32_t timeoutMs) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  // Estabilidade
  WiFi.setSleep(false);

  // Zera estado anterior
  WiFi.disconnect(true, true);
  delay(250);

  // Scan para priorizar SSID conhecido com melhor sinal
  Serial.println("🔎 [WiFi] Scan redes...");
  int n = WiFi.scanNetworks(false, true);
  int bestKnown = -1;
  int bestRssi  = -10000;

  if (n > 0) {
    for (int i = 0; i < n; i++) {
      String ssidFound = WiFi.SSID(i);
      int rssiFound = WiFi.RSSI(i);

      for (int k = 0; k < WIFI_COUNT; k++) {
        if (ssidFound == wifiList[k].ssid) {
          if (rssiFound > bestRssi) { bestRssi = rssiFound; bestKnown = k; }
        }
      }
    }
  }
  WiFi.scanDelete();

  int order[WIFI_COUNT];
  int p = 0;

  if (bestKnown >= 0) {
    order[p++] = bestKnown;
    for (int i = 0; i < WIFI_COUNT; i++) if (i != bestKnown) order[p++] = i;
    Serial.printf("📶 [WiFi] Melhor rede conhecida: '%s' (RSSI %d)\n", wifiList[bestKnown].ssid, bestRssi);
  } else {
    for (int i = 0; i < WIFI_COUNT; i++) order[i] = i;
    Serial.println("ℹ️ [WiFi] Nenhuma rede conhecida apareceu no scan. Tentando lista na ordem...");
  }

  for (int oi = 0; oi < WIFI_COUNT; oi++) {
    int idx = order[oi];
    const char* ssid = wifiList[idx].ssid;
    const char* pass = wifiList[idx].pass;

    Serial.printf("⏳ [WiFi] Tentando '%s'\n", ssid);

    WiFi.disconnect(true, true);
    delay(150);

    WiFi.begin(ssid, pass);

    uint32_t ts = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - ts) < timeoutMs) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("✓ [WiFi] Conectado em '%s' | IP: %s | RSSI: %d\n",
                    ssid, WiFi.localIP().toString().c_str(), WiFi.RSSI());
      return true;
    }

    Serial.printf("✗ [WiFi] Falhou em '%s'\n", ssid);
  }

  Serial.println("✗ [WiFi] Falha (offline).");
  return false;
}

bool sincronizarNTP() {
  Serial.println("⏳ [NTP] Sincronizando relógio...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  struct tm timeinfo;
  int tentativas = 0;
  while (!getLocalTime(&timeinfo) && tentativas < 12) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }
  Serial.println();

  if (tentativas >= 12) {
    Serial.println("✗ [NTP] Falha na sincronização");
    return false;
  }

  time(&ultimoNTP);
  Serial.printf("✓ [NTP] OK: %02d/%02d/%04d %02d:%02d:%02d\n",
                timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return true;
}

bool obterHoraLocal(struct tm &t) {
  if (!getLocalTime(&t)) return false;
  if (t.tm_year < (2020 - 1900)) return false;
  return true;
}

bool isJanelaDia(const struct tm &t) {
  const int h = t.tm_hour;
  const int m = t.tm_min;

  if (h < DIA_INICIO_H) return false;
  if (h > DIA_FIM_H)    return false;

  if (h == DIA_FIM_H) return (m <= DIA_FIM_M); // 19:00 ainda é dia; 19:01+ é noite
  return true;                                  // 07:00 até 18:59
}

bool ledDeveFicarOnNaNoite(const struct tm &t) {
  int secNaHora = (t.tm_min * 60) + t.tm_sec;
  return (secNaHora >= 0 && secNaHora < (NOITE_ON_MIN * 60));
}

uint64_t calcularSonoNoiteUs(const struct tm &t) {
  int secNaHora = (t.tm_min * 60) + t.tm_sec;
  int janelaOnSec = NOITE_ON_MIN * 60;

  int sleepSec = 0;
  if (secNaHora < janelaOnSec) {
    sleepSec = janelaOnSec - secNaHora;
    if (sleepSec < 5) sleepSec = 5;
  } else {
    sleepSec = 3600 - secNaHora;
    if (sleepSec < 10) sleepSec = 10;
  }
  return (uint64_t)sleepSec * 1000000ULL;
}

// ====================================================
// Sinric (Power + envio dos 3 Ranges conforme imagem)
// ====================================================
static inline void enviarRange(const char* instance, int value) {
  tank.sendRangeValueEvent(instance, value);
}

bool onPowerStateCb(const String& /*deviceId*/, bool &state) {
  g_powerOn = state;
  Serial.printf("⚡ [Power] %s\n", g_powerOn ? "ON (monitoramento ativo)" : "OFF (pausado)");
  if (!g_powerOn) apagarTodosLEDs();
  return true;
}

// ====================================================
// Medição / Tempo / Sinric
// ====================================================
Medicao realizarMedicao() {
  Medicao r{};
  r.distCm = medirDistanciaMedianaCm();

  if (isnan(r.distCm)) {
    Serial.println("✗ [Sensor] Leitura inválida. Fallback: 0% e 0 L úteis.");
    r.litrosUteis = 0;
    r.nivelPct    = 0;
  } else {
    r.litrosUteis = distanciaToLitrosUteis(r.distCm);
    r.nivelPct    = distanciaToNivelPctUteis(r.distCm);
  }

  r.vbat   = lerBateriaV();
  r.batPct = bateriaPct(r.vbat);
  return r;
}

bool garantirTempo(bool wifiOK, bool &ntpOK) {
  ntpOK = false;
  if (!wifiOK) return false;

  time_t agora;
  time(&agora);

  if (ultimoNTP == 0 || (agora - ultimoNTP) > 6 * 3600) {
    ntpOK = sincronizarNTP();
  } else {
    struct tm t;
    ntpOK = obterHoraLocal(t);
  }
  return ntpOK;
}

bool enviarSinric(const Medicao &m) {
  tank.onPowerState(onPowerStateCb);

  SinricPro.onConnected([] { Serial.println("✓ [SinricPro] Conectado!"); });
  SinricPro.onDisconnected([] { Serial.println("✗ [SinricPro] Desconectado"); });

  Serial.println("⏳ [SinricPro] Iniciando...");
  SinricPro.begin(APP_KEY, APP_SECRET);

  uint32_t ts = millis();
  while (!SinricPro.isConnected() && millis() - ts < SINRIC_TIMEOUT_MS) {
    SinricPro.handle();
    delay(50);
  }

  if (!SinricPro.isConnected()) {
    Serial.println("✗ [SinricPro] Offline (sem envio).");
    return false;
  }

  // Sincroniza estado do Power no app
  tank.sendPowerStateEvent(g_powerOn);

  // Se Power OFF: não envia ranges (evita “atualizar” enquanto pausado)
  if (!g_powerOn) {
    for (int i = 0; i < 10; i++) { SinricPro.handle(); delay(50); }
    Serial.println("⏸️ [Sinric] Power OFF: envio pausado.");
    return true;
  }

  // Envia somente os 3 Ranges conforme imagem
  enviarRange(DP_NIVEL,   m.nivelPct);
  enviarRange(DP_LITROS,  m.litrosUteis);
  if (m.batPct >= 0) enviarRange(DP_BATERIA, m.batPct);

  // Alertas (baseados em % útil)
  if (!notificadoNivelCritico && m.nivelPct <= LIM_CRITICO_NIVEL) {
    tank.sendPushNotification("⚠️ Nível de água MUITO baixo!");
    notificadoNivelCritico = true;
  } else if (notificadoNivelCritico && m.nivelPct > (LIM_CRITICO_NIVEL + 3)) {
    notificadoNivelCritico = false;
  }

  if (m.batPct >= 0 && !notificadoBatCritica && m.batPct <= LIM_CRITICO_BAT) {
    tank.sendPushNotification("🔋 Bateria fraca — recarregue o sistema.");
    notificadoBatCritica = true;
  } else if (notificadoBatCritica && m.batPct > (LIM_CRITICO_BAT + 5)) {
    notificadoBatCritica = false;
  }

  for (int i = 0; i < 30; i++) { SinricPro.handle(); delay(50); }
  Serial.println("📤 [Sinric] Dados enviados.");
  return true;
}

// ====================================================
// Fluxo NOITE
// ====================================================
void executarModoNoite() {
  Serial.println("\n================= MODO NOITE =================");
  Serial.println("Regra: Deep sleep ON | LED 3min na hora cheia");
  Serial.println("================================================\n");

  // Mesmo com Power OFF, mantém regra de deep sleep e LEDs apagados
  struct tm t = {};
  bool horaOK = obterHoraLocal(t);

  bool acenderLED = false;
  if (horaOK && g_powerOn) acenderLED = ledDeveFicarOnNaNoite(t);

  Medicao m{};
  const char* corNivel = "LED OFF (noite)";

  if (g_powerOn) {
    Serial.println("📏 [Medição] Lendo Sensor...");
    m = realizarMedicao();
    if (acenderLED) corNivel = aplicarLEDNivel(m.nivelPct);
    else apagarTodosLEDs();
  } else {
    apagarTodosLEDs();
  }

  Serial.println("┌──────────────────────────────────────────────┐");
  Serial.printf ("│ ⚡ Power:         %-26s│\n", g_powerOn ? "ON" : "OFF (pausado)");
  if (g_powerOn) {
    Serial.printf ("│ 💧 Nível útil:     %6d %%                  │\n", m.nivelPct);
    Serial.printf ("│ 🚰 Litros úteis:   %6d L                   │\n", m.litrosUteis);
    if (!isnan(m.distCm)) Serial.printf("│ 📏 Distância:      %6.1f cm                │\n", m.distCm);
    else                  Serial.println("│ 📏 Distância:        --.- cm                │");
    if (m.batPct >= 0)    Serial.printf("│ 🔋 Bateria:        %6.2fV (%3d%%)           │\n", m.vbat, m.batPct);
    else                  Serial.println("│ 🔌 Alimentação:    USB/indefinido           │");
    Serial.printf ("│ 💡 LED:            %-23s │\n", corNivel);
  } else {
    Serial.println("│ ⏸️ Monitoramento:  pausado (sem medição)     │");
    Serial.println("│ 💡 LED:            OFF                       │");
  }
  Serial.println("└──────────────────────────────────────────────┘\n");

  bool wifiOK = false;
  bool ntpOK  = false;

  if (g_powerOn) {
    wifiOK = conectarWiFiMulti(WIFI_TIMEOUT_MS);
    if (wifiOK) (void)garantirTempo(wifiOK, ntpOK);
    if (wifiOK) (void)enviarSinric(m);
  } else {
    // Power OFF: ainda pode sincronizar power no app se você quiser;
    // aqui mantemos economia máxima e NÃO liga WiFi.
  }

  desligarRadios();

  uint64_t sleepUs = 10ULL * 60ULL * 1000000ULL; // fallback 10 min
  if (horaOK) sleepUs = calcularSonoNoiteUs(t);

  Serial.printf("\n💤 [Deep Sleep] Dormindo %.1f min...\n", (double)sleepUs / 60000000.0);
  Serial.println("═══════════════════════════════════════════════\n");

  if (acenderLED && g_powerOn) {
    aplicarHoldLEDsEspelhadoAntesDeDormir();
  } else {
    apagarTodosLEDs();
    aplicarHoldLEDsOffAntesDeDormir();
  }

  esp_sleep_enable_timer_wakeup(sleepUs);
  esp_deep_sleep_start();
}

// ====================================================
// Fluxo DIA
// ====================================================
void executarCicloDia() {
  if (!g_powerOn) {
    apagarTodosLEDs();
    Serial.println("⏸️ [DIA] Power OFF: monitoramento pausado (sem medição / sem envio).");
    return;
  }

  Serial.println("📏 [Medição] Lendo Sensor...");
  Medicao m = realizarMedicao();

  const char* corNivel = aplicarLEDNivel(m.nivelPct);

  bool wifiOK = conectarWiFiMulti(WIFI_TIMEOUT_MS);

  bool ntpOK = false;
  if (wifiOK) (void)garantirTempo(wifiOK, ntpOK);
  else Serial.println("⚠️ [WiFi] Offline. Sem NTP/Sinric neste ciclo.");

  struct tm t = {};
  bool horaOK = obterHoraLocal(t);

  Serial.println("┌──────────────────────────────────────────────┐");
  Serial.printf ("│ ⚡ Power:         %-26s│\n", "ON");
  Serial.printf ("│ 💧 Nível útil:     %6d %%                  │\n", m.nivelPct);
  Serial.printf ("│ 🚰 Litros úteis:   %6d L                   │\n", m.litrosUteis);
  if (!isnan(m.distCm)) Serial.printf("│ 📏 Distância:      %6.1f cm                │\n", m.distCm);
  else                  Serial.println("│ 📏 Distância:        --.- cm                │");
  if (m.batPct >= 0)    Serial.printf("│ 🔋 Bateria:        %6.2fV (%3d%%)           │\n", m.vbat, m.batPct);
  else                  Serial.println("│ 🔌 Alimentação:    USB/indefinido           │");
  Serial.printf ("│ 💡 LED:            %-23s │\n", corNivel);
  if (horaOK) {
    Serial.printf ("│ 🕒 Hora:           %02d:%02d:%02d               │\n", t.tm_hour, t.tm_min, t.tm_sec);
  } else {
    Serial.println("│ 🕒 Hora:           --:--:--               │");
  }
  Serial.println("└──────────────────────────────────────────────┘\n");

  if (wifiOK) (void)enviarSinric(m);

  desligarRadios();
}

bool decidirModoInicialDiaOuNoite(bool &horaOK_out, struct tm &t_out) {
  horaOK_out = obterHoraLocal(t_out);
  if (!horaOK_out) return true; // DIA por segurança
  return isJanelaDia(t_out);
}

// ====================================================
// SETUP / LOOP
// ====================================================
void setup() {
  Serial.begin(115200);
  delay(250);
  bootCount++;

  liberarHoldLEDsNoBoot();
  desligarWiFiBtLogoNoBoot();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(LED_AZUL,    OUTPUT);
  pinMode(LED_AMARELO, OUTPUT);
  pinMode(LED_VERDE,   OUTPUT);
  pinMode(LED_VERM,    OUTPUT);

  apagarTodosLEDs();

  Serial.println("\n==========================================");
  Serial.printf("RuralWaterTankMonitor - BOOT #%d\n", bootCount);
  Serial.println("DIA 07:00-19:00: sem deep sleep, LED sempre ON");
  Serial.println("NOITE 19:01-06:59: deep sleep, LED 3min/h na hora cheia");
  Serial.println("SINRIC (imagem): Range nivel_agua | Range litros_uteis | Alcance bateria_pct | Power");
  Serial.printf ("Tanque: %dL | Boia: %dcm abaixo do topo | Saída: %dcm da base\n",
                TANK_CAP_L, BOIA_ABAIXO_BORDA_CM, SAIDA_ACIMA_BASE_CM);
  Serial.printf ("Altura interna estimada (cm): %.1f (g_alturaVazioCm + %d)\n",
                alturaInternaEstimCm(), SAIDA_ACIMA_BASE_CM);
  Serial.printf ("Power inicial: %s\n", g_powerOn ? "ON" : "OFF");
  Serial.println("==========================================\n");

  // Boot: tenta NTP (somente se Power ON)
  if (g_powerOn) {
    bool wifiOK = conectarWiFiMulti(WIFI_TIMEOUT_MS);
    bool ntpOK = false;
    if (wifiOK) {
      (void)garantirTempo(wifiOK, ntpOK);
      desligarRadios();
    }
  }

  struct tm t = {};
  bool horaOK = false;
  bool modoDia = decidirModoInicialDiaOuNoite(horaOK, t);

  if (modoDia) {
    Serial.println("\n================= MODO DIA ===================");
    Serial.println("Regra: Sem deep sleep | LED sempre aceso");
    Serial.println("================================================\n");
    executarCicloDia();
  } else {
    executarModoNoite(); // não retorna
  }
}

void loop() {
  static unsigned long lastCycleMs = 0;

  struct tm t = {};
  bool horaOK = obterHoraLocal(t);

  if (horaOK) {
    if (!isJanelaDia(t)) executarModoNoite(); // não retorna
  }

  unsigned long nowMs = millis();
  if (lastCycleMs == 0 || (nowMs - lastCycleMs) >= DAY_CYCLE_MS) {
    lastCycleMs = nowMs;
    executarCicloDia();
  }

  delay(250);
}
