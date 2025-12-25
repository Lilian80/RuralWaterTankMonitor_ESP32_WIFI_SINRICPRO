/******************************************************* 
        #### RuralWaterTankMonitor-ESP32 WIFI/SINRICPRO ####

# Sistema embarcado para medir o nível de água em reservatório Fortlev 1000 L usando ESP32 + HC-SR04, com indicação local por LEDs e telemetria/notificações via WIFI/Sinric Pro.
 
# Projetado para ambientes remotos (pastos e áreas externas), com arquitetura voltada à sustentabilidade: ciclos de leitura, deep sleep para baixo consumo e monitoramento de bateria para maximizar autonomia.

# Alimentação baseada em bateria 18650 (1S), com carga via TP4056 e step-up MT3608 para 5 V. O firmware envia nível/distância/bateria ao Sinric Pro e utiliza LEDs (Azul/Verde/Amarelo/Vermelho) como indicador visual do status do reservatório. 

## Como compilar
1. Copie `secrets_nivel.example.h` para `secrets_nivel.h`
2. Preencha as credenciais (Sinric e Wi-Fi)
3. Abra `RuralWaterTankMonitor-ESP32/SINRIC.ino` no Arduino IDE e envie para o ESP32


 * ESP32 + Sonar (HC-SR04)
 * PRODUÇÃO: Deep Sleep + RTC HOLD
 *
 * Regras do LED (nível):
 * - DIA: LED de nível fica ACESO continuamente
 * - NOITE: LED fica APAGADO e acende por 3 minutos a cada hora
 *
 * LEDs (padrão): 25/26/27/33
 * Ligação: GPIO -> resistor -> LED -> GND (ativo em HIGH)
 *
 * Autor: Lilian de Ávila Costa
 * Versão: 1.2.0
 * Revisão: 17/12/2025
 ******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <SinricPro.h>
#include <SinricProDevice.h>
#include <Capabilities/RangeController.h>
#include <Capabilities/PushNotification.h>
#include <time.h>

#include "secrets_nivel.h"   // APP_KEY, APP_SECRET, DEVICE_ID, WIFI_SSID_1/2, WIFI_PASS_1/2

#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#ifdef ESP32
  #include "esp_bt.h"
#endif

// ====================================================
// CONFIG PRODUÇÃO
// ====================================================
static const uint64_t SLEEP_TIME_US      = 10ULL * 60ULL * 1000000ULL; // 10 min
static const uint32_t WIFI_TIMEOUT_MS    = 10000;
static const uint32_t SINRIC_TIMEOUT_MS  = 15000;

// NTP / Horário (UTC-3)
static const char* NTP_SERVER          = "pool.ntp.org";
static const long  GMT_OFFSET_SEC      = -3 * 3600;
static const int   DAYLIGHT_OFFSET_SEC = 0;

// Janela noturna
static const int HORA_INICIO_NOITE = 21;
static const int HORA_FIM_NOITE    = 5;

// À noite: acender 3 min a cada hora
static const int NOITE_ON_MIN      = 3;   // aceso por 3 minutos
static const int NOITE_PERIOD_MIN  = 60;  // a cada 60 minutos

RTC_DATA_ATTR time_t ultimoNTP = 0;
RTC_DATA_ATTR int bootCount = 0;

// ====================================================
// WIFI (2 redes)
// ====================================================
struct WiFiConfig { const char* ssid; const char* pass; };
WiFiConfig wifiList[] = {
  {WIFI_SSID_1, WIFI_PASS_1},
  {WIFI_SSID_2, WIFI_PASS_2}
};
const int WIFI_COUNT = sizeof(wifiList) / sizeof(wifiList[0]);

// ====================================================
// HARDWARE
// ====================================================
const int PIN_TRIG = 5;
const int PIN_ECHO = 18; // HC-SR04 ECHO -> divisor 5V->3.3V no ESP32

// LEDs (RTC GPIOs) - padrão definido
const int LED_AZUL    = 25;
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
const int LIM_CRITICO_NIVEL = 5;
const int LIM_CRITICO_BAT   = 15;

RTC_DATA_ATTR bool notificadoNivelCritico = false;
RTC_DATA_ATTR bool notificadoBatCritica   = false;

// ====================================================
// SINRIC PRO
// ====================================================
class WaterLevelIndicator
: public SinricProDevice
, public RangeController<WaterLevelIndicator>
, public PushNotification<WaterLevelIndicator> {
  friend class RangeController<WaterLevelIndicator>;
  friend class PushNotification<WaterLevelIndicator>;
public:
  WaterLevelIndicator(const String &deviceId) : SinricProDevice(deviceId, "WaterLevelIndicator") {}
};
WaterLevelIndicator &tank = SinricPro[DEVICE_ID];

const char* DP_NIVEL     = "nivel_agua";
const char* DP_DISTANCIA = "distancia_cm";
const char* DP_BATERIA   = "bateria_pct";
const char* DP_H_CHEIO   = "altura_cheio_cm";
const char* DP_H_VAZIO   = "altura_vazio_cm";

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
  if      (nivel >= 75) { ledWrite(LED_AZUL,    true); return "🔵 AZUL (>=75%)"; }
  else if (nivel >= 50) { ledWrite(LED_VERDE,   true); return "🟢 VERDE (50-74%)"; }
  else if (nivel >= 25) { ledWrite(LED_AMARELO, true); return "🟡 AMARELO (25-49%)"; }
  else                  { ledWrite(LED_VERM,    true); return "🔴 VERMELHO (<25%)"; }
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

// Mantém OFF no deep sleep
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

// Mantém o estado atual dos LEDs durante deep sleep (espelhando GPIO -> RTC)
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

int distanciaToNivelPct(float distCm) {
  int pct = map((int)round(distCm), g_alturaVazioCm, g_alturaCheioCm, 0, 100);
  return constrain(pct, 0, 100);
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
  return constrain((int)round(pct), 0, 100);
}

// ====================================================
// WiFi / NTP / Modo Noturno
// ====================================================
bool conectarWiFiMulti(uint32_t timeoutMs) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  for (int i = 0; i < WIFI_COUNT; i++) {
    Serial.printf("⏳ [WiFi] Tentando '%s'\n", wifiList[i].ssid);
    WiFi.begin(wifiList[i].ssid, wifiList[i].pass);

    uint32_t ts = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - ts < timeoutMs) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("✓ [WiFi] Conectado em '%s' | IP: %s\n",
                    wifiList[i].ssid, WiFi.localIP().toString().c_str());
      return true;
    }
  }
  Serial.println("✗ [WiFi] Falha (offline).");
  return false;
}

bool sincronizarNTP() {
  Serial.println("⏳ [NTP] Sincronizando relógio...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  struct tm timeinfo;
  int tentativas = 0;
  while (!getLocalTime(&timeinfo) && tentativas < 10) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }
  Serial.println();

  if (tentativas >= 10) {
    Serial.println("✗ [NTP] Falha na sincronização");
    return false;
  }

  time(&ultimoNTP);
  Serial.printf("✓ [NTP] OK: %02d/%02d/%04d %02d:%02d:%02d\n",
                timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return true;
}

bool isModoNoturno() {
  struct tm t;
  if (!getLocalTime(&t)) {
    Serial.println("⚠️ [Horário] Sem hora (assumindo DIA).");
    return false;
  }
  int hora = t.tm_hour;
  return (hora >= HORA_INICIO_NOITE || hora < HORA_FIM_NOITE);
}

// Regra da noite: LED acende nos primeiros 3 minutos de cada hora
bool deveAcenderLEDNaNoite() {
  struct tm t;
  if (!getLocalTime(&t)) return false; // sem hora -> conserva (não acende à noite)
  int minuto = t.tm_min;
  return (minuto >= 0 && minuto < NOITE_ON_MIN);
}

// ====================================================
// Sinric
// ====================================================
void enviarDP(const char* inst, int val) {
  tank.sendRangeValueEvent(inst, val);
}

bool onRangeValueCb(const String&, const String& instance, int &value) {
  if (instance == DP_H_CHEIO) {
    g_alturaCheioCm = constrain(value, 1, 120);
    Serial.printf("[Calibração] Altura CHEIO: %d cm\n", g_alturaCheioCm);
    return true;
  }
  if (instance == DP_H_VAZIO) {
    g_alturaVazioCm = constrain(value, 20, 200);
    Serial.printf("[Calibração] Altura VAZIO: %d cm\n", g_alturaVazioCm);
    return true;
  }
  return false;
}

// ====================================================
// SETUP (um ciclo por boot)
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
  Serial.printf("SENSOR NIVEL DE ÁGUA 1000L - BOOT #%d\n", bootCount);
  Serial.println("Regra LED: DIA sempre aceso | NOITE 3min/h");
  Serial.println("==========================================\n");

  // 1) Medição
  Serial.println("📏 [Medição] Lendo Sensor...");
  float distCm = medirDistanciaMedianaCm();

  int nivel = 0;
  if (isnan(distCm)) {
    Serial.println("✗ [Sensor] Leitura inválida. Nível=0% (fallback).");
    nivel = 0;
  } else {
    nivel = distanciaToNivelPct(distCm);
  }

  float vbat = lerBateriaV();
  int   batPct = bateriaPct(vbat);

  // 2) Conecta WiFi para NTP + Sinric (quando possível)
  bool wifiOK = conectarWiFiMulti(WIFI_TIMEOUT_MS);

  bool ntpOK = false;
  bool modoNoturno = false;

  if (wifiOK) {
    time_t agora;
    time(&agora);
    if (ultimoNTP == 0 || (agora - ultimoNTP) > 12 * 3600) {
      ntpOK = sincronizarNTP();
    } else {
      ntpOK = true;
    }
    modoNoturno = ntpOK ? isModoNoturno() : false; // sem NTP -> trata como DIA
  } else {
    // sem WiFi, sem NTP confiável -> conserva LED como DIA (mais seguro)
    modoNoturno = false;
  }

  // 3) Decide se LED fica aceso neste ciclo (antes do sleep)
  bool acenderLED = true;

  if (modoNoturno) {
    // À noite só acende na janela de 3 min por hora
    acenderLED = deveAcenderLEDNaNoite();
  } else {
    // De dia sempre aceso
    acenderLED = true;
  }

  const char* corNivel = "LED OFF (noite)";
  if (acenderLED) {
    corNivel = aplicarLEDNivel(nivel);
  } else {
    apagarTodosLEDs();
  }

  Serial.println("┌───────────────────────────────────────┐");
  Serial.printf ("│ 💧 Nível:     %6d %%                 │\n", nivel);
  if (!isnan(distCm)) Serial.printf("│ 📏 Distância: %6.1f cm               │\n", distCm);
  else                Serial.println("│ 📏 Distância:   --.- cm               │");
  if (batPct >= 0)    Serial.printf("│ 🔋 Bateria:   %6.2fV (%3d%%)          │\n", vbat, batPct);
  else                Serial.println("│ 🔌 Alimentação: USB/indefinido        │");
  Serial.printf ("│ 💡 LED:       %-21s │\n", corNivel);
  Serial.println("└───────────────────────────────────────┘\n");

  // 4) Sinric (somente se WiFi OK)
  if (wifiOK) {
    tank.onRangeValue(DP_H_CHEIO, onRangeValueCb);
    tank.onRangeValue(DP_H_VAZIO, onRangeValueCb);

    SinricPro.onConnected([] { Serial.println("✓ [SinricPro] Conectado!"); });
    SinricPro.onDisconnected([] { Serial.println("✗ [SinricPro] Desconectado"); });

    Serial.println("⏳ [SinricPro] Iniciando...");
    SinricPro.begin(APP_KEY, APP_SECRET);

    uint32_t ts = millis();
    while (!SinricPro.isConnected() && millis() - ts < SINRIC_TIMEOUT_MS) {
      SinricPro.handle();
      delay(50);
    }

    if (SinricPro.isConnected()) {
      enviarDP(DP_NIVEL, nivel);
      if (!isnan(distCm)) enviarDP(DP_DISTANCIA, (int)round(distCm));
      if (batPct >= 0)    enviarDP(DP_BATERIA, batPct);

      if (!notificadoNivelCritico && nivel <= LIM_CRITICO_NIVEL) {
        tank.sendPushNotification("⚠️ Nível de água MUITO baixo!");
        notificadoNivelCritico = true;
      }
      if (notificadoNivelCritico && nivel > (LIM_CRITICO_NIVEL + 3)) {
        notificadoNivelCritico = false;
      }

      if (batPct >= 0 && !notificadoBatCritica && batPct <= LIM_CRITICO_BAT) {
        tank.sendPushNotification("🔋 Bateria fraca — recarregue o sistema.");
        notificadoBatCritica = true;
      }
      if (notificadoBatCritica && batPct > (LIM_CRITICO_BAT + 5)) {
        notificadoBatCritica = false;
      }

      for (int i = 0; i < 30; i++) { SinricPro.handle(); delay(50); }
      Serial.println("📤 [Sinric] Dados enviados.");
    } else {
      Serial.println("✗ [SinricPro] Offline (sem envio).");
    }
  } else {
    Serial.println("⚠️ [WiFi] Offline. Sem envio ao app.");
  }

  // 5) Desliga rádio para economizar
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // 6) Deep sleep: se LED deve ficar aceso, segura estado; senão, segura OFF
  Serial.printf("\n💤 [Deep Sleep] Dormindo %d min...\n", (int)(SLEEP_TIME_US / 60000000ULL));
  Serial.println("═══════════════════════════════════════════════\n");

  if (acenderLED) {
    aplicarHoldLEDsEspelhadoAntesDeDormir(); // mantém LED do nível durante o sleep
  } else {
    apagarTodosLEDs();
    aplicarHoldLEDsOffAntesDeDormir();       // garante tudo OFF durante o sleep
  }

  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);
  esp_deep_sleep_start();
}

void loop() {
  // nunca executa (deep sleep reinicia no setup)
}
