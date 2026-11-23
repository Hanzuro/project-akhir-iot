// =====================================================================
//  TA_IoT v2 - Inkubator ESP32 (edge-first)
//  Perubahan utama dari v1 (F1-F8):
//   F1 RPC switch dashboard (setValue / getValue)
//   F2 Kontrol lokal TIDAK PERNAH diblokir jaringan (WiFi/MQTT di task terpisah, core 0)
//   F3 Kegagalan DHT tidak menghentikan kontrol (fail-safe, alarm_code 9)
//   F4 Buzzer + LED mengikuti tabel keputusan, buzzer = bip berkala
//   F5 Ambang suhu disatukan: 28 C
//   F6 Keputusan alarm dihitung di edge -> alarm_code & alarm_text dikirim
//   F7 LDR: rata-rata bergerak + histeresis + debounce
//   F8 Kredensial masih di file ini (dipindah ke secrets.h di langkah P1)
//
//  Library: PubSubClient >= 2.8 (butuh setSocketTimeout), DHT sensor library (Adafruit)
// =====================================================================
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// --- KONFIGURASI WIFI & THINGSBOARD (dipindah ke secrets.h di P1) ---
const char* ssid        = "Harvey";
const char* password    = "12345678";
const char* mqtt_server = "eu.thingsboard.cloud";
const int   mqtt_port   = 1883;                  // TLS 8883 dikerjakan di P3
const char* token       = "j8ukjicm2Ab4Rj1rPZb5"; // di-regenerate di P2

// --- PINOUT ---
#define DHTPIN     4
#define DHTTYPE    DHT11
#define LDR_PIN    34
#define BUZZER_PIN 5
#define LED_PIN    2

#define LED_ACTIVE_HIGH true   // false jika LED dipasang terbalik
#define LED_ON  (LED_ACTIVE_HIGH ? HIGH : LOW)
#define LED_OFF (LED_ACTIVE_HIGH ? LOW : HIGH)

// --- PARAMETER LOGIKA EDGE ---
const float TEMP_THRESHOLD = 28.0;   // batas dingin/panas (C)
const float TEMP_HYST      = 1.0;    // histeresis suhu (DHT11 resolusi 1 C). Set 0 untuk mematikan
const int   LDR_DARK_ABOVE = 3000;   // nilai rata-rata > ini  -> kandidat GELAP
const int   LDR_LIGHT_BELOW = 2700;  // nilai rata-rata < ini  -> kandidat TERANG
const uint32_t LDR_DEBOUNCE_MS = 1000;   // kandidat harus bertahan selama ini
const uint32_t LDR_SAMPLE_MS   = 100;    // interval sampel LDR
const int   LDR_WINDOW = 10;             // jumlah sampel rata-rata bergerak
const uint32_t DHT_INTERVAL_MS = 2000;
const int   DHT_FAIL_LIMIT = 3;          // gagal berturut-turut -> sensor_error

// Pola bip buzzer (non-blocking)
const uint32_t BEEP_PERIOD_MS = 1500;
const uint32_t BEEP_ON_MS     = 120;

// --- PARAMETER JARINGAN ---
const uint32_t SEND_INTERVAL_MS = 2000;
const uint32_t WIFI_RETRY_MS    = 15000;
const uint32_t MQTT_RETRY_MS    = 5000;

// --- RPC (samakan dengan pengaturan widget Switch di dashboard) ---
const char* RPC_REQ_PREFIX  = "v1/devices/me/rpc/request/";
const char* RPC_RESP_PREFIX = "v1/devices/me/rpc/response/";
const char* RPC_SET_METHOD  = "setValue";
const char* RPC_GET_METHOD  = "getValue";

// ---------------------------------------------------------------------
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient mqtt(espClient);

// Status bersama antara task kontrol (core 1) dan task jaringan (core 0)
struct EdgeState {
  float   suhu;
  bool    suhuValid;
  bool    sensorError;
  int     cahaya;
  bool    lampuUtamaNyala;
  bool    ledOn;
  bool    buzzerAktif;     // alarm meminta buzzer (bukan keadaan sesaat bip)
  uint8_t alarmCode;
};
EdgeState shared = {0, false, false, 0, true, false, false, 0};
SemaphoreHandle_t stateMutex;

volatile bool manualSwitch = false;  // switch lampu cadangan dari dashboard
volatile bool pushNow = false;       // minta kirim telemetri segera

// ---------------------------------------------------------------------
const char* alarmText(uint8_t code) {
  switch (code) {
    case 0: return "AMAN";
    case 1: return "DARURAT: lampu mati + suhu rendah";
    case 2: return "DARURAT: lampu mati, suhu tinggi";
    case 3: return "DARURAT: lampu nyala tapi suhu rendah, cek inkubator";
    case 9: return "DARURAT: sensor suhu gagal";
    default: return "?";
  }
}

// =====================================================================
//  TASK JARINGAN (core 0): WiFi, MQTT, RPC, publish
// =====================================================================
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  size_t prefixLen = strlen(RPC_REQ_PREFIX);
  if (strncmp(topic, RPC_REQ_PREFIX, prefixLen) != 0) return;
  const char* reqId = topic + prefixLen;

  char msg[200];
  unsigned int n = (len < sizeof(msg) - 1) ? len : sizeof(msg) - 1;
  memcpy(msg, payload, n);
  msg[n] = 0;
  Serial.printf("[RPC] %s -> %s\n", reqId, msg);

  bool isSet = strstr(msg, RPC_SET_METHOD) != nullptr;
  bool isGet = strstr(msg, RPC_GET_METHOD) != nullptr;
  if (!isSet && !isGet) return;

  if (isSet) {
    const char* p = strstr(msg, "\"params\"");
    if (p && (p = strchr(p, ':'))) {
      p++;
      while (*p == ' ' || *p == '"') p++;
      manualSwitch = (*p == 't' || *p == 'T' || *p == '1');
      pushNow = true;
    }
  }

  char respTopic[96];
  snprintf(respTopic, sizeof(respTopic), "%s%s", RPC_RESP_PREFIX, reqId);
  mqtt.publish(respTopic, manualSwitch ? "true" : "false");
}

void connectMqtt() {
  Serial.print("[MQTT] Menghubungkan ke ThingsBoard...");
  if (mqtt.connect("ESP32_Incubator", token, NULL)) {
    Serial.println("terhubung");
    mqtt.subscribe("v1/devices/me/rpc/request/+");
    pushNow = true;
  } else {
    Serial.printf("gagal, rc=%d\n", mqtt.state());
  }
}

void publishTelemetry(const EdgeState& s) {
  char buf[384];
  int n = snprintf(buf, sizeof(buf), "{");
  if (s.suhuValid && !s.sensorError) {
    n += snprintf(buf + n, sizeof(buf) - n, "\"suhu\":%.1f,", s.suhu);
  }
  n += snprintf(buf + n, sizeof(buf) - n,
    "\"cahaya\":%d,\"lampu_utama\":%s,\"lampu_cadangan\":%s,"
    "\"lampu_cadangan_manual\":%s,\"buzzer\":%s,"
    "\"alarm_code\":%d,\"alarm_text\":\"%s\",\"sensor_error\":%s}",
    s.cahaya,
    s.lampuUtamaNyala ? "true" : "false",
    s.ledOn ? "true" : "false",
    manualSwitch ? "true" : "false",
    s.buzzerAktif ? "true" : "false",
    s.alarmCode, alarmText(s.alarmCode),
    s.sensorError ? "true" : "false");

  Serial.println(buf);
  mqtt.publish("v1/devices/me/telemetry", buf);

  // Sinkronkan status switch ke atribut (untuk widget yang membaca atribut)
  char attr[48];
  snprintf(attr, sizeof(attr), "{\"lampu_cadangan_manual\":%s}", manualSwitch ? "true" : "false");
  mqtt.publish("v1/devices/me/attributes", attr);
}

void networkTask(void* param) {
  uint32_t lastWifiTry = 0, lastMqttTry = 0, lastSend = 0;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastWifiTry > WIFI_RETRY_MS) {
        lastWifiTry = millis();
        Serial.println("[WiFi] mencoba menyambung ulang...");
        WiFi.reconnect();
      }
    } else if (!mqtt.connected()) {
      if (millis() - lastMqttTry > MQTT_RETRY_MS) {
        lastMqttTry = millis();
        connectMqtt();
      }
    } else {
      mqtt.loop();
      if (pushNow || millis() - lastSend >= SEND_INTERVAL_MS) {
        pushNow = false;
        lastSend = millis();
        EdgeState snap;
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        snap = shared;
        xSemaphoreGive(stateMutex);
        publishTelemetry(snap);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =====================================================================
//  KONTROL LOKAL (core 1 / loop): tidak bergantung pada jaringan
// =====================================================================
int ldrBuf[LDR_WINDOW];
int ldrIdx = 0;
long ldrSum = 0;
uint32_t lastLdrSample = 0, lastDhtRead = 0;

bool lampuUtamaNyala = true;
bool pendingChange = false;
uint32_t pendingSince = 0;

float suhu = 0;
bool suhuValid = false, sensorError = false, dingin = false;
int dhtFail = 0;

void sampleLdr() {
  int v = analogRead(LDR_PIN);
  ldrSum += v - ldrBuf[ldrIdx];
  ldrBuf[ldrIdx] = v;
  ldrIdx = (ldrIdx + 1) % LDR_WINDOW;
}

void updateLampState(int avg) {
  bool candidate = lampuUtamaNyala;
  if (lampuUtamaNyala && avg > LDR_DARK_ABOVE) candidate = false;
  else if (!lampuUtamaNyala && avg < LDR_LIGHT_BELOW) candidate = true;

  if (candidate != lampuUtamaNyala) {
    if (!pendingChange) { pendingChange = true; pendingSince = millis(); }
    else if (millis() - pendingSince >= LDR_DEBOUNCE_MS) {
      lampuUtamaNyala = candidate;
      pendingChange = false;
    }
  } else {
    pendingChange = false;
  }
}

void readDht() {
  float t = dht.readTemperature();
  if (isnan(t)) {
    if (++dhtFail >= DHT_FAIL_LIMIT) sensorError = true;
    Serial.println("[DHT] gagal baca");
    return;
  }
  dhtFail = 0;
  sensorError = false;
  if (!suhuValid) dingin = (t < TEMP_THRESHOLD);
  suhuValid = true;
  suhu = t;
  // histeresis: keluar dari 'dingin' butuh >= ambang + hist; masuk 'dingin' saat < ambang
  if (dingin) { if (t >= TEMP_THRESHOLD + TEMP_HYST) dingin = false; }
  else        { if (t <  TEMP_THRESHOLD)              dingin = true;  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LED_OFF);

  // Tes LED saat startup
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, LED_ON);  delay(200);
    digitalWrite(LED_PIN, LED_OFF); delay(200);
  }

  dht.begin();

  // Isi awal buffer LDR agar rata-rata langsung valid
  int first = analogRead(LDR_PIN);
  for (int i = 0; i < LDR_WINDOW; i++) ldrBuf[i] = first;
  ldrSum = (long)first * LDR_WINDOW;
  lampuUtamaNyala = (first <= LDR_DARK_ABOVE);

  stateMutex = xSemaphoreCreateMutex();

  // Jaringan: dimulai non-blocking, dikelola task core 0
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(3);
  xTaskCreatePinnedToCore(networkTask, "net", 8192, NULL, 1, NULL, 0);

  Serial.println("Kontrol lokal aktif (tidak menunggu jaringan)");
}

void loop() {
  uint32_t now = millis();

  if (now - lastLdrSample >= LDR_SAMPLE_MS) {
    lastLdrSample = now;
    sampleLdr();
    updateLampState((int)(ldrSum / LDR_WINDOW));
  }
  if (now - lastDhtRead >= DHT_INTERVAL_MS) {
    lastDhtRead = now;
    readDht();
  }

  // ---- TABEL KEPUTUSAN EDGE ----
  bool gelap = !lampuUtamaNyala;
  bool autoLed = false, buzzerAktif = false;
  uint8_t code = 0;

  if (sensorError) {                 // fail-safe
    code = 9; autoLed = gelap; buzzerAktif = true;
  } else if (!suhuValid) {           // pemanasan sensor
    code = 0; autoLed = gelap; buzzerAktif = false;
  } else if (gelap && dingin) {
    code = 1; autoLed = true;  buzzerAktif = true;
  } else if (gelap && !dingin) {
    code = 2; autoLed = false; buzzerAktif = true;
  } else if (!gelap && !dingin) {
    code = 0; autoLed = false; buzzerAktif = false;
  } else {                           // lampu nyala tapi dingin
    code = 3; autoLed = false; buzzerAktif = true;
  }

  // LED = otomatis ATAU switch dashboard (alarm otomatis tidak bisa dimatikan switch)
  bool ledOn = autoLed || manualSwitch;
  digitalWrite(LED_PIN, ledOn ? LED_ON : LED_OFF);

  // Buzzer bip berkala, non-blocking
  bool bipSekarang = buzzerAktif && ((now % BEEP_PERIOD_MS) < BEEP_ON_MS);
  digitalWrite(BUZZER_PIN, bipSekarang ? HIGH : LOW);

  // Perbarui status bersama; kirim segera jika ada perubahan penting
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  bool changed = (shared.alarmCode != code) || (shared.ledOn != ledOn) ||
                 (shared.lampuUtamaNyala != lampuUtamaNyala) ||
                 (shared.buzzerAktif != buzzerAktif);
  shared.suhu = suhu;
  shared.suhuValid = suhuValid;
  shared.sensorError = sensorError;
  shared.cahaya = (int)(ldrSum / LDR_WINDOW);
  shared.lampuUtamaNyala = lampuUtamaNyala;
  shared.ledOn = ledOn;
  shared.buzzerAktif = buzzerAktif;
  shared.alarmCode = code;
  xSemaphoreGive(stateMutex);
  if (changed) pushNow = true;

  delay(5);
}
