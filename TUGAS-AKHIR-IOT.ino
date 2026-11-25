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
//   F8 Kredensial dipindah ke secrets.h (P1)
//   P3 MQTT over TLS port 8883 (verifikasi sertifikat + sinkron waktu NTP)
//   P6 Offline: buffer store-and-forward bertimestamp, status online/offline, watchdog
//
//  Library: PubSubClient >= 2.8 (butuh setSocketTimeout), DHT sensor library (Adafruit)
// =====================================================================
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <sys/time.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <PubSubClient.h>
#include <DHT.h>

// --- KONFIGURASI WIFI & THINGSBOARD (nilai rahasia ada di secrets.h) ---
#include "secrets.h"
#include "ca_cert.h"   // root CA ThingsBoard Cloud (isi dari tb-cloud-root-ca.pem)
const char* ssid        = WIFI_SSID;
const char* password    = WIFI_PASSWORD;
const char* mqtt_server = MQTT_HOST;
const int   mqtt_port   = MQTT_PORT;      // 8883 = MQTT over TLS (P3)
const char* token       = DEVICE_TOKEN;

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

// --- PARAMETER OFFLINE / BUFFER (P6) ---
const int      RING_SIZE          = 600;     // kapasitas buffer (data) ~ 100 menit pada 10 dtk/data
const uint32_t OFFLINE_SAMPLE_MS  = 10000;   // simpan 1 data tiap 10 dtk saat offline (+ saat ada kejadian)
const uint32_t FLUSH_INTERVAL_MS  = 1000;    // kirim ulang 1 batch per interval setelah online
const int      FLUSH_BATCH        = 4;       // maks data per batch (dibatasi ukuran buffer MQTT)
const uint32_t STATUS_INTERVAL_MS = 10000;   // log status ke Serial
const uint32_t WDT_TIMEOUT_S      = 10;      // watchdog hardware untuk loop kontrol
const uint32_t NET_STALL_RESTART_MS = 90000; // restart jika task jaringan macet selama ini

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
// Set 1 HANYA untuk diagnosis (mematikan verifikasi sertifikat). Jangan dipakai di versi akhir.
#define TLS_INSECURE_DEBUG 0

WiFiClientSecure espClient;
bool caReady = false;
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

// Tipe data buffer offline. HARUS didefinisikan sebelum fungsi pertama:
// Arduino IDE membuat prototipe fungsi otomatis di atas fungsi pertama.
struct Rec {
  int64_t ms;        // waktu sejak boot (ms), dikonversi ke epoch saat dikirim
  float   suhu;
  int16_t cahaya;
  uint8_t alarmCode;
  uint8_t flags;
};
enum { F_SUHU = 1, F_SENSOR_ERR = 2, F_LAMPU_UTAMA = 4, F_LED = 8, F_BUZZER = 16, F_MANUAL = 32 };


volatile uint32_t netAlive = 0;      // detak task jaringan (diawasi loop kontrol)
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

bool timeSynced() { return time(nullptr) > 1700000000; }  // sertifikat butuh waktu valid

void connectMqtt() {
  if (!caReady) {
    Serial.println("[TLS] ca_cert.h belum diisi sertifikat root CA");
    return;
  }
  Serial.print("[MQTT] Menghubungkan ke ThingsBoard (TLS)...");
  if (mqtt.connect("ESP32_Incubator", token, NULL)) {
    Serial.println("terhubung");
    mqtt.subscribe("v1/devices/me/rpc/request/+");
    pushNow = true;
  } else {
    Serial.printf("gagal, rc=%d\n", mqtt.state());
    char tlsErr[100];
    if (espClient.lastError(tlsErr, sizeof(tlsErr)) != 0) Serial.printf("[TLS] error: %s\n", tlsErr);
  }
}

// ---------------------------------------------------------------------
//  BUFFER STORE-AND-FORWARD (hanya diakses oleh task jaringan)
// ---------------------------------------------------------------------
Rec ring[RING_SIZE];
int ringHead = 0, ringCount = 0;

void ringPush(const EdgeState& s) {
  Rec r;
  r.ms = esp_timer_get_time() / 1000;
  r.suhu = s.suhu;
  r.cahaya = (int16_t)s.cahaya;
  r.alarmCode = s.alarmCode;
  r.flags = ((s.suhuValid && !s.sensorError) ? F_SUHU : 0) |
            (s.sensorError ? F_SENSOR_ERR : 0) |
            (s.lampuUtamaNyala ? F_LAMPU_UTAMA : 0) |
            (s.ledOn ? F_LED : 0) |
            (s.buzzerAktif ? F_BUZZER : 0) |
            (manualSwitch ? F_MANUAL : 0);
  if (ringCount == RING_SIZE) {          // penuh: timpa data terlama
    ringHead = (ringHead + 1) % RING_SIZE;
    ringCount--;
  }
  ring[(ringHead + ringCount) % RING_SIZE] = r;
  ringCount++;
}

int recToJson(char* out, size_t cap, const Rec& r, int64_t ts) {
  int n = snprintf(out, cap, "{\"ts\":%lld,\"values\":{", (long long)ts);
  if (r.flags & F_SUHU) n += snprintf(out + n, cap - n, "\"suhu\":%.1f,", r.suhu);
  n += snprintf(out + n, cap - n,
    "\"cahaya\":%d,\"lampu_utama\":%s,\"lampu_cadangan\":%s,"
    "\"lampu_cadangan_manual\":%s,\"buzzer\":%s,"
    "\"alarm_code\":%d,\"alarm_text\":\"%s\",\"sensor_error\":%s}}",
    r.cahaya,
    (r.flags & F_LAMPU_UTAMA) ? "true" : "false",
    (r.flags & F_LED) ? "true" : "false",
    (r.flags & F_MANUAL) ? "true" : "false",
    (r.flags & F_BUZZER) ? "true" : "false",
    r.alarmCode, alarmText(r.alarmCode),
    (r.flags & F_SENSOR_ERR) ? "true" : "false");
  return n;
}

// Susun batch "[{ts,values},...]" dari data terlama. Mengembalikan jumlah data yang dimasukkan.
int buildBatch(char* out, size_t cap) {
  int64_t nowUp = esp_timer_get_time() / 1000;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  int64_t nowEpoch = (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;

  size_t n = 0;
  out[n++] = '[';
  int used = 0;
  for (int i = 0; i < FLUSH_BATCH && i < ringCount; i++) {
    const Rec& r = ring[(ringHead + i) % RING_SIZE];
    int64_t ts = nowEpoch - (nowUp - r.ms);
    char one[320];
    int m = recToJson(one, sizeof(one), r, ts);
    if (m <= 0 || m >= (int)sizeof(one)) break;
    if (n + m + 2 >= cap) break;
    if (used > 0) out[n++] = ',';
    memcpy(out + n, one, m);
    n += m;
    used++;
  }
  out[n++] = ']';
  out[n] = 0;
  return used;
}

void ringPop(int count) {
  ringHead = (ringHead + count) % RING_SIZE;
  ringCount -= count;
}

// ---------------------------------------------------------------------
bool publishTelemetry(const EdgeState& s) {
  char buf[448];
  int n = snprintf(buf, sizeof(buf), "{");
  if (s.suhuValid && !s.sensorError) {
    n += snprintf(buf + n, sizeof(buf) - n, "\"suhu\":%.1f,", s.suhu);
  }
  n += snprintf(buf + n, sizeof(buf) - n,
    "\"cahaya\":%d,\"lampu_utama\":%s,\"lampu_cadangan\":%s,"
    "\"lampu_cadangan_manual\":%s,\"buzzer\":%s,"
    "\"alarm_code\":%d,\"alarm_text\":\"%s\",\"sensor_error\":%s,\"buf_pending\":%d}",
    s.cahaya,
    s.lampuUtamaNyala ? "true" : "false",
    s.ledOn ? "true" : "false",
    manualSwitch ? "true" : "false",
    s.buzzerAktif ? "true" : "false",
    s.alarmCode, alarmText(s.alarmCode),
    s.sensorError ? "true" : "false",
    ringCount);

  Serial.println(buf);
  bool ok = mqtt.publish("v1/devices/me/telemetry", buf);

  if (ok) {
    // Sinkronkan status switch ke atribut (untuk widget yang membaca atribut)
    char attr[48];
    snprintf(attr, sizeof(attr), "{\"lampu_cadangan_manual\":%s}", manualSwitch ? "true" : "false");
    mqtt.publish("v1/devices/me/attributes", attr);
  }
  return ok;
}

EdgeState takeSnapshot() {
  EdgeState snap;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  snap = shared;
  xSemaphoreGive(stateMutex);
  return snap;
}

void networkTask(void* param) {
  uint32_t lastWifiTry = 0, lastMqttTry = 0, lastSend = 0;
  uint32_t lastBuf = 0, lastFlush = 0, lastStatus = 0;
  bool ntpStarted = false, ntpReported = false, wasOnline = false;

  for (;;) {
    netAlive = millis();
    bool online = (WiFi.status() == WL_CONNECTED) && timeSynced() && mqtt.connected();

    if (online != wasOnline) {
      wasOnline = online;
      if (online) Serial.printf("[NET] ONLINE - data tertunda: %d\n", ringCount);
      else        Serial.println("[NET] OFFLINE - kontrol lokal tetap aktif, data disimpan di buffer");
    }

    // ---- Mode offline: simpan data berkala + saat ada kejadian (perubahan alarm/LED/switch) ----
    if (!online && (pushNow || millis() - lastBuf >= OFFLINE_SAMPLE_MS)) {
      pushNow = false;
      lastBuf = millis();
      ringPush(takeSnapshot());
    }

    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastWifiTry > WIFI_RETRY_MS) {
        lastWifiTry = millis();
        Serial.println("[WiFi] mencoba menyambung ulang...");
        WiFi.reconnect();
      }
    } else if (!timeSynced()) {
      if (!ntpStarted) {
        configTime(0, 0, "pool.ntp.org", "time.google.com");
        ntpStarted = true;
        Serial.println("[NTP] sinkronisasi waktu...");
      }
    } else if (!mqtt.connected()) {
      if (!ntpReported) { ntpReported = true; Serial.println("[NTP] waktu tersinkron"); }
      if (millis() - lastMqttTry > MQTT_RETRY_MS) {
        lastMqttTry = millis();
        connectMqtt();
      }
    } else {
      mqtt.loop();

      // Telemetri langsung
      if (pushNow || millis() - lastSend >= SEND_INTERVAL_MS) {
        pushNow = false;
        lastSend = millis();
        EdgeState snap = takeSnapshot();
        if (!publishTelemetry(snap)) {
          ringPush(snap);
          Serial.println("[BUF] publish gagal, data disimpan");
        }
      }

      // Kirim ulang data yang tertunda (terlama dulu)
      if (ringCount > 0 && millis() - lastFlush >= FLUSH_INTERVAL_MS) {
        lastFlush = millis();
        char batch[960];
        int used = buildBatch(batch, sizeof(batch));
        if (used > 0 && mqtt.publish("v1/devices/me/telemetry", batch)) {
          ringPop(used);
          Serial.printf("[BUF] terkirim %d data, sisa %d\n", used, ringCount);
        }
      }
    }

    if (millis() - lastStatus >= STATUS_INTERVAL_MS) {
      lastStatus = millis();
      Serial.printf("[STATUS] wifi=%d mqtt=%d buf=%d heap=%u\n",
                    WiFi.status() == WL_CONNECTED, mqtt.connected(), ringCount, (unsigned)ESP.getFreeHeap());
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
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(10);
  mqtt.setSocketTimeout(5);

#if TLS_INSECURE_DEBUG
  espClient.setInsecure();
  caReady = true;
  Serial.println("[TLS] PERINGATAN: mode insecure (tanpa verifikasi sertifikat)");
#else
  caReady = (strstr(TB_ROOT_CA, "PASTE_") == nullptr);
  if (caReady) espClient.setCACert(TB_ROOT_CA);
#endif
  espClient.setHandshakeTimeout(30);

  // Stack lebih besar: handshake TLS (mbedtls) butuh banyak stack
  netAlive = millis();
  xTaskCreatePinnedToCore(networkTask, "net", 12288, NULL, 1, NULL, 0);

  // Watchdog hardware untuk loop kontrol: jika loop() macet > WDT_TIMEOUT_S, ESP32 restart
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = { WDT_TIMEOUT_S * 1000, 0, true };
  esp_task_wdt_reconfigure(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);

  Serial.println("Kontrol lokal aktif (tidak menunggu jaringan)");
}

void loop() {
  esp_task_wdt_reset();
  uint32_t now = millis();

  // Task jaringan macet (mis. handshake/DNS menggantung)? restart agar pulih sendiri
  if (now - netAlive > NET_STALL_RESTART_MS) {
    Serial.println("[WDT] task jaringan macet, restart");
    delay(100);
    ESP.restart();
  }

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