#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// --- KONFIGURASI WIFI & THINGSBOARD ---
const char* ssid = "Harvey";
const char* password = "12345678";
const char* mqtt_server = "eu.thingsboard.cloud";
const char* token = "j8ukjicm2Ab4Rj1rPZb5";

// --- PINOUT ---
#define DHTPIN 4
#define DHTTYPE DHT11
#define LDR_PIN 34
#define BUZZER_PIN 5
#define LED_PIN 2   // LED cadangan: GPIO2 -> resistor 220Ω -> anoda (+) LED, katoda (-) -> GND

// Wiring di atas = active HIGH (HIGH menyalakan LED).
// Jika LED dipasang terbalik (anoda ke 3V3 lewat resistor, katoda ke GPIO2), ubah ke false.
#define LED_ACTIVE_HIGH true

#define LED_ON  (LED_ACTIVE_HIGH ? HIGH : LOW)
#define LED_OFF (LED_ACTIVE_HIGH ? LOW : HIGH)

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

const int LIGHT_THRESHOLD = 3000;     // Sesuaikan dengan gelap/terang ruangan
const float TEMP_THRESHOLD = 32.0;    // Batas suhu
unsigned long lastSend = 0;

void setup_wifi() {
  delay(10);
  Serial.print("Menghubungkan ke WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke ThingsBoard...");
    if (client.connect("ESP32_Incubator", token, NULL)) {
      Serial.println("terhubung");
    } else {
      Serial.print("gagal, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LED_OFF);

  // Tes LED cadangan: kedip 2x saat startup
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, LED_ON);
    delay(200);
    digitalWrite(LED_PIN, LED_OFF);
    delay(200);
  }

  dht.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  if (millis() - lastSend > 2000) {
    lastSend = millis();

    int ldrValue = analogRead(LDR_PIN);
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (isnan(t) || isnan(h)) {
      Serial.println("Gagal membaca sensor DHT");
      return;
    }

    // LOGIKA KONTROL LOKAL
    // Asumsi: nilai LDR tinggi = gelap = lampu utama mati
    // (jika terbalik, ganti jadi: ldrValue < LIGHT_THRESHOLD)
    bool lampuUtamaMati = (ldrValue > LIGHT_THRESHOLD);
    bool suhuTurun      = (t < TEMP_THRESHOLD);

    // LED cadangan: nyala hanya jika gelap
    bool cadanganNyala = lampuUtamaMati;

    // Buzzer: nyala hanya jika gelap DAN suhu rendah
    bool buzzerNyala = (lampuUtamaMati && suhuTurun);

    digitalWrite(LED_PIN, cadanganNyala ? LED_ON : LED_OFF);
    digitalWrite(BUZZER_PIN, buzzerNyala ? HIGH : LOW);

    // BENTUK JSON PAYLOAD
    String payload = "{";
    payload += "\"suhu\":" + String(t) + ",";
    payload += "\"kelembapan\":" + String(h) + ",";
    payload += "\"cahaya\":" + String(ldrValue) + ",";
    payload += "\"lampu_utama\":" + String(lampuUtamaMati ? "false" : "true") + ",";
    payload += "\"lampu_cadangan\":" + String(cadanganNyala ? "true" : "false") + ",";
    payload += "\"buzzer\":" + String(buzzerNyala ? "true" : "false");
    payload += "}";

    Serial.println(payload);

    // KIRIM KE THINGSBOARD
    client.publish("v1/devices/me/telemetry", (char*) payload.c_str());
  }
}
