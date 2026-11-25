// secrets.h  -- JANGAN di-commit / dibagikan (sudah ada di .gitignore)
#pragma once

#define WIFI_SSID     "Harvey"
#define WIFI_PASSWORD "12345678"

#define MQTT_HOST     "mqtt.eu.thingsboard.cloud"
#define MQTT_PORT     8883            // MQTT over TLS

// Token lama masih dipakai sementara agar perangkat tetap online.
// GANTI dengan token baru setelah langkah P2 (regenerate di ThingsBoard).
#define DEVICE_TOKEN  "j8ukjicm2Ab4Rj1rPZb5"