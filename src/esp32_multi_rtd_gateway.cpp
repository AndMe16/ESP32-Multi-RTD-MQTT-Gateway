/*
  ESP32 Multi-RTD MQTT Gateway

  Generic ESP32 firmware that:
  - Reads temperature data from multiple RTD PT100 sensors (MAX31865)
  - Acts as an IoT Gateway using MQTT (ThingsBoard compatible)
  - Publishes telemetry per virtual device
  - Supports OTA firmware updates
*/

/////////////////////////////////////
// Libraries
/////////////////////////////////////

#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <Espressif_Updater.h>
#include <Preferences.h>
#include <ArduinoJson.h>

/////////////////////////////////////
// Configuration
/////////////////////////////////////

constexpr char MQTT_BROKER_HOST[] PROGMEM = "demo.thingsboard.io";
constexpr uint16_t MQTT_BROKER_PORT PROGMEM = 1883U;
constexpr uint16_t MAX_MESSAGE_SIZE PROGMEM = 1024U;

constexpr char CURRENT_FIRMWARE_TITLE[] PROGMEM = "ESP32_MULTI_RTD_GATEWAY";
constexpr char CURRENT_FIRMWARE_VERSION[] PROGMEM = "1.0.0";
constexpr char FW_STATE_UPDATED[] PROGMEM = "UPDATED";

constexpr uint8_t FIRMWARE_FAILURE_RETRIES PROGMEM = 24U;
constexpr uint16_t FIRMWARE_PACKET_SIZE PROGMEM = 4096U;

constexpr uint32_t SERIAL_DEBUG_BAUD PROGMEM = 115200U;

/////////////////////////////////////
// RTD Configuration
/////////////////////////////////////

constexpr uint8_t SENSOR_COUNT = 3;
constexpr uint16_t RREF PROGMEM = 430;
constexpr uint16_t RNOMINAL PROGMEM = 100;

// SPI pin configuration per sensor
const uint16_t SPI_CS[SENSOR_COUNT]  = {18, 32, 27};
const uint16_t SPI_SDI[SENSOR_COUNT] = {17, 33, 14};
const uint16_t SPI_SDO[SENSOR_COUNT] = {16, 25, 12};
const uint16_t SPI_CLK[SENSOR_COUNT] = {4,  26, 13};

/////////////////////////////////////
// Timing
/////////////////////////////////////

constexpr unsigned long TELEMETRY_INTERVAL_MS = 900000; // 15 min
constexpr unsigned long CONNECTION_CHECK_MS   = 1000;   // 1 sec

/////////////////////////////////////
// Global Objects
/////////////////////////////////////

Preferences preferences;

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
Espressif_Updater updater;

Adafruit_MAX31865 rtdSensors[SENSOR_COUNT] = {
  Adafruit_MAX31865(SPI_CS[0], SPI_SDI[0], SPI_SDO[0], SPI_CLK[0]),
  Adafruit_MAX31865(SPI_CS[1], SPI_SDI[1], SPI_SDO[1], SPI_CLK[1]),
  Adafruit_MAX31865(SPI_CS[2], SPI_SDI[2], SPI_SDO[2], SPI_CLK[2])
};

/////////////////////////////////////
// Runtime State
/////////////////////////////////////

String wifi_ssid;
String wifi_password;
unsigned char wifi_ap_bssid[6];
int wifi_channel;
String mqtt_access_token;

bool mqtt_connected = false;
bool firmware_info_sent = false;
bool ota_subscription_active = false;

unsigned long lastTelemetryTime = 0;
unsigned long lastConnectionCheck = 0;
int ota_progress_counter = 0;

/////////////////////////////////////
// JSON Buffers
/////////////////////////////////////

StaticJsonDocument<256> connectMsg;
StaticJsonDocument<1024> telemetryMsg;
StaticJsonDocument<256> attributeMsg;

char outputTelemetry[1024];
char outputConnect[256];
char outputAttribute[256];

/////////////////////////////////////
// WiFi Functions
/////////////////////////////////////

void initWiFi() {
  Serial.println(F("Connecting to WiFi..."));
  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str(), wifi_channel, wifi_ap_bssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(F("\nWiFi connected"));
}

bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  initWiFi();
  return true;
}

/////////////////////////////////////
// OTA Callbacks
/////////////////////////////////////

void updatedCallback(const bool& success) {
  if (success) {
    Serial.println(F("Firmware updated successfully. Rebooting..."));
    esp_restart();
  } else {
    Serial.println(F("Firmware update failed"));
  }
}

void progressCallback(const size_t& current, const size_t& total) {
  if (++ota_progress_counter >= 10) {
    float progress = (float)current * 100.0f / total;
    tb.sendAttributeData("ota_progress", progress);
    ota_progress_counter = 0;
  }
}

/////////////////////////////////////
// RTD Sampling
/////////////////////////////////////

float readTemperature(uint8_t index) {
  uint16_t rtd = rtdSensors[index].readRTD();
  return rtdSensors[index].temperature(RNOMINAL, RREF);
}

/////////////////////////////////////
// Telemetry
/////////////////////////////////////

void sendTelemetry() {
  telemetryMsg.clear();
  JsonArray sensors = telemetryMsg.createNestedArray("sensors");

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    JsonObject sensor = sensors.createNestedObject();
    sensor["sensor_id"] = i + 1;
    sensor["temperature_celsius"] = readTemperature(i);
  }

  serializeJson(telemetryMsg, outputTelemetry);
  tb.Send_Json_String("v1/gateway/telemetry", outputTelemetry);
  Serial.println(outputTelemetry);
}

/////////////////////////////////////
// Setup
/////////////////////////////////////

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);

  preferences.begin("credentials", true);
  wifi_ssid = preferences.getString("ssid", "");
  wifi_password = preferences.getString("password", "");
  preferences.getBytes("mac", wifi_ap_bssid, sizeof(wifi_ap_bssid));
  wifi_channel = preferences.getInt("channel", 0);
  mqtt_access_token = preferences.getString("mqtt_token", "");

  initWiFi();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    rtdSensors[i].begin(MAX31865_3WIRE);
  }
}

/////////////////////////////////////
// Main Loop
/////////////////////////////////////

void loop() {
  unsigned long now = millis();

  if (now - lastConnectionCheck >= CONNECTION_CHECK_MS) {
    ensureWiFi();

    if (!tb.connected()) {
      mqtt_connected = false;

      if (!tb.connect(MQTT_BROKER_HOST, mqtt_access_token.c_str(), MQTT_BROKER_PORT)) {
        return;
      }

      firmware_info_sent =
        tb.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION) &&
        tb.Firmware_Send_State(FW_STATE_UPDATED);

      OTA_Update_Callback callback(
        &progressCallback,
        &updatedCallback,
        CURRENT_FIRMWARE_TITLE,
        CURRENT_FIRMWARE_VERSION,
        &updater,
        FIRMWARE_FAILURE_RETRIES,
        FIRMWARE_PACKET_SIZE
      );

      ota_subscription_active = tb.Subscribe_Firmware_Update(callback);
      mqtt_connected = true;
    }

    lastConnectionCheck = now;
  }

  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    if (mqtt_connected) sendTelemetry();
    lastTelemetryTime = now;
  }

  tb.loop();
}
