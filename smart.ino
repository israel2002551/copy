#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ================= NETWORK & MQTT CONFIG =================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MQTT_BROKER   = "broker.emqx.io";
const int   MQTT_PORT     = 1883;

// Change "DEVICE123" to a unique string to prevent cross-talk
const char* TOPIC_STATUS       = "smartfan/DEVICE123/status";
const char* TOPIC_SET          = "smartfan/DEVICE123/set";
const char* TOPIC_AVAILABILITY = "smartfan/DEVICE123/availability";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences preferences;

const char* PREF_NAMESPACE = "smartfan";

// ================= PIN DEFINITIONS =================
#define DHTPIN 32
#define DHTTYPE DHT22

#define AIR_SENSOR_PIN 34
#define BUZZER_PIN 33

#define ENA_PIN 25
#define IN1_PIN 26
#define IN2_PIN 27

// ================= PERIPHERALS =================
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

// ================= SYSTEM STATES =================
enum FanMode { MODE_AUTO = 0, MODE_MANUAL = 1 };
FanMode currentMode = MODE_AUTO;

int manualSpeedTarget = 0; // 0 to 255
int currentSpeed = 0;
bool fanActive = false;

// Buzzer alert state
bool buzzerArmed = true;
bool buzzerPlaying = false;
unsigned long buzzerStartTime = 0;
const unsigned long BUZZER_DURATION = 5000;

// Non-blocking timers
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 2000;
unsigned long lastMqttReconnect = 0;

// ================= FORWARD DECLARATIONS =================
void setupWiFi();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleBuzzer(float temp, unsigned long currentMillis);
int  calculateAutoSpeed(float temp);
void applyMotorSpeed(int speed);
void updateDisplay(float temp, int speed, int airPercent);
void publishStatus(float temp, int airPercent, int speed);
void handleSensorError();

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // LCD Initialization
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Fan Init");

  // DHT & Analog Input
  dht.begin();
  analogReadResolution(12);
  analogSetPinAttenuation(AIR_SENSOR_PIN, ADC_11db);

  // Motor Driver Pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  // ESP32 Core v3.x PWM
  ledcAttach(ENA_PIN, 5000, 8);
  ledcWrite(ENA_PIN, 0);

  // Buzzer Pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize NVS Preferences and restore saved state
  preferences.begin(PREF_NAMESPACE, false);
  uint8_t savedMode = preferences.getUChar("mode", (uint8_t)MODE_AUTO);
  currentMode = (savedMode == (uint8_t)MODE_MANUAL) ? MODE_MANUAL : MODE_AUTO;
  manualSpeedTarget = preferences.getInt("speed", 0);

  Serial.printf("[NVS] Loaded -> Mode: %s | Manual Speed: %d\n",
                (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL",
                manualSpeedTarget);

  // Wi-Fi & MQTT Configuration
  setupWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Method 1: Set MQTT Keep-Alive to 4s (broker triggers LWT at 4 * 1.5 = 6s)
  mqttClient.setKeepAlive(4);

  lcd.clear();
}

// ================= LOOP =================
void loop() {
  unsigned long currentMillis = millis();

  // Maintain Wi-Fi and MQTT connectivity
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      if (currentMillis - lastMqttReconnect > 5000) {
        lastMqttReconnect = currentMillis;
        reconnectMQTT();
      }
    } else {
      mqttClient.loop();
    }
  }

  // Handle buzzer shutoff timing
  if (buzzerPlaying && (currentMillis - buzzerStartTime >= BUZZER_DURATION)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerPlaying = false;
  }

  // Periodic sensor read, fan control, and MQTT publishing
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentMillis;

    float temperature = dht.readTemperature();
    int airRaw = analogRead(AIR_SENSOR_PIN);
    int airPercent = constrain(map(airRaw, 0, 4095, 0, 100), 0, 100);

    if (isnan(temperature)) {
      handleSensorError();
      return;
    }

    // Safety Buzzer check
    handleBuzzer(temperature, currentMillis);

    // Motor Arbitration
    if (currentMode == MODE_AUTO) {
      currentSpeed = calculateAutoSpeed(temperature);
    } else {
      currentSpeed = manualSpeedTarget;
    }
    applyMotorSpeed(currentSpeed);

    // Refresh UI & Telemetry
    updateDisplay(temperature, currentSpeed, airPercent);
    publishStatus(temperature, airPercent, currentSpeed);
  }
}

// ================= MOTOR CONTROL =================
int calculateAutoSpeed(float temp) {
  // Hysteresis deadband: turns on at 27.0°C, shuts off below 26.5°C
  if (fanActive) {
    if (temp < 26.5) fanActive = false;
  } else {
    if (temp >= 27.0) fanActive = true;
  }

  if (!fanActive) return 0;

  // Scale speed from PWM 80 (27.0°C) to PWM 255 (32.0°C)
  int speed = map((int)(temp * 10), 270, 320, 80, 255);
  return constrain(speed, 80, 255);
}

void applyMotorSpeed(int speed) {
  if (speed <= 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, 0);
  } else {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    ledcWrite(ENA_PIN, speed);
  }
}

// ================= BUZZER =================
void handleBuzzer(float temp, unsigned long currentMillis) {
  // Sound alarm once upon hitting 30.0°C
  if (temp >= 30.0 && buzzerArmed && !buzzerPlaying) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerStartTime = currentMillis;
    buzzerPlaying = true;
    buzzerArmed = false;
  }

  // Re-arm when temperature drops below 29.0°C
  if (temp < 29.0) {
    buzzerArmed = true;
  }
}

// ================= DISPLAY =================
void updateDisplay(float temp, int speed, int airPercent) {
  char line0[17];
  char modeChar = (currentMode == MODE_AUTO) ? 'A' : 'M';
  snprintf(line0, sizeof(line0), "%c T:%4.1f%cC F:%-3d", modeChar, temp, (char)223, speed);
  lcd.setCursor(0, 0);
  lcd.print(line0);

  char line1[17];
  snprintf(line1, sizeof(line1), "Air:%3d%%        ", airPercent);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

void handleSensorError() {
  applyMotorSpeed(0);
  lcd.setCursor(0, 0);
  lcd.print("Sensor Read Err ");
  lcd.setCursor(0, 1);
  lcd.print("Fan Halted      ");
  Serial.println("[ERR] DHT22 failure! Motor halted.");
}

// ================= NETWORKING & MQTT =================
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWi-Fi connected. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWi-Fi connection failed. Will retry in background.");
  }
}

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  String clientId = "ESP32Fan-" + String(random(0xffff), HEX);

  // LWT configuration:
  // Publishes retained "offline" to TOPIC_AVAILABILITY if connection drops unexpectedly
  const char* willTopic   = TOPIC_AVAILABILITY;
  uint8_t     willQoS     = 1;
  boolean     willRetain  = true;
  const char* willMessage = "offline";

  Serial.print("Attempting MQTT connection...");
  if (mqttClient.connect(clientId.c_str(), willTopic, willQoS, willRetain, willMessage)) {
    Serial.println(" connected.");

    // Announce online status with retain flag set
    mqttClient.publish(TOPIC_AVAILABILITY, "online", true);

    // Subscribe to incoming commands
    mqttClient.subscribe(TOPIC_SET);
  } else {
    Serial.printf(" failed, rc=%d. Retrying in 5 seconds.\n", mqttClient.state());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.printf("[JSON] Deserialization error: %s\n", error.c_str());
    return;
  }

  // Parse mode switch command
  if (doc.containsKey("mode")) {
    const char* modeStr = doc["mode"];
    FanMode newMode = currentMode;

    if (strcasecmp(modeStr, "AUTO") == 0) {
      newMode = MODE_AUTO;
    } else if (strcasecmp(modeStr, "MANUAL") == 0) {
      newMode = MODE_MANUAL;
    }

    // Write to flash only if changed
    if (newMode != currentMode) {
      currentMode = newMode;
      preferences.putUChar("mode", (uint8_t)currentMode);
      Serial.printf("[NVS] Mode changed and saved: %s\n", modeStr);
    }
  }

  // Parse manual speed command
  if (doc.containsKey("speed")) {
    int newSpeed = constrain(doc["speed"].as<int>(), 0, 255);

    // Write to flash only if changed
    if (newSpeed != manualSpeedTarget) {
      manualSpeedTarget = newSpeed;
      preferences.putInt("speed", manualSpeedTarget);
      Serial.printf("[NVS] Speed changed and saved: %d\n", manualSpeedTarget);
    }
  }
}

void publishStatus(float temp, int airPercent, int speed) {
  if (!mqttClient.connected()) return;

  StaticJsonDocument<256> doc;
  doc["temp"] = serialized(String(temp, 1));
  doc["air"] = airPercent;
  doc["mode"] = (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL";
  doc["speed"] = speed;
  doc["alarm"] = buzzerPlaying;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  mqttClient.publish(TOPIC_STATUS, buffer, len);
}
