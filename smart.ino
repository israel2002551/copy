#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= NETWORK & MQTT CONFIG =================
// IMPORTANT: Enter your 2.4 GHz Wi-Fi credentials below (Case-Sensitive):
const char* WIFI_SSID     = "YOUR_WIFI_SSID";     // <-- Replace with your Wi-Fi name
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // <-- Replace with your Wi-Fi password

const char* MQTT_BROKER   = "broker.emqx.io";
const int   MQTT_PORT     = 1883;

// Change "DEVICE123" to a unique string to prevent cross-talk
#define DEVICE_ID "DEVICE123"
const char* TOPIC_STATUS       = "smartfan/" DEVICE_ID "/status";
const char* TOPIC_SET          = "smartfan/" DEVICE_ID "/set";
const char* TOPIC_AVAILABILITY = "smartfan/" DEVICE_ID "/availability";

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

// Compatibility for ESP32 Arduino Core 2.x and 3.x
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define USE_ESP32_CORE_V3 1
#elif defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= 0x030000)
  #define USE_ESP32_CORE_V3 1
#else
  #define USE_ESP32_CORE_V3 0
  #define PWM_CHANNEL 0
#endif

// ================= PERIPHERALS =================
LiquidCrystal_I2C* lcd = nullptr;
bool lcdAvailable = false;
DHT dht(DHTPIN, DHTTYPE);

// Custom degree symbol character for LCD
byte degreeChar[8] = {
  0b00110,
  0b01001,
  0b01001,
  0b00110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

// ================= SYSTEM STATES =================
enum FanMode { MODE_AUTO = 0, MODE_MANUAL = 1 };
FanMode currentMode = MODE_AUTO;

int manualSpeedTarget = 0; // 0 to 255
int currentSpeed = 0;
bool fanActive = false;

// Auto thermostatic parameters
float autoThresholdTemp = 27.0; // Cooling starts above this (°C)
const float AUTO_MAX_TEMP = 32.0; // Max speed at this temp (°C)
const float AUTO_HYSTERESIS = 0.5; // Turn off at (autoThresholdTemp - AUTO_HYSTERESIS)

// Telemetry cache
float currentTemp = 25.0;
float currentHumidity = 50.0;
int currentAirPercent = 0;
int dhtConsecutiveErrors = 0;
bool sensorFault = false;

// NVS flash wear protection (debounced writing)
bool pendingNvsSave = false;
unsigned long nvsSaveTimer = 0;
const unsigned long NVS_SAVE_DELAY = 3000; // write after 3s of stability

// Buzzer alert state
bool buzzerArmed = true;
bool buzzerPlaying = false;
unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 5000;
const float ALARM_TEMP_TRIGGER = 30.0;
const float ALARM_TEMP_REARM = 29.0;

// Non-blocking timers
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 2000;
unsigned long lastMqttReconnect = 0;
unsigned long lastWifiCheck = 0;

// ================= FORWARD DECLARATIONS =================
void setupWiFi();
void checkWiFiConnection(unsigned long currentMillis);
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleBuzzer(float temp, unsigned long currentMillis);
int  calculateAutoSpeed(float temp);
void applyMotorSpeed(int speed);
void writePwm(int duty);
void updateDisplay(float temp, float hum, int speed, int airPercent);
void publishStatus(float temp, float hum, int airPercent, int speed);
void handleSensorError();

// ================= SETUP =================
void setup() {
  // Disable ESP32 brownout detector to prevent reset on Wi-Fi RF power surge
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(300);

  Serial.println("\n===========================================");
  Serial.println("   AeroSync Pro ESP32 Smart Fan System     ");
  Serial.println("===========================================");

  // STEP 1: I2C Bus & LCD Auto-Detection (Safe with timeout to prevent boot hang)
  Serial.println("[BOOT 1/6] Scanning I2C bus on GPIO 21 (SDA) / 22 (SCL)...");
  Wire.begin(21, 22);
  Wire.setTimeOut(50); // Set 50ms timeout to prevent lockup if SDA/SCL are floating or unconnected

  uint8_t detectedLcdAddr = 0;
  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) {
    detectedLcdAddr = 0x27;
  } else {
    Wire.beginTransmission(0x3F);
    if (Wire.endTransmission() == 0) {
      detectedLcdAddr = 0x3F;
    }
  }

  if (detectedLcdAddr != 0) {
    Serial.printf("[BOOT 1/6] LCD detected at I2C address 0x%02X.\n", detectedLcdAddr);
    lcd = new LiquidCrystal_I2C(detectedLcdAddr, 16, 2);
    lcd->init();
    lcd->backlight();
    lcd->createChar(0, degreeChar);
    lcd->setCursor(0, 0);
    lcd->print("Smart Fan v2.0");
    lcd->setCursor(0, 1);
    lcd->print("Booting...");
    lcdAvailable = true;
  } else {
    Serial.println("[BOOT 1/6] No LCD found at 0x27 or 0x3F. Continuing in headless mode.");
    lcdAvailable = false;
  }

  // STEP 2: DHT & Analog Input
  Serial.println("[BOOT 2/6] Initializing DHT22 and Analog Input...");
  dht.begin();
  analogReadResolution(12);
  analogSetPinAttenuation(AIR_SENSOR_PIN, ADC_11db);

  // STEP 3: Motor Driver Pins & PWM
  Serial.println("[BOOT 3/6] Setting up Motor Driver Pins & PWM...");
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

#if USE_ESP32_CORE_V3
  ledcAttach(ENA_PIN, 5000, 8);
  ledcWrite(ENA_PIN, 0);
#else
  ledcSetup(PWM_CHANNEL, 5000, 8);
  ledcAttachPin(ENA_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);
#endif

  // STEP 4: Buzzer Pin
  Serial.println("[BOOT 4/6] Initializing Buzzer Pin...");
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // STEP 5: Initialize NVS Preferences and restore saved state
  Serial.println("[BOOT 5/6] Restoring saved states from NVS Flash...");
  if (preferences.begin(PREF_NAMESPACE, false)) {
    uint8_t savedMode = preferences.getUChar("mode", (uint8_t)MODE_AUTO);
    currentMode = (savedMode == (uint8_t)MODE_MANUAL) ? MODE_MANUAL : MODE_AUTO;
    manualSpeedTarget = preferences.getInt("speed", 0);
    autoThresholdTemp = preferences.getFloat("auto_th", 27.0);

    Serial.printf("[NVS] Restored -> Mode: %s | Manual Speed: %d | Auto Threshold: %.1fC\n",
                  (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL",
                  manualSpeedTarget,
                  autoThresholdTemp);
  } else {
    Serial.println("[NVS] Warning: Preferences init failed, using default parameters.");
  }

  // STEP 6: Wi-Fi & MQTT Configuration
  Serial.println("[BOOT 6/6] Configuring Wi-Fi & MQTT Client...");
  setupWiFi();
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(15);
  mqttClient.setSocketTimeout(3); // Non-blocking socket timeout

  if (lcdAvailable && lcd != nullptr) {
    lcd->clear();
  }

  Serial.println("[SYSTEM] System initialization complete. Entering main loop.\n");
}

// ================= LOOP =================
void loop() {
  unsigned long currentMillis = millis();

  // 1. Maintain Wi-Fi and MQTT connectivity (Non-blocking)
  checkWiFiConnection(currentMillis);

  // 2. Handle buzzer shutoff timing
  if (buzzerPlaying && (currentMillis - buzzerStartTime >= buzzerDuration)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerPlaying = false;
  }

  // 3. Debounced NVS flash saving to prevent flash wear
  if (pendingNvsSave && (currentMillis - nvsSaveTimer >= NVS_SAVE_DELAY)) {
    preferences.putInt("speed", manualSpeedTarget);
    preferences.putFloat("auto_th", autoThresholdTemp);
    pendingNvsSave = false;
    Serial.printf("[NVS] Debounced parameters committed to flash.\n");
  }

  // 4. Periodic sensor sampling, fan control, and MQTT publishing
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentMillis;

    float t = dht.readTemperature();
    float h = dht.readHumidity();
    int airRaw = analogRead(AIR_SENSOR_PIN);
    currentAirPercent = constrain(map(airRaw, 0, 4095, 0, 100), 0, 100);

    // Resilient DHT reading: filter transient glitches
    if (isnan(t) || isnan(h)) {
      dhtConsecutiveErrors++;
      Serial.printf("[WARN] DHT read glitch (%d/3)\n", dhtConsecutiveErrors);
      if (dhtConsecutiveErrors >= 3) {
        sensorFault = true;
        handleSensorError();
        publishStatus(currentTemp, currentHumidity, currentAirPercent, 0);
        return;
      }
      // If glitch is transient, keep previous valid values and continue loop
    } else {
      dhtConsecutiveErrors = 0;
      sensorFault = false;
      currentTemp = t;
      currentHumidity = h;
    }

    // Safety Buzzer check
    handleBuzzer(currentTemp, currentMillis);

    // Motor Arbitration
    if (currentMode == MODE_AUTO) {
      currentSpeed = calculateAutoSpeed(currentTemp);
    } else {
      currentSpeed = manualSpeedTarget;
    }
    applyMotorSpeed(currentSpeed);

    // Refresh UI & Telemetry
    updateDisplay(currentTemp, currentHumidity, currentSpeed, currentAirPercent);
    publishStatus(currentTemp, currentHumidity, currentAirPercent, currentSpeed);
  }
}

// ================= MOTOR CONTROL =================
void writePwm(int duty) {
#if USE_ESP32_CORE_V3
  ledcWrite(ENA_PIN, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
}

int calculateAutoSpeed(float temp) {
  float offThreshold = autoThresholdTemp - AUTO_HYSTERESIS;

  // Hysteresis deadband to prevent rapid cycling
  if (fanActive) {
    if (temp < offThreshold) fanActive = false;
  } else {
    if (temp >= autoThresholdTemp) fanActive = true;
  }

  if (!fanActive) return 0;

  // Linear scaling from PWM 80 to PWM 255 between autoThresholdTemp and AUTO_MAX_TEMP
  int speed = map((int)(temp * 10), (int)(autoThresholdTemp * 10), (int)(AUTO_MAX_TEMP * 10), 80, 255);
  return constrain(speed, 80, 255);
}

void applyMotorSpeed(int speed) {
  speed = constrain(speed, 0, 255);

  if (speed <= 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    writePwm(0);
    fanActive = false;
  } else {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);

    // Kickstart pulse to overcome DC motor static friction if starting from zero
    if (currentSpeed == 0 && speed > 0 && speed < 110) {
      writePwm(180);
      delay(40); // very brief kickstart pulse
    }

    writePwm(speed);
  }
  currentSpeed = speed;
}

// ================= BUZZER =================
void handleBuzzer(float temp, unsigned long currentMillis) {
  // Sound alarm once upon hitting critical threshold
  if (temp >= ALARM_TEMP_TRIGGER && buzzerArmed && !buzzerPlaying) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerStartTime = currentMillis;
    buzzerDuration = 5000;
    buzzerPlaying = true;
    buzzerArmed = false;
    Serial.println("[ALARM] Over-temperature triggered!");
  }

  // Re-arm when temperature safely cools down
  if (temp < ALARM_TEMP_REARM) {
    buzzerArmed = true;
  }
}

// ================= DISPLAY =================
void updateDisplay(float temp, float hum, int speed, int airPercent) {
  if (!lcdAvailable || lcd == nullptr) return;

  char modeChar = (currentMode == MODE_AUTO) ? 'A' : 'M';
  char netChar  = mqttClient.connected() ? '*' : (WiFi.status() == WL_CONNECTED ? 'w' : 'x');

  // Line 0: "A 28.5[deg]C H:55% *" (16 chars)
  char line0[17];
  snprintf(line0, sizeof(line0), "%c %4.1f%cC H:%2.0f%% %c",
           modeChar, temp, (char)0, hum, netChar);
  lcd->setCursor(0, 0);
  lcd->print(line0);

  // Line 1: Speed PWM and Air Pollution
  char line1[17];
  if (buzzerPlaying) {
    snprintf(line1, sizeof(line1), "** TEMP ALARM! **");
  } else {
    snprintf(line1, sizeof(line1), "PWM:%-3d Air:%2d%%  ", speed, airPercent);
  }
  lcd->setCursor(0, 1);
  lcd->print(line1);
}

void handleSensorError() {
  applyMotorSpeed(0);
  if (lcdAvailable && lcd != nullptr) {
    lcd->setCursor(0, 0);
    lcd->print("! SENSOR ERROR !");
    lcd->setCursor(0, 1);
    lcd->print("Fan Halted Safely");
  }
  Serial.println("[ERR] Persistent DHT22 failure! Motor halted for safety.");
}

// ================= NETWORKING & MQTT =================
void setupWiFi() {
  delay(100);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  Serial.flush();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Disable Wi-Fi modem sleep to prevent voltage oscillation
  WiFi.setTxPower(WIFI_POWER_11dBm); // Cap transmit power to prevent 400mA current spikes

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Will retry in background.");
  }
}

void checkWiFiConnection(unsigned long currentMillis) {
  // Maintain Wi-Fi and MQTT connectivity
  if (WiFi.status() != WL_CONNECTED) {
    if (currentMillis - lastWifiCheck > 5000) {
      lastWifiCheck = currentMillis;
      Serial.println("Reconnecting to WiFi...");
      WiFi.reconnect();
    }
    return;
  }

  // Handle MQTT reconnect if WiFi is healthy
  if (!mqttClient.connected()) {
    if (currentMillis - lastMqttReconnect > 5000) {
      lastMqttReconnect = currentMillis;
      reconnectMQTT();
    }
  } else {
    mqttClient.loop();
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

  Serial.print("[MQTT] Connecting to broker...");
  if (mqttClient.connect(clientId.c_str(), willTopic, willQoS, willRetain, willMessage)) {
    Serial.println(" Connected successfully.");

    // Announce online status with retain flag set
    mqttClient.publish(TOPIC_AVAILABILITY, "online", true);

    // Subscribe to incoming commands
    mqttClient.subscribe(TOPIC_SET);

    // Immediate status broadcast on connect
    publishStatus(currentTemp, currentHumidity, currentAirPercent, currentSpeed);
  } else {
    Serial.printf(" Failed, state=%d. Retrying in 5s.\n", mqttClient.state());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Compatible with ArduinoJson v6 and v7
#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument doc;
#else
  StaticJsonDocument<384> doc;
#endif

  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.printf("[JSON] Deserialization error: %s\n", error.c_str());
    return;
  }

  bool stateChanged = false;

  // 1. Mode command
  if (doc["mode"].is<const char*>()) {
    const char* modeStr = doc["mode"];
    FanMode newMode = currentMode;

    if (strcasecmp(modeStr, "AUTO") == 0) {
      newMode = MODE_AUTO;
    } else if (strcasecmp(modeStr, "MANUAL") == 0) {
      newMode = MODE_MANUAL;
    }

    if (newMode != currentMode) {
      currentMode = newMode;
      preferences.putUChar("mode", (uint8_t)currentMode);
      stateChanged = true;
      Serial.printf("[CMD] Mode changed to: %s\n", (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL");
    }
  }

  // 2. Manual speed command
  if (doc["speed"].is<int>()) {
    int newSpeed = constrain(doc["speed"].as<int>(), 0, 255);
    if (newSpeed != manualSpeedTarget) {
      manualSpeedTarget = newSpeed;
      pendingNvsSave = true;
      nvsSaveTimer = millis();
      stateChanged = true;
      Serial.printf("[CMD] Target speed updated: %d\n", manualSpeedTarget);
    }
  }

  // 3. Auto target temperature threshold command
  if (doc["auto_thresh"].is<float>()) {
    float newThresh = constrain(doc["auto_thresh"].as<float>(), 20.0, 38.0);
    if (abs(newThresh - autoThresholdTemp) > 0.1) {
      autoThresholdTemp = newThresh;
      pendingNvsSave = true;
      nvsSaveTimer = millis();
      stateChanged = true;
      Serial.printf("[CMD] Auto Threshold updated: %.1fC\n", autoThresholdTemp);
    }
  }

  // 4. Buzzer test / mute command
  if (doc["buzzer"].is<const char*>()) {
    const char* bcmd = doc["buzzer"];
    if (strcasecmp(bcmd, "test") == 0) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerStartTime = millis();
      buzzerDuration = 1000; // 1s test beep
      buzzerPlaying = true;
      stateChanged = true;
    } else if (strcasecmp(bcmd, "mute") == 0) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerPlaying = false;
      buzzerArmed = false; // Stay muted until re-armed
      stateChanged = true;
    }
  }

  // 5. Immediate response if state changed (zero UI lag)
  if (stateChanged) {
    if (currentMode == MODE_AUTO) {
      currentSpeed = calculateAutoSpeed(currentTemp);
    } else {
      currentSpeed = manualSpeedTarget;
    }
    applyMotorSpeed(currentSpeed);
    updateDisplay(currentTemp, currentHumidity, currentSpeed, currentAirPercent);
    publishStatus(currentTemp, currentHumidity, currentAirPercent, currentSpeed);
  }
}

void publishStatus(float temp, float hum, int airPercent, int speed) {
  if (!mqttClient.connected()) return;

#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument doc;
#else
  StaticJsonDocument<384> doc;
#endif

  // Clean numeric JSON values
  doc["temp"]         = (float)((int)(temp * 10.0 + 0.5)) / 10.0;
  doc["humidity"]     = (float)((int)(hum * 10.0 + 0.5)) / 10.0;
  doc["air"]          = airPercent;
  doc["mode"]         = (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL";
  doc["speed"]        = speed;
  doc["speed_pct"]    = (int)map(speed, 0, 255, 0, 100);
  doc["alarm"]        = buzzerPlaying;
  doc["buzzer_armed"] = buzzerArmed;
  doc["target_temp"]  = (float)((int)(autoThresholdTemp * 10.0 + 0.5)) / 10.0;
  doc["sensor_ok"]    = !sensorFault;
  doc["wifi_rssi"]    = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  doc["uptime"]       = millis() / 1000;

  char buffer[384];
  size_t len = serializeJson(doc, buffer);
  mqttClient.publish(TOPIC_STATUS, buffer, len);
}
