#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

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

// ================= CLIMATE & FAN CONFIGURATION =================
const float SET_TEMPERATURE   = 18.0; // Target cooling threshold: turns ON at >= 18.0°C
const float TEMP_HYSTERESIS   = 0.5;  // Turns OFF below (18.0 - 0.5) = 17.5°C
const float MAX_FAN_TEMP      = 26.0; // Reaches 100% full throttle at 26.0°C
const float ALARM_TEMP        = 30.0; // High-temperature safety alarm threshold

// System State
int currentSpeed = 0;
bool fanActive = false;

float currentTemp = 18.0;
float currentHumidity = 50.0;
int currentAirPercent = 0;

int dhtConsecutiveErrors = 0;
bool sensorFault = false;

// Buzzer Alert State
bool buzzerArmed = true;
bool buzzerPlaying = false;
unsigned long buzzerStartTime = 0;
const unsigned long BUZZER_DURATION = 3000; // 3-second alert beep

// Non-blocking sampling timer
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 2000; // Sample every 2 seconds

// ================= FORWARD DECLARATIONS =================
void handleBuzzer(float temp, unsigned long currentMillis);
int  calculateSpeed(float temp);
void applyMotorSpeed(int speed);
void writePwm(int duty);
void updateDisplay(float temp, float hum, int speed, int airPercent);
void handleSensorError();

// ================= SETUP =================
void setup() {
  // Disable brownout detector to prevent voltage drop resets
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(200);

  Serial.println("\n===========================================");
  Serial.println("     Smart Fan Standalone Controller       ");
  Serial.println("===========================================");
  Serial.printf("[CONFIG] Set Temperature: %.1f°C | Max Throttle at: %.1f°C\n", SET_TEMPERATURE, MAX_FAN_TEMP);

  // STEP 1: I2C Bus & LCD Auto-Detection (Non-blocking with timeout)
  Serial.println("[BOOT 1/4] Scanning I2C bus (GPIO 21 SDA / GPIO 22 SCL)...");
  Wire.begin(21, 22);
  Wire.setTimeOut(50); // Prevent hang if LCD is disconnected or lines are floating

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
    Serial.printf("[BOOT 1/4] LCD detected at address 0x%02X. Initializing...\n", detectedLcdAddr);
    lcd = new LiquidCrystal_I2C(detectedLcdAddr, 16, 2);
    lcd->init();
    lcd->backlight();
    lcd->createChar(0, degreeChar);
    lcd->setCursor(0, 0);
    lcd->print("Smart Fan Ready");
    char line1[17];
    snprintf(line1, sizeof(line1), "Target: %2.0f%cC", SET_TEMPERATURE, (char)0);
    lcd->setCursor(0, 1);
    lcd->print(line1);
    lcdAvailable = true;
    delay(1000);
    lcd->clear();
  } else {
    Serial.println("[BOOT 1/4] No I2C LCD detected. Running in headless serial mode.");
    lcdAvailable = false;
  }

  // STEP 2: DHT22 & Analog Gas/Air Sensor
  Serial.println("[BOOT 2/4] Initializing DHT22 and Analog Input...");
  dht.begin();
  analogReadResolution(12);
  analogSetPinAttenuation(AIR_SENSOR_PIN, ADC_11db);

  // STEP 3: Motor Driver Pins & PWM Configuration
  Serial.println("[BOOT 3/4] Configuring Motor Driver Pins & PWM...");
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
  Serial.println("[BOOT 4/4] Initializing Buzzer Pin...");
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("[SYSTEM] System initialization complete. Entering automatic control loop.\n");
}

// ================= MAIN LOOP =================
void loop() {
  unsigned long currentMillis = millis();

  // 1. Handle buzzer alarm shutoff timing
  if (buzzerPlaying && (currentMillis - buzzerStartTime >= BUZZER_DURATION)) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerPlaying = false;
  }

  // 2. Periodic sensor sampling and thermostatic fan control
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentMillis;

    float t = dht.readTemperature();
    float h = dht.readHumidity();
    int airRaw = analogRead(AIR_SENSOR_PIN);
    currentAirPercent = constrain(map(airRaw, 0, 4095, 0, 100), 0, 100);

    // Filter transient DHT22 read glitches (intermittent CRC errors)
    if (isnan(t) || isnan(h)) {
      dhtConsecutiveErrors++;
      Serial.printf("[WARN] DHT read glitch (%d/3)\n", dhtConsecutiveErrors);
      if (dhtConsecutiveErrors >= 3) {
        sensorFault = true;
        handleSensorError();
        return;
      }
      // If glitch is transient, keep previous valid values and continue
    } else {
      dhtConsecutiveErrors = 0;
      sensorFault = false;
      currentTemp = t;
      currentHumidity = h;
    }

    // Safety Buzzer check (Trigger alarm if temperature exceeds 30.0°C)
    handleBuzzer(currentTemp, currentMillis);

    // Thermostatic Fan Speed Calculation based on 18.0°C Setpoint
    currentSpeed = calculateSpeed(currentTemp);
    applyMotorSpeed(currentSpeed);

    // Refresh Display and Serial Telemetry
    updateDisplay(currentTemp, currentHumidity, currentSpeed, currentAirPercent);

    int pct = map(currentSpeed, 0, 255, 0, 100);
    Serial.printf("[STATUS] Temp: %4.1f°C | Hum: %2.0f%% | Air: %2d%% | Fan: %3d PWM (%3d%%) | State: %s\n",
                  currentTemp, currentHumidity, currentAirPercent, currentSpeed, pct,
                  (currentSpeed > 0) ? "COOLING" : "IDLE (Below 18°C)");
  }
}

// ================= THERMOSTATIC MOTOR CONTROL =================
int calculateSpeed(float temp) {
  float offThreshold = SET_TEMPERATURE - TEMP_HYSTERESIS; // 18.0 - 0.5 = 17.5°C

  // Hysteresis deadband to prevent rapid on/off cycling around 18.0°C
  if (fanActive) {
    if (temp < offThreshold) fanActive = false;
  } else {
    if (temp >= SET_TEMPERATURE) fanActive = true;
  }

  if (!fanActive) return 0;

  // Scale speed linearly:
  // At 18.0°C -> PWM 80 (minimum gentle breeze)
  // At 26.0°C+ -> PWM 255 (maximum throttle)
  int speed = map((int)(temp * 10), (int)(SET_TEMPERATURE * 10), (int)(MAX_FAN_TEMP * 10), 80, 255);
  return constrain(speed, 80, 255);
}

void writePwm(int duty) {
#if USE_ESP32_CORE_V3
  ledcWrite(ENA_PIN, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
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

    // Kickstart pulse to overcome static DC motor friction if spinning up from rest
    if (currentSpeed == 0 && speed > 0 && speed < 110) {
      writePwm(180);
      delay(40); // 40ms kickstart pulse
    }

    writePwm(speed);
  }
  currentSpeed = speed;
}

// ================= BUZZER =================
void handleBuzzer(float temp, unsigned long currentMillis) {
  if (temp >= ALARM_TEMP && buzzerArmed && !buzzerPlaying) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerStartTime = currentMillis;
    buzzerPlaying = true;
    buzzerArmed = false;
    Serial.println("[ALARM] Over-temperature warning triggered (>= 30°C)!");
  }

  // Re-arm when temperature cools down below 28.5°C
  if (temp < (ALARM_TEMP - 1.5)) {
    buzzerArmed = true;
  }
}

// ================= LCD DISPLAY =================
void updateDisplay(float temp, float hum, int speed, int airPercent) {
  if (!lcdAvailable || lcd == nullptr) return;

  // Line 0: "T: 18.5[deg]C H:52%"
  char line0[17];
  snprintf(line0, sizeof(line0), "T:%4.1f%cC H:%2.0f%% ", temp, (char)0, hum);
  lcd->setCursor(0, 0);
  lcd->print(line0);

  // Line 1: Fan Speed and Air Quality
  char line1[17];
  if (buzzerPlaying) {
    snprintf(line1, sizeof(line1), "** TEMP ALARM! **");
  } else if (speed == 0) {
    snprintf(line1, sizeof(line1), "Fan:OFF Air:%2d%% ", airPercent);
  } else {
    int pct = map(speed, 0, 255, 0, 100);
    snprintf(line1, sizeof(line1), "Fan:%3d%% Air:%2d%% ", pct, airPercent);
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
