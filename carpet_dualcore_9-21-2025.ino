/*
  ESP32 Dual-Core FMS Mat (5x VL53L0X)
  ------------------------------------
  Core 1 (APP):  Initializes I2C + sensors; reads every 250 ms; drives LEDs/buzzer (S1..S4) + depth tone for S5; enqueues samples
  Core 0 (NET):  Initializes Wi-Fi; dequeues and POSTs one-by-one via HTTPClient

  Pins (as requested):
    XSHUT: S1=25, S2=32, S3=27, S4=33, S5=26
    BUZZER: 19
    LEDs:   S1=4, S2=16, S3=5, S4=17
    I2C:    SDA=21, SCL=22
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ---------- Network config ----------
static const char* WIFI_SSID     = "QSTP VC";
static const char* WIFI_PASSWORD = "qstp1234";
static const char* POST_URL      = "http://54.86.18.41/api/sensors"; // switch to https://... if needed

// ---------- Timing ----------
static const uint32_t SAMPLE_PERIOD_MS = 250;

// ---------- I2C ----------
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// ---------- XSHUT pins (as provided) ----------
#define SENSOR1_XSHUT 25
#define SENSOR2_XSHUT 32
#define SENSOR3_XSHUT 27
#define SENSOR4_XSHUT 33
#define SENSOR5_XSHUT 26

// ---- Thresholds (in mm) ----
#define THRESHOLD_SENSOR14 400   // for sensors 1 and 4
#define THRESHOLD_SENSOR23 400   // for sensors 2 and 3
#define THRESHOLD_SENSOR5  450   // NEW: depth threshold for sensor 5 (tune as needed)

// ---------- LEDs + Buzzer ----------
#define BUZZER_PIN    19
int LED_1 = 4;
int LED_2 = 16;
int LED_3 = 5;
int LED_4 = 17;

// ---------- Buzzer tones ----------
#define TONE_SENSOR14 1200
#define TONE_SENSOR23 800
#define TONE_SENSOR5  1500   // NEW: distinct tone for depth threshold on S5
#define BEEP_DURATION 200

// ---------- Sensors ----------
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;
Adafruit_VL53L0X sensor3;
Adafruit_VL53L0X sensor4;
Adafruit_VL53L0X sensor5;

bool sensor1_ok = false;
bool sensor2_ok = false;
bool sensor3_ok = false;
bool sensor4_ok = false;
bool sensor5_ok = false;

VL53L0X_RangingMeasurementData_t measure1, measure2, measure3, measure4, measure5;

// ---------- FreeRTOS ----------
struct Sample {
  float s1, s2, s3, s4, s5;
};

#define SAMPLE_QUEUE_LEN 40
QueueHandle_t gQueue;
TaskHandle_t  hTaskSensor;
TaskHandle_t  hTaskNet;

// Init synchronization
volatile bool gHwReady = false;
volatile bool gNetReady = false;
volatile bool gBeeped  = false;

// ---------- Forward decls ----------
bool initializeSensor(Adafruit_VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddress, const char *name);
void printMeasurement(VL53L0X_RangingMeasurementData_t &m);
void taskSensor(void* pv);
void taskNet(void* pv);
void threeShortBeeps();

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(150);

  // GPIOs
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
  digitalWrite(LED_4, LOW);

  // Queue
  gQueue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(Sample));
  if (!gQueue) {
    Serial.println("Queue alloc failed; halting.");
    while (true) delay(1000);
  }

  // Create tasks (sensor on Core 1, net on Core 0)
  xTaskCreatePinnedToCore(taskSensor, "sensor", 8192, nullptr, 2, &hTaskSensor, 1);
  xTaskCreatePinnedToCore(taskNet,    "net",    8192, nullptr, 2, &hTaskNet,    0);
}

void loop() {
  // not used; work is in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ---------- Sensor task (Core 1) ----------
void taskSensor(void* pv) {
  Serial.println("[HW] Init starting...");

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(20);

  // XSHUT as outputs
  pinMode(SENSOR1_XSHUT, OUTPUT);
  pinMode(SENSOR2_XSHUT, OUTPUT);
  pinMode(SENSOR3_XSHUT, OUTPUT);
  pinMode(SENSOR4_XSHUT, OUTPUT);
  pinMode(SENSOR5_XSHUT, OUTPUT);

  // Reset all low
  digitalWrite(SENSOR1_XSHUT, LOW);
  digitalWrite(SENSOR2_XSHUT, LOW);
  digitalWrite(SENSOR3_XSHUT, LOW);
  digitalWrite(SENSOR4_XSHUT, LOW);
  digitalWrite(SENSOR5_XSHUT, LOW);
  delay(10);

  // Initialize sensors and assign unique addresses
  sensor1_ok = initializeSensor(sensor1, SENSOR1_XSHUT, 0x30, "S1");
  sensor2_ok = initializeSensor(sensor2, SENSOR2_XSHUT, 0x31, "S2");
  sensor3_ok = initializeSensor(sensor3, SENSOR3_XSHUT, 0x32, "S3");
  sensor4_ok = initializeSensor(sensor4, SENSOR4_XSHUT, 0x33, "S4");
  sensor5_ok = initializeSensor(sensor5, SENSOR5_XSHUT, 0x34, "S5 (depth)");

  Serial.println("[HW] Initialization complete.");
  Serial.println("S1\tS2\tS3\tS4\tS5");

  gHwReady = true;

  uint32_t lastRead = millis();

  for (;;) {
    // Play 3 short beeps after BOTH HW and NET are ready (do once)
    if (gHwReady && gNetReady && !gBeeped) {
      threeShortBeeps();
      gBeeped = true;
    }

    // Read every 250 ms
    uint32_t now = millis();
    if ((int32_t)(now - lastRead) >= 0) {
      lastRead += SAMPLE_PERIOD_MS;

      // Read all sensors (invalid/out-of-range → 0)
      if (sensor1_ok) sensor1.rangingTest(&measure1, false); else measure1.RangeStatus = 4;
      if (sensor2_ok) sensor2.rangingTest(&measure2, false); else measure2.RangeStatus = 4;
      if (sensor3_ok) sensor3.rangingTest(&measure3, false); else measure3.RangeStatus = 4;
      if (sensor4_ok) sensor4.rangingTest(&measure4, false); else measure4.RangeStatus = 4;
      if (sensor5_ok) sensor5.rangingTest(&measure5, false); else measure5.RangeStatus = 4;

      float d1 = (measure1.RangeStatus == 4 || measure1.RangeMilliMeter > 8000) ? 0.0f : (float)measure1.RangeMilliMeter;
      float d2 = (measure2.RangeStatus == 4 || measure2.RangeMilliMeter > 8000) ? 0.0f : (float)measure2.RangeMilliMeter;
      float d3 = (measure3.RangeStatus == 4 || measure3.RangeMilliMeter > 8000) ? 0.0f : (float)measure3.RangeMilliMeter;
      float d4 = (measure4.RangeStatus == 4 || measure4.RangeMilliMeter > 8000) ? 0.0f : (float)measure4.RangeMilliMeter;
      float d5 = (measure5.RangeStatus == 4 || measure5.RangeMilliMeter > 8000) ? 0.0f : (float)measure5.RangeMilliMeter;

      // Original alert logic (S1..S4 kept)
      if ((sensor1_ok && measure1.RangeMilliMeter < THRESHOLD_SENSOR14) ||
          (sensor4_ok && measure4.RangeMilliMeter < THRESHOLD_SENSOR14)) {
        tone(BUZZER_PIN, TONE_SENSOR14, BEEP_DURATION);
        delay(BEEP_DURATION);
        if (sensor1_ok && measure1.RangeMilliMeter < THRESHOLD_SENSOR14) digitalWrite(LED_1, HIGH);
        else if (sensor4_ok && measure4.RangeMilliMeter < THRESHOLD_SENSOR14) digitalWrite(LED_4, HIGH);
      }
      else if ((sensor2_ok && measure2.RangeMilliMeter < THRESHOLD_SENSOR23) ||
               (sensor3_ok && measure3.RangeMilliMeter < THRESHOLD_SENSOR23)) {
        tone(BUZZER_PIN, TONE_SENSOR23, BEEP_DURATION);
        delay(BEEP_DURATION);
        if (sensor2_ok && measure2.RangeMilliMeter < THRESHOLD_SENSOR23) digitalWrite(LED_2, HIGH);
        else if (sensor3_ok && measure3.RangeMilliMeter < THRESHOLD_SENSOR23) digitalWrite(LED_3, HIGH);
      }
      // NEW: depth threshold alert (no LED, distinct tone)
      else if (sensor5_ok && measure5.RangeMilliMeter < THRESHOLD_SENSOR5) {
        tone(BUZZER_PIN, TONE_SENSOR5, BEEP_DURATION);
        delay(BEEP_DURATION);
        // no LEDs for S5
      }
      else {
        digitalWrite(LED_1, LOW);
        digitalWrite(LED_2, LOW);
        digitalWrite(LED_3, LOW);
        digitalWrite(LED_4, LOW);
      }

      // Print readings
      printMeasurement(measure1); Serial.print("\t");
      printMeasurement(measure2); Serial.print("\t");
      printMeasurement(measure3); Serial.print("\t");
      printMeasurement(measure4); Serial.print("\t");
      if (measure5.RangeStatus != 4 && measure5.RangeMilliMeter <= 8000) {
        Serial.print(measure5.RangeMilliMeter); Serial.print("mm");
      } else {
        Serial.print("---");
      }
      Serial.println();

      // Enqueue sample (drop oldest if full so we never block)
      Sample s{d1, d2, d3, d4, d5};
      if (xQueueSendToBack(gQueue, &s, 0) != pdTRUE) {
        Sample dump;
        xQueueReceive(gQueue, &dump, 0);
        xQueueSendToBack(gQueue, &s, 0);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ---------- Network task (Core 0) ----------
void taskNet(void* pv) {
  Serial.println("[NET] Wi-Fi init starting...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(250));
    if (millis() - t0 > 20000) {
      Serial.println("[NET] Wi-Fi timeout; retrying...");
      WiFi.disconnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      t0 = millis();
    }
  }

  Serial.print("[NET] Wi-Fi connected. IP: ");
  Serial.println(WiFi.localIP());
  gNetReady = true;

  HTTPClient http;

  for (;;) {
    // Beep after both ready (sensor task also checks; either can play if first)
    if (gHwReady && gNetReady && !gBeeped) {
      threeShortBeeps();
      gBeeped = true;
    }

    // If no samples, idle briefly
    if (uxQueueMessagesWaiting(gQueue) == 0) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Pop one sample
    Sample s;
    if (xQueueReceive(gQueue, &s, 0) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Ensure Wi-Fi is still up; if not, requeue the sample and retry later
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[NET] Wi-Fi lost; reconnecting...");
      WiFi.disconnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      uint32_t start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
      // Put back for later if still down
      if (WiFi.status() != WL_CONNECTED) {
        if (xQueueSendToBack(gQueue, &s, 0) != pdTRUE) {
          Sample dump; xQueueReceive(gQueue, &dump, 0);
          xQueueSendToBack(gQueue, &s, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
        continue;
      }
    }

    // Build minimal JSON (five readings as strings)
    String body;
    body.reserve(96);
    body  = "{\"sensor1\":\""; body += String(s.s1, 1);
    body += "\",\"sensor2\":\""; body += String(s.s2, 1);
    body += "\",\"sensor3\":\""; body += String(s.s3, 1);
    body += "\",\"sensor4\":\""; body += String(s.s4, 1);
    body += "\",\"sensor5\":\""; body += String(s.s5, 1);
    body += "\"}";

    // POST
    http.begin(POST_URL);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST((uint8_t*)body.c_str(), body.length());
    if (code > 0 && code / 100 == 2) {
      Serial.print("[NET] POST OK (");
      Serial.print(code);
      Serial.print(") body: ");
      Serial.println(body);
    } else {
      Serial.print("[NET] POST FAIL (");
      Serial.print(code);
      Serial.println(") re-queueing sample.");
      // Requeue for retry
      if (xQueueSendToBack(gQueue, &s, 0) != pdTRUE) {
        Sample dump; xQueueReceive(gQueue, &dump, 0);
        xQueueSendToBack(gQueue, &s, 0);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    http.end();
  }
}

// ---------- Helpers ----------
bool initializeSensor(Adafruit_VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddress, const char *name) {
  digitalWrite(xshutPin, HIGH);
  delay(10);
  if (!sensor.begin()) {
    Serial.print("[HW] "); Serial.print(name); Serial.println(" failed to initialize!");
    digitalWrite(xshutPin, LOW);
    return false;
  }
  sensor.setAddress(newAddress);
  Serial.print("[HW] "); Serial.print(name); Serial.println(" initialized.");
  return true;
}

void printMeasurement(VL53L0X_RangingMeasurementData_t &measure) {
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter <= 8000) {
    Serial.print(measure.RangeMilliMeter);
    Serial.print("mm");
  } else {
    Serial.print("---");
  }
}

void threeShortBeeps() {
  // 3 short beeps on BUZZER_PIN
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1500, 120);
    delay(180);
  }
}
