#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_NeoPixel.h>

// ================== Hardware ==================
#define SDA_PIN 21
#define SCL_PIN 22

// Motor A: IN1, Motor B: IN3
#define IN1 25
#define IN3 27

// NeoPixel strip (60 LEDs total)
#define LED_PIN   23          // free GPIO on ESP32
#define LED_COUNT 60
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
MPU6050 mpu(Wire);

// Control constants
//add end value for pitch where at end value all leds are on
//make blue override red only after a certain value
//look at ghaiths code
const float P_DEADZONE   = 3.0;    // pitch threshold
const float R_DEADZONE   = 9;  // roll threshold
const float MAX_ANGLE  = 10.0;   // degrees; beyond this, PWM caps
const int   MAX_SPEED  = 200;    // 0..255 cap
const int   MIN_SPEED  = 30;     // starting PWM

// HW loop period for control/POST enqueue
static const uint32_t HW_PERIOD_MS = 250;

// ================== Wi-Fi / POST ==================
static const char* WIFI_SSID     = "QSTP VC";
static const char* WIFI_PASSWORD = "qstp1234";
// Endpoint expects: {"pitch":"..","yaw":"..","roll":".."}
static const char* POST_URL      = "http://54.86.18.41/api/bardata";

// ================== Queue (HW -> NET) ==================
struct PoseSample {
  float pitch;
  float yaw;
  float roll;
};
#define SAMPLE_QUEUE_LEN 30
QueueHandle_t gQueue;

// Init flags
volatile bool gHwReady  = false;
volatile bool gNetReady = false;

// ================== Tasks ==================
TaskHandle_t taskHWHandle;
TaskHandle_t taskNetHandle;

void taskHW(void*);   // Core 0: Hardware
void taskNet(void*);  // Core 1: WiFi + HTTP POST

// ================== Motor helpers (PWM) ==================
inline void motorPWM(int pin, int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(pin, pwm);
}
inline void motorsOff() {
  analogWrite(IN1, 0);
  analogWrite(IN3, 0);
}

// ================== LED helpers ==================
// Map PWM (0..MAX_SPEED) -> number of LEDs (0..30), ceiling division
inline int ledsFromPWM(int pwm) {
  if (pwm <= 0) return 0;
  int count = (pwm * 30 + (MAX_SPEED - 1)) / MAX_SPEED; // ceil(pwm/MAX_SPEED*30)
  return constrain(count, 0, 30);
}

void showAllGreenOneShot(uint16_t ms) {
  for (int i = 0; i < LED_COUNT; ++i) strip.setPixelColor(i, strip.Color(0, 255, 0));
  strip.show();
  delay(ms);
  strip.clear();
  strip.show();
}

// Draw red bars for pitch mode (A or B). Called every HW loop.
void drawPitchLEDs(int pwmA, int pwmB) {
  strip.clear();

  // --- SWAP APPLIED ---
  // Motor A → LEDs 1..30 (index 0..29), filling from LED 30 backward to 1
  int aCount = ledsFromPWM(pwmA);
  if (aCount > 0) {
    int start = 29;                 // LED 30 index
    int end   = 29 - (aCount - 1);  // go backward
    for (int i = start; i >= end; --i) {
      strip.setPixelColor(i, strip.Color(255, 0, 0)); // RED
    }
  }

  // Motor B → LEDs 31..60 (index 30..59), filling from LED 31 forward to 60
  int bCount = ledsFromPWM(pwmB);
  if (bCount > 0) {
    int start = 30;                 // LED 31 index
    int end   = 30 + (bCount - 1);  // go forward
    for (int i = start; i <= end; ++i) {
      strip.setPixelColor(i, strip.Color(255, 0, 0)); // RED
    }
  }

  strip.show();
}

// Draw blue for roll-override mode (all 60 LEDs)
void drawRollLEDsBlue() {
  for (int i = 0; i < LED_COUNT; ++i) strip.setPixelColor(i, strip.Color(0, 0, 255));
  strip.show();
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  // NeoPixel init (keep brightness modest if powered from ESP32 5V)
  strip.begin();
  strip.setBrightness(40);
  strip.show();              // all off

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("[HW] Initializing MPU6050...");
  if (mpu.begin() != 0) {
    Serial.println("[HW] MPU6050 init failed. Check wiring/power.");
    while (true) delay(1000);
  }

  Serial.println("[HW] Calibrating... keep device still");
  //mpu.calcOffsets();
  Serial.println("[HW] Calibration done.");

  // One-time green flash after calibration
  showAllGreenOneShot(1000);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  motorsOff();

  // Queue
  gQueue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(PoseSample));
  if (!gQueue) {
    Serial.println("[SYS] Queue allocation failed, halting.");
    while (true) delay(1000);
  }

  // Create tasks (Core 0 = HW, Core 1 = NET)
  BaseType_t ok1 = xTaskCreatePinnedToCore(taskHW,  "HW",  4096, nullptr, 2, &taskHWHandle,  0);
  BaseType_t ok2 = xTaskCreatePinnedToCore(taskNet, "NET", 6144, nullptr, 2, &taskNetHandle, 1);
  if (ok1 != pdPASS || ok2 != pdPASS) {
    Serial.println("[SYS] Task creation failed, halting.");
    while (true) delay(1000);
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000)); // unused; work runs in tasks
}

// ================== Core 0: Hardware ==================
void taskHW(void*) {
  Serial.println("[HW] Task started (Core 0).");
  gHwReady = true;

  uint32_t lastCtrl = millis();

  for (;;) {
    // High-rate sensor fusion
    mpu.update();
    vTaskDelay(pdMS_TO_TICKS(2));

    // 250 ms control & queue
    uint32_t now = millis();
    if ((int32_t)(now - lastCtrl) >= (int32_t)HW_PERIOD_MS) {
      lastCtrl += HW_PERIOD_MS;

      // Read angles (deg)
      float pitch = mpu.getAngleX();
      float roll  = mpu.getAngleY();
      float yaw   = mpu.getAngleZ();

// Check if we're in the flipped orientation (around 180°)
bool flipped = (pitch > 90.0f || pitch < -90.0f);

// Map pitch to -90° to 90° range for consistent control logic
if (pitch > 90.0f) {pitch = pitch - 180.0f;}
else if (pitch < -90.0f) {pitch = pitch + 180.0f;}



      int pwmA = 0, pwmB = 0;
      bool rollOverride = false;

      // ---- Combined logic with roll override ----
      if (fabs(roll) > R_DEADZONE) {
        // ROLL MODE: both motors ON with same PWM (from roll magnitude)
        float norm = (fabs(roll) - R_DEADZONE) / (MAX_ANGLE - R_DEADZONE);
        norm = constrain(norm, 0.0f, 1.0f);
        int pwm = MIN_SPEED + (int)(norm * (MAX_SPEED - MIN_SPEED));
        pwmA = pwm;
        pwmB = pwm;
        rollOverride = true;
      } else {
        // PITCH MODE: one motor ON depending on pitch sign
        if (pitch > P_DEADZONE) {
          float norm = (pitch - P_DEADZONE) / (MAX_ANGLE - P_DEADZONE);
          norm = constrain(norm, 0.0f, 1.0f);
          pwmA = MIN_SPEED + (int)(norm * (MAX_SPEED - MIN_SPEED));
          pwmB = 0;
        } else if (pitch < -P_DEADZONE) {
          float norm = ((-pitch) - P_DEADZONE) / (MAX_ANGLE - P_DEADZONE);
          norm = constrain(norm, 0.0f, 1.0f);
          pwmB = MIN_SPEED + (int)(norm * (MAX_SPEED - MIN_SPEED));
          pwmA = 0;
        } else {
          pwmA = 0;
          pwmB = 0;
        }
      }
      // Then flip outputs if needed:
if (flipped) {
    // Swap motor outputs
    int temp = pwmA;
    pwmA = pwmB;
    pwmB = temp;}
    
      // Apply PWMs or OFF in deadzone
      if (pwmA == 0 && pwmB == 0) {
        motorsOff();
      } else {
        motorPWM(IN1, pwmA);
        motorPWM(IN3, pwmB);
      }

      // ---- LED update (every HW loop) ----
      if (pwmA == 0 && pwmB == 0) {
        strip.clear();
        strip.show();
      } else if (rollOverride) {
        // All blue when roll dominates
        drawRollLEDsBlue();
      } else {
        // Red sections for pitch
        drawPitchLEDs(pwmA, pwmB);
      }

      // Print HW status
      char line[220];
      snprintf(line, sizeof(line),
               "[HW] Pitch: %.1f  Roll: %.1f  Yaw: %.1f  PWM_A: %d  PWM_B: %d  Mode:%s",
               pitch, roll, yaw, pwmA, pwmB, rollOverride ? "ROLL" : "PITCH");
      Serial.println(line);

      // Enqueue a sample for networking (drop oldest if full)
      PoseSample s{pitch, yaw, roll};
      if (xQueueSendToBack(gQueue, &s, 0) != pdTRUE) {
        PoseSample dump;
        xQueueReceive(gQueue, &dump, 0);
        xQueueSendToBack(gQueue, &s, 0);
      }
    }
  }
}

// ================== Core 1: Networking ==================
void taskNet(void*) {
  Serial.println("[NET] Task started (Core 1).");
  Serial.println("[NET] Connecting to Wi-Fi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(250));
    Serial.print(".");
    if (millis() - t0 > 20000) {
      Serial.println("\n[NET] Wi-Fi connect timeout, retrying...");
      WiFi.disconnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      t0 = millis();
    }
  }
  Serial.print("\n[NET] Wi-Fi connected. IP: ");
  Serial.println(WiFi.localIP());
  gNetReady = true;

  HTTPClient http;

  for (;;) {
    if (uxQueueMessagesWaiting(gQueue) == 0) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    PoseSample s;
    if (xQueueReceive(gQueue, &s, 0) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Reconnect if needed
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[NET] Wi-Fi lost; reconnecting...");
      WiFi.disconnect(true);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      uint32_t start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
      if (WiFi.status() != WL_CONNECTED) {
        if (xQueueSendToBack(gQueue, &s, 0) != pdTRUE) {
          PoseSample dump; xQueueReceive(gQueue, &dump, 0);
          xQueueSendToBack(gQueue, &s, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
        continue;
      }
    }

    // Build JSON payload (values as strings, 1 decimal)
    String body;
    body.reserve(96);
    body  = "{\"pitch\":\""; body += String(s.pitch, 1);
    body += "\",\"yaw\":\"";  body += String(s.yaw,   1);
    body += "\",\"roll\":\""; body += String(s.roll,  1);
    body += "\"}";

    http.begin(POST_URL);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST((uint8_t*)body.c_str(), body.length());

    if (code > 0 && code / 100 == 2) {
      Serial.print("[NET] POST OK (");
      Serial.print(code);
      Serial.print(") ");
      Serial.println(body);
    } else {
      Serial.print("[NET] POST FAIL (");
      Serial.print(code);
      Serial.println(") re-queueing sample.");

      if (xQueueSendToBack(gQueue, &s, 0) != pdTRUE) {
        PoseSample dump; xQueueReceive(gQueue, &dump, 0);
        xQueueSendToBack(gQueue, &s, 0);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    http.end();
  }
}
