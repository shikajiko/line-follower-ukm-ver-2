#include "function.h"
#include "button.h"
#include "IO.h"
#include "line_sensor.h"
#include "display.h"
#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <driver/pcnt.h>
#include <stdio.h>
#include <string.h>
#include "locomotion.h"

#ifndef WIFI_SSID
#define WIFI_SSID "Robotika@test"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "12345678"
#endif

#define BATTERY_CHECK_INTERVAL_MS 500u   // how often to sample
#define BATTERY_LOW_CONFIRM_MS 3000u     // must stay low this long (avoids
                                          // false trip from voltage sag under
                                          // motor load)
#define BATTERY_SAMPLES 8
#define BATTERY_ALARM_TOGGLE_MS 120u     // fast pulse reads as more urgent
                                          // than a steady tone

// ============ GLOBAL VARIABLES ============

// ---- Definitions for externs declared in function.h ----
// (Declared `extern` in function.h so other translation units can see
// them; defined exactly once here, in this file.)
int16_t gy25_yaw = 0;   // x100 degrees
int16_t gy25_pitch = 0; // x100 degrees
int16_t gy25_roll = 0;  // x100 degrees

int16_t motor1_speed = 0;
int16_t motor2_speed = 0;

// GY25 UART communication
uint32_t gy25_last_read = 0;
bool gy25_timeout = false;

// Line position calculation
uint32_t line_position_last_update = 0;
uint16_t line_min_threshold = 100;  // Minimum ADC value to consider as line
uint16_t line_max_threshold = 4000; // Maximum ADC value
float line_position_range =
    1000.0f; // Output range for position (-range to +range)

// Encoder PCNT counters
int32_t enc1_count = 0;
int32_t enc2_count = 0;
int32_t enc1_last_count = 0;
int32_t enc2_last_count = 0;

// Servo position
uint16_t servo1_pulse = 1500;
uint16_t servo2_pulse = 1500;

// Motor PWM (LEDC)
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RES_BITS 8

// Some drivers need a short time enabled while direction inputs change.
#define MOTOR_ENABLE_HOLD_MS 300u

// battery check
static uint32_t battery_last_check_ms = 0;
static uint32_t battery_low_start_ms = 0;
static bool battery_alarm_active = false;
static float battery_last_voltage = 0.0f;
static uint32_t battery_display_until = 0;
bool battery_owns_display = false;

static bool is_calibrating = false;

// VBAT sense calibration
// VBAT_SCALE = VBAT(mV) / SENSE(mV). Include divider ratio + adisny trim.
#ifndef VBAT_SCALE
#define VBAT_SCALE 6.4f
#endif

// Test state machine
#define TEST_STATE_IDLE 0
#define TEST_STATE_GY25 1
#define TEST_STATE_BUTTON 2
#define TEST_STATE_BUZZER 3
#define TEST_STATE_ENC1 4
#define TEST_STATE_ENC2 5
#define TEST_STATE_SERVO1 6
#define TEST_STATE_SERVO2 7
#define TEST_STATE_MOTOR1 8
#define TEST_STATE_MOTOR2 9
#define TEST_STATE_LED_POWER 10
#define TEST_STATE_VBATT 11
#define TEST_STATE_LINE_SENSOR 12
#define TEST_STATE_PID_LINE 13
#define TEST_STATE_WEB_GAMEPAD 14
#define TEST_STATE_DONE 15

uint8_t current_test_state = TEST_STATE_IDLE;
uint32_t test_state_timer = 0;
uint8_t test_running = 0;
uint8_t test_phase = 0;

// Single-sensor check mode (cek sensor satu-per-satu, mirip program basic)
bool single_sensor_active = false;
uint8_t single_sensor_channel = 0;
bool led_power_last_out = false;
uint32_t led_power_last_debug_ms = 0;
bool line_sensor_entry_armed = false;

static WebServer webGamepadServer(80);
static bool webGamepadServerStarted = false;
static bool webGamepadWifiConfigured = false;
static bool webGamepadWifiConnected = false;
static uint32_t webGamepadLastRetryMs = 0;
static char webGamepadIpText[24] = "--";
static char webGamepadWifiText[24] = "idle";
static char webGamepadCommandText[32] = "stop";
static uint8_t webGamepadSpeed = 160;

static bool webGamepadCredentialsReady() {
  return strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0 &&
         strcmp(WIFI_PASSWORD, "YOUR_WIFI_PASSWORD") != 0;
}

static void webGamepadStopMotors() {
  setMotor(1, 0);
  setMotor(2, 0);
  snprintf(webGamepadCommandText, sizeof(webGamepadCommandText), "stop");
}

static void webGamepadSetDrive(const char *direction, uint8_t speed) {
  int16_t left = 0;
  int16_t right = 0;

  if (strcmp(direction, "fwd") == 0) {
    left = speed;
    right = speed;
  } else if (strcmp(direction, "back") == 0) {
    left = -static_cast<int16_t>(speed);
    right = -static_cast<int16_t>(speed);
  } else if (strcmp(direction, "left") == 0) {
    left = -static_cast<int16_t>(speed);
    right = speed;
  } else if (strcmp(direction, "right") == 0) {
    left = speed;
    right = -static_cast<int16_t>(speed);
  }

  setMotor(1, left);
  setMotor(2, right);

  if (left == 0 && right == 0) {
    snprintf(webGamepadCommandText, sizeof(webGamepadCommandText), "stop");
  } else {
    snprintf(webGamepadCommandText, sizeof(webGamepadCommandText), "%s %u",
             direction, speed);
  }

  Serial.printf("[WEB] %s L:%d R:%d SPEED:%u\n", direction, left, right, speed);
}

static void webGamepadServeRoot() {
  static const char page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Robot Gamepad</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #07111f;
      --panel: rgba(12, 22, 39, 0.92);
      --border: rgba(77, 225, 161, 0.22);
      --text: #edf5ff;
      --muted: #9fb0c6;
      --accent: #4de1a1;
      --danger: #ff6b6b;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: Arial, Helvetica, sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at top, rgba(77, 225, 161, 0.15), transparent 28%),
        radial-gradient(circle at bottom right, rgba(255, 204, 102, 0.12), transparent 25%),
        linear-gradient(180deg, #08101c 0%, #050910 100%);
      display: grid;
      place-items: center;
      padding: 16px;
    }
    .card {
      width: min(760px, 100%);
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 24px;
      padding: 20px;
      box-shadow: 0 24px 80px rgba(0, 0, 0, 0.45);
    }
    h1 { margin: 0 0 8px; font-size: 28px; }
    p { margin: 6px 0; color: var(--muted); }
    .status {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 10px;
      margin: 16px 0;
    }
    .pill {
      border: 1px solid rgba(255, 255, 255, 0.08);
      border-radius: 999px;
      padding: 10px 12px;
      background: rgba(255, 255, 255, 0.04);
      font-size: 14px;
    }
    .speed { display: grid; gap: 8px; margin: 10px 0 18px; }
    input[type="range"] { width: 100%; }
    .pad {
      display: grid;
      grid-template-columns: repeat(3, minmax(72px, 1fr));
      gap: 12px;
      max-width: 380px;
      margin: 14px auto;
    }
    button {
      border: 0;
      border-radius: 18px;
      min-height: 64px;
      font-weight: 700;
      font-size: 16px;
      color: var(--text);
      background: rgba(255, 255, 255, 0.08);
      box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.06);
    }
    button.primary {
      background: linear-gradient(180deg, rgba(77, 225, 161, 0.28), rgba(77, 225, 161, 0.12));
      box-shadow: inset 0 0 0 1px rgba(77, 225, 161, 0.35);
    }
    button.danger {
      background: linear-gradient(180deg, rgba(255, 107, 107, 0.32), rgba(255, 107, 107, 0.14));
      box-shadow: inset 0 0 0 1px rgba(255, 107, 107, 0.35);
    }
    .footer {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      justify-content: space-between;
      margin-top: 14px;
      color: var(--muted);
      font-size: 13px;
    }
    .actions {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 10px;
      margin-top: 12px;
    }
  </style>
</head>
<body>
  <main class="card">
    <h1>ESP32 Robot Gamepad</h1>
    <p>Open this page from the IP shown on the LCD, then drive the robot with the buttons below.</p>
    <div class="status">
      <div class="pill" id="wifi">WiFi: --</div>
      <div class="pill" id="ip">IP: --</div>
      <div class="pill" id="cmd">CMD: stop</div>
    </div>
    <div class="speed">
      <label for="speed">Speed: <span id="speedLabel">160</span></label>
      <input id="speed" type="range" min="0" max="255" value="160">
    </div>
    <div class="pad">
      <div></div>
      <button class="primary hold" data-dir="fwd">Forward</button>
      <div></div>
      <button class="primary hold" data-dir="left">Left</button>
      <button class="danger" data-stop="true">STOP</button>
      <button class="primary hold" data-dir="right">Right</button>
      <div></div>
      <button class="primary hold" data-dir="back">Back</button>
      <div></div>
    </div>
    <div class="actions">
      <button class="danger" onclick="finishTest()">Finish Test</button>
      <button onclick="refreshStatus()">Refresh Status</button>
    </div>
    <div class="footer">
      <div>BTN1 on the robot advances to DONE.</div>
      <div>BTN2 returns to the previous test.</div>
    </div>
  </main>
  <script>
    const wifiEl = document.getElementById('wifi');
    const ipEl = document.getElementById('ip');
    const cmdEl = document.getElementById('cmd');
    const speedEl = document.getElementById('speed');
    const speedLabelEl = document.getElementById('speedLabel');

    speedEl.addEventListener('input', () => {
      speedLabelEl.textContent = speedEl.value;
    });

    async function sendCmd(dir) {
      await fetch(`/cmd?dir=${encodeURIComponent(dir)}&speed=${encodeURIComponent(speedEl.value)}`, { cache: 'no-store' });
      await refreshStatus();
    }

    async function stopCmd() {
      await fetch('/cmd?dir=stop&speed=0', { cache: 'no-store' });
      await refreshStatus();
    }

    async function finishTest() {
      await fetch('/finish', { cache: 'no-store' });
      await refreshStatus();
    }

    async function refreshStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        const state = await response.json();
        wifiEl.textContent = `WiFi: ${state.wifi}`;
        ipEl.textContent = `IP: ${state.ip}`;
        cmdEl.textContent = `CMD: ${state.cmd}`;
        if (state.speed !== undefined) {
          speedEl.value = state.speed;
          speedLabelEl.textContent = state.speed;
        }
      } catch (error) {
        wifiEl.textContent = 'WiFi: offline';
      }
    }

    function wireMomentaryButtons() {
      document.querySelectorAll('button[data-dir]').forEach((button) => {
        const dir = button.dataset.dir;
        const start = async (event) => {
          event.preventDefault();
          await sendCmd(dir);
        };
        const stop = async (event) => {
          event.preventDefault();
          await stopCmd();
        };

        button.addEventListener('pointerdown', start);
        button.addEventListener('pointerup', stop);
        button.addEventListener('pointercancel', stop);
        button.addEventListener('pointerleave', stop);
        button.addEventListener('lostpointercapture', stop);
      });

      const stopButton = document.querySelector('button[data-stop="true"]');
      if (stopButton) {
        stopButton.addEventListener('click', async (event) => {
          event.preventDefault();
          await stopCmd();
        });
      }
    }

    refreshStatus();
    wireMomentaryButtons();
    setInterval(refreshStatus, 1000);
  </script>
</body>
</html>
)rawliteral";

  webGamepadServer.sendHeader("Cache-Control", "no-store");
  webGamepadServer.send_P(200, "text/html", page);
}

static void webGamepadServeStatus() {
  String payload = "{";
  payload += "\"wifi\":\"";
  payload += webGamepadWifiText;
  payload += "\",";
  payload += "\"ip\":\"";
  payload += webGamepadIpText;
  payload += "\",";
  payload += "\"cmd\":\"";
  payload += webGamepadCommandText;
  payload += "\",";
  payload += "\"speed\":";
  payload += webGamepadSpeed;
  payload += "}";

  webGamepadServer.sendHeader("Cache-Control", "no-store");
  webGamepadServer.send(200, "application/json", payload);
}

static void webGamepadServeCommand() {
  const String dir = webGamepadServer.arg("dir");
  const int speedValue = webGamepadServer.arg("speed").toInt();
  const uint8_t speed = static_cast<uint8_t>(constrain(speedValue, 0, 255));

  if (dir == "stop" || dir.length() == 0) {
    webGamepadStopMotors();
  } else {
    webGamepadSpeed = speed;
    webGamepadSetDrive(dir.c_str(), speed);
  }

  webGamepadServer.sendHeader("Cache-Control", "no-store");
  webGamepadServer.send(200, "text/plain", "OK");
}

static void webGamepadFinishTest() {
  webGamepadStopMotors();
  test_running = 1;
  current_test_state = TEST_STATE_DONE;
  test_state_timer = millis();
  test_phase = 0;
  snprintf(webGamepadWifiText, sizeof(webGamepadWifiText), "done");
  webGamepadServer.sendHeader("Cache-Control", "no-store");
  webGamepadServer.send(200, "text/plain", "DONE");
}

static void webGamepadEnsureServerStarted() {
  if (webGamepadServerStarted) {
    return;
  }

  webGamepadServer.on("/", HTTP_GET, webGamepadServeRoot);
  webGamepadServer.on("/status", HTTP_GET, webGamepadServeStatus);
  webGamepadServer.on("/cmd", HTTP_GET, webGamepadServeCommand);
  webGamepadServer.on("/finish", HTTP_GET, webGamepadFinishTest);
  webGamepadServer.onNotFound(
      []() { webGamepadServer.send(404, "text/plain", "Not found"); });
  webGamepadServer.begin();
  webGamepadServerStarted = true;
  Serial.println("[WEB] Gamepad server started on port 80");
}

static void webGamepadEnsureWifiConnected(uint32_t now) {
  if (!webGamepadCredentialsReady()) {
    snprintf(webGamepadWifiText, sizeof(webGamepadWifiText), "set creds");
    snprintf(webGamepadIpText, sizeof(webGamepadIpText), "--");
    return;
  }

  if (!webGamepadWifiConfigured) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    webGamepadWifiConfigured = true;
    webGamepadLastRetryMs = now;
    snprintf(webGamepadWifiText, sizeof(webGamepadWifiText), "connecting");
    Serial.printf("[WEB] Connecting to SSID: %s\n", WIFI_SSID);
  }

  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    if (!webGamepadWifiConnected) {
      webGamepadWifiConnected = true;
      IPAddress ip = WiFi.localIP();
      snprintf(webGamepadIpText, sizeof(webGamepadIpText), "%u.%u.%u.%u", ip[0],
               ip[1], ip[2], ip[3]);
      Serial.printf("[WEB] Connected. %s\n", webGamepadIpText);
      if (!webGamepadServerStarted) {
        webGamepadEnsureServerStarted();
      }
    }
    snprintf(webGamepadWifiText, sizeof(webGamepadWifiText), "connected");
    return;
  }

  if (webGamepadWifiConnected) {
    webGamepadWifiConnected = false;
    webGamepadStopMotors();
  }

  if ((now - webGamepadLastRetryMs) >= 15000) {
    webGamepadLastRetryMs = now;
    // Retry by calling begin again (avoid disconnect which may alter lwIP
    // state)
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    snprintf(webGamepadWifiText, sizeof(webGamepadWifiText), "retrying");
    Serial.println("[WEB] WiFi retry");
  } else {
    snprintf(webGamepadWifiText, sizeof(webGamepadWifiText), "connecting");
  }

  snprintf(webGamepadIpText, sizeof(webGamepadIpText), "--");
}

// ============ INIT FUNCTIONS ============
void initpower() {
  // Prevent brief LOW glitch when switching pin mode to OUTPUT.
  // Setting the output value first keeps the rail enabled during boot.
  digitalWrite(SW_POWER_PIN, HIGH);
  pinMode(SW_POWER_PIN, OUTPUT);
}
void power(bool state) { digitalWrite(SW_POWER_PIN, state ? HIGH : LOW); }
void initOLED() {
  // Ensure I2C uses the configured OLED pins from IO.h
  Wire.begin(SDA_PIN, SCL_PIN);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Halt if display fails
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));
  display.display();

  delay(1000);
  Serial.println("[OLED] Initialized successfully");
}

void initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void initButton() {
  pinMode(BUTTON1_PIN, BUTTON_PINMODE);
  pinMode(BUTTON2_PIN, BUTTON_PINMODE);
  pinMode(BUTTON3_PIN, BUTTON_PINMODE);
  pinMode(BUTTON4_PIN, BUTTON_PINMODE);

  // Wire up the debounced Button struct array (pins + initial state).
  initButtonStruct();

  // Illumination LED for line sensor reflection test (Normal Mode)
  pinMode(LED_POWER_PIN, OUTPUT);
  digitalWrite(LED_POWER_PIN, LOW);
}

void initEncoder() {
  pinMode(ENC1_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
}

void initMotor() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(INH1_PIN, OUTPUT);

  // Setup LEDC PWM for motor direction pins
  ledcSetup(MOTOR1_IN1_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
  ledcSetup(MOTOR1_IN2_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
  ledcSetup(MOTOR2_IN3_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);
  ledcSetup(MOTOR2_IN4_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES_BITS);

  ledcAttachPin(IN1_PIN, MOTOR1_IN1_CH);
  ledcAttachPin(IN2_PIN, MOTOR1_IN2_CH);
  ledcAttachPin(IN3_PIN, MOTOR2_IN3_CH);
  ledcAttachPin(IN4_PIN, MOTOR2_IN4_CH);

  // Start with all outputs low (motors disabled)
  ledcWrite(MOTOR1_IN1_CH, 0);
  ledcWrite(MOTOR1_IN2_CH, 0);
  ledcWrite(MOTOR2_IN3_CH, 0);
  ledcWrite(MOTOR2_IN4_CH, 0);
  digitalWrite(INH1_PIN, LOW);
}

void initADC() {
  // Setup ADC for line sensor reading on MUX_ADC_PIN
  analogReadResolution(12); // 12-bit ADC (0-4095)
  pinMode(MUX_ADC_PIN, INPUT);
  pinMode(VBAT_SENSE_PIN, INPUT);
}

void initMUX() {
  // Setup MUX select pins (S0, S1, S2, S3)
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(MUX_S2_PIN, OUTPUT);
  pinMode(MUX_S3_PIN, OUTPUT);

  // Start at channel 0
  digitalWrite(MUX_S0_PIN, LOW);
  digitalWrite(MUX_S1_PIN, LOW);
  digitalWrite(MUX_S2_PIN, LOW);
  digitalWrite(MUX_S3_PIN, LOW);

  // Setup default thresholds for Line Sensor (via API — the threshold
  // array itself lives in line_sensor.cpp).
  ensureLineSensorThresholdDefaults();
}

void initGY25() {
  // Setup Serial1 for GY25 using proven working settings (see gy25.txt)
  Serial1.begin(115200, SERIAL_8N1, GY25_RX_PIN, GY25_TX_PIN);
  gy25_last_read = millis();

#if GY25_DEBUG
  Serial.printf("[GY25] Serial1 started: baud=115200 RX=%d TX=%d\n",
                GY25_RX_PIN, GY25_TX_PIN);
  // Drain garbage bytes so framing starts clean.
  while (Serial1.available()) {
    (void)Serial1.read();
  }
#endif
}

void initServo() {
  // Setup PWM channels for servo control on PWM1 (GPIO6) and PWM2 (GPIO7)
  // Servo frequency: 50Hz (20ms period), 12-bit resolution
  ledcSetup(0, 50, 12); // Channel 0: 50Hz, 12-bit
  ledcSetup(1, 50, 12); // Channel 1: 50Hz, 12-bit
  ledcAttachPin(PWM1_PIN, 0);
  ledcAttachPin(PWM2_PIN, 1);

  setServo(1, 1500);
  setServo(2, 1500);
}

void setupPCNT() {
  // Configure PCNT for Encoder 1 (GPIO41)
  pcnt_config_t pcnt_config_1;
  memset(&pcnt_config_1, 0, sizeof(pcnt_config_t));
  pcnt_config_1.pulse_gpio_num = ENC1_A_PIN;
  pcnt_config_1.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config_1.counter_h_lim = 10000;
  pcnt_config_1.counter_l_lim = -10000;
  pcnt_config_1.unit = PCNT_UNIT_0;
  pcnt_config_1.channel = PCNT_CHANNEL_0;
  pcnt_config_1.pos_mode = PCNT_COUNT_INC;
  pcnt_config_1.neg_mode = PCNT_COUNT_DIS;

  pcnt_unit_config(&pcnt_config_1);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  // Configure PCNT for Encoder 2 (GPIO42)
  pcnt_config_t pcnt_config_2;
  memset(&pcnt_config_2, 0, sizeof(pcnt_config_t));
  pcnt_config_2.pulse_gpio_num = ENC2_A_PIN;
  pcnt_config_2.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config_2.counter_h_lim = 10000;
  pcnt_config_2.counter_l_lim = -10000;
  pcnt_config_2.unit = PCNT_UNIT_1;
  pcnt_config_2.channel = PCNT_CHANNEL_0;
  pcnt_config_2.pos_mode = PCNT_COUNT_INC;
  pcnt_config_2.neg_mode = PCNT_COUNT_DIS;

  pcnt_unit_config(&pcnt_config_2);
  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);
}

// ============ UTILITY FUNCTIONS ============
float readBatteryVoltage() {
    return (analogReadMilliVolts(VBAT_SENSE_PIN) * VBAT_SCALE) / 1000.0f;
}

void printSerial(const char *msg) { Serial.println(msg); }

// ============ GY25 MODULE ============

#ifndef GY25_DEBUG
#define GY25_DEBUG 0
#endif

#if GY25_DEBUG
static void gy25HexDump(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X%s", buf[i], (i + 1 == len) ? "" : " ");
  }
}
#endif

int16_t readGY25Yaw() {
  static uint32_t last_dbg_ms = 0;
  static uint32_t bad_header_count = 0;
  static uint32_t bad_frame_count = 0;
  static uint8_t frame[8];
  static uint8_t frame_idx = 0;

  // Parse binary frames (8 bytes):
  // [0]=0xAA, [1..2]=yaw, [3..4]=pitch, [5..6]=roll, [7]=0x55
  while (Serial1.available()) {
    const int v = Serial1.read();
    if (v < 0) {
      break;
    }
    const uint8_t b = static_cast<uint8_t>(v);

    if (frame_idx == 0 && b != 0xAA) {
      bad_header_count++;
      continue;
    }

    frame[frame_idx++] = b;
    if (frame_idx < sizeof(frame)) {
      continue;
    }

    frame_idx = 0;

    if (frame[7] != 0x55) {
      bad_frame_count++;
      continue;
    }

    gy25_yaw =
        (static_cast<int16_t>(frame[1]) << 8) | static_cast<int16_t>(frame[2]);
    gy25_pitch =
        (static_cast<int16_t>(frame[3]) << 8) | static_cast<int16_t>(frame[4]);
    gy25_roll =
        (static_cast<int16_t>(frame[5]) << 8) | static_cast<int16_t>(frame[6]);
    gy25_last_read = millis();
    gy25_timeout = false;

#if GY25_DEBUG
    {
      const uint32_t now = millis();
      if (now - last_dbg_ms >= 250) {
        last_dbg_ms = now;
        Serial.printf("[GY25] OK Y=%.2f P=%.2f R=%.2f avail=%d badHdr=%lu "
                      "badFrm=%lu bytes: ",
                      gy25_yaw / 100.0f, gy25_pitch / 100.0f,
                      gy25_roll / 100.0f, Serial1.available(),
                      static_cast<unsigned long>(bad_header_count),
                      static_cast<unsigned long>(bad_frame_count));
        gy25HexDump(frame, sizeof(frame));
        Serial.println();
      }
    }
#endif
    return gy25_yaw;
  }

#if GY25_DEBUG
  {
    const uint32_t now = millis();
    if (now - last_dbg_ms >= 1000) {
      last_dbg_ms = now;
      Serial.printf(
          "[GY25] status avail=%d age=%lums Y=%.2f P=%.2f R=%.2f timeout=%u "
          "badHdr=%lu badFrm=%lu\n",
          Serial1.available(), static_cast<unsigned long>(now - gy25_last_read),
          gy25_yaw / 100.0f, gy25_pitch / 100.0f, gy25_roll / 100.0f,
          gy25_timeout ? 1U : 0U, static_cast<unsigned long>(bad_header_count),
          static_cast<unsigned long>(bad_frame_count));
    }
  }
#endif

  if (millis() - gy25_last_read > 1000) {
    gy25_timeout = true;
  }

  return gy25_yaw;
}

// ============ ENCODER PCNT MODULE ============

int32_t readEncoder(uint8_t encoderNum) {
  int16_t count = 0;
  if (REVERSE_MOTOR) {
    if (encoderNum == 1) encoderNum = 2;
    else if (encoderNum == 2) encoderNum = 1;
  }

  if (encoderNum == 1) {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    enc1_count = count;
  } else if (encoderNum == 2) {
    pcnt_get_counter_value(PCNT_UNIT_1, &count);
    enc2_count = count;
  }

  return (int32_t)count;
}

void resetEncoder(uint8_t encoderNum) {
  if (encoderNum == 1) {
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    enc1_count = 0;
    enc1_last_count = 0;
  } else if (encoderNum == 2) {
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
    enc2_count = 0;
    enc2_last_count = 0;
  }
}

// ==== BATTERY CHECK ====

static float readBatteryVoltageAveraged() {
  uint32_t mv_sum = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLES; i++) {
    mv_sum += analogReadMilliVolts(VBAT_SENSE_PIN);
  }
  const float mv = mv_sum / static_cast<float>(BATTERY_SAMPLES);
  return (mv * VBAT_SCALE) / 1000.0f;
}

static void updateBatteryAlarmState(uint32_t now) {
  if (now - battery_last_check_ms < BATTERY_CHECK_INTERVAL_MS) {
    return;
  }
  battery_last_check_ms = now;
  battery_last_voltage = readBatteryVoltageAveraged();

  if (battery_last_voltage < LIMIT_BATTERY) {
    if (battery_low_start_ms == 0) {
      battery_low_start_ms = now;
    }
    if (!battery_alarm_active &&
        (now - battery_low_start_ms) >= BATTERY_LOW_CONFIRM_MS) {
      battery_alarm_active = true;
      Serial.printf("[BATTERY] LOW confirmed (%.2fV < %.2fV). Alarm ON.\n",
                    battery_last_voltage, LIMIT_BATTERY);
    }
  } else {
    battery_low_start_ms = 0;
    if (battery_alarm_active) {
      battery_alarm_active = false;
      Serial.println("[BATTERY] Voltage recovered. Alarm OFF.");
    }
  }
}

void checkBatteryAlarm() {
    const uint32_t now = millis();
    updateBatteryAlarmState(now);

    if (battery_alarm_active && current_test_state != TEST_STATE_BUZZER) {
        const bool on = ((now / BATTERY_ALARM_TOGGLE_MS) % 2) == 0;
        digitalWrite(BUZZER_PIN, on ? HIGH : LOW);

        if (!battery_owns_display) {
            battery_owns_display = true;
            battery_display_until = now + 1000;
        }

        if (battery_owns_display &&
            (int32_t)(now - battery_display_until) >= 0) {
            battery_owns_display = false;
            battery_display_until = 0;
        }
        
    } else {
        digitalWrite(BUZZER_PIN, LOW);
        battery_owns_display = false;
        battery_display_until = 0;
    }
}

// ============ SERVO MODULE ============

void setServo(uint8_t servoNum, uint16_t pulseUs) {
  // Convert pulse width in microseconds to PWM duty cycle
  uint16_t duty = (pulseUs * 4095) / 20000;
  if (duty > 4095)
    duty = 4095;

  if (servoNum == 1) {
    ledcWrite(0, duty);
    servo1_pulse = pulseUs;
  } else if (servoNum == 2) {
    ledcWrite(1, duty);
    servo2_pulse = pulseUs;
  }
}

// ============ MOTOR MODULE ============

void setMotor(uint8_t motorNum, int16_t speed) {
  if (speed > 255)
    speed = 255;
  if (speed < -255)
    speed = -255;

  bool forward = (speed > 0);
  uint8_t pwm_val = abs(speed);

  if (motorNum == 1) {
    ledcWrite(MOTOR1_IN1_CH, forward ? pwm_val : 0);
    ledcWrite(MOTOR1_IN2_CH, forward ? 0 : pwm_val);
    motor1_speed = speed;
  } else if (motorNum == 2) {
    ledcWrite(MOTOR2_IN3_CH, forward ? pwm_val : 0);
    ledcWrite(MOTOR2_IN4_CH, forward ? 0 : pwm_val);
    motor2_speed = speed;
  }

  // Shared enable ON if at least one motor has non-zero command
  static uint32_t last_active_ms = 0;
  const uint32_t now = millis();
  const bool active = (motor1_speed != 0) || (motor2_speed != 0);
  if (active) {
    last_active_ms = now;
  }
  const bool enable = active || (now - last_active_ms < MOTOR_ENABLE_HOLD_MS);
  digitalWrite(INH1_PIN, enable ? HIGH : LOW);
}


// ============ LINE SENSOR 16CH MODULE ============
// Reading, thresholding, and calibration now live entirely in
// line_sensor.cpp behind the API declared in line_sensor.h. This file
// only calls that API (readLineSensors(), getLineSensorRaw/Digital(),
// lineSensorCalibrationBegin/Update/End(), etc.) — it never touches the
// underlying arrays.

// void printPIDDebug() {
//   Serial.printf(
//       "[PID] KP:%.3f KI:%.3f KD:%.3f BASE:%.1f POS:%.2f LINE:%s RUN:%s\n",
//       pid_current_Kp, pid_current_Ki, pid_current_Kd, pid_base_speed,
//       pid_line_position, line_detected ? "ON" : "OFF",
//       pid_enabled ? "YES" : "NO");
// }

// ============ SINGLE SENSOR CHECK MODE ============

// Cek sensor MUX satu-per-satu (mirip "tes robot basic").
// - BTN1 = pindah ke channel berikutnya (0->15->0) + buzzer beep.
// - LED iluminasi dijaga konstan PWM 200 selama mode aktif.
// - Keluar mode: tahan BTN1+BTN2 bersamaan (ditangani di runTestSequence()).
void runSingleSensorCheck() {
  const uint32_t now = millis();

  // Jaga LED iluminasi konstan di PWM 200.
  analogWrite(LED_POWER_PIN, 200);

  // Pindah channel dengan BTN1, tapi abaikan bila BTN2 juga ditekan supaya
  // kombo keluar (BTN1+BTN2) tidak ikut memindah channel.
  static uint32_t buzzer_off_ms = 0;
  if (isButtonPressed(BTN_1) && isButtonUp(BTN_2)) {
    single_sensor_channel = (single_sensor_channel + 1) % 16;
    digitalWrite(BUZZER_PIN, HIGH); // beep pendek non-blocking
    buzzer_off_ms = now + 60;
  }
  if (buzzer_off_ms != 0 && now >= buzzer_off_ms) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzer_off_ms = 0;
  }

  // Baca channel aktif melalui API (selectMUXChannel + analogRead internal).
  uint16_t val = readLineSensorChannel(single_sensor_channel);

  // Tampilan OLED.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("CEK SENSOR 1-1");
  display.setCursor(0, 12);
  display.printf("CH: %u", single_sensor_channel);
  display.setCursor(0, 24);
  display.printf("VAL: %u", val);
  display.setCursor(0, 36);
  display.print("LED PWM: 200");
  display.setCursor(0, 52);
  display.print("Hold B1+B2 = Exit");
  display.display();

  // Serial debug periodik.
  static uint32_t last_serial_ms = 0;
  if (now - last_serial_ms >= 50) {
    last_serial_ms = now;
    Serial.printf("[SENSOR1] MUX CH%u | Value: %u\n", single_sensor_channel,
                  val);
  }
}

// ============ TEST STATE MACHINE ============

void runTestSequence() {
  uint32_t now = millis();

  // loadPIDSettings();
  bool btn3_poweroff = buttonHeld(BTN_3, 3000);

  if (isButtonDown(BTN_3)) {
    if (btn3_poweroff) {
      displayOLED("POWER OFF", "BTN3 > 3s", "Shutting down", "");
      power(false);
      test_running = 0;
      current_test_state = TEST_STATE_IDLE;
      return;
    } else if (current_test_state != TEST_STATE_PID_LINE) {
      char countdown_buf[32];
      uint32_t held_ms = buttonHeldMs(BTN_3);
      uint32_t remain_ms = (held_ms < 3000) ? (3000 - held_ms) : 0;
      float remain_s = remain_ms / 1000.0f;
      snprintf(countdown_buf, sizeof(countdown_buf), "%.1f s", remain_s);
      displayOLED("HOLD BTN3", "Power off in", countdown_buf, "Release=Cancel");
      return;
    }
  }

  // Toggle mode cek sensor satu-per-satu: tahan BTN1+BTN2 bersamaan ~1s.
  static uint32_t combo_hold_start = 0;
  static bool combo_handled = false;
  bool both_down = isButtonDown(BTN_1) && isButtonDown(BTN_2);
  if (both_down) {
    if (combo_hold_start == 0) {
      combo_hold_start = now;
    }
    if (!combo_handled && (now - combo_hold_start) >= 1000) {
      combo_handled = true;
      single_sensor_active = !single_sensor_active;
      if (single_sensor_active) {
        stopMotors(); // safety saat masuk mode
        single_sensor_channel = 0;
      } else {
        analogWrite(LED_POWER_PIN, 0);  // lepas PWM
        pinMode(LED_POWER_PIN, OUTPUT); // kembalikan kontrol digitalWrite
        digitalWrite(LED_POWER_PIN, LOW);
      }
    }
  } else {
    combo_hold_start = 0;
    combo_handled = false;
  }

  if (single_sensor_active) {
    runSingleSensorCheck();
    return; // konsumsi loop; lewati state-machine biasa
  }

  if (!test_running) {
    current_test_state = TEST_STATE_IDLE;
  }

  if (isButtonPressed(BTN_1) && current_test_state == TEST_STATE_IDLE) {
    test_running = 1;
    current_test_state = TEST_STATE_GY25;
    test_state_timer = now;
    test_phase = 0;
  } else if (current_test_state != TEST_STATE_BUTTON) {
    // Global navigation (manual): BTN1=Next, BTN2=Prev
    if (isButtonPressed(BTN_1) && current_test_state != TEST_STATE_IDLE &&
        current_test_state != TEST_STATE_DONE &&
        current_test_state != TEST_STATE_LINE_SENSOR &&
        current_test_state != TEST_STATE_PID_LINE) {
      current_test_state++;
      test_state_timer = now;
      test_phase = 0;
    } else if (isButtonPressed(BTN_2) &&
               current_test_state > TEST_STATE_GY25 &&
               current_test_state != TEST_STATE_IDLE &&
               current_test_state != TEST_STATE_PID_LINE) {
      current_test_state--;
      test_state_timer = now;
      test_phase = 0;
    }
  }

  // Cleanup/entry actions on state transitions (manual navigation)
  static uint8_t last_state = TEST_STATE_IDLE;
  if (last_state != current_test_state) {
    if (last_state == TEST_STATE_MOTOR1 || last_state == TEST_STATE_MOTOR2) {
      stopMotors();
    }
    if (last_state == TEST_STATE_PID_LINE) {
      stopMotors();
      // pid_enabled = false;
      // pid_menu_active = false;
      // pid_last_update_ms = 0;
    }
    if (last_state == TEST_STATE_WEB_GAMEPAD) {
      webGamepadStopMotors();
    }
    if (last_state == TEST_STATE_BUZZER) {
      digitalWrite(BUZZER_PIN, LOW);
    }
    if (current_test_state == TEST_STATE_ENC1) {
      resetEncoder(1);
    } else if (current_test_state == TEST_STATE_ENC2) {
      resetEncoder(2);
    }
    last_state = current_test_state;
  }

  // Ensure buzzer is off outside buzzer test
  if (current_test_state != TEST_STATE_BUZZER) {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // LED_POWER behavior:
  // - LED_POWER state: blink continuously as visual test
  // - LINE_SENSOR state: force ON for photodiode reflection
  bool led_reflection_out = false;

  if (current_test_state == TEST_STATE_LED_POWER) {
    led_reflection_out = (((now / 300) % 2) == 0);
  } else if (current_test_state == TEST_STATE_LINE_SENSOR) {
    led_reflection_out = true;
  } else if (current_test_state == TEST_STATE_PID_LINE) {
    led_reflection_out = true;
  }

  digitalWrite(LED_POWER_PIN, led_reflection_out ? HIGH : LOW);

  // Debug LED output transitions and periodic status in LED test mode
  if (led_reflection_out != led_power_last_out) {
    Serial.printf("[LED] STATE:%u PIN:%u OUT:%s\n", current_test_state,
                  LED_POWER_PIN, led_reflection_out ? "ON" : "OFF");
    led_power_last_out = led_reflection_out;
  }
  if (current_test_state == TEST_STATE_LED_POWER &&
      (now - led_power_last_debug_ms) >= 500) {
    led_power_last_debug_ms = now;
    Serial.printf("[LED_TEST] BLINK:%s BTN1=NEXT\n",
                  led_reflection_out ? "ON" : "OFF");
  }

  if (!test_running) {
    displayOLED("READY", "Press BTN1", "to start test", "");
    return;
  }

  switch (current_test_state) {

  case TEST_STATE_GY25: {
    readGY25Yaw();
    char l2[32];
    char l3[32];
    snprintf(l2, sizeof(l2), "Y:%.2f P:%.2f", gy25_yaw / 100.0f,
             gy25_pitch / 100.0f);
    snprintf(l3, sizeof(l3), "R:%.2f %s", gy25_roll / 100.0f,
             gy25_timeout ? "TIMEOUT" : "OK");
    displayOLED("TEST: GY25", l2, l3, "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_BUTTON: {
    // Show live button states on OLED.
    // In this test, navigation is by HOLD (so you can tap buttons to test),
    // using the shared buttonHeld() hold-detection.
    const int b1 = isButtonDown(BTN_1);
    const int b2 = isButtonDown(BTN_2);
    const int b3 = isButtonDown(BTN_3);
    const int b4 = isButtonDown(BTN_4);

    if (buttonHeld(BTN_1, 800)) {
      current_test_state++;
      test_state_timer = now;
      test_phase = 0;
      return;
    }
    if (buttonHeld(BTN_2, 800) && current_test_state > TEST_STATE_GY25) {
      current_test_state--;
      test_state_timer = now;
      test_phase = 0;
      return;
    }

    char l1[32];
    char l2[32];
    snprintf(l1, sizeof(l1), "B1:%d B2:%d", b1, b2);
    snprintf(l2, sizeof(l2), "B3:%d B4:%d", b3, b4);
    displayOLED("TEST: BUTTON", l1, l2, "Hold1=Next Hold2=Prev");
    break;
  }

  case TEST_STATE_BUZZER: {
    // Keep buzzing while in this test (manual navigation).
    const bool on = (((now - test_state_timer) / 200) % 2) == 0;
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
    displayOLED("TEST: BUZZER", "Buzzing...", "", "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_ENC1: {
    int32_t enc_val = readEncoder(1);
    char buf[32];
    sprintf(buf, "ENC1: %ld", enc_val);
    displayOLED("TEST: ENCODER 1", buf, "Rotate wheel L",
                "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_ENC2: {
    int32_t enc_val = readEncoder(2);
    char buf[32];
    sprintf(buf, "ENC2: %ld", enc_val);
    displayOLED("TEST: ENCODER 2", buf, "Rotate wheel R",
                "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_SERVO1: {
    // Repeat sweep while staying in this state (manual navigation)
    uint32_t elapsed = (now - test_state_timer) % 2000;
    uint16_t pulse;

    if (elapsed < 1000) {
      pulse = 1000 + (elapsed / 2);
    } else if (elapsed < 2000) {
      pulse = 1500 + ((elapsed - 1000) / 2);
    } else {
      pulse = 2000;
    }

    setServo(1, pulse);
    char buf[32];
    sprintf(buf, "SRV1: %u us", pulse);
    displayOLED("TEST: SERVO 1", buf, "", "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_SERVO2: {
    // Repeat sweep while staying in this state (manual navigation)
    uint32_t elapsed = (now - test_state_timer) % 2000;
    uint16_t pulse;

    if (elapsed < 1000) {
      pulse = 1000 + (elapsed / 2);
    } else if (elapsed < 2000) {
      pulse = 1500 + ((elapsed - 1000) / 2);
    } else {
      pulse = 2000;
    }

    setServo(2, pulse);
    char buf[32];
    sprintf(buf, "SRV2: %u us", pulse);
    displayOLED("TEST: SERVO 2", buf, "", "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_MOTOR1: {
    // Repeat ramp pattern while staying in this state (manual navigation)
    // Add short brake/dead-time between direction changes for reliability.
    // Cycle: 0-2.5s FWD ramp, 2.5-2.7s BRAKE, 2.7-5.2s REV ramp
    uint32_t elapsed = (now - test_state_timer) % 5200;
    if (test_phase == 0) {
      resetEncoder(1);
      test_phase = 1;
    }

    int16_t speed_cmd = 0;
    const char *mode = "BRK";
    if (elapsed < 2500) {
      mode = "FWD";
      speed_cmd = (elapsed * 255) / 2500; // forward ramp
    } else if (elapsed < 2700) {
      mode = "BRK";
      speed_cmd = 0;
    } else {
      mode = "REV";
      speed_cmd = -((int32_t)(elapsed - 2700) * 255) / 2500; // reverse ramp
    }

    setMotor(1, speed_cmd);
    int32_t enc_val = readEncoder(1);

    char buf1[32];
    char buf2[32];
    sprintf(buf1, "M1 %s PWM:%d", mode, abs(speed_cmd));
    sprintf(buf2, "ENC1: %ld", enc_val);
    displayOLED("TEST: MOTOR 1", buf1, buf2, "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_MOTOR2: {
    // Repeat ramp pattern while staying in this state (manual navigation)
    // Add short brake/dead-time between direction changes for reliability.
    // Cycle: 0-2.5s FWD ramp, 2.5-2.7s BRAKE, 2.7-5.2s REV ramp
    uint32_t elapsed = (now - test_state_timer) % 5200;
    if (test_phase == 0) {
      resetEncoder(2);
      test_phase = 1;
    }

    int16_t speed_cmd = 0;
    const char *mode = "BRK";
    if (elapsed < 2500) {
      mode = "FWD";
      speed_cmd = (elapsed * 255) / 2500; // forward ramp
    } else if (elapsed < 2700) {
      mode = "BRK";
      speed_cmd = 0;
    } else {
      mode = "REV";
      speed_cmd = -((int32_t)(elapsed - 2700) * 255) / 2500; // reverse ramp
    }

    setMotor(2, speed_cmd);
    int32_t enc_val = readEncoder(2);

    char buf1[32];
    char buf2[32];
    sprintf(buf1, "M2 %s PWM:%d", mode, abs(speed_cmd));
    sprintf(buf2, "ENC2: %ld", enc_val);
    displayOLED("TEST: MOTOR 2", buf1, buf2, "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_LED_POWER: {
    displayOLED("TEST: LED POWER", "LED ON for line", "sensor reflection",
                "BTN1=Next");

    // Stay in this test until operator presses BTN1 to continue
    break;
  }

  case TEST_STATE_VBATT: {
    // Read VBAT sense ADC (shows voltage at the sense pin; apply divider ratio
    // externally if needed).
    uint32_t raw_sum = 0;
    uint32_t mv_sum = 0;
    const uint8_t samples = 8;
    for (uint8_t i = 0; i < samples; i++) {
      raw_sum += analogRead(VBAT_SENSE_PIN);
      mv_sum += analogReadMilliVolts(VBAT_SENSE_PIN);
      delay(2);
    }
    const uint16_t raw = static_cast<uint16_t>(raw_sum / samples);
    const uint16_t mv = static_cast<uint16_t>(mv_sum / samples);

    const float vbat_mv = mv * VBAT_SCALE;

    char l2[32];
    char l3[32];
    snprintf(l2, sizeof(l2), "RAW:%u", static_cast<unsigned>(raw));
    snprintf(l3, sizeof(l3), "S:%umV B:%.2fV", static_cast<unsigned>(mv),
             vbat_mv / 1000.0f);
    displayOLED("TEST: VBATT", l2, l3, "BTN1=Next BTN2=Prev");
    break;
  }

  case TEST_STATE_LINE_SENSOR: {
    // All calibration state (min/max/threshold) and the raw/digital
    // arrays now live behind line_sensor.h — this state only drives the
    // workflow (when to begin/update/end calibration, what to show) and
    // reads results back out through the getters.
    if (test_phase == 0) {
      test_state_timer = now;
      test_phase = 1;
      is_calibrating = false;
      line_sensor_entry_armed = false;
    }

    if (!line_sensor_entry_armed) {
      if (!isButtonPressed(BTN_1)) {
        line_sensor_entry_armed = true;
      }
    }

    // --- Proses Kontrol Kalibrasi via BUTTON 4 ---
    if (isButtonPressed(BTN_4)) {
      is_calibrating = !is_calibrating; // Toggle mode kalibrasi
      if (is_calibrating) {
        // Mulai kalibrasi: reset min/max via API
        lineSensorCalibrationBegin();
      } else {
        // Selesai kalibrasi: hitung threshold tiap channel via API
        lineSensorCalibrationEnd();
        Serial.print("[LINE] CALIBRATION DONE. Thresholds: ");
        for (int i = 0; i < 16; i++) {
          Serial.printf("%d:%u ", i, getLineSensorThreshold(i));
        }
        Serial.println();
      }
    }

    // Lanjut ke Test WEB GAMEPAD via Button 1 (di-handle terpisah untuk case
    // ini)
    if (line_sensor_entry_armed && isButtonPressed(BTN_1) &&
        !is_calibrating) {
      current_test_state = TEST_STATE_WEB_GAMEPAD;
      test_state_timer = now;
      test_phase = 0;
      break; // Keluar dari frame ini dan masuk ke test selanjutnya
    }

    // Update pembacaan Min & Max saat mode Kalibrasi AKTIF (via API).
    if (is_calibrating) {
      lineSensorCalibrationUpdate();
    }

    uint8_t disp_group = ((now - test_state_timer) / 1000) % 4;
    uint8_t ch_start = disp_group * 4;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    // Tampilkan label berbeda di OLED saat mode kalibrasi jalan
    if (is_calibrating) {
      display.printf("CALIBRATING... (BTN4)");
    } else {
      display.printf("RAW CH%u-%u", ch_start, ch_start + 3);
    }

    display.setCursor(0, 8);
    display.printf("%u:%u %u:%u", ch_start, getLineSensorRaw(ch_start),
                   ch_start + 1, getLineSensorRaw(ch_start + 1));

    display.setCursor(0, 16);
    display.printf("%u:%u %u:%u", ch_start + 2, getLineSensorRaw(ch_start + 2),
                   ch_start + 3, getLineSensorRaw(ch_start + 3));

    const int bar_y = 24;
    const int bar_h = 39;
    const int slot_w = 8;
    const int box_w = 7;

    for (int i = 0; i < 16; i++) {
      int x = i * slot_w;
      display.drawRect(x, bar_y, box_w, bar_h, SSD1306_WHITE);
      if (getLineSensorDigital(i)) {
        display.fillRect(x + 1, bar_y + 1, box_w - 2, bar_h - 2, SSD1306_WHITE);
      }
    }

    display.display();

    static uint32_t last_ls_serial_ms = 0;
    if (now - last_ls_serial_ms >= 100) {
      last_ls_serial_ms = now;
      // Dump semua channel 0-15 dalam satu baris agar cepat dibaca.
      Serial.printf("[LINE] %s RAW:", is_calibrating ? "CAL:ON" : "CAL:OFF");
      for (int i = 0; i < 16; i++) {
        Serial.printf(" %d:%u", i, getLineSensorRaw(i));
      }
      Serial.print(" DIG:");
      for (int i = 0; i < 16; i++) {
        Serial.print(getLineSensorDigital(i) ? '1' : '0');
      }
      Serial.println();
    }

    break;
  }

  case TEST_STATE_PID_LINE: {
    // (kept as-is: fully commented out in the original source)
    break;
  }

  case TEST_STATE_WEB_GAMEPAD: {
    if (test_phase == 0) {
      webGamepadStopMotors();
      webGamepadWifiConfigured = false;
      webGamepadWifiConnected = false;
      webGamepadLastRetryMs = now;
      snprintf(webGamepadWifiText, sizeof(webGamepadWifiText), "idle");
      snprintf(webGamepadIpText, sizeof(webGamepadIpText), "--");
      snprintf(webGamepadCommandText, sizeof(webGamepadCommandText), "stop");
      test_state_timer = now;
      test_phase = 1;
    }

    webGamepadEnsureWifiConnected(now);

    if (WiFi.status() == WL_CONNECTED) {
      if (webGamepadServerStarted) {
        webGamepadServer.handleClient();
      }
    }

    if (webGamepadWifiConnected) {
      char ipLine[32];
      char wifiLine[32];
      char cmdLine[32];
      snprintf(ipLine, sizeof(ipLine), "IP: %s", webGamepadIpText);
      snprintf(wifiLine, sizeof(wifiLine), "WIFI: %s", webGamepadWifiText);
      snprintf(cmdLine, sizeof(cmdLine), "CMD: %s", webGamepadCommandText);
      displayOLED("TEST: WEB GAMEPAD", ipLine, wifiLine, cmdLine);
    } else {
      char wifiLine[32];
      snprintf(wifiLine, sizeof(wifiLine), "WIFI: %s", webGamepadWifiText);
      displayOLED("TEST: WEB GAMEPAD", wifiLine, "Open IP after WiFi",
                  "BTN1=Done BTN2=Prev");
    }
    break;
  }

  case TEST_STATE_DONE: {
    displayOLED("ALL TESTS DONE", "", "BTN1=Restart", "BTN2=Prev");

    if (isButtonPressed(BTN_1)) {
      test_running = 0;
      current_test_state = TEST_STATE_IDLE;
    }
    break;
  }

  default:
    displayOLED("ERROR", "Unknown state", "", "");
    break;
  }
}