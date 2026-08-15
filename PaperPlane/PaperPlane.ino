/*
  XIAO ESP32S3 - Dual coreless motor control via web sliders (WebSocket)
  ------------------------------------------------------------------
  Low-latency design choices:
   - WebSocket instead of HTTP polling: no request/response overhead per update
   - Plain-text message format ("A:120", "B:90") instead of JSON: avoids
     parsing overhead on both browser and MCU
   - HTML/JS page served from PROGMEM: no filesystem (LittleFS/SPIFFS) access,
     so first load is instant and RAM/flash use stays low
   - Slider input uses the "input" JS event (fires continuously while dragging)
     and sends every change immediately over the already-open WS socket
   - PWM via LEDC hardware peripheral, updated directly in the WS callback
     (no queueing, no delay() anywhere in the code)
   - Watchdog failsafe: motors auto-stop if no message received for 400 ms
     (protects against WiFi drop while a slider is mid-drag)

  Libraries required (Library Manager):
   - "AsyncTCP" by ESP32Async
   - "ESPAsyncWebServer" (a.k.a. "ESP Async WebServer") by ESP32Async
   - ESP32 board package v3.x (Boards Manager) for the ledcAttach() API

  Wiring - single PWM pin per motor (speed only, no direction control):
   Motor A PWM -> D1 (GPIO2)
   Motor B PWM -> D2 (GPIO3)
   Driver GND  -> XIAO GND (common ground, mandatory)
   Driver VM   -> external motor supply (NOT the 3V3 pin)
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ---------- USER CONFIG ----------
//const char* WIFI_SSID     = "Natanon_2.4G";
//const char* WIFI_PASSWORD = "0818939584";

const char* WIFI_SSID     = "fuji";
const char* WIFI_PASSWORD = "valent420";

// Set true to have the XIAO create its own WiFi network instead of joining one.
// Useful for robots/vehicles with no router in range -> lowest, most stable latency.
const bool USE_ACCESS_POINT = false;
const char* AP_SSID     = "PaperPlane";
const char* AP_PASSWORD = "12345678"; // min 8 chars

// Motor pins (one PWM pin per motor - speed only)
#define MOTOR_A_PIN 2   // D1
#define MOTOR_B_PIN 3   // D2

const int PWM_FREQ = 20000;   // 20 kHz: above audible range, quiet motors
const int PWM_RES  = 8;       // 8-bit -> duty 0-255

const unsigned long FAILSAFE_TIMEOUT_MS = 400; // stop motors if link goes quiet
const unsigned long MONITOR_INTERVAL_MS = 500;  // how often to print pin values
// ----------------------------------

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile uint8_t speedA = 0; // 0..255
volatile uint8_t speedB = 0; // 0..255
volatile unsigned long lastMsgTime = 0;
unsigned long lastMonitorPrint = 0;

void setMotor(int pin, uint8_t value) {
  ledcWrite(pin, value);
}

void stopAllMotors() {
  speedA = 0;
  speedB = 0;
  setMotor(MOTOR_A_PIN, 0);
  setMotor(MOTOR_B_PIN, 0);
}

/// ---------- Web page (served from flash, no filesystem needed) ----------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Motor Control</title>
<style>
  body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:20px;}
  h2{font-weight:400;}
  .sliders-row{
    display:flex;
    justify-content:center;
    align-items:flex-start;
    gap:60px;
    margin:40px auto;
  }
  .slider-box{
    display:flex;
    flex-direction:column;
    align-items:center;
  }
  .slider-vertical{
    /* fixed on-screen size after rotation */
    width:60px;
    height:280px;
    display:flex;
    justify-content:center;
    align-items:center;
  }
  input[type=range]{
    -webkit-appearance:none;
    appearance:none;
    background:#333;
    border-radius:10px;
    outline:none;
    /* base (unrotated) size: becomes height after 90deg rotation */
    width:280px;
    height:60px;
    /* rotate to vertical: min ends up at bottom, max at top */
    transform:rotate(-90deg);
  }
  input[type=range]::-webkit-slider-thumb{
    -webkit-appearance:none;
    width:50px;height:50px;border-radius:50%;
    background:#2ecc71;cursor:pointer;
  }
  input[type=range]::-moz-range-thumb{
    width:50px;height:50px;border-radius:50%;
    background:#2ecc71;cursor:pointer;border:none;
  }
  .val{font-size:24px;margin-top:12px;}
  #status{margin-top:20px;font-size:14px;color:#888;}
  .connected{color:#2ecc71 !important;}
</style>
</head>
<body>
<div class="sliders-row">
  <div class="slider-box">
    <h2>Motor A</h2>
    <div class="slider-vertical">
      <input type="range" min="0" max="255" value="0" id="sliderA">
    </div>
    <div class="val" id="valA">0</div>
  </div>

  <div class="slider-box">
    <h2>Motor B</h2>
    <div class="slider-vertical">
      <input type="range" min="0" max="255" value="0" id="sliderB">
    </div>
    <div class="val" id="valB">0</div>
  </div>
</div>

<div id="status">connecting...</div>

<script>
let ws;
let sliderA = document.getElementById('sliderA');
let sliderB = document.getElementById('sliderB');
let valA = document.getElementById('valA');
let valB = document.getElementById('valB');
let statusEl = document.getElementById('status');

function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = () => { statusEl.textContent = 'connected'; statusEl.classList.add('connected'); };
  ws.onclose = () => {
    statusEl.textContent = 'disconnected - retrying...';
    statusEl.classList.remove('connected');
    setTimeout(connect, 500);
  };
  ws.onerror = () => ws.close();
}
connect();

function send(id, v) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(id + ':' + v);
}

// 'input' fires continuously while dragging -> lowest latency feedback
sliderA.addEventListener('input', () => {
  valA.textContent = sliderA.value;
  send('A', sliderA.value);
});
sliderB.addEventListener('input', () => {
  valB.textContent = sliderB.value;
  send('B', sliderB.value);
});

// Heartbeat: resend current values periodically even while a slider is held
// still (no 'input' events firing). This keeps the board's failsafe timer
// fed so it only trips on a real disconnect, not on a stationary slider.
setInterval(() => {
  send('A', sliderA.value);
  send('B', sliderB.value);
}, 150);

// NOTE: by default the sliders HOLD their position after release, so the
// motor keeps running at whatever speed you left it at.
// If you prefer RC-style "let go = stop" behavior instead, uncomment these:
// sliderA.addEventListener('change', () => { sliderA.value = 0; valA.textContent = 0; send('A', 0); });
// sliderB.addEventListener('change', () => { sliderB.value = 0; valB.textContent = 0; send('B', 0); });
</script>
</body>
</html>
)HTML";
// ---------- WebSocket event handling ----------
void handleWsMessage(uint8_t *data, size_t len) {
  // Expect short plain-text messages: "A:123" or "B:45" (0-255)
  if (len < 3) return;
  char buf[16];
  size_t n = min(len, sizeof(buf) - 1);
  memcpy(buf, data, n);
  buf[n] = '\0';

  char motor = buf[0];
  if (buf[1] != ':') return;
  int value = atoi(buf + 2);
  value = constrain(value, 0, 255);

  lastMsgTime = millis();

  if (motor == 'A') {
    speedA = (uint8_t)value;
    setMotor(MOTOR_A_PIN, speedA);
  } else if (motor == 'B') {
    speedB = (uint8_t)value;
    setMotor(MOTOR_B_PIN, speedB);
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS client #%u connected\n", client->id());
      lastMsgTime = millis();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      stopAllMotors(); // failsafe: stop immediately on disconnect
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        handleWsMessage(data, len);
      }
      break;
    }
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // PWM setup (ESP32 core 3.x LEDC API)
  ledcAttach(MOTOR_A_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_B_PIN, PWM_FREQ, PWM_RES);
  stopAllMotors();

  if (USE_ACCESS_POINT) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("AP started. Connect to WiFi '");
    Serial.print(AP_SSID);
    Serial.println("' then browse to http://192.168.4.1");
  } else {
    WiFi.mode(WIFI_STA);
    // Disables WiFi power-save so response latency stays low and consistent
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");

    const unsigned long STA_TIMEOUT_MS = 30000; // give up after 30s
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startAttempt < STA_TIMEOUT_MS) {
      delay(300);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected. Browse to http://");
      Serial.println(WiFi.localIP());
    } else {
      // Fallback: couldn't join the network in time -> start our own AP
      // so the board is still reachable instead of hanging forever.
      Serial.println("Failed to connect to WiFi. Starting fallback AP...");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(AP_SSID, AP_PASSWORD);
      Serial.print("Fallback AP '");
      Serial.print(AP_SSID);
      Serial.println("' started. Browse to http://192.168.4.1");
    }
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.begin();
}

void loop() {
  // Periodic cleanup of stale WS client structures (recommended by the library)
  ws.cleanupClients();

  // Failsafe: if we haven't heard from the browser in a while, stop motors.
  // Guards against a dropped WiFi link leaving a motor running at speed.
  if (lastMsgTime != 0 && (millis() - lastMsgTime > FAILSAFE_TIMEOUT_MS)) {
    if (speedA != 0 || speedB != 0) {
      stopAllMotors();
    }
  }

  // Periodically print current pin/speed values to Serial for monitoring.
  // Non-blocking (no delay()), so it doesn't add latency to motor control.
  if (millis() - lastMonitorPrint >= MONITOR_INTERVAL_MS) {
    lastMonitorPrint = millis();
    Serial.printf("[Monitor] Motor A (pin %d): %3u/255   Motor B (pin %d): %3u/255   clients: %u\n",
                  MOTOR_A_PIN, speedA, MOTOR_B_PIN, speedB, ws.count());
  }
}
