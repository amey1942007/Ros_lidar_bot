/*
 * esp32_wifi_gamepad.ino
 *
 * ESP32 DevKit V1 — Wi-Fi virtual Xbox gamepad bridge
 *
 * Flow:
 *   Mobile browser  →  WebSocket JSON  →  ESP32  →  UART frame  →  RPi (/dev/ttyACM1)
 *
 * The ESP32 joins your home/lab Wi-Fi (Station mode). Open the IP address
 * printed in Serial Monitor on your phone to get the virtual Xbox pad.
 *
 * UART frame (32 bytes) sent to RPi over USB-CDC Serial at 115200 baud:
 *   [0]     0xA5   start byte 1
 *   [1]     0x5A   start byte 2
 *   [2-3]   uint16 LE  — sequence number
 *   [4-27]  8 x float32 LE — axes: LX LY RX RY LT RT DX DY
 *   [28-29] uint16 LE  — button bitmask
 *             bit 0=A  1=B  2=X  3=Y  4=LB  5=RB  6=Back  7=Start
 *             bit 8=Xbox  9=L3  10=R3
 *   [30]    XOR checksum of bytes [0-29]
 *   [31]    0xFE   end byte
 *
 * Dependencies (install via Arduino Library Manager):
 *   - ESPAsyncWebServer  (by lacamera / me-no-dev)
 *   - AsyncTCP           (by dvarrel / me-no-dev)
 *   - ArduinoJson        (by Benoit Blanchon, v7 or v6)
 *
 * Board: "ESP32 Dev Module" in Arduino IDE
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// ── Wi-Fi credentials (Station mode) ─────────────────────────────────────────
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// ── Watchdog ──────────────────────────────────────────────────────────────────
// ms without a WS packet before sending a safe-stop (all-zero) frame
#define WS_TIMEOUT_MS 500

// ── UART baud rate — must match joy_teleop_node uart_baud parameter ───────────
#define UART_BAUD 115200

// ── Frame constants ───────────────────────────────────────────────────────────
static const uint8_t FRAME_START1 = 0xA5;
static const uint8_t FRAME_START2 = 0x5A;
static const uint8_t FRAME_END    = 0xFE;
static const size_t  FRAME_LEN    = 32;

// ── Global state ──────────────────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");

// Shared gamepad state (written from WS callback, read from loop)
volatile float    gAxes[8]    = {};  // LX LY RX RY LT RT DX DY
volatile uint16_t gButtons    = 0;
volatile uint32_t lastWsMs    = 0;
uint16_t          seqNum      = 0;

// ── Virtual Xbox gamepad HTML + JavaScript ────────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no"/>
<title>Robot Gamepad</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
body{background:#111;display:flex;flex-direction:column;align-items:center;
     justify-content:space-between;height:100dvh;overflow:hidden;
     font-family:system-ui,sans-serif;color:#eee;user-select:none;}
#status{font-size:13px;padding:6px 10px;border-radius:20px;margin-top:4px;
        background:rgba(255,255,255,.06);}
#status.ok{color:#4f4;}
#status.err{color:#f44;}
#gamepad{display:flex;width:100%;flex:1;align-items:center;
         justify-content:space-around;padding:8px;}

/* Joystick */
.stick-wrap{position:relative;width:130px;height:130px;touch-action:none;
  background:rgba(255,255,255,.07);border-radius:50%;
  border:2px solid rgba(255,255,255,.15);}
.stick-knob{position:absolute;width:54px;height:54px;background:#3af;border-radius:50%;
  top:50%;left:50%;transform:translate(-50%,-50%);box-shadow:0 0 14px #3af8;}

/* D-pad */
#dpad{display:grid;grid-template-columns:44px 44px 44px;
      grid-template-rows:44px 44px 44px;gap:3px;}
.dpad-btn{background:rgba(255,255,255,.1);border-radius:6px;display:flex;
  align-items:center;justify-content:center;font-size:18px;touch-action:none;
  border:1px solid rgba(255,255,255,.15);}
.dpad-btn.active{background:rgba(80,160,255,.55);}
.dpad-center{background:rgba(255,255,255,.04);}

/* Face buttons */
#face{display:grid;grid-template-columns:44px 44px;
      grid-template-rows:44px 44px;gap:6px;}
.face-btn{border-radius:50%;display:flex;align-items:center;justify-content:center;
  font-size:13px;font-weight:700;touch-action:none;
  box-shadow:0 4px 8px #0006;}
.face-btn.active{filter:brightness(1.5) saturate(1.4);}
#btnY{background:#c8a000;color:#000;}
#btnB{background:#b00;color:#fff;}
#btnX{background:#005bc4;color:#fff;}
#btnA{background:#076;color:#fff;}

/* Shoulders / triggers */
#shoulders{display:flex;width:100%;justify-content:space-between;
           padding:0 10px;gap:8px;}
.shoulder-row{display:flex;flex-direction:column;gap:6px;align-items:center;}
.sh-btn{padding:8px 22px;background:rgba(255,255,255,.1);border-radius:8px;touch-action:none;
  border:1px solid rgba(255,255,255,.15);font-size:13px;color:#eee;}
.sh-btn.active{background:rgba(80,160,255,.5);}

/* Start / Back / Xbox */
#middle-btns{display:flex;gap:12px;align-items:center;}
.mid-btn{padding:6px 14px;background:rgba(255,255,255,.1);border-radius:20px;touch-action:none;
  border:1px solid rgba(255,255,255,.15);font-size:12px;color:#eee;}
.mid-btn.active{background:rgba(80,160,255,.4);}
</style>
</head>
<body>
<div id="status">Connecting…</div>

<div id="shoulders">
  <div class="shoulder-row">
    <div class="sh-btn" id="btnLB" data-bit="4">LB</div>
    <div class="sh-btn" id="btnLT" data-axis="4" data-val="-1">LT</div>
  </div>
  <div id="middle-btns">
    <div class="mid-btn" id="btnBack"  data-bit="6">☰</div>
    <div class="mid-btn" id="btnXbox"  data-bit="8">⬤</div>
    <div class="mid-btn" id="btnStart" data-bit="7">≡</div>
  </div>
  <div class="shoulder-row">
    <div class="sh-btn" id="btnRB" data-bit="5">RB</div>
    <div class="sh-btn" id="btnRT" data-axis="5" data-val="-1">RT</div>
  </div>
</div>

<div id="gamepad">
  <div class="stick-wrap" id="leftStick">
    <div class="stick-knob" id="leftKnob"></div>
  </div>

  <div id="dpad">
    <div></div>
    <div class="dpad-btn" id="dUp"    data-dv="1"  data-dh="0" >▲</div>
    <div></div>
    <div class="dpad-btn" id="dLeft"  data-dv="0"  data-dh="1" >◀</div>
    <div class="dpad-center"></div>
    <div class="dpad-btn" id="dRight" data-dv="0"  data-dh="-1">▶</div>
    <div></div>
    <div class="dpad-btn" id="dDown"  data-dv="-1" data-dh="0" >▼</div>
    <div></div>
  </div>

  <div id="face">
    <div></div>
    <div class="face-btn" id="btnY" data-bit="3">Y</div>
    <div class="face-btn" id="btnX" data-bit="2">X</div>
    <div class="face-btn" id="btnA" data-bit="0">A</div>
    <div class="face-btn" id="btnB" data-bit="1">B</div>
    <div></div>
  </div>

  <div class="stick-wrap" id="rightStick">
    <div class="stick-knob" id="rightKnob"></div>
  </div>
</div>

<script>
const axes    = new Float32Array(8);  // LX LY RX RY LT RT DX DY
let   buttons = 0;
let   ws, loopId;
const statusEl = document.getElementById('status');

function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen  = () => { statusEl.textContent = 'Connected ✓'; statusEl.className = 'ok'; startLoop(); };
  ws.onclose = () => { statusEl.textContent = 'Disconnected — retrying…'; statusEl.className = 'err'; clearInterval(loopId); setTimeout(connect, 1500); };
  ws.onerror = () => ws.close();
}

function startLoop() {
  clearInterval(loopId);
  loopId = setInterval(() => {
    if (ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify({ a: Array.from(axes), b: buttons }));
  }, 33);  // ~30 Hz
}

// ── Joystick ─────────────────────────────────────────────────────────────────
function makeStick(wrapId, knobId, axX, axY) {
  const wrap = document.getElementById(wrapId);
  const knob = document.getElementById(knobId);
  let tid = null;

  function update(px, py) {
    const rect = wrap.getBoundingClientRect();
    const R  = rect.width  / 2;
    const KR = knob.offsetWidth / 2;
    const ox = px - rect.left  - R;
    const oy = py - rect.top   - R;
    const d  = Math.sqrt(ox * ox + oy * oy);
    const maxD = R - KR;
    const s  = d > maxD ? maxD / d : 1;
    const nx = ox * s, ny = oy * s;
    knob.style.transform = `translate(calc(-50% + ${nx}px), calc(-50% + ${ny}px))`;
    axes[axX] =  nx / maxD;
    axes[axY] = -ny / maxD;  // invert Y: up = +1
  }

  wrap.addEventListener('touchstart', e => {
    e.preventDefault();
    const t = e.changedTouches[0]; tid = t.identifier;
    update(t.clientX, t.clientY);
  }, { passive: false });
  wrap.addEventListener('touchmove', e => {
    e.preventDefault();
    for (const t of e.changedTouches) if (t.identifier === tid) update(t.clientX, t.clientY);
  }, { passive: false });
  const reset = () => { tid = null; knob.style.transform = 'translate(-50%,-50%)'; axes[axX] = 0; axes[axY] = 0; };
  wrap.addEventListener('touchend',   reset);
  wrap.addEventListener('touchcancel',reset);
}

window.addEventListener('load', () => {
  makeStick('leftStick',  'leftKnob',  0, 1);
  makeStick('rightStick', 'rightKnob', 2, 3);
});

// ── Generic button helper ────────────────────────────────────────────────────
function addBtn(el) {
  const bit  = el.dataset.bit  != null ? parseInt(el.dataset.bit)   : null;
  const aIdx = el.dataset.axis != null ? parseInt(el.dataset.axis)  : null;
  const aVal = el.dataset.val  != null ? parseFloat(el.dataset.val) : 0;
  const set = pressed => {
    el.classList.toggle('active', pressed);
    if (bit  != null) { if (pressed) buttons |= (1 << bit); else buttons &= ~(1 << bit); }
    if (aIdx != null) { axes[aIdx] = pressed ? aVal : 0; }
  };
  el.addEventListener('touchstart',  e => { e.preventDefault(); set(true);  }, { passive: false });
  el.addEventListener('touchend',    e => { e.preventDefault(); set(false); }, { passive: false });
  el.addEventListener('touchcancel', e => { e.preventDefault(); set(false); }, { passive: false });
  el.addEventListener('mousedown', () => set(true));
  el.addEventListener('mouseup',   () => set(false));
  el.addEventListener('mouseleave',() => set(false));
}

['btnA','btnB','btnX','btnY','btnLB','btnRB','btnLT','btnRT',
 'btnBack','btnStart','btnXbox'].forEach(id => addBtn(document.getElementById(id)));

// ── D-pad (one direction at a time) ──────────────────────────────────────────
document.querySelectorAll('.dpad-btn').forEach(btn => {
  const dh = parseFloat(btn.dataset.dh || 0);
  const dv = parseFloat(btn.dataset.dv || 0);
  const clearDpad = () => {
    axes[6] = 0; axes[7] = 0;
    document.querySelectorAll('.dpad-btn').forEach(b => b.classList.remove('active'));
  };
  const press = () => { clearDpad(); axes[6] = dh; axes[7] = dv; btn.classList.add('active'); };
  const rel   = () => clearDpad();
  btn.addEventListener('touchstart',  e => { e.preventDefault(); press(); }, { passive: false });
  btn.addEventListener('touchend',    e => { e.preventDefault(); rel();   }, { passive: false });
  btn.addEventListener('touchcancel', e => { e.preventDefault(); rel();   }, { passive: false });
  btn.addEventListener('mousedown', press);
  btn.addEventListener('mouseup',   rel);
  btn.addEventListener('mouseleave',rel);
});

connect();
</script>
</body>
</html>
)rawliteral";

// ── Build and send a 32-byte UART frame ──────────────────────────────────────
void sendFrame(const float axes[8], uint16_t btns) {
  uint8_t frame[FRAME_LEN];
  frame[0] = FRAME_START1;
  frame[1] = FRAME_START2;

  // Sequence number (LE)
  frame[2] = seqNum & 0xFF;
  frame[3] = (seqNum >> 8) & 0xFF;
  seqNum++;

  // 8 axes as float32 LE
  for (int i = 0; i < 8; i++) {
    float v = axes[i];
    memcpy(&frame[4 + i * 4], &v, 4);
  }

  // Button bitmask (LE)
  frame[28] = btns & 0xFF;
  frame[29] = (btns >> 8) & 0xFF;

  // XOR checksum of bytes 0-29
  uint8_t chk = 0;
  for (int i = 0; i < 30; i++) chk ^= frame[i];
  frame[30] = chk;

  frame[31] = FRAME_END;

  Serial.write(frame, FRAME_LEN);
}

// ── WebSocket event handler ───────────────────────────────────────────────────
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    // Accept only single-frame complete text messages
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = '\0';  // safe to null-terminate; ESPAsyncWebServer over-allocates by 1
      JsonDocument doc;
      if (deserializeJson(doc, (char *)data) != DeserializationError::Ok) return;

      JsonArray jA = doc["a"];
      float tmpAxes[8] = {};
      for (int i = 0; i < 8 && i < (int)jA.size(); i++)
        tmpAxes[i] = jA[i].as<float>();

      uint16_t tmpBtns = (uint16_t)(doc["b"] | 0);

      noInterrupts();
      memcpy((void *)gAxes, tmpAxes, sizeof(tmpAxes));
      gButtons = tmpBtns;
      lastWsMs = millis();
      interrupts();
    }
  }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(UART_BAUD);
  delay(300);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[ESP32] Connecting to Wi-Fi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print('.');
    if (millis() - t0 > 15000) {
      Serial.println("\n[ESP32] Wi-Fi timeout — rebooting");
      ESP.restart();
    }
  }
  Serial.print("\n[ESP32] Connected! Open this IP on your phone: http://");
  Serial.println(WiFi.localIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();
  Serial.println("[ESP32] Web server ready.");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
  ws.cleanupClients();

  uint32_t now = millis();
  bool timedOut = (now - lastWsMs) > WS_TIMEOUT_MS;

  float   axes[8];
  uint16_t btns;

  if (timedOut) {
    memset(axes, 0, sizeof(axes));
    btns = 0;
  } else {
    noInterrupts();
    memcpy(axes, (const void *)gAxes, sizeof(axes));
    btns = gButtons;
    interrupts();
  }

  sendFrame(axes, btns);
  delay(20);  // ~50 Hz transmit rate
}
