#!/usr/bin/env python3
"""
web_gamepad_node.py — Virtual Xbox gamepad served from the RPi5 over Tailscale.

Hosts a mobile-friendly virtual Xbox controller webpage (HTTP + WebSocket)
directly on the RPi5.  The player opens the page on their phone via the RPi5's
Tailscale IP address; touches on the on-screen sticks and buttons are sent back
as JSON over WebSocket and published on /joy (sensor_msgs/Joy).

The companion joy_teleop_node subscribes to /joy and drives /cmd_vel, camera,
and dashboard actions — it requires no changes.

Usage
─────
  # Install dependency once:
  pip install aiohttp

  # Run:
  ros2 run Ros_lidar_bot web_gamepad

  # Override port (default 8765):
  ros2 run Ros_lidar_bot web_gamepad --ros-args -p port:=9000

  # Open in phone browser:
  http://<tailscale-ip>:8765

WebSocket protocol (browser → RPi)
────────────────────────────────────
  Text frame — JSON: {"a": [LX, LY, RX, RY, LT, RT, DX, DY], "b": <bitmask>}
  axes:    float32 in range [-1.0, 1.0]
           LT/RT: 0 = not pressed, -1 = fully pressed
           DX:    D-pad horizontal  +1=left, -1=right
           DY:    D-pad vertical    +1=up,   -1=down
  buttons: uint16 bitmask
           bit 0=A  1=B  2=X  3=Y  4=LB  5=RB  6=Back  7=Start  8=Xbox  9=L3  10=R3

sensor_msgs/Joy mapping
────────────────────────
  axes[0]=LX  axes[1]=LY  axes[2]=RX  axes[3]=RY
  axes[4]=LT  axes[5]=RT  axes[6]=DX  axes[7]=DY
  buttons[0..10] = bitmask bits 0..10 (each 0 or 1)

Requires:
  pip install aiohttp
"""

import asyncio
import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

try:
    from aiohttp import web
    _AIOHTTP_OK = True
except ImportError:
    _AIOHTTP_OK = False

# ── Virtual Xbox gamepad HTML page ────────────────────────────────────────────
_GAMEPAD_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no"/>
<title>Robot Controller</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
body{
  background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);
  display:flex;flex-direction:column;align-items:center;
  justify-content:space-between;height:100dvh;overflow:hidden;
  font-family:system-ui,sans-serif;color:#eee;user-select:none;
}

/* ── Status bar ── */
#statusbar{
  width:100%;display:flex;align-items:center;justify-content:space-between;
  padding:6px 14px;background:rgba(0,0,0,.35);font-size:12px;
}
#dot{width:10px;height:10px;border-radius:50%;background:#555;margin-right:6px;
     box-shadow:0 0 6px #555;transition:background .3s,box-shadow .3s;}
#dot.ok{background:#4f4;box-shadow:0 0 10px #4f4;}
#dot.err{background:#f44;box-shadow:0 0 10px #f44;}
#statustext{flex:1;}
#iptext{opacity:.5;}

/* ── Shoulder row ── */
#shoulders{
  display:flex;width:100%;justify-content:space-between;
  padding:0 10px;gap:6px;
}
.shoulder-row{display:flex;flex-direction:column;gap:5px;align-items:center;}
.sh-btn{
  padding:9px 24px;background:rgba(255,255,255,.1);border-radius:10px;
  border:1px solid rgba(255,255,255,.18);font-size:13px;color:#eee;
  touch-action:none;box-shadow:0 3px 8px #0005;
  transition:background .1s;
}
.sh-btn.active{background:rgba(80,160,255,.55);box-shadow:0 0 12px rgba(80,160,255,.4);}
#middle-btns{display:flex;gap:10px;align-items:center;}
.mid-btn{
  padding:6px 14px;background:rgba(255,255,255,.1);border-radius:20px;
  border:1px solid rgba(255,255,255,.15);font-size:12px;color:#eee;
  touch-action:none;transition:background .1s;
}
.mid-btn.active{background:rgba(80,160,255,.45);}

/* ── Main gamepad row ── */
#gamepad{
  display:flex;width:100%;flex:1;align-items:center;
  justify-content:space-around;padding:8px 10px;
}

/* ── Joystick ── */
.stick-wrap{
  position:relative;width:130px;height:130px;touch-action:none;
  background:radial-gradient(circle,rgba(255,255,255,.08),rgba(255,255,255,.03));
  border-radius:50%;border:2px solid rgba(255,255,255,.15);
  box-shadow:0 0 20px rgba(0,0,0,.4) inset;
}
.stick-knob{
  position:absolute;width:52px;height:52px;border-radius:50%;
  background:radial-gradient(circle at 35% 35%,#5af,#258);
  top:50%;left:50%;transform:translate(-50%,-50%);
  box-shadow:0 0 14px rgba(51,170,255,.6),0 4px 8px #0006;
  transition:box-shadow .1s;
}
.stick-wrap:active .stick-knob{box-shadow:0 0 22px rgba(51,170,255,.9),0 4px 8px #0006;}

/* ── D-pad ── */
#dpad{display:grid;grid-template-columns:46px 46px 46px;
      grid-template-rows:46px 46px 46px;gap:3px;}
.dpad-btn{
  background:rgba(255,255,255,.1);border-radius:7px;display:flex;
  align-items:center;justify-content:center;font-size:20px;touch-action:none;
  border:1px solid rgba(255,255,255,.15);
  box-shadow:0 3px 6px #0004;transition:background .1s;
}
.dpad-btn.active{background:rgba(80,160,255,.55);box-shadow:0 0 10px rgba(80,160,255,.4);}
.dpad-center{background:rgba(255,255,255,.04);border-radius:4px;}

/* ── Face buttons ── */
#face{display:grid;grid-template-columns:46px 46px;
      grid-template-rows:46px 46px;gap:7px;}
.face-btn{
  border-radius:50%;display:flex;align-items:center;justify-content:center;
  font-size:13px;font-weight:700;touch-action:none;
  box-shadow:0 4px 10px #0007;transition:filter .1s;
}
.face-btn.active{filter:brightness(1.55) saturate(1.3);}
#btnY{background:radial-gradient(circle at 35% 35%,#e8b800,#8a6c00);color:#000;}
#btnB{background:radial-gradient(circle at 35% 35%,#e03030,#7a0000);color:#fff;}
#btnX{background:radial-gradient(circle at 35% 35%,#1060e0,#052080);color:#fff;}
#btnA{background:radial-gradient(circle at 35% 35%,#10a050,#055025);color:#fff;}
</style>
</head>
<body>

<div id="statusbar">
  <div id="dot"></div>
  <span id="statustext">Connecting…</span>
  <span id="iptext"></span>
</div>

<div id="shoulders">
  <div class="shoulder-row">
    <div class="sh-btn" id="btnLB" data-bit="4">LB</div>
    <div class="sh-btn" id="btnLT" data-axis="4" data-val="-1">LT</div>
  </div>
  <div id="middle-btns">
    <div class="mid-btn" id="btnBack"  data-bit="6">☰</div>
    <div class="mid-btn" id="btnXbox"  data-bit="8">⏺</div>
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
// ── State ─────────────────────────────────────────────────────────────────────
const axes    = new Float32Array(8);  // LX LY RX RY LT RT DX DY
let   buttons = 0;
let   ws, loopId;
const dot        = document.getElementById('dot');
const statusText = document.getElementById('statustext');
const ipText     = document.getElementById('iptext');

// ── WebSocket ─────────────────────────────────────────────────────────────────
function connect() {
  const url = 'ws://' + location.host + '/ws';
  ipText.textContent = location.host;
  ws = new WebSocket(url);
  ws.onopen = () => {
    dot.className = 'ok';
    statusText.textContent = 'Connected ✓';
    clearInterval(loopId);
    loopId = setInterval(send, 33);   // ~30 Hz
  };
  ws.onclose = () => {
    dot.className = 'err';
    statusText.textContent = 'Disconnected — retrying…';
    clearInterval(loopId);
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
}

function send() {
  if (ws && ws.readyState === WebSocket.OPEN)
    ws.send(JSON.stringify({ a: Array.from(axes), b: buttons }));
}

// ── Joystick ──────────────────────────────────────────────────────────────────
function makeStick(wrapId, knobId, axX, axY) {
  const wrap = document.getElementById(wrapId);
  const knob = document.getElementById(knobId);
  let tid = null;

  function update(px, py) {
    const rect = wrap.getBoundingClientRect();
    const R  = rect.width / 2;
    const KR = knob.offsetWidth / 2;
    const ox = px - rect.left - R;
    const oy = py - rect.top  - R;
    const d  = Math.hypot(ox, oy);
    const maxD = R - KR;
    const s  = d > maxD ? maxD / d : 1;
    const nx = ox * s, ny = oy * s;
    knob.style.transform = `translate(calc(-50% + ${nx}px), calc(-50% + ${ny}px))`;
    axes[axX] =  nx / maxD;
    axes[axY] = -ny / maxD;   // invert Y: up = +1
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
  const reset = () => {
    tid = null;
    knob.style.transform = 'translate(-50%,-50%)';
    axes[axX] = 0; axes[axY] = 0;
  };
  wrap.addEventListener('touchend',   reset);
  wrap.addEventListener('touchcancel',reset);
}

window.addEventListener('load', () => {
  makeStick('leftStick',  'leftKnob',  0, 1);
  makeStick('rightStick', 'rightKnob', 2, 3);
});

// ── Generic button ────────────────────────────────────────────────────────────
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
  // Mouse fallback for desktop testing
  el.addEventListener('mousedown', () => set(true));
  el.addEventListener('mouseup',   () => set(false));
  el.addEventListener('mouseleave',() => set(false));
}

['btnA','btnB','btnX','btnY','btnLB','btnRB','btnLT','btnRT',
 'btnBack','btnStart','btnXbox'].forEach(id => addBtn(document.getElementById(id)));

// ── D-pad ─────────────────────────────────────────────────────────────────────
const dBtns = document.querySelectorAll('.dpad-btn');
function clearDpad() { axes[6] = 0; axes[7] = 0; dBtns.forEach(b => b.classList.remove('active')); }
dBtns.forEach(btn => {
  const dh = parseFloat(btn.dataset.dh || 0);
  const dv = parseFloat(btn.dataset.dv || 0);
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
"""


# ── WebSocket connection handler ───────────────────────────────────────────────
class WebGamepadNode(Node):
    """
    Serves the virtual gamepad webpage and translates WebSocket messages to /joy.
    """

    def __init__(self):
        super().__init__('web_gamepad')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('publish_hz', 30.0)

        self._host = self.get_parameter('host').value
        self._port = self.get_parameter('port').value
        self._pub  = self.create_publisher(Joy, '/joy', 10)

        # Rate limiter: only publish at most publish_hz even if WS sends faster
        self._min_dt = 1.0 / self.get_parameter('publish_hz').value
        self._last_pub = 0.0

        # aiohttp app / runner kept for clean shutdown
        self._app    = None
        self._runner = None
        self._loop   = None

        if not _AIOHTTP_OK:
            self.get_logger().error(
                'aiohttp is not installed. Run:  pip install aiohttp\n'
                'web_gamepad_node cannot start without it.')
            return

        self._ws_clients: set = set()

        # Start the web server in a background thread with its own event loop
        self._web_thread = threading.Thread(
            target=self._run_server, daemon=True, name='web_gamepad_server')
        self._web_thread.start()

        # Log Tailscale / local IPs so user knows where to connect
        self.create_timer(1.5, self._log_url)   # delay to let server start first

    # ── Startup ───────────────────────────────────────────────────────────────
    def _run_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._start_app())
        self._loop.run_forever()

    async def _start_app(self):
        self._app = web.Application()
        self._app.router.add_get('/',    self._handle_index)
        self._app.router.add_get('/ws',  self._handle_ws)

        self._runner = web.AppRunner(self._app)
        await self._runner.setup()
        site = web.TCPSite(self._runner, self._host, self._port)
        await site.start()

    def _log_url(self):
        """Log IP addresses so the user knows where to connect."""
        try:
            hostname = socket.gethostname()
            local_ip = socket.gethostbyname(hostname)
        except Exception:
            local_ip = '127.0.0.1'

        self.get_logger().info(
            f'Web gamepad listening on port {self._port}.\n'
            f'  Local:     http://{local_ip}:{self._port}\n'
            f'  Tailscale: check `tailscale ip -4` on the RPi and use '
            f'http://<tailscale-ip>:{self._port}\n'
            f'  Any iface: http://0.0.0.0:{self._port}')
        # Cancel the one-shot timer
        self.destroy_timer(self._log_url_timer)

    # We store the timer so we can cancel it; create_timer returns the timer obj
    def create_timer(self, period, callback):
        t = super().create_timer(period, callback)
        if callback == self._log_url:
            self._log_url_timer = t
        return t

    # ── HTTP handlers ─────────────────────────────────────────────────────────
    async def _handle_index(self, request):
        return web.Response(
            text=_GAMEPAD_HTML,
            content_type='text/html',
            charset='utf-8')

    async def _handle_ws(self, request):
        ws_resp = web.WebSocketResponse()
        await ws_resp.prepare(request)
        self._ws_clients.add(ws_resp)
        self.get_logger().info(
            f'Gamepad client connected from {request.remote}')
        try:
            async for msg in ws_resp:
                if msg.type == web.WSMsgType.TEXT:  # type: ignore[attr-defined]
                    self._on_ws_message(msg.data)
                elif msg.type in (web.WSMsgType.ERROR,   # type: ignore[attr-defined]
                                  web.WSMsgType.CLOSE):  # type: ignore[attr-defined]
                    break
        except Exception as exc:
            self.get_logger().warn(f'WebSocket error: {exc}')
        finally:
            self._ws_clients.discard(ws_resp)
            # Publish a zero Joy when client disconnects (safe stop)
            self._publish_joy([0.0] * 8, 0)
            self.get_logger().info('Gamepad client disconnected — publishing zero Joy')
        return ws_resp

    # ── Message decoding ──────────────────────────────────────────────────────
    def _on_ws_message(self, data: str):
        try:
            obj = json.loads(data)
            raw_axes    = obj.get('a', [0.0] * 8)
            raw_buttons = int(obj.get('b', 0))
        except (json.JSONDecodeError, TypeError, ValueError):
            return

        # Rate-limit publishing
        now = time.monotonic()
        if now - self._last_pub < self._min_dt:
            return
        self._last_pub = now

        axes = [float(v) for v in raw_axes[:8]]
        while len(axes) < 8:
            axes.append(0.0)

        # Call publish from the asyncio thread — rclpy publishers are thread-safe
        self._publish_joy(axes, raw_buttons)

    def _publish_joy(self, axes, buttons_mask: int):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes    = axes
        # Expand bitmask to list of 0/1 integers (11 buttons defined)
        msg.buttons = [(buttons_mask >> i) & 1 for i in range(11)]
        self._pub.publish(msg)

    # ── Shutdown ──────────────────────────────────────────────────────────────
    def shutdown_server(self):
        if self._runner and self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self._runner.cleanup(), self._loop).result(timeout=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = WebGamepadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_server()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
