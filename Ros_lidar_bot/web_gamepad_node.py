#!/usr/bin/env python3
"""
web_gamepad_node.py — Virtual Xbox Gamepad served directly from the robot.

Zero external dependencies: uses Python standard library only (http.server,
socket, threading). Works out-of-the-box on Jetson Orin, Raspberry Pi, Ubuntu,
macOS, and any Linux without needing pip or aiohttp.

Features:
  • Built-in HTTP server serving responsive mobile & desktop Cyberpunk gamepad UI.
  • RFC 6455 compliant native WebSocket server + HTTP POST fallback.
  • Decoupled ROS 2 30Hz publisher timer for smooth, uninterrupted /joy stream.
  • Multi-touch Pointer Events with setPointerCapture for simultaneous stick
    and button operation without gesture interruption.
  • Mecanum & Differential drive support:
    - Left stick: linear.x (forward/back) + linear.y (strafe left/right)
    - Right stick: angular.z (yaw rotate) + camera tilt
"""

import base64
import hashlib
import json
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# ── Cyberpunk Mobile & Desktop Web Controller HTML ────────────────────────────
_GAMEPAD_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover"/>
<title>Robot Web Controller</title>
<style>
  :root {
    --bg-primary: #0b0f19;
    --bg-card: rgba(18, 24, 38, 0.85);
    --border-card: rgba(255, 255, 255, 0.14);
    --cyan: #00f0ff;
    --emerald: #10b981;
    --amber: #f59e0b;
    --red: #ef4444;
    --text-primary: #f8fafc;
    --text-dim: #94a3b8;
  }
  * {
    box-sizing: border-box;
    margin: 0;
    padding: 0;
    -webkit-tap-highlight-color: transparent;
  }
  html, body {
    width: 100vw;
    height: 100vh;
    overflow: hidden;
    touch-action: none;
    user-select: none;
    -webkit-user-select: none;
    overscroll-behavior: none;
    position: fixed;
    background: radial-gradient(circle at 50% 20%, #171f33 0%, #080b12 85%);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", sans-serif;
    color: var(--text-primary);
    display: flex;
    flex-direction: column;
    justify-content: space-between;
  }

  /* ── Header ── */
  #header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 16px;
    background: var(--bg-card);
    backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px);
    border-bottom: 1px solid var(--border-card);
    z-index: 10;
  }
  .status-badge {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.5px;
    text-transform: uppercase;
  }
  .pulse-dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: #64748b;
    box-shadow: 0 0 6px rgba(100, 116, 139, 0.6);
    transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  }
  .pulse-dot.online {
    background: var(--emerald);
    box-shadow: 0 0 12px var(--emerald), 0 0 24px rgba(16, 185, 129, 0.4);
    animation: pulse 1.8s infinite;
  }
  .pulse-dot.offline {
    background: var(--red);
    box-shadow: 0 0 12px var(--red);
  }
  @keyframes pulse {
    0%, 100% { transform: scale(1); opacity: 1; }
    50% { transform: scale(1.25); opacity: 0.75; }
  }
  .ip-info {
    font-size: 12px;
    color: var(--text-dim);
    font-family: monospace;
    font-weight: 600;
  }
  .estop-btn {
    background: linear-gradient(135deg, #ef4444, #991b1b);
    color: #fff;
    border: 1px solid rgba(255, 255, 255, 0.3);
    border-radius: 8px;
    padding: 7px 16px;
    font-size: 12px;
    font-weight: 800;
    letter-spacing: 1px;
    box-shadow: 0 2px 10px rgba(239, 68, 68, 0.4);
    cursor: pointer;
    touch-action: none;
  }
  .estop-btn:active {
    transform: scale(0.94);
    filter: brightness(1.2);
  }

  /* ── Shoulders & Speed Bar ── */
  #shoulder-panel {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 8px 16px 0 16px;
    gap: 8px;
  }
  .shoulder-group {
    display: flex;
    gap: 6px;
  }
  .trigger-btn {
    background: var(--bg-card);
    border: 1px solid var(--border-card);
    border-radius: 10px;
    padding: 8px 14px;
    font-size: 11px;
    font-weight: 700;
    color: var(--text-primary);
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 2px;
    min-width: 60px;
    box-shadow: 0 4px 10px rgba(0, 0, 0, 0.3);
    cursor: pointer;
    touch-action: none;
    transition: transform 0.08s, background 0.08s, border-color 0.08s;
  }
  .trigger-btn span {
    font-size: 9px;
    color: var(--text-dim);
    font-weight: 500;
  }
  .trigger-btn.active {
    background: rgba(0, 240, 255, 0.25);
    border-color: var(--cyan);
    color: var(--cyan);
    box-shadow: 0 0 16px rgba(0, 240, 255, 0.4);
    transform: translateY(2px);
  }

  /* ── Live Speed HUD (Center) ── */
  #speed-hud {
    display: flex;
    gap: 16px;
    background: rgba(0, 0, 0, 0.5);
    padding: 6px 16px;
    border-radius: 20px;
    border: 1px solid rgba(255, 255, 255, 0.1);
  }
  .hud-item {
    display: flex;
    flex-direction: column;
    align-items: center;
  }
  .hud-label {
    font-size: 9px;
    color: var(--text-dim);
    font-weight: 600;
    text-transform: uppercase;
  }
  .hud-val {
    font-size: 13px;
    font-weight: 800;
    color: var(--cyan);
    font-family: monospace;
  }

  /* ── Main Controls Workspace ── */
  #workspace {
    flex: 1;
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 10px 18px 24px 18px;
    gap: 10px;
    position: relative;
  }

  /* ── Joysticks ── */
  .stick-container {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
  }
  .stick-label {
    font-size: 10px;
    font-weight: 700;
    color: var(--text-dim);
    letter-spacing: 0.5px;
    text-transform: uppercase;
  }
  .stick-base {
    position: relative;
    width: 140px;
    height: 140px;
    border-radius: 50%;
    background: radial-gradient(circle, rgba(255,255,255,0.06) 0%, rgba(15,23,42,0.6) 70%, rgba(11,15,25,0.9) 100%);
    border: 2px solid rgba(255, 255, 255, 0.15);
    box-shadow: 0 0 25px rgba(0, 0, 0, 0.8) inset, 0 8px 20px rgba(0, 0, 0, 0.5);
    touch-action: none;
    cursor: grab;
  }
  .stick-cross {
    position: absolute;
    top: 50%;
    left: 50%;
    width: 80%;
    height: 1px;
    background: rgba(255, 255, 255, 0.08);
    transform: translate(-50%, -50%);
    pointer-events: none;
  }
  .stick-cross.v {
    transform: translate(-50%, -50%) rotate(90deg);
  }
  .stick-thumb {
    position: absolute;
    width: 58px;
    height: 58px;
    border-radius: 50%;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    background: radial-gradient(circle at 35% 35%, #38bdf8, #0284c7 60%, #0369a1);
    border: 2px solid rgba(255, 255, 255, 0.4);
    box-shadow: 0 6px 14px rgba(0, 0, 0, 0.6), 0 0 16px rgba(56, 189, 248, 0.5);
    pointer-events: none;
    transition: box-shadow 0.15s;
  }
  .stick-base.active .stick-thumb {
    box-shadow: 0 8px 20px rgba(0, 0, 0, 0.8), 0 0 26px var(--cyan);
  }

  /* ── Center Column: D-Pad & System Buttons ── */
  #center-column {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 16px;
  }

  .dpad-grid {
    display: grid;
    grid-template-columns: 36px 36px 36px;
    grid-template-rows: 36px 36px 36px;
    gap: 2px;
  }
  .dpad-btn {
    background: var(--bg-card);
    border: 1px solid var(--border-card);
    border-radius: 6px;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 14px;
    color: var(--cyan);
    box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
    cursor: pointer;
    touch-action: none;
    transition: transform 0.06s, background 0.06s, box-shadow 0.06s;
  }
  .dpad-btn.active {
    background: rgba(0, 240, 255, 0.3);
    border-color: var(--cyan);
    box-shadow: 0 0 14px rgba(0, 240, 255, 0.5);
    transform: scale(0.92);
  }
  .dpad-center {
    background: rgba(255, 255, 255, 0.03);
    border-radius: 4px;
  }

  .middle-group {
    display: flex;
    gap: 10px;
  }
  .sys-btn {
    background: var(--bg-card);
    border: 1px solid var(--border-card);
    border-radius: 16px;
    padding: 6px 14px;
    font-size: 11px;
    font-weight: 700;
    color: var(--text-dim);
    box-shadow: 0 3px 8px rgba(0, 0, 0, 0.3);
    cursor: pointer;
    touch-action: none;
  }
  .sys-btn.active {
    background: rgba(255, 255, 255, 0.2);
    color: #fff;
  }

  /* ── Face Buttons (ABXY) ── */
  .face-cluster {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
  }
  .face-grid {
    display: grid;
    grid-template-columns: 46px 46px;
    grid-template-rows: 46px 46px;
    gap: 8px;
    transform: rotate(45deg);
    padding: 6px;
  }
  .face-btn {
    border-radius: 50%;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    font-size: 14px;
    font-weight: 900;
    transform: rotate(-45deg);
    border: 1.5px solid rgba(255, 255, 255, 0.25);
    box-shadow: 0 6px 14px rgba(0, 0, 0, 0.6);
    cursor: pointer;
    touch-action: none;
    transition: transform 0.06s, filter 0.06s;
  }
  .face-btn span {
    font-size: 8px;
    font-weight: 700;
    text-transform: uppercase;
    opacity: 0.85;
  }
  .face-btn.active {
    transform: rotate(-45deg) scale(0.92);
    filter: brightness(1.45) saturate(1.4);
    box-shadow: 0 0 20px currentColor;
  }

  #btn-y { background: radial-gradient(circle at 35% 35%, #fbbf24, #b45309); color: #1e1b18; }
  #btn-b { background: radial-gradient(circle at 35% 35%, #f87171, #b91c1c); color: #fff; }
  #btn-a { background: radial-gradient(circle at 35% 35%, #34d399, #047857); color: #fff; }
  #btn-x { background: radial-gradient(circle at 35% 35%, #38bdf8, #0369a1); color: #fff; }
</style>
</head>
<body>

  <!-- Top Status Header -->
  <div id="header">
    <div class="status-badge">
      <div id="pulse-dot" class="pulse-dot"></div>
      <span id="conn-text">CONNECTING…</span>
    </div>
    <div class="ip-info" id="ip-badge">PORT 8765</div>
    <button class="estop-btn" id="btn-estop">E-STOP ⏹</button>
  </div>

  <!-- Shoulder / Trigger Bar -->
  <div id="shoulder-panel">
    <div class="shoulder-group">
      <button class="trigger-btn" id="btn-lt" data-axis="4" data-val="-1.0">LT<span>SPD -</span></button>
      <button class="trigger-btn" id="btn-lb" data-bit="4">LB<span>TURN -</span></button>
    </div>

    <!-- Live Speed HUD -->
    <div id="speed-hud">
      <div class="hud-item">
        <span class="hud-label">Linear</span>
        <span class="hud-val" id="hud-lin">0.25 m/s</span>
      </div>
      <div class="hud-item">
        <span class="hud-label">Angular</span>
        <span class="hud-val" id="hud-ang">0.80 rad/s</span>
      </div>
    </div>

    <div class="shoulder-group">
      <button class="trigger-btn" id="btn-rb" data-bit="5">RB<span>TURN +</span></button>
      <button class="trigger-btn" id="btn-rt" data-axis="5" data-val="-1.0">RT<span>SPD +</span></button>
    </div>
  </div>

  <!-- Main Gamepad Workspace -->
  <div id="workspace">

    <!-- Left Stick (Drive: Forward/Back + Strafe) -->
    <div class="stick-container">
      <div class="stick-base" id="left-stick-base">
        <div class="stick-cross"></div>
        <div class="stick-cross v"></div>
        <div class="stick-thumb" id="left-stick-thumb"></div>
      </div>
      <span class="stick-label">MOVE (FWD / STRAFE)</span>
    </div>

    <!-- Center Area: D-Pad & Utility Buttons -->
    <div id="center-column">
      <div class="middle-group">
        <button class="sys-btn" id="btn-back" data-bit="6">☰ BACK</button>
        <button class="sys-btn" id="btn-start" data-bit="7">START ≡</button>
      </div>

      <!-- D-Pad (Camera Snapping) -->
      <div class="dpad-grid">
        <div></div>
        <button class="dpad-btn" id="dpad-up" data-dv="1.0" data-dh="0.0">▲</button>
        <div></div>
        <button class="dpad-btn" id="dpad-left" data-dv="0.0" data-dh="1.0">◀</button>
        <div class="dpad-center"></div>
        <button class="dpad-btn" id="dpad-right" data-dv="0.0" data-dh="-1.0">▶</button>
        <div></div>
        <button class="dpad-btn" id="dpad-down" data-dv="-1.0" data-dh="0.0">▼</button>
        <div></div>
      </div>
    </div>

    <!-- Right Side: ABXY & Camera Pan/Tilt -->
    <div class="face-cluster">
      <div class="face-grid">
        <button class="face-btn" id="btn-y" data-bit="3">Y<span>CAL</span></button>
        <button class="face-btn" id="btn-b" data-bit="1">B<span>MAP</span></button>
        <button class="face-btn" id="btn-x" data-bit="2">X<span>VIS</span></button>
        <button class="face-btn" id="btn-a" data-bit="0">A<span>SEQ</span></button>
      </div>
    </div>

    <!-- Right Stick (Camera Pan & Tilt) -->
    <div class="stick-container">
      <div class="stick-base" id="right-stick-base">
        <div class="stick-cross"></div>
        <div class="stick-cross v"></div>
        <div class="stick-thumb" id="right-stick-thumb"></div>
      </div>
      <span class="stick-label">ROTATE / CAMERA</span>
    </div>

  </div>

<script>
// ── State Representation ──────────────────────────────────────────────────────
const axes = new Float32Array(8);
let buttonsMask = 0;
let ws = null;
let streamInterval = null;
let useHttpFallback = false;

const pulseDot = document.getElementById('pulse-dot');
const connText = document.getElementById('conn-text');
const ipBadge  = document.getElementById('ip-badge');
const hudLin   = document.getElementById('hud-lin');
const hudAng   = document.getElementById('hud-ang');

let curLin = 0.25;
let curAng = 0.80;

function vibrate(ms = 20) {
  if (navigator.vibrate) {
    try { navigator.vibrate(ms); } catch (e) {}
  }
}

// ── Connection: WebSocket with HTTP POST Fallback ─────────────────────────────
function connectWebSocket() {
  const host = location.host || (location.hostname + ':8765');
  ipBadge.textContent = host;
  const wsUrl = (location.protocol === 'https:' ? 'wss://' : 'ws://') + host + '/ws';

  try {
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      useHttpFallback = false;
      pulseDot.className = 'pulse-dot online';
      connText.textContent = 'ONLINE (WS 30Hz)';
      if (streamInterval) clearInterval(streamInterval);
      streamInterval = setInterval(sendState, 33);
      vibrate(40);
    };

    ws.onclose = () => {
      ws = null;
      startHttpFallback();
      setTimeout(connectWebSocket, 2000);
    };

    ws.onerror = () => {
      if (ws) ws.close();
    };
  } catch (err) {
    startHttpFallback();
  }
}

function startHttpFallback() {
  if (useHttpFallback) return;
  useHttpFallback = true;
  pulseDot.className = 'pulse-dot online';
  connText.textContent = 'ONLINE (HTTP 20Hz)';
  if (streamInterval) clearInterval(streamInterval);
  streamInterval = setInterval(sendState, 50);
}

function sendState() {
  const payload = JSON.stringify({
    a: Array.from(axes),
    b: buttonsMask
  });

  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(payload);
  } else if (useHttpFallback) {
    fetch('/api/joy', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: payload
    }).catch(() => {});
  }
}

// ── Pointer-Captured Virtual Joysticks ────────────────────────────────────────
function setupJoystick(baseId, thumbId, axisXIdx, axisYIdx, invertX = false, invertY = false) {
  const base  = document.getElementById(baseId);
  const thumb = document.getElementById(thumbId);
  let activePointerId = null;

  function handlePointer(clientX, clientY) {
    const rect = base.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const maxRadius = (rect.width - thumb.offsetWidth) / 2;

    const dx = clientX - centerX;
    const dy = clientY - centerY;
    const dist = Math.hypot(dx, dy);
    const angle = Math.atan2(dy, dx);

    const clampedDist = Math.min(dist, maxRadius);
    const clampedX = Math.cos(angle) * clampedDist;
    const clampedY = Math.sin(angle) * clampedDist;

    thumb.style.transform = `translate(calc(-50% + ${clampedX}px), calc(-50% + ${clampedY}px))`;

    let normX = clampedX / maxRadius;
    let normY = clampedY / maxRadius;

    axes[axisXIdx] = invertX ? normX : -normX;
    axes[axisYIdx] = invertY ? normY : -normY;
  }

  function resetStick() {
    activePointerId = null;
    base.classList.remove('active');
    thumb.style.transform = 'translate(-50%, -50%)';
    axes[axisXIdx] = 0.0;
    axes[axisYIdx] = 0.0;
  }

  base.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    activePointerId = e.pointerId;
    base.setPointerCapture(e.pointerId);
    base.classList.add('active');
    handlePointer(e.clientX, e.clientY);
    vibrate(15);
  });

  base.addEventListener('pointermove', (e) => {
    if (e.pointerId === activePointerId) {
      e.preventDefault();
      handlePointer(e.clientX, e.clientY);
    }
  });

  base.addEventListener('pointerup', (e) => {
    if (e.pointerId === activePointerId) {
      e.preventDefault();
      try { base.releasePointerCapture(e.pointerId); } catch (err) {}
      resetStick();
    }
  });

  base.addEventListener('pointercancel', (e) => {
    if (e.pointerId === activePointerId) {
      resetStick();
    }
  });
}

// Left stick: Translate (Strafe left/right, Forward/Back)
setupJoystick('left-stick-base', 'left-stick-thumb', 0, 1, false, false);

// Right stick: Rotate + Camera Tilt
setupJoystick('right-stick-base', 'right-stick-thumb', 2, 3, false, true);

// ── Standard Button Binding with PointerCapture ───────────────────────────────
function bindButton(btn) {
  const bit = btn.dataset.bit != null ? parseInt(btn.dataset.bit, 10) : null;
  const axisIdx = btn.dataset.axis != null ? parseInt(btn.dataset.axis, 10) : null;
  const axisVal = btn.dataset.val != null ? parseFloat(btn.dataset.val) : 0.0;
  let activePointer = null;

  function press(pressed) {
    btn.classList.toggle('active', pressed);
    if (bit != null) {
      if (pressed) buttonsMask |= (1 << bit);
      else buttonsMask &= ~(1 << bit);
    }
    if (axisIdx != null) {
      axes[axisIdx] = pressed ? axisVal : 0.0;
    }
  }

  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    activePointer = e.pointerId;
    btn.setPointerCapture(e.pointerId);
    press(true);
    vibrate(25);

    if (btn.id === 'btn-rt') {
      curLin = Math.min(0.50, +(curLin + 0.05).toFixed(2));
      hudLin.textContent = curLin.toFixed(2) + ' m/s';
    } else if (btn.id === 'btn-lt') {
      curLin = Math.max(0.05, +(curLin - 0.05).toFixed(2));
      hudLin.textContent = curLin.toFixed(2) + ' m/s';
    } else if (btn.id === 'btn-rb') {
      curAng = Math.min(2.0, +(curAng + 0.10).toFixed(2));
      hudAng.textContent = curAng.toFixed(2) + ' rad/s';
    } else if (btn.id === 'btn-lb') {
      curAng = Math.max(0.20, +(curAng - 0.10).toFixed(2));
      hudAng.textContent = curAng.toFixed(2) + ' rad/s';
    }
  });

  btn.addEventListener('pointerup', (e) => {
    if (e.pointerId === activePointer) {
      e.preventDefault();
      try { btn.releasePointerCapture(e.pointerId); } catch (err) {}
      activePointer = null;
      press(false);
    }
  });

  btn.addEventListener('pointercancel', () => {
    activePointer = null;
    press(false);
  });
}

document.querySelectorAll('.trigger-btn, .face-btn, .sys-btn').forEach(bindButton);

// ── D-Pad (Camera Snap-to-Limit) ──────────────────────────────────────────────
const dpadButtons = document.querySelectorAll('.dpad-btn');
function clearDpad() {
  axes[6] = 0.0;
  axes[7] = 0.0;
  dpadButtons.forEach(b => b.classList.remove('active'));
}

dpadButtons.forEach(btn => {
  const dh = parseFloat(btn.dataset.dh || 0.0);
  const dv = parseFloat(btn.dataset.dv || 0.0);
  let pId = null;

  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    pId = e.pointerId;
    btn.setPointerCapture(e.pointerId);
    clearDpad();
    axes[6] = dh;
    axes[7] = dv;
    btn.classList.add('active');
    vibrate(30);
  });

  btn.addEventListener('pointerup', (e) => {
    if (e.pointerId === pId) {
      e.preventDefault();
      try { btn.releasePointerCapture(e.pointerId); } catch (err) {}
      clearDpad();
    }
  });

  btn.addEventListener('pointercancel', () => clearDpad());
});

// ── Emergency Stop Button ─────────────────────────────────────────────────────
const estopBtn = document.getElementById('btn-estop');
estopBtn.addEventListener('pointerdown', (e) => {
  e.preventDefault();
  vibrate(100);
  for (let i = 0; i < 8; i++) axes[i] = 0.0;
  buttonsMask = 0;
  document.getElementById('left-stick-thumb').style.transform = 'translate(-50%, -50%)';
  document.getElementById('right-stick-thumb').style.transform = 'translate(-50%, -50%)';
  document.querySelectorAll('.active').forEach(el => el.classList.remove('active'));
  sendState();
});

window.addEventListener('DOMContentLoaded', connectWebSocket);
</script>
</body>
</html>
"""


# ── Shared Node Reference for HTTP Handler ────────────────────────────────────
_NODE_INSTANCE = None


class GamepadHTTPRequestHandler(BaseHTTPRequestHandler):
    """
    Standard Library HTTP Request Handler supporting:
      1. GET /         → Serves virtual gamepad HTML interface
      2. GET /ws       → RFC 6455 WebSocket Upgrade
      3. POST /api/joy → HTTP POST state update fallback
    """

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            content = _GAMEPAD_HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/ws':
            self._handle_websocket_upgrade()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/api/joy':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length) if length > 0 else b'{}'
            if _NODE_INSTANCE is not None:
                _NODE_INSTANCE.process_payload(body.decode('utf-8', errors='ignore'))
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        else:
            self.send_response(404)
            self.end_headers()

    def _handle_websocket_upgrade(self):
        """RFC 6455 WebSocket Handshake & Masked Frame Parser."""
        key = self.headers.get('Sec-WebSocket-Key', '')
        if not key:
            self.send_response(400)
            self.end_headers()
            return

        guid = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
        accept_key = base64.b64encode(hashlib.sha1((key.strip() + guid).encode('utf-8')).digest()).decode('utf-8')

        self.send_response(101, 'Switching Protocols')
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', accept_key)
        self.end_headers()

        sock = self.request
        sock.settimeout(5.0)

        if _NODE_INSTANCE is not None:
            _NODE_INSTANCE.on_client_connected(self.client_address)

        try:
            while True:
                head = sock.recv(2)
                if not head or len(head) < 2:
                    break
                b1, b2 = head[0], head[1]
                opcode = b1 & 0x0F

                # Close frame
                if opcode == 8:
                    break
                # Ping frame → reply with pong
                elif opcode == 9:
                    sock.sendall(bytes([0x8A, 0x00]))
                    continue

                payload_len = b2 & 0x7F
                if payload_len == 126:
                    ext = sock.recv(2)
                    if len(ext) < 2:
                        break
                    payload_len = struct.unpack('>H', ext)[0]
                elif payload_len == 127:
                    ext = sock.recv(8)
                    if len(ext) < 8:
                        break
                    payload_len = struct.unpack('>Q', ext)[0]

                mask = sock.recv(4)
                if len(mask) < 4:
                    break

                payload = bytearray()
                while len(payload) < payload_len:
                    chunk = sock.recv(min(payload_len - len(payload), 4096))
                    if not chunk:
                        break
                    payload.extend(chunk)

                unmasked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))

                # Text frame
                if opcode == 1 and _NODE_INSTANCE is not None:
                    _NODE_INSTANCE.process_payload(unmasked.decode('utf-8', errors='ignore'))
        except (socket.timeout, ConnectionResetError, BrokenPipeError, OSError):
            pass
        finally:
            if _NODE_INSTANCE is not None:
                _NODE_INSTANCE.on_client_disconnected()

    def log_message(self, format, *args):
        # Suppress noisy HTTP access logs
        pass


# ── ROS 2 Web Gamepad Node ────────────────────────────────────────────────────
class WebGamepadNode(Node):
    """
    Hosts the virtual Xbox controller web interface and publishes /joy
    at a steady 30 Hz using standard library only.
    """

    def __init__(self):
        super().__init__('web_gamepad')
        global _NODE_INSTANCE
        _NODE_INSTANCE = self

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('publish_hz', 30.0)

        self._host = self.get_parameter('host').value
        self._port = int(self.get_parameter('port').value)
        self._publish_hz = float(self.get_parameter('publish_hz').value)

        self._pub = self.create_publisher(Joy, '/joy', 10)

        # State storage
        self._axes = [0.0] * 8
        self._buttons_mask = 0
        self._last_rx_time = 0.0
        self._client_connected = False
        self._had_active_motion = False
        self._lock = threading.Lock()

        # Periodic ROS 2 publisher timer
        self._pub_timer = self.create_timer(1.0 / self._publish_hz, self._timer_publish_joy)

        # Start standard library ThreadingHTTPServer
        self._server = ThreadingHTTPServer((self._host, self._port), GamepadHTTPRequestHandler)
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True, name='web_server')
        self._server_thread.start()

        # Log connection banner to terminal
        self._log_banner()

    def _log_banner(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            local_ip = '127.0.0.1'

        print(
            f'\n\033[1;36m'
            f'╔═══════════════════════════════════════════════════════════════╗\n'
            f'║ 🎮 WEB GAMEPAD READY (Port {self._port})                              ║\n'
            f'╠───────────────────────────────────────────────────────────────╣\n'
            f'║  Local / Wi-Fi URL:  http://{local_ip}:{self._port:<5}                  ║\n'
            f'║  Tailscale URL:      http://<tailscale-ip>:{self._port:<5}            ║\n'
            f'║  Open either address on your laptop or phone browser!         ║\n'
            f'╚═══════════════════════════════════════════════════════════════╝\033[0m\n',
            flush=True,
        )

    def on_client_connected(self, client_addr):
        with self._lock:
            self._client_connected = True
        self.get_logger().info(f'Web Controller connected from {client_addr}')

    def on_client_disconnected(self):
        with self._lock:
            self._client_connected = False
            self._axes = [0.0] * 8
            self._buttons_mask = 0
        self.get_logger().info('Web Controller disconnected')

    def process_payload(self, text: str):
        try:
            data = json.loads(text)
            raw_axes = data.get('a', [0.0] * 8)
            buttons = int(data.get('b', 0))

            padded_axes = [float(v) for v in raw_axes[:8]]
            while len(padded_axes) < 8:
                padded_axes.append(0.0)

            with self._lock:
                self._axes = padded_axes
                self._buttons_mask = buttons
                self._last_rx_time = time.monotonic()
                self._client_connected = True
        except (json.JSONDecodeError, TypeError, ValueError):
            pass

    def _timer_publish_joy(self):
        now = time.monotonic()
        with self._lock:
            is_fresh = self._client_connected and ((now - self._last_rx_time) < 1.5)
            axes = list(self._axes)
            buttons_mask = self._buttons_mask

        if is_fresh:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = axes
            msg.buttons = [(buttons_mask >> i) & 1 for i in range(11)]
            self._pub.publish(msg)
            self._had_active_motion = True
        elif self._had_active_motion:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [0.0] * 8
            msg.buttons = [0] * 11
            self._pub.publish(msg)
            self._had_active_motion = False

    def shutdown(self):
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WebGamepadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
