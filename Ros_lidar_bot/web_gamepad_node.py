#!/usr/bin/env python3
"""
web_gamepad_node.py — Virtual Xbox gamepad served directly from the RPi5.

Hosts a high-performance, mobile-optimized virtual Xbox controller webpage
(HTTP + WebSocket) directly on the RPi5. Accessible from any phone on the
same Wi-Fi or Tailscale network without needing a physical controller plugged in.

Key Architectural Improvements:
  1. Decoupled ROS 2 Publishing:
     A dedicated ROS 2 timer publishes /joy at a rock-solid 30 Hz. WebSocket
     frames update node state asynchronously, completely eliminating packet-drop
     stutters and false timeouts caused by Wi-Fi latency.
  2. Pointer Events & Multi-Touch:
     Uses Pointer Events with setPointerCapture() so driving with the left thumb
     while operating camera or buttons with the right thumb works with 100%
     independence. Gestures (pinch, scroll, swipe) cannot hijack or reset stick input.
  3. Correct ROS 2 Conventions:
     - Left stick horizontal: Left = +1.0 (turn left), Right = -1.0 (turn right).
     - Left stick vertical: Up = +1.0 (forward), Down = -1.0 (reverse).
     - Camera right stick: Up = +1.0 (tilt up), Down = -1.0 (tilt down).
     - Camera pan: Left = +1.0 (pan left), Right = -1.0 (pan right).
     - Matching Xbox button indices:
       0=A (seq), 1=B (save map), 2=X (vision), 3=Y (imu cal),
       4=LB (turn-), 5=RB (turn+), 6=Back, 7=Start, 8=Xbox/Stop.
  4. Live HUD & E-STOP:
     Interactive speed controls with live visual feedback, haptic vibration,
     and an instant Emergency Stop.
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

# ── High-Tech Cyberpunk Mobile Gamepad HTML ───────────────────────────────────
_GAMEPAD_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover"/>
<title>RPi5 Bot Controller</title>
<style>
  :root {
    --bg-primary: #0b0f19;
    --bg-card: rgba(18, 24, 38, 0.75);
    --border-card: rgba(255, 255, 255, 0.12);
    --cyan: #00f0ff;
    --blue: #3b82f6;
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

  /* ── Top Status Bar ── */
  #header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 16px;
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
    font-size: 11px;
    color: var(--text-dim);
    font-family: monospace;
  }
  .estop-btn {
    background: linear-gradient(135deg, #ef4444, #991b1b);
    color: #fff;
    border: 1px solid rgba(255, 255, 255, 0.3);
    border-radius: 8px;
    padding: 6px 14px;
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
    padding: 6px 14px 0 14px;
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
    gap: 12px;
    background: rgba(0, 0, 0, 0.4);
    padding: 6px 14px;
    border-radius: 20px;
    border: 1px solid rgba(255, 255, 255, 0.08);
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

  /* ── Center D-Pad & Actions ── */
  #center-column {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 16px;
  }

  /* D-Pad */
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

  /* Middle System Buttons */
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
    <div class="ip-info" id="ip-badge">HOST: --</div>
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

    <!-- Left Stick (Drive: Linear + Angular) -->
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
// axes: [0: LX, 1: LY, 2: RX, 3: RY, 4: LT, 5: RT, 6: DX, 7: DY]
const axes = new Float32Array(8);
let buttonsMask = 0;
let ws = null;
let streamInterval = null;

const pulseDot  = document.getElementById('pulse-dot');
const connText  = document.getElementById('conn-text');
const ipBadge   = document.getElementById('ip-badge');
const hudLin    = document.getElementById('hud-lin');
const hudAng    = document.getElementById('hud-ang');

// Local speed state tracker for UI
let curLin = 0.25;
let curAng = 0.80;

function vibrate(ms = 20) {
  if (navigator.vibrate) {
    try { navigator.vibrate(ms); } catch (e) {}
  }
}

// ── WebSocket Client ──────────────────────────────────────────────────────────
function connectWebSocket() {
  const host = location.host || 'localhost:8765';
  ipBadge.textContent = host;
  const wsUrl = (location.protocol === 'https:' ? 'wss://' : 'ws://') + host + '/ws';

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    pulseDot.className = 'pulse-dot online';
    connText.textContent = 'ONLINE (30Hz)';
    if (streamInterval) clearInterval(streamInterval);
    streamInterval = setInterval(sendState, 33); // 30 Hz stream
    vibrate(40);
  };

  ws.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      if (data.lin_speed != null) {
        curLin = data.lin_speed;
        hudLin.textContent = curLin.toFixed(2) + ' m/s';
      }
      if (data.ang_speed != null) {
        curAng = data.ang_speed;
        hudAng.textContent = curAng.toFixed(2) + ' rad/s';
      }
    } catch (e) {}
  };

  ws.onclose = () => {
    pulseDot.className = 'pulse-dot offline';
    connText.textContent = 'RECONNECTING…';
    if (streamInterval) clearInterval(streamInterval);
    setTimeout(connectWebSocket, 1200);
  };

  ws.onerror = () => {
    ws.close();
  };
}

function sendState() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({
      a: Array.from(axes),
      b: buttonsMask
    }));
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

    // Normalised [-1.0 .. 1.0]
    let normX = clampedX / maxRadius;
    let normY = clampedY / maxRadius;

    // Apply polarity:
    // For ROS linear: Up = +1.0, Down = -1.0. (Screen coords have down = +y, so negate normY).
    // For ROS angular: Left = +1.0, Right = -1.0. (Screen coords have right = +x, so negate normX).
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

// ── Setup Left & Right Sticks ─────────────────────────────────────────────────
// Left stick: Drive
//   axis 0 (LX): Left = +1.0 (turn left), Right = -1.0 (turn right)
//   axis 1 (LY): Up = +1.0 (forward), Down = -1.0 (reverse)
setupJoystick('left-stick-base', 'left-stick-thumb', 0, 1, false, false);

// Right stick: Camera
//   axis 2 (RX): Left = +1.0 (pan left), Right = -1.0 (pan right)
//   axis 3 (RY): Up = -1.0 (SDL style: joy_teleop inverts it to tilt up)
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

    // Speed adjustment feedback simulation
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

// Bind all standard action and shoulder buttons
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

// Initialize connection on page load
window.addEventListener('DOMContentLoaded', connectWebSocket);
</script>
</body>
</html>
"""


# ── ROS 2 Web Gamepad Node ────────────────────────────────────────────────────
class WebGamepadNode(Node):
    """
    Hosts the virtual Xbox controller web interface and publishes /joy
    at a rock-solid, periodic rate to completely eliminate wireless stutters.
    """

    def __init__(self):
        super().__init__('web_gamepad')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('publish_hz', 30.0)

        self._host = self.get_parameter('host').value
        self._port = self.get_parameter('port').value
        self._publish_hz = float(self.get_parameter('publish_hz').value)

        self._pub = self.create_publisher(Joy, '/joy', 10)

        # Internal state updated by WebSocket and published periodically by ROS timer
        self._axes = [0.0] * 8
        self._buttons_mask = 0
        self._last_rx_time = 0.0
        self._client_connected = False
        self._had_active_motion = False

        # Periodic ROS 2 publisher timer (eliminates network rate-limiting jitter)
        self._pub_timer = self.create_timer(1.0 / self._publish_hz, self._timer_publish_joy)

        # aiohttp web server objects
        self._app = None
        self._runner = None
        self._loop = None
        self._ws_clients = set()

        if not _AIOHTTP_OK:
            self.get_logger().error(
                'aiohttp is not installed! Run:\n'
                '  sudo apt install -y python3-aiohttp\n'
                'or\n'
                '  pip install aiohttp --break-system-packages')
            return

        # Start aiohttp in a dedicated background daemon thread
        self._web_thread = threading.Thread(
            target=self._run_server, daemon=True, name='web_gamepad_server')
        self._web_thread.start()

        # Log connection information
        self._log_timer = self.create_timer(1.0, self._log_url)

    # ── Periodic ROS 2 Publisher Timer ─────────────────────────────────────────
    def _timer_publish_joy(self):
        """
        Runs at steady publish_hz (e.g. 30 Hz).
        Publishes the latest received controller state if client is active,
        or ensures zero stop commands are sent if client disconnects.
        """
        now = time.monotonic()
        is_fresh = (self._client_connected and (now - self._last_rx_time) < 1.5)

        if is_fresh:
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = list(self._axes)
            msg.buttons = [(self._buttons_mask >> i) & 1 for i in range(11)]
            self._pub.publish(msg)
            self._had_active_motion = True
        elif self._had_active_motion:
            # Client timed out or disconnected mid-motion: publish safe-stop
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [0.0] * 8
            msg.buttons = [0] * 11
            self._pub.publish(msg)
            self._had_active_motion = False
            self.get_logger().warn('Web gamepad stream paused/disconnected — sent safe stop',
                                   throttle_duration_sec=3.0)

    # ── aiohttp Server ─────────────────────────────────────────────────────────
    def _run_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._start_app())
        self._loop.run_forever()

    async def _start_app(self):
        self._app = web.Application()
        self._app.router.add_get('/', self._handle_index)
        self._app.router.add_get('/ws', self._handle_ws)

        self._runner = web.AppRunner(self._app)
        await self._runner.setup()
        site = web.TCPSite(self._runner, self._host, self._port)
        await site.start()

    def _log_url(self):
        self.destroy_timer(self._log_timer)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            local_ip = '127.0.0.1'

        self.get_logger().info(
            f'\n╔═══════════════════════════════════════════════════════════════╗\n'
            f'║ 🎮 WEB GAMEPAD READY (Listening on 0.0.0.0:{self._port})             ║\n'
            f'╠───────────────────────────────────────────────────────────────╣\n'
            f'║  Wi-Fi Network URL:   http://{local_ip}:{self._port:<5}                   ║\n'
            f'║  Tailscale Network:   http://<tailscale-ip>:{self._port:<5}             ║\n'
            f'║  Open either address on your mobile phone browser to drive!   ║\n'
            f'╚═══════════════════════════════════════════════════════════════╝')

    # ── HTTP & WebSocket Handlers ──────────────────────────────────────────────
    async def _handle_index(self, _request):
        return web.Response(text=_GAMEPAD_HTML, content_type='text/html', charset='utf-8')

    async def _handle_ws(self, request):
        # heartbeat=3.0 sends ping frames every 3s to keep wireless connection alive
        ws_resp = web.WebSocketResponse(heartbeat=3.0)
        await ws_resp.prepare(request)
        self._ws_clients.add(ws_resp)
        self._client_connected = True
        self.get_logger().info(f'Phone connected from {request.remote}')

        try:
            async for msg in ws_resp:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        raw_axes = data.get('a', [0.0] * 8)
                        self._buttons_mask = int(data.get('b', 0))

                        # Ensure 8 axes: [LX, LY, RX, RY, LT, RT, DX, DY]
                        padded_axes = [float(v) for v in raw_axes[:8]]
                        while len(padded_axes) < 8:
                            padded_axes.append(0.0)
                        self._axes = padded_axes
                        self._last_rx_time = time.monotonic()
                    except (json.JSONDecodeError, TypeError, ValueError):
                        pass
                elif msg.type in (web.WSMsgType.ERROR, web.WSMsgType.CLOSE):
                    break
        except Exception as exc:
            self.get_logger().warn(f'WebSocket connection notice: {exc}')
        finally:
            self._ws_clients.discard(ws_resp)
            if not self._ws_clients:
                self._client_connected = False
                self._axes = [0.0] * 8
                self._buttons_mask = 0
            self.get_logger().info('Phone disconnected')

        return ws_resp

    # ── Clean Shutdown ────────────────────────────────────────────────────────
    def shutdown_server(self):
        if self._runner and self._loop and self._loop.is_running():
            try:
                asyncio.run_coroutine_threadsafe(
                    self._runner.cleanup(), self._loop).result(timeout=2.0)
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
        try:
            node.shutdown_server()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
