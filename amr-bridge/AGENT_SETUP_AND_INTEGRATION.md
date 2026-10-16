# AMR cloud bridge — setup & integration guide (for agents)

This document is the **complete** playbook for connecting a Pi robot
(`Ros_lidar_bot` + `robot_dashboard_node`) to **https://amrslam.tech/control**.

**Audience:** another coding / ops agent integrating this into the robot repo.
**Website repo:** `amey1942007/AMR` (this package lives at `amr-bridge/`).
**Robot repo:** `amey1942007/Ros_lidar_bot`.

---

## 0. What is already done (do not redo)

| Layer | Status |
| --- | --- |
| Site `/control` UI (2D LiveMap, status Hz/age, tools, Xbox fullscreen) | Merged to `main` |
| Supabase project `khucmrlbstdlbxcuboub` | Live |
| Tables: `amr_bots`, `operator_profiles`, `permitted_operators`, `amr_bridge_config` | Applied |
| RPC `touch_amr_bot` (bridge heartbeat) | Applied |
| Edge Function `operator-auth` (invite-only username login) | Deployed |
| Auth users / allowlist (`amey` Owner, operators) | Seeded |
| Protocol TS ↔ Python | In sync (`src/lib/telemetry/protocol.ts` ↔ `amr_telemetry_bridge/protocol.py`) |

**You only integrate the Pi side.** No website code changes are required for a
normal connect.

---

## 1. Architecture (one paragraph)

```
browser  https://amrslam.tech/control
    │  Supabase Auth + Realtime broadcast
    ▼
Supabase Realtime channel  amr-bot-{amr4|hardware-bot}
    ▲ telemetry / map / hb / log          │ command
    │                                     ▼
Pi: amr_telemetry_bridge  ◄──HTTP──►  robot_dashboard_node :8080
         │                              (SSE /events, /api/*)
         └─ optional ──HTTP──►  web_gamepad_node :8765  (/api/joy → ROS /joy)
```

- Bridge **does not** talk ROS directly. It scrapes the local dashboard HTTP API.
- Dashboard must be up on `AMR_DASHBOARD_URL` (default `http://127.0.0.1:8080`).
- Offline threshold on the site: **3 s** without heartbeat / telemetry.

---

## 2. Prerequisites on the Pi

1. ROS 2 Humble (or whatever `Ros_lidar_bot` uses) workspace builds.
2. `robot_dashboard_node` launches and serves:
   - `GET /events` (SSE state + map + log)
   - `POST /api/estop|nudge|nav_goal|nav_cancel|vision|save_map|run|stop|blacklist`
3. Optional but recommended for Xbox pad parity:
   - `web_gamepad_node` on port **8765** with `POST /api/joy` body `{"a":[8 floats],"b":int}`.
4. Outbound HTTPS from Pi → `*.supabase.co` (Realtime + REST). Tailscale optional for ops SSH; **not** required for `/control` if Supabase is reachable.
5. Python 3.10+ with venv capability.

---

## 3. Files to copy from this repo (`amr-bridge/`)

| Source (website repo) | Destination on Pi |
| --- | --- |
| `amr_telemetry_bridge/` | Installable package (see §4) |
| `requirements.txt` | Same |
| `launch/amr_bridge.launch.py` | `Ros_lidar_bot/launch/amr_bridge.launch.py` |
| `scripts/run_bridge.sh` | Optional helper / systemd |
| `.env.pi` (secrets — **do not commit**) | `/etc/amr-bridge.env` mode `600` |
| `.env.pi.example` | Template only |

Also read (do not invent APIs):

- `launch/INCLUDE_IN_ROSLIDAR.md` — short include snippet
- `README.md` — overview
- This file — full agent playbook

---

## 4. Install Python bridge on the Pi

```bash
# Example layout — adjust to the robot workspace
cd ~/ros2_ws/src
# Option A: clone/copy amr-bridge next to Ros_lidar_bot
cp -a /path/to/AMR/amr-bridge ~/amr-bridge

cd ~/amr-bridge
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
# Make the package importable:
pip install -e .
# OR: export PYTHONPATH=~/amr-bridge:$PYTHONPATH
```

If the package has no `pyproject.toml` / `setup.py`, keep using:

```bash
export PYTHONPATH="/home/pi/amr-bridge${PYTHONPATH:+:$PYTHONPATH}"
python -m amr_telemetry_bridge
```

`amr_bridge.launch.py` runs `python3 -m amr_telemetry_bridge` — that Python must
see the package and have `supabase`, `httpx`, `websockets` installed
(`requirements.txt`).

**Recommended:** point the launch `python` argument at the venv:

```text
python:=/home/pi/amr-bridge/.venv/bin/python
```

---

## 5. Secrets: `/etc/amr-bridge.env`

```bash
# On a machine that has the private .env.pi from the AMR website repo:
sudo install -m 600 amr-bridge/.env.pi /etc/amr-bridge.env
```

Required keys:

```bash
AMR_BOT_ID=hardware-bot            # must match the site bot picker
AMR_SUPABASE_URL=https://….supabase.co
AMR_SUPABASE_KEY=sb_publishable_…  # publishable key — NOT service_role
AMR_BRIDGE_TOKEN=…                 # must match sha256 in public.amr_bridge_config
AMR_BRIDGE_EMAIL=…                 # bridge Auth user (private channels)
AMR_BRIDGE_PASSWORD=…              # never commit; env file only
AMR_PUBLIC_CHANNEL=0               # 1 enables the legacy public channel
AMR_DASHBOARD_URL=http://127.0.0.1:8080
```

`AMR_BRIDGE_EMAIL` / `AMR_BRIDGE_PASSWORD` sign the bridge in so it can join
`bot:<id>:status|cmd|cmdq|ctl`. Without them, private channels stay off.
`AMR_PUBLIC_CHANNEL` defaults to off, so the legacy public channel stays off
too unless it is set to `1`. Do not set `AMR_SUPABASE_SERVICE_KEY`.

Optional:

```bash
AMR_CAMERA_URL=https://…/camera.mjpg   # public MJPEG if browser can't scrape Pi
AMR_CAMERA_JPEG_MAX_KB=24              # inline JPEG budget when scraping /camera.mjpg
AMR_GAMEPAD_URL=http://127.0.0.1:8765  # web_gamepad_node — full Xbox /joy
AMR_PRIVATE_CHANNEL=1                  # 0 disables private topics
AMR_CONTROL_POLL_S=2                   # bot_control poll, clamped to 2–5 seconds
```

**Agent rules for secrets**

- Never print `AMR_BRIDGE_TOKEN`, `AMR_BRIDGE_PASSWORD`, or keys into chat logs, PRs, or commits.
- Never put bridge token / service role into `NEXT_PUBLIC_*`.
- If token is lost: regenerate, store new plaintext only on Pi, update
  `public.amr_bridge_config.token_sha256` in Supabase (sha256 of token).

---

## 6. Integrate into Ros_lidar_bot launch

### 6.1 Copy launch file

```bash
cp amr-bridge/launch/amr_bridge.launch.py \
  ~/ros2_ws/src/Ros_lidar_bot/launch/amr_bridge.launch.py
# rebuild / source so share path resolves if you use get_package_share_directory
cd ~/ros2_ws && colcon build --packages-select Ros_lidar_bot && source install/setup.bash
```

### 6.2 Include from bringup (required pattern)

Add **after** the dashboard node is started; delay ~5 s so `:8080` is listening:

```python
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

bridge = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('Ros_lidar_bot'),
            'launch',
            'amr_bridge.launch.py',
        )
    ),
    launch_arguments={
        'bot_id': 'amr4',                       # or hardware-bot
        'env_file': '/etc/amr-bridge.env',
        'dashboard': 'http://127.0.0.1:8080',
        'python': '/home/pi/amr-bridge/.venv/bin/python',  # adjust path
    }.items(),
)

delayed_bridge = TimerAction(period=5.0, actions=[bridge])
# Add delayed_bridge to LaunchDescription(...) next to dashboard.
```

### 6.3 Standalone smoke (no full stack)

```bash
# Terminal A — dashboard already running on :8080
# Terminal B:
source ~/amr-bridge/.venv/bin/activate
export PYTHONPATH=~/amr-bridge:$PYTHONPATH
ros2 launch Ros_lidar_bot amr_bridge.launch.py bot_id:=amr4
# or:
AMR_ENV_FILE=/etc/amr-bridge.env ~/amr-bridge/scripts/run_bridge.sh
```

Healthy logs look like:

```text
[amr_bridge] starting bot_id=amr4 env_file=/etc/amr-bridge.env
amr_telemetry_bridge 0.1.0 starting (bot=amr4)
realtime subscribed (amr-bot-amr4)
connecting SSE http://127.0.0.1:8080/events
```

Missing env → launch prints `[amr_bridge] missing env [...]` and **does not** start.

---

## 7. Bot ID mapping

| `AMR_BOT_ID` / launch `bot_id` | Site label | Realtime channel |
| --- | --- | --- |
| `amr4` | AMR4 | `amr-bot-amr4` |
| `hardware-bot` | Hardware bot | `amr-bot-hardware-bot` |

Run **one bridge process per robot**. Wrong id → site shows offline for the
selected bot even if the other channel is live.

---

## 8. Dashboard HTTP contract (must not break)

Bridge depends on these. If you rename paths in `robot_dashboard_node`, update
`amr_telemetry_bridge/bridge.py` in lockstep.

### Inbound (Pi → browser via bridge)

| Dashboard | Bridge action |
| --- | --- |
| `GET /events` SSE `t=state` (has `pose`) | Broadcast `telemetry` (rate-limited 10 Hz full / 1 Hz board) |
| SSE `t=map` | Broadcast `map` |
| SSE `t=log` | Broadcast `log` |
| `GET /camera.mjpg` | Occasional JPEG → `cameraJpegB64` when no `AMR_CAMERA_URL` |

Important state fields forwarded (subset): `pose`, `map_pose`, `vel`, `cmd`,
`imu`, `scan` as `[angle_rad, range_m]`, `rates` (`hz` + `age`), `nodes`, `tf`,
`nav`, `path`, `goal`, `yolo_det`→`yoloDet`, `sem`, `frontier`, `sys`, `tool`,
`vision`, `saving`.

### Outbound (browser command → dashboard)

Private-channel commands that pass epoch / token / lease / seq checks skip
the rank gates. E-stop skips seq, timestamp, and lease. The public channel
still applies the rank gates while `AMR_PUBLIC_CHANNEL=1` (default is off).
That setting reopens a spoofable rank (the client writes it) and those
commands have no 0.5 s deadman. Motion is posted to the dashboard or gamepad,
which publish `/cmd_vel`. The bridge does not publish `/cmd_vel_safe`.

| Command `type` | Rank gate (public channel only) | Dashboard |
| --- | --- | --- |
| `estop` | ≥1 | `POST /api/estop` |
| `nudge` / `drive` | ≥1 | `POST /api/nudge` `{lin,ang}` |
| `joy` | ≥1 | `POST {AMR_GAMEPAD_URL}/api/joy` `{a,b}` **or** nudge approx |
| `nav_goal` | ≥1 | `POST /api/nav_goal` `{x,y}` odom |
| `nav_cancel` | ≥1 | `POST /api/nav_cancel` |
| `vision` | ≥1 | `POST /api/vision` `{on}` |
| `save_map` | ≥2 | `POST /api/save_map` `{name}` |
| `run_tool` | ≥1 | `POST /api/run` `{tool,params}` |
| `stop_tool` | ≥1 | `POST /api/stop` |
| `blacklist` add/clear | ≥1 | `POST /api/blacklist` |
| `set_rate` | any | Bridge-local full/board mode only |

Tools expected by `/control` UI: `imu_test`, `imu_calibration`, `drive_distance`
(same as dashboard `TOOLS` dict).

---

## 9. Xbox / joy path

Site fullscreen controller streams:

```json
{ "type": "joy", "axes": [8 floats], "buttons": <bitmask int> }
```

Layout matches `web_gamepad_node` (Xbox):

- axes[0], axes[1] — left stick (strafe / forward)
- axes[2], axes[3] — right stick (yaw / tilt)
- axes[4], axes[5] — LT / RT
- axes[6], axes[7] — D-pad
- button bits — A/B/X/Y, LB/RB, Back/Start (see `web_gamepad_node.py`)

**Best:** set `AMR_GAMEPAD_URL=http://127.0.0.1:8765` and keep `web_gamepad_node`
running so `/joy` is published at 30 Hz into the existing teleop stack.

**Fallback:** without gamepad URL, bridge maps left stick → `/api/nudge` only
(no face buttons / shoulders).

---

## 10. Verify end-to-end

1. Start robot bringup (dashboard + bridge).
2. Confirm bridge logs: SSE connected + realtime subscribed.
3. Open https://amrslam.tech/control/ → sign in as allowlisted user (`amey`).
4. Select bot matching `AMR_BOT_ID`.
5. Expect status **online** (not BOT OFFLINE); lag ms updating; topics Hz/age.
6. **Take control** → Drive / Controller / nav goal / E-STOP.
7. Optional: Start vision → camera panel; run IMU test → console logs.

### Failure matrix

| Symptom | Likely cause |
| --- | --- |
| BOT OFFLINE forever | Bridge not running; wrong `AMR_BOT_ID`; no Supabase egress; missing token |
| Launch exits immediately with missing env | `/etc/amr-bridge.env` incomplete |
| SSE reconnect loop | Dashboard not on `AMR_DASHBOARD_URL` |
| Online but Drive noop | Not controller lock holder; rank 0 Viewer |
| Joy weak / no buttons | `AMR_GAMEPAD_URL` unset (nudge fallback) |
| Camera empty | Vision off; no JPEG scrape; set `AMR_CAMERA_URL` |
| `touch_amr_bot` errors in logs | Token mismatch vs `amr_bridge_config` |

---

## 11. Site / Supabase (reference only — already live)

Agents integrating the **robot** should not change these unless explicitly asked.

- Channels: `amr-bot-amr4`, `amr-bot-hardware-bot`
- Events: `telemetry`, `map`, `hb`, `log`, `command`
- Presence lock for control authority (higher rank preempts)
- Login: invite-only usernames via Edge Function `operator-auth`
- Migrations under `supabase/migrations/` in the website repo

Site env (GitHub Pages / build), not Pi:

```bash
NEXT_PUBLIC_SUPABASE_URL=…
NEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY=…
NEXT_PUBLIC_AUTH_ADAPTER=supabase
NEXT_PUBLIC_SITE_URL=https://amrslam.tech
```

---

## 12. Checklist for the integrating agent

Copy this into the robot PR description:

```text
[ ] Copied amr_bridge.launch.py into Ros_lidar_bot/launch/
[ ] Installed amr_telemetry_bridge + requirements in a venv
[ ] Installed /etc/amr-bridge.env (600) with BOT_ID, URL, KEY, TOKEN, DASHBOARD
[ ] Included delayed bridge in bringup after robot_dashboard_node
[ ] (Optional) AMR_GAMEPAD_URL → web_gamepad_node :8765
[ ] (Optional) AMR_CAMERA_URL public MJPEG
[ ] Smoke: bridge logs SSE + realtime subscribed
[ ] Smoke: /control shows online for correct bot_id
[ ] Smoke: Take control → nudge / joy / estop
[ ] No secrets committed to Ros_lidar_bot
```

---

## 13. Minimal “do this now” command block

```bash
# 1) Secrets
sudo install -m 600 /path/to/AMR/amr-bridge/.env.pi /etc/amr-bridge.env

# 2) Python
cd /path/to/AMR/amr-bridge
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
export PYTHONPATH="$PWD:$PYTHONPATH"

# 3) Launch file into robot package + include TimerAction (see §6)

# 4) Test with dashboard up
python -m amr_telemetry_bridge
# → then https://amrslam.tech/control/
```

---

## 14. Source of truth / keep in sync

| Concern | File |
| --- | --- |
| Command + telemetry types (TS) | `src/lib/telemetry/protocol.ts` |
| Bot ids / rates (Python) | `amr-bridge/amr_telemetry_bridge/protocol.py` |
| Bridge I/O | `amr-bridge/amr_telemetry_bridge/bridge.py` |
| Launch | `amr-bridge/launch/amr_bridge.launch.py` |
| Dashboard HTTP | `Ros_lidar_bot` `robot_dashboard_node.py` |
| Gamepad HTTP | `Ros_lidar_bot` `web_gamepad_node.py` |
| Control UI | `src/components/control/*` |

If you change dashboard APIs or protocol fields, update **both** repos and this guide.

---

*Last updated for AMR website `amr-bridge` + `/control` dashboard parity (merged PR #4).*
