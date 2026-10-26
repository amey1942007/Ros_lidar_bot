# vlm-integration-windows (laptop side only)

Orphan branch of `amey1942007/Ros_lidar_bot`. It shares **no history** with `main`. It holds the
**Windows laptop** VLM pipeline. It never runs on the Pi and is never deployed to the robot. There is
no ROS here and no simulation.

```
 Windows laptop (THIS branch, no ROS)                    Raspberry Pi 5 (branch main, ROS 2 Jazzy)
 ┌───────────────────────────────────────┐  Tailscale  ┌─────────────────────────────────────────┐
 │ Tkinter GUI  (webcam | bot toggle)    │  + token    │ vlm_bridge_node (PLANNED, not built yet) │
 │ agent loop: plan -> tools -> results  │─REST :8090─▶│  -> Nav2, /camera_cmd, /yolo, slam ...   │
 │ VLM: Ollama qwen3-vl:8b on RTX 4060   │◀─WS :8090───│  status/events, heartbeat watchdog       │
 │ E-STOP sender (own thread)            │─POST :8091─▶│  E-STOP latch (no reset endpoint)        │
 └───────────────────────────────────────┘             └─────────────────────────────────────────┘
```
**The VLM and all reasoning run on the laptop. Every robot tool (motion, pan/tilt, detection, mapping,
E-STOP) runs on the Pi.** The laptop only sends commands and receives results. In **webcam mode** the
robot tools are **stubs**: nothing moves, and every result is marked `"stub": true`.

## What is on `main` (what this talks to)
Summary of `main` @ `6b394ea` (read remotely, not fetched into this repo):
- **Bringup** `launch/launch_robot.launch.py`: robot_state_publisher, `imu_node` (`/imu`), `driver_node`
  (motors, 1 s command timeout), `odom_node`, sllidar (`/scan_raw` → min-range filter → `/scan`), EKF
  (`/odom`), `camera_servo` (pan/tilt), slam_toolbox (`/map`) and Nav2 (`/navigate_to_pose`).
- **Motors**: `driver_node`. **Safety**: `safety_stop_node` (`/cmd_vel` → `/cmd_vel_safe`) exists but is
  **disabled in the main launch today**; PR #23 re-enables it and adds a latched `/emergency_stop`.
- **Camera**: owned by `yolo.py` (640×480) today; the plan is an always-on `camera_node` on the Pi.
  Detections on `/yolo` (JSON); `semantic_slam_node` writes a semantic map.
- **Pan servo**: `camera_servo_node`, `/camera_cmd` (rate control), continuous-rotation pan with **no
  encoder**, so the pan angle is an estimate. `pan_home_deg` is a config param (null until Amey sets it).
- **E-STOP**: dashboard E-STOP today zeroes `/cmd_vel`; the design makes it latched. **Only Amey resets it,
  from the dashboard.**
- **Web gamepad** :8765, **dashboard** :8080 (HTTP + SSE, camera MJPEG), **lidar** UDP 8089.
- **Exploration**: `frontier_explorer_node`.
- **Bridge**: the Pi-side `vlm_bridge_node` (FastAPI, REST+WS :8090, E-STOP :8091) is **planned, not built**.
  Bot mode in this repo is written against that contract (design doc rev 4).

## Layout
| Path | What |
|---|---|
| `config/laptop.yaml` | model, Ollama URL, bot Tailscale IP, token file, ports (api 8090, estop 8091, gui 8070), `pan_home_deg`, limits |
| `vlm/ollama_client.py` | streaming Ollama `/api/chat` client (thinking + content, TTFT, tok/s) |
| `agent/tools.py` | tool registry + JSON schemas (no E-STOP reset tool, no battery tool) |
| `agent/loop.py` | agent loop: observe → stream reasoning → JSON plan (2-retry parse rule) → run tools automatically, cancel per tool / per run, step + time limits, pause/resume, JSONL log in `logs/` |
| `agent/stopwords.py` | stop words checked before text reaches the VLM |
| `robot/bridge_client.py` | bot mode: REST + WS (heartbeat every 500 ms), Bearer token |
| `robot/estop.py` | E-STOP sender on its own thread, POST `:8091/estop`, 3 retries |
| `robot/stub.py`, `robot/webcam.py` | webcam mode: laptop camera + stub tools |
| `gui/app.py` | Tkinter GUI |
| `scripts/test_snapshot.py` | one frame → model → reply, timings, VRAM |
| `tests/` | unit tests (agent loop, E-STOP, stop words) |

## Safety rules (enforced in code and tests)
- No tool, endpoint or code path clears the E-STOP. **Resume** in the GUI does not clear it either; the agent
  stays paused until the Pi reports the latch reset from the dashboard.
- E-STOP button (also `Esc`/`F12`) sends on its own thread, so it never waits on inference.
- **Stop words** in the command box (case-insensitive, whole words): `halt`, `freeze`, `emergency`, `ruko`,
  `tham`. They cancel the plan and tools at once and send `stop()` (the stub in webcam mode). `emergency` also
  fires the latched E-STOP on :8091. The text is never sent to the VLM. V1 has no voice input.
- A command typed while a task runs is a **human command**: the task pauses, the command runs, then the
  task re-observes and resumes.
- The agent may start exploration and save maps without asking.

## Setup (Windows, existing venv)
```powershell
cd D:\Vision-Pipeline\vlm-integration-windows
D:\Vision-Pipeline\Scripts\python.exe -m pip install -r requirements.txt   # all already present; torch NOT touched
ollama pull qwen3-vl:8b        # fallback: qwen3-vl:4b  (set model.name in config/laptop.yaml)
```
Bot mode also needs `bot.tailscale_ip` in the config, and the token in the file at `bot.token_file`
(default `%USERPROFILE%\.vlm_bridge\token`, never committed).

## Run
```powershell
D:\Vision-Pipeline\Scripts\python.exe -m unittest discover -s tests -t .
D:\Vision-Pipeline\Scripts\python.exe scripts\test_snapshot.py            # webcam -> model
D:\Vision-Pipeline\Scripts\python.exe gui\app.py                          # GUI
```

## GUI (CustomTkinter, dark)
Install the GUI dependency without letting pip resolve anything else (protects the venv's torch):
`D:\Vision-Pipeline\Scripts\python.exe -m pip install --no-deps customtkinter darkdetect`

Run (opens maximised):
```powershell
D:\Vision-Pipeline\Scripts\Activate.ps1
cd D:\Vision-Pipeline\vlm-integration-windows
python gui\app.py
```
Keys: F11 toggles fullscreen, Esc leaves fullscreen (neither touches the E-STOP), F12 = E-STOP. Enter sends the prompt.
