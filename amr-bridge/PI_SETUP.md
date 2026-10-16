# Pi remaining setup (after this repo is pulled)

Repo integration is done in `Ros_lidar_bot`. On the **robot**, finish:

## 1. Secrets

Obtain the private `amr-bridge/.env.pi` from the AMR website owner (not in git).

```bash
sudo install -m 600 /path/to/.env.pi /etc/amr-bridge.env
sudo nano /etc/amr-bridge.env   # set AMR_BOT_ID=hardware-bot (or amr4)
```

Required: `AMR_SUPABASE_URL`, `AMR_SUPABASE_KEY` (publishable key), `AMR_BRIDGE_TOKEN`,
`AMR_DASHBOARD_URL`. Private channels also need `AMR_BRIDGE_EMAIL` and
`AMR_BRIDGE_PASSWORD`. `AMR_PUBLIC_CHANNEL=1` (default) keeps the legacy public
channel until cut-over. Do not set `AMR_SUPABASE_SERVICE_KEY`.
Recommended: `AMR_GAMEPAD_URL=http://127.0.0.1:8765`.

## 2. Bridge Python venv

```bash
cd ~/ros2_ws/src/Ros_lidar_bot/amr-bridge
python3 -m venv .venv && source .venv/bin/activate
pip install -U pip && pip install -e .
```

## 3. Rebuild ROS package

```bash
cd ~/ros2_ws
colcon build --packages-select Ros_lidar_bot
source install/setup.bash
```

## 4. Launch + verify

```bash
ros2 launch Ros_lidar_bot launch_robot.launch.py \
  amr_bot_id:=hardware-bot \
  amr_bridge_python:=$HOME/ros2_ws/src/Ros_lidar_bot/amr-bridge/.venv/bin/python
```

Healthy bridge logs: `realtime subscribed` + `connecting SSE http://127.0.0.1:8080/events`.

Then open https://amrslam.tech/control/ → sign in → pick **Hardware bot** →
online → Take control → Drive / E-STOP.

## Optional

- Public camera: set `AMR_CAMERA_URL` (Cloudflare Tunnel / Tailscale Funnel).
- Disable cloud: `enable_amr_bridge:=false`.
- Standalone bridge smoke (dashboard already on :8080):

```bash
ros2 launch Ros_lidar_bot amr_bridge.launch.py \
  bot_id:=hardware-bot \
  python:=$HOME/ros2_ws/src/Ros_lidar_bot/amr-bridge/.venv/bin/python
```
