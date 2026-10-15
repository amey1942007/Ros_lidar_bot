# RPi5 prep you can do **now** (before pulling this PR)

These steps do not need the new bridge code on the Pi yet. Do them while the PR merges / you wait to pull.

## 1. Outbound network to Supabase

From the Pi:

```bash
curl -I --connect-timeout 5 https://khucmrlbstdlbxcuboub.supabase.co/
# Expect some HTTP response (even 404). Timeout / DNS fail = fix Wi‑Fi/firewall first.
```

`/control` does **not** need Tailscale for telemetry if Supabase HTTPS works. Tailscale is still useful for SSH and local dashboard (`:8080`).

## 2. Get secrets ready (do not commit)

Ask the AMR website owner for the private `amr-bridge/.env.pi` (or regenerate the bridge token in Supabase and update `public.amr_bridge_config.token_sha256`).

On the Pi, install the file **before** you pull if you already have it:

```bash
sudo install -m 600 /path/to/.env.pi /etc/amr-bridge.env
sudo nano /etc/amr-bridge.env
```

Set at least:

```bash
AMR_BOT_ID=hardware-bot          # must match site bot picker
AMR_SUPABASE_URL=https://khucmrlbstdlbxcuboub.supabase.co
AMR_SUPABASE_KEY=…               # publishable/anon (not service_role)
AMR_BRIDGE_TOKEN=…               # plaintext whose sha256 is in amr_bridge_config
AMR_DASHBOARD_URL=http://127.0.0.1:8080
AMR_GAMEPAD_URL=http://127.0.0.1:8765
```

Verify permissions:

```bash
sudo ls -l /etc/amr-bridge.env   # should be -rw------- root root
```

## 3. Create an empty venv home (deps after pull)

```bash
mkdir -p ~/amr-bridge-venv-prep
python3 -m venv ~/amr-bridge-venv-prep
# After you pull the PR into the robot repo, you will instead use:
#   ~/ros2_ws/src/Ros_lidar_bot/amr-bridge/.venv
# and: pip install -e .
```

Optional: pre-install deps into that prep venv so first bringup is faster:

```bash
source ~/amr-bridge-venv-prep/bin/activate
pip install -U pip
pip install 'supabase>=2.4.0' 'httpx>=0.27.0' 'websockets>=12.0'
```

## 4. Confirm local dashboard contract (current code)

With your **existing** bringup already running:

```bash
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/
# 200 expected when robot_dashboard is up

curl -s -N --max-time 2 http://127.0.0.1:8080/events | head -c 200
# Should stream SSE (or at least connect)

curl -s -o /dev/null -w '%{http_code}\n' -X POST http://127.0.0.1:8080/api/estop -H 'Content-Type: application/json' -d '{}'
# 200 expected
```

After the PR is on the Pi, also check gamepad HTTP (new route):

```bash
curl -s -X POST http://127.0.0.1:8765/api/joy \
  -H 'Content-Type: application/json' \
  -d '{"a":[0,0,0,0,0,0,0,0],"b":0}'
# {"ok": true}
```

## 5. Site login smoke (laptop/phone — no Pi code)

1. Open https://amrslam.tech/control/
2. Sign in as an allowlisted operator (`amey` / your invite username)
3. Confirm the bot picker shows **Hardware bot** / **AMR4**
4. Without a bridge running, expect **BOT OFFLINE** — that is normal until step 6 after pull

## 6. After you pull / merge lands on the Pi

```bash
cd ~/ros2_ws/src/Ros_lidar_bot
git fetch origin && git checkout hardware && git pull

cd amr-bridge
python3 -m venv .venv && source .venv/bin/activate
pip install -U pip && pip install -e .

cd ~/ros2_ws
colcon build --packages-select Ros_lidar_bot
source install/setup.bash

ros2 launch Ros_lidar_bot launch_robot.launch.py \
  amr_bot_id:=hardware-bot \
  amr_bridge_python:=$HOME/ros2_ws/src/Ros_lidar_bot/amr-bridge/.venv/bin/python
```

Healthy logs: `realtime subscribed` + `connecting SSE http://127.0.0.1:8080/events`.  
Then `/control` → Hardware bot → online → Take control.
