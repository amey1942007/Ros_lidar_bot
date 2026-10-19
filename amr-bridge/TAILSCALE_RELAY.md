# Pi on campus Wi‑Fi + Tailscale → amrslam.tech/control

## Your setup

The Raspberry Pi 5 is on **campus Wi‑Fi** and also joined to **Tailscale**.
Tailscale is for SSH / ops (MagicDNS, `100.x` address). The website never
connects to Tailscale. Data path:

```
ROS on Pi  →  amr_telemetry_bridge (on the Pi)  →  Supabase Realtime
                                                      ↓
                                              amrslam.tech/control
```

`robot_dashboard` is **optional** and **off by default**. Use
`AMR_SOURCE=ros` so the bridge reads `/odom`, `/scan`, `/map`, … directly.

## Why `/control` was empty before

The bridge was not calling Supabase (`last_seen_at` days stale). Campus Wi‑Fi
sometimes blocks HTTPS to `*.supabase.co`. Tailscale alone does not fix
outbound HTTPS unless you use a **Tailscale exit node**.

Check from the Pi:

```bash
curl -I --connect-timeout 5 https://khucmrlbstdlbxcuboub.supabase.co/auth/v1/health
```

- Works → run the bridge on the Pi (recommended).
- Fails → use a Tailscale **exit node** on a home PC / phone, or temporarily a
  phone hotspot, then start the bridge on the Pi again.

## Recommended bringup (no dashboard)

```bash
# /etc/amr-bridge.env
AMR_BOT_ID=hardware-bot
AMR_SOURCE=ros
AMR_SUPABASE_URL=https://khucmrlbstdlbxcuboub.supabase.co
AMR_SUPABASE_KEY=sb_publishable_…
AMR_BRIDGE_TOKEN=…          # must match amr_bridge_config.token_sha256
AMR_BRIDGE_EMAIL=amr-bridge@operators.amrslam.tech
AMR_BRIDGE_PASSWORD=…
AMR_PUBLIC_CHANNEL=0
AMR_PRIVATE_CHANNEL=1

bash amr-bridge/scripts/install_bridge_service.sh --local
sudo systemctl enable --now amr-bridge

ros2 launch Ros_lidar_bot launch_robot.launch.py \
  enable_robot_dashboard:=false \
  enable_amr_bridge:=false \
  amr_bot_id:=hardware-bot
```

(`enable_robot_dashboard` defaults to **false**. Set `:=true` only if you want
the local `:8080` UI.)

## Cloud demo (no Pi required)

From a machine that can reach Supabase (this proves the website path):

```bash
export AMR_SUPABASE_URL=https://khucmrlbstdlbxcuboub.supabase.co
export AMR_SUPABASE_KEY=sb_publishable_…
export AMR_BRIDGE_TOKEN=…   # same as on the Pi
export AMR_BOT_ID=hardware-bot
python amr-bridge/scripts/demo_cloud_telemetry.py
```

Sends `hb`, full `telemetry` (pose, scan, rates, nodes, tf, nav, path, goal,
yolo, frontier, sys, …), `map`, and `log`, and calls `touch_amr_bot`.
Open https://amrslam.tech/control/ → Hardware bot → should go **Live**.

## Optional: keep dashboard

```bash
ros2 launch … enable_robot_dashboard:=true
# /etc/amr-bridge.env → AMR_SOURCE=dashboard
```

Only needed for the Pi browser UI or legacy HTTP SSE.
