# Tailscale relay + do we need robot_dashboard?

## Why `/control` showed no telemetry last time

The website **never talks to the robot**. It only listens on Supabase Realtime
(`amr-bot-<id>`). Evidence from the live AMR Supabase project when this was
diagnosed:

| Signal | Value |
| --- | --- |
| `hardware-bot.last_seen_at` | ~2.5 days stale |
| Bridge Auth last sign-in | same window as last touch |
| Recent `bridge_touch_failures` | 0 (process was not calling at all) |

So the Pi bridge was **not connected to Supabase** (not running, missing
`/etc/amr-bridge.env` secrets, or campus Wi‑Fi blocking `*.supabase.co`). The
control panel correctly showed offline / no data.

`amr_bots.online` used to stay `true` forever after the last successful touch.
A `pg_cron` job now flips it to `false` after 20 s without a heartbeat.

## Do you need `robot_dashboard`?

**For amrslam.tech/control: no, not if the bridge runs on the Pi with ROS.**

| Piece | Role |
| --- | --- |
| `robot_dashboard` | Local browser UI on `:8080` + aggregates ROS → SSE + HTTP command API |
| `amr_telemetry_bridge` | Pi (or Tailscale relay) → Supabase Realtime → website |
| Website `/control` | Supabase Realtime only |

Two modes (`AMR_SOURCE`):

1. **`dashboard` (default)** — bridge scrapes `http://127.0.0.1:8080/events` and
   posts `/api/nudge`, `/api/estop`, …. Needs the dashboard process (or any HTTP
   that speaks the same API). **Required for a Tailscale relay host** that is
   not on the robot's ROS graph.
2. **`ros`** — bridge subscribes to `/odom`, `/scan`, `/map`, … and publishes
   `/cmd_vel` itself. **Dashboard can be off:**
   ```bash
   ros2 launch Ros_lidar_bot launch_robot.launch.py \
     enable_robot_dashboard:=false \
     amr_bridge_python:=…/amr-bridge/.venv/bin/python
   # and in /etc/amr-bridge.env:
   AMR_SOURCE=ros
   ```
   Tools that only exist on the dashboard (vision toggle, exploration tool
   runner, save-map UI helpers) return a clear error in this mode. Drive,
   e-stop, and nav goals still work.

You can delete the dashboard from a bringup that only cares about the cloud
panel by using `enable_robot_dashboard:=false` + `AMR_SOURCE=ros`. Keep it if
you still want the Pi web UI.

## Recommended pipeline with Tailscale always on

Campus / IITD Wi‑Fi often blocks HTTPS from the Pi to Supabase. Tailscale does
not fix that by itself. Use one of:

### A. Bridge on the Pi (simplest when Supabase HTTPS works)

```bash
# /etc/amr-bridge.env
AMR_SOURCE=ros
AMR_BOT_ID=hardware-bot
AMR_SUPABASE_URL=https://khucmrlbstdlbxcuboub.supabase.co
AMR_SUPABASE_KEY=sb_publishable_…
AMR_BRIDGE_TOKEN=…
AMR_BRIDGE_EMAIL=amr-bridge@operators.amrslam.tech
AMR_BRIDGE_PASSWORD=…

bash amr-bridge/scripts/install_bridge_service.sh --local
sudo systemctl enable --now amr-bridge
# Avoid a double process from ROS launch:
ros2 launch … enable_amr_bridge:=false enable_robot_dashboard:=false
```

### B. Tailscale relay (when the Pi cannot reach Supabase)

Run the bridge on any **always-on** machine that is on Tailscale **and** can
reach Supabase (home PC, phone-hotspot laptop, small VPS with Tailscale).

On the Pi (dashboard still needed as the HTTP front for ROS):

```bash
bash amr-bridge/scripts/pi_tailscale_serve.sh   # Serve :8080 inside the tailnet
# or just leave robot_dashboard on 0.0.0.0:8080 reachable via Tailscale IP
```

On the relay host:

```bash
bash amr-bridge/scripts/install_bridge_service.sh --relay \
  --bot hardware-bot \
  --dashboard http://<pi-magicdns-or-100.x.y.z>:8080

sudo nano /etc/amr-bridge-hardware-bot.env   # fill keys
sudo systemctl enable --now amr-bridge@hardware-bot
journalctl -u amr-bridge@hardware-bot -f
```

When the robot boots on Tailscale, the relay’s `wait_for_dashboard.sh` unblocks,
SSE flows, `touch_amr_bot` updates `last_seen_at`, and `/control` goes **Live**.

Do **not** use Tailscale Funnel for the dashboard (public internet). Serve is
tailnet-only.

## Verify

1. `select id, online, last_seen_at from amr_bots;` — `last_seen_at` moves every ~5 s when the bridge is up.
2. Bridge log: `realtime subscribed` and `bridge stats: … telemetry_ok=…` growing.
3. https://amrslam.tech/control/ → Hardware bot → **Live**.
