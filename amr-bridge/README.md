# amr-bridge

Cloud telemetry bridge for [Ros_lidar_bot](https://github.com/amey1942007/Ros_lidar_bot) → Supabase Realtime → [amrslam.tech/control](https://amrslam.tech/control/).

> **Integrating into the robot?** Start here:  
> **[`AGENT_SETUP_AND_INTEGRATION.md`](./AGENT_SETUP_AND_INTEGRATION.md)** — full setup + launch include + verify checklist for another agent.  
> Short include snippet: [`launch/INCLUDE_IN_ROSLIDAR.md`](./launch/INCLUDE_IN_ROSLIDAR.md).

## Supabase (already applied on project **AMR**)

| Piece | Status |
| --- | --- |
| `operator_profiles` + ranks | Applied · `amey@amrslam.tech` = **Owner (3)** |
| `amr_bots` registry | `amr4`, `hardware-bot` |
| `touch_amr_bot` RPC | Bridge heartbeat with shared token (no service_role needed) |
| Auth redirect | Site URL `https://amrslam.tech` |

## Why `/control` (not a bot subdomain)

Same-origin auth with the marketing site, existing AuthGuard + Supabase session, one CSP surface. Camera can still be a separate Cloudflare Tunnel URL when the browser cannot reach the Pi over Tailscale.

## Do you need `robot_dashboard`?

**Not for the website.** `/control` only sees Supabase Realtime. The dashboard is a local Pi UI + optional HTTP front for the bridge.

- `AMR_SOURCE=ros` on the Pi → bridge reads ROS topics; set `enable_robot_dashboard:=false` if you do not want `:8080`.
- `AMR_SOURCE=dashboard` (default) → bridge needs the dashboard (or a Tailscale relay pulling from it). Required when the bridge runs off-robot.

See [`TAILSCALE_RELAY.md`](./TAILSCALE_RELAY.md) for the Tailscale relay path (campus Wi‑Fi often blocks Pi → Supabase).

## Include in ROS launch

**Yes.** Use `launch/amr_bridge.launch.py` from your bringup (after the local dashboard is up). See [`launch/INCLUDE_IN_ROSLIDAR.md`](launch/INCLUDE_IN_ROSLIDAR.md).

```bash
ros2 launch Ros_lidar_bot amr_bridge.launch.py bot_id:=amr4
```

## Pi setup

```bash
cd amr-bridge
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Secrets (gitignored). Generated during cloud setup — copy to the Pi:
sudo install -m 600 .env.pi /etc/amr-bridge.env

export $(grep -v '^#' /etc/amr-bridge.env | xargs)
python -m amr_telemetry_bridge
# or via ROS:
# ros2 launch … amr_bridge.launch.py bot_id:=amr4
```

Run one process per robot (`AMR_BOT_ID=amr4` or `hardware-bot`).

## Authority

The private path (`bot:<id>:cmd` / `:cmdq`) does not use ranks. A command is
accepted only when the epoch, token, and lease check out and the seq is new
(or a restart after a short gap). E-stop skips the seq, timestamp, and lease
checks. After an accepted e-stop the bridge ignores joy, nudge, and drive for
1 s unless that command's seq is newer than the e-stop's. `/api/estop` does
not latch: the dashboard streams zeros on `/cmd_vel` for 1.5 s, cancels
navigation, and kills the running tool, and the gamepad keeps the last axes
fresh for 1.5 s. The legacy public channel still applies the rank table below while
`AMR_PUBLIC_CHANNEL=1`. That flag defaults to off. Turning it on reopens the
old topic: the rank in the message is whatever the sender writes, so it can
be spoofed, and commands on that topic do not arm the 0.5 s deadman.

| Rank | Label | Drive | Preempt |
| --- | --- | --- | --- |
| 0 | Viewer | no | — |
| 1 | Operator | yes | equal+ |
| 2 | Admin | yes | yes |
| 3 | Owner | yes | yes |

On the legacy channel, control lock uses Realtime presence. E-STOP is allowed for any operator+ without holding the lock.

## Adaptive rate

Browsers request `full` (10 Hz + lidar/camera/3D) or `board` (1 Hz vitals). When mean lag exceeds ~800 ms the control panel falls back to the status board.

## Protocol

`src/lib/telemetry/protocol.ts` ↔ `amr_telemetry_bridge/protocol.py`

The bridge publishes `telemetry`, `map`, `log`, and `hb` on the public topic `amr-bot-<id>` whenever that join reaches `SUBSCRIBED`, including when `AMR_PUBLIC_CHANNEL` is off. That flag (default off) only listens for `command` on the same topic. Commands there are spoofable and do not arm the deadman.

Private topics: `bot:<id>:status` (bridge sends `telemetry`, `map`, `log`, `hb`), `bot:<id>:cmd` and `bot:<id>:cmdq` (bridge receives `command`), `bot:<id>:ctl` (bridge receives `control`). Sign-in uses `AMR_BRIDGE_EMAIL` / `AMR_BRIDGE_PASSWORD`. A failed `:status` join does not drop the public topic. Motion still goes out through the dashboard and gamepad HTTP APIs onto `/cmd_vel`.

Every 10 s the process logs one `bridge stats:` line (ready flags, send counters, SSE frames, last pose and scan size). Send failures log at WARNING, at most once per 30 s per exception type.

`/scan` on `telemetry` is capped (`AMR_SCAN_MAX_POINTS`, default 90) and included on every `ceil(10/AMR_SCAN_HZ)`-th full-rate frame, starting with the first (default 5 Hz). Other full-rate frames send `"scan": null`. The current website treats a null scan as an empty list, so those frames clear the drawn scan until the page keeps the previous one. Frames over `AMR_TELEMETRY_MAX_BYTES` (default 200000) drop `cameraJpegB64`, then `scan`, then `frontier` / `yoloDet` / `sem` / `path`. An oversize `map` frame is dropped. `AMR_PUBLIC_REST=1` also POSTs the public payload to Realtime's REST broadcast endpoint at most 5 times a second (default off).

Env names (values stay in `/etc/amr-bridge.env`, never in git): `AMR_SUPABASE_URL`, `AMR_SUPABASE_KEY` (publishable), `AMR_BRIDGE_TOKEN`, `AMR_BRIDGE_EMAIL`, `AMR_BRIDGE_PASSWORD`, `AMR_PUBLIC_CHANNEL`, `AMR_SCAN_MAX_POINTS` (90, clamp 20–360), `AMR_SCAN_HZ` (5, clamp 1–10), `AMR_TELEMETRY_MAX_BYTES` (200000, clamp 20000–250000), `AMR_PUBLIC_REST` (0). Do not set `AMR_SUPABASE_SERVICE_KEY`.
