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
checks. The legacy public channel still applies the rank table below while
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

Legacy public topics (flag `AMR_PUBLIC_CHANNEL`, default off): `amr-bot-amr4`, `amr-bot-hardware-bot`. Events: `telemetry`, `map`, `log`, `hb`, `command`.

Private topics: `bot:<id>:status` (bridge sends `telemetry`, `map`, `log`, `hb`), `bot:<id>:cmd` and `bot:<id>:cmdq` (bridge receives `command`), `bot:<id>:ctl` (bridge receives `control`). Sign-in uses `AMR_BRIDGE_EMAIL` / `AMR_BRIDGE_PASSWORD`. Motion still goes out through the dashboard and gamepad HTTP APIs onto `/cmd_vel`.

Env names (values stay in `/etc/amr-bridge.env`, never in git): `AMR_SUPABASE_URL`, `AMR_SUPABASE_KEY` (publishable), `AMR_BRIDGE_TOKEN`, `AMR_BRIDGE_EMAIL`, `AMR_BRIDGE_PASSWORD`, `AMR_PUBLIC_CHANNEL`. Do not set `AMR_SUPABASE_SERVICE_KEY`.
