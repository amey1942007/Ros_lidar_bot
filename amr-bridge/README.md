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

| Rank | Label | Drive | Preempt |
| --- | --- | --- | --- |
| 0 | Viewer | no | — |
| 1 | Operator | yes | equal+ |
| 2 | Admin | yes | yes |
| 3 | Owner | yes | yes |

Control lock uses Realtime presence. E-STOP is allowed for any operator+ without holding the lock.

## Adaptive rate

Browsers request `full` (10 Hz + lidar/camera/3D) or `board` (1 Hz vitals). When mean lag exceeds ~800 ms the control panel falls back to the status board.

## Protocol

`src/lib/telemetry/protocol.ts` ↔ `amr_telemetry_bridge/protocol.py`  
Channels: `amr-bot-amr4`, `amr-bot-hardware-bot` · Events: `telemetry`, `map`, `hb`, `command`
