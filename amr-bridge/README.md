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

Legacy public topics (flag `AMR_PUBLIC_CHANNEL`, default off): `amr-bot-amr4`, `amr-bot-hardware-bot`. Events: `telemetry`, `map`, `log`, `hb`, `command`.

Private topics: `bot:<id>:status` (bridge sends `telemetry`, `map`, `log`, `hb`, `cmd_ack`), `bot:<id>:cmd` and `bot:<id>:cmdq` (bridge receives `command`), `bot:<id>:ctl` (bridge receives `control`). Sign-in uses `AMR_BRIDGE_EMAIL` / `AMR_BRIDGE_PASSWORD`. Motion still goes out through the dashboard and gamepad HTTP APIs onto `/cmd_vel`.

## Command acks

The site opens `bot:<id>:cmd` as a private channel with broadcast ack. That transport ack only means the realtime server took the broadcast. The bridge still answers **every** command on `bot:<id>:cmd` and `bot:<id>:cmdq` with one application event:

- topic: `bot:<id>:status`
- event: `cmd_ack`
- payload: `{seq: number, ok: boolean, reason?: string}`

`seq` is the command seq. A message with no integer seq is acked as `0`. `reason` is omitted when `ok` is true. It is one of:

| reason | when |
| --- | --- |
| `bad_token` | `:cmd` token does not match, or a token carried on `:cmdq` does not match |
| `bad_sender` | `:cmdq` sender is missing or is not the current controller |
| `stale_epoch` | epoch is missing or is not the current epoch |
| `lower_seq` | seq is below the last accepted seq and the restart gap has not elapsed |
| `duplicate_seq` | seq repeats the last accepted seq for this epoch |
| `no_lease` | lease is missing or already expired |
| `invalid_payload` | body, bot id, timestamp, or a field value is not usable |
| `unknown_type` | `type` is not a command the bridge handles |
| `estop_hold` | joy, nudge, or drive ignored after an e-stop, or dropped because an e-stop cleared it before it was checked |
| `rate_limited` | the callback cap was full, or a newer stick replaced this one before it was checked |

E-stop still skips seq, timestamp, and lease checks. It still requires the bot, the current epoch, and the token (`:cmd`) or sender (`:cmdq`). An accepted e-stop is acked `ok: true` only after the stop/zero posts have been scheduled. The ack is queued at that moment, while those posts and any joy POST already in flight are still running. A failed stop is retried; that retry does not send a second ack.

Ack sends run on their own task. The queue holds at most 64 and drops the oldest. E-stop and motion only append. A stalled realtime send stays on the ack task. The ack and the logs carry `seq`, `ok`, and `reason` only.

Motion is unchanged: the dashboard and gamepad HTTP APIs publish `/cmd_vel`, and `safety_stop` is still the only publisher of `/cmd_vel_safe`.

Env names (values stay in `/etc/amr-bridge.env`, never in git): `AMR_SUPABASE_URL`, `AMR_SUPABASE_KEY` (publishable), `AMR_BRIDGE_TOKEN`, `AMR_BRIDGE_EMAIL`, `AMR_BRIDGE_PASSWORD`, `AMR_PUBLIC_CHANNEL`. Do not set `AMR_SUPABASE_SERVICE_KEY`.
