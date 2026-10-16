"""Mirrors src/lib/telemetry/protocol.ts (keep in sync)."""

from __future__ import annotations

BOT_IDS = ("amr4", "hardware-bot")
HEARTBEAT_OFFLINE_MS = 3000
FULL_RATE_HZ = 10
BOARD_RATE_HZ = 1
BRIDGE_VERSION = "0.1.0"

# Private Realtime topics from the applied policies (bot:<id>:status|ctl|cmd|cmdq).
PRIVATE_TOPIC_KINDS = ("status", "cmd", "cmdq", "ctl")
# Events the bridge sends on bot:<id>:status. :cmd / :cmdq use "command";
# :ctl uses "control" (see send_command / take_control).
STATUS_EVENTS = ("telemetry", "map", "log", "hb")
CMD_EVENT = "command"
CTL_EVENT = "control"


def channel_name(bot_id: str) -> str:
    """Legacy public topic. Still used while AMR_PUBLIC_CHANNEL is on."""
    return f"amr-bot-{bot_id}"


def private_topic(bot_id: str, kind: str) -> str:
    if kind not in PRIVATE_TOPIC_KINDS:
        raise ValueError(f"unknown private topic kind {kind!r}")
    return f"bot:{bot_id}:{kind}"
