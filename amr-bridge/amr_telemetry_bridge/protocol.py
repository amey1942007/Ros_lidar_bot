"""Mirrors src/lib/telemetry/protocol.ts (keep in sync)."""

from __future__ import annotations

BOT_IDS = ("amr4", "hardware-bot")
HEARTBEAT_OFFLINE_MS = 3000
FULL_RATE_HZ = 10
BOARD_RATE_HZ = 1
BRIDGE_VERSION = "0.1.0"


def channel_name(bot_id: str) -> str:
    return f"amr-bot-{bot_id}"
