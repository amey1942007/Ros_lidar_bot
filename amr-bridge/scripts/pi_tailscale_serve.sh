#!/usr/bin/env bash
# Expose the robot dashboard + gamepad inside the Tailscale tailnet only
# (Tailscale Serve — NOT Funnel). Other tailnet nodes can then run the bridge
# as a relay: AMR_DASHBOARD_URL=http://<this-hostname>:8080
#
# Run on the Pi (once, or via systemd). Requires Tailscale with HTTPS/MagicDNS.
set -euo pipefail

DASH_PORT="${AMR_DASHBOARD_PORT:-8080}"
GAME_PORT="${AMR_GAMEPAD_PORT:-8765}"

command -v tailscale >/dev/null || { echo "tailscale CLI not found"; exit 1; }

echo "Serving robot dashboard on the tailnet (Serve, not public Funnel)…"
# Background persistent config (Tailscale 1.52+).
sudo tailscale serve --bg --yes "${DASH_PORT}" || \
  sudo tailscale serve --bg "${DASH_PORT}"

if curl -s -m 1 -o /dev/null "http://127.0.0.1:${GAME_PORT}/" 2>/dev/null; then
  echo "Also serving web gamepad on :${GAME_PORT} (path /gamepad)…"
  sudo tailscale serve --bg --yes --set-path=/gamepad "${GAME_PORT}" || \
    sudo tailscale serve --bg --set-path=/gamepad "${GAME_PORT}" || true
fi

echo
sudo tailscale serve status || true
HOST="$(tailscale status --json 2>/dev/null | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("Self",{}).get("DNSName","").rstrip("."))' 2>/dev/null || true)"
if [ -n "$HOST" ]; then
  echo
  echo "On any always-on Tailscale machine, set:"
  echo "  AMR_DASHBOARD_URL=http://${HOST%:*}:8080"
  echo "  # or HTTPS Serve URL if you use https://$HOST/"
  echo "  AMR_BOT_ID=hardware-bot   # or amr4"
  echo "Then: sudo systemctl enable --now amr-bridge@hardware-bot"
fi
