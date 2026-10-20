#!/usr/bin/env bash
# Wait until AMR_DASHBOARD_URL answers (local or Tailscale MagicDNS).
# Used by systemd so the bridge does not thrash while the robot is booting.
set -euo pipefail

SOURCE="$(echo "${AMR_SOURCE:-dashboard}" | tr '[:upper:]' '[:lower:]')"
if [[ "$SOURCE" == "ros" || "$SOURCE" == "rclpy" || "$SOURCE" == "direct" ]]; then
  echo "[amr-bridge] AMR_SOURCE=$SOURCE — skipping dashboard wait"
  exit 0
fi

BASE="${AMR_DASHBOARD_URL:-http://127.0.0.1:8080}"
BASE="${BASE%/}"
URL="${BASE}/events"
TIMEOUT_S="${AMR_DASHBOARD_WAIT_S:-90}"
deadline=$((SECONDS + TIMEOUT_S))

echo "[amr-bridge] waiting up to ${TIMEOUT_S}s for dashboard at ${URL}"
while (( SECONDS < deadline )); do
  code="$(curl -s -m 3 -o /dev/null -w '%{http_code}' -H 'Accept: text/event-stream' "$URL" || true)"
  # SSE may hang open (000 after timeout) or return 200. Treat 2xx/3xx as up;
  # also accept a hung stream that already connected (curl 28 with partial).
  if [[ "$code" =~ ^[23][0-9][0-9]$ ]]; then
    echo "[amr-bridge] dashboard reachable (HTTP ${code})"
    exit 0
  fi
  # Alternate probe: plain GET on /
  root="$(curl -s -m 2 -o /dev/null -w '%{http_code}' "${BASE}/" || true)"
  if [[ "$root" =~ ^[23][0-9][0-9]$ ]]; then
    echo "[amr-bridge] dashboard root reachable (HTTP ${root})"
    exit 0
  fi
  sleep 2
done

echo "[amr-bridge] dashboard still unreachable after ${TIMEOUT_S}s — starting anyway (SSE loop will retry)" >&2
exit 0
