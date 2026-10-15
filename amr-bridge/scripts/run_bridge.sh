#!/usr/bin/env bash
# Start amr_telemetry_bridge (used by systemd or ros2 ExecuteProcess).
set -euo pipefail
ENV_FILE="${AMR_ENV_FILE:-/etc/amr-bridge.env}"
if [[ -f "$ENV_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="${ROOT}${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m amr_telemetry_bridge "$@"
