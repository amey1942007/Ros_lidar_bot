#!/usr/bin/env bash
# Install amr_telemetry_bridge as a systemd service.
#
# Modes:
#   --local     On the robot: dashboard is http://127.0.0.1:8080
#   --relay     On an always-on Tailscale host: pull from the Pi over the tailnet
#
# Usage (local Pi):
#   bash amr-bridge/scripts/install_bridge_service.sh --local
#
# Usage (Tailscale relay laptop/desktop/VPS that can reach Supabase):
#   bash amr-bridge/scripts/install_bridge_service.sh --relay \
#     --bot hardware-bot \
#     --dashboard http://hardware-bot:8080
#
# Why relay: campus / IITD Wi-Fi often blocks HTTPS to *.supabase.co from the Pi.
# The relay sits on Tailscale + a network that can reach Supabase, and pulls SSE
# from the robot over MagicDNS. /control still only talks to Supabase.
set -euo pipefail

MODE=""
BOT_ID="hardware-bot"
DASHBOARD="http://127.0.0.1:8080"
GAMEPAD="http://127.0.0.1:8765"
ENV_SRC=""
INSTALL_ROOT="/opt/amr-bridge"
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BRIDGE_SRC="$REPO_ROOT/amr-bridge"

say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31mxx %s\033[0m\n' "$*"; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --local) MODE=local; shift ;;
    --relay) MODE=relay; shift ;;
    --bot) BOT_ID="$2"; shift 2 ;;
    --dashboard) DASHBOARD="$2"; shift 2 ;;
    --gamepad) GAMEPAD="$2"; shift 2 ;;
    --env) ENV_SRC="$2"; shift 2 ;;
    --install-root) INSTALL_ROOT="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,25p' "$0"; exit 0 ;;
    *) die "unknown arg: $1" ;;
  esac
done

[ -n "$MODE" ] || die "pass --local or --relay (see --help)"
[ -d "$BRIDGE_SRC/amr_telemetry_bridge" ] || die "amr-bridge not found at $BRIDGE_SRC"
[[ "$BOT_ID" == "hardware-bot" || "$BOT_ID" == "amr4" ]] || die "bot must be hardware-bot or amr4"

if [ "$MODE" = "relay" ]; then
  ENV_FILE="/etc/amr-bridge-${BOT_ID}.env"
  UNIT_DST="/etc/systemd/system/amr-bridge@.service"
  UNIT_ENABLE="amr-bridge@${BOT_ID}.service"
else
  ENV_FILE="/etc/amr-bridge.env"
  UNIT_DST="/etc/systemd/system/amr-bridge.service"
  UNIT_ENABLE="amr-bridge.service"
fi

say "1/5 Installing bridge tree to $INSTALL_ROOT"
sudo mkdir -p "$INSTALL_ROOT"
sudo rsync -a --delete \
  --exclude '.venv' --exclude '__pycache__' --exclude '.env*' \
  "$BRIDGE_SRC/" "$INSTALL_ROOT/"
sudo chmod +x "$INSTALL_ROOT/scripts/"*.sh

say "2/5 Creating venv + deps"
if [ ! -x "$INSTALL_ROOT/.venv/bin/python" ]; then
  sudo python3 -m venv "$INSTALL_ROOT/.venv"
fi
sudo "$INSTALL_ROOT/.venv/bin/pip" install -U pip
sudo "$INSTALL_ROOT/.venv/bin/pip" install -r "$INSTALL_ROOT/requirements.txt"
sudo "$INSTALL_ROOT/.venv/bin/python" -c \
  "import sys; sys.path.insert(0,'.'); from amr_telemetry_bridge.bridge import assert_realtime_api; assert_realtime_api(); print('realtime OK')" \
  || die "realtime API check failed"

say "3/5 Writing $ENV_FILE (created once; never overwritten)"
if [ -n "$ENV_SRC" ]; then
  sudo install -m 600 -o root -g root "$ENV_SRC" "$ENV_FILE"
elif [ ! -f "$ENV_FILE" ]; then
  sudo install -m 600 -o root -g root /dev/null "$ENV_FILE"
  sudo tee "$ENV_FILE" >/dev/null <<EOF
# amr_telemetry_bridge — ${MODE} mode for ${BOT_ID}
AMR_BOT_ID=${BOT_ID}
AMR_SUPABASE_URL=https://khucmrlbstdlbxcuboub.supabase.co
AMR_SUPABASE_KEY=
AMR_BRIDGE_TOKEN=
AMR_BRIDGE_EMAIL=
AMR_BRIDGE_PASSWORD=
AMR_PUBLIC_CHANNEL=0
AMR_PRIVATE_CHANNEL=1
AMR_DASHBOARD_URL=${DASHBOARD}
AMR_GAMEPAD_URL=${GAMEPAD}
# How long systemd waits for the dashboard before starting the bridge anyway.
AMR_DASHBOARD_WAIT_S=90
# Do NOT set AMR_SUPABASE_SERVICE_KEY.
EOF
  echo "Fill secrets: sudo nano $ENV_FILE"
else
  echo "$ENV_FILE already exists — leaving it alone."
fi

say "4/5 Installing systemd unit"
RUN_USER="${SUDO_USER:-$USER}"
RUN_GROUP="$(id -gn "$RUN_USER")"
if [ "$MODE" = "relay" ]; then
  tmp="$(mktemp)"
  sed -e "s/REPLACE_USER/${RUN_USER}/g" -e "s/REPLACE_GROUP/${RUN_GROUP}/g" \
    "$INSTALL_ROOT/systemd/amr-bridge@.service" > "$tmp"
  sudo install -m 644 "$tmp" "$UNIT_DST"
  rm -f "$tmp"
else
  tmp="$(mktemp)"
  # Local unit: run as the installing user, not as a template.
  sed -e "s/^User=%i$/User=${RUN_USER}/" \
      -e "s|^WorkingDirectory=.*|WorkingDirectory=${INSTALL_ROOT}|" \
      -e "s|^ExecStart=.*|ExecStart=${INSTALL_ROOT}/.venv/bin/python -u -m amr_telemetry_bridge|" \
      -e "s|^ReadOnlyPaths=.*|ReadOnlyPaths=${ENV_FILE} ${INSTALL_ROOT}|" \
      "$INSTALL_ROOT/systemd/amr-bridge.service" > "$tmp"
  # Point EnvironmentFile + add ExecStartPre wait.
  if ! grep -q wait_for_dashboard "$tmp"; then
    sed -i "/^ExecStart=/i ExecStartPre=${INSTALL_ROOT}/scripts/wait_for_dashboard.sh" "$tmp"
  fi
  sudo install -m 644 "$tmp" "$UNIT_DST"
  rm -f "$tmp"
fi

say "5/5 Enabling $UNIT_ENABLE"
sudo systemctl daemon-reload
echo
echo "Next:"
echo "  1) sudo nano $ENV_FILE   # set AMR_SUPABASE_KEY + AMR_BRIDGE_TOKEN (+ email/password)"
if [ "$MODE" = "relay" ]; then
  echo "  2) On the Pi: bash $INSTALL_ROOT/scripts/pi_tailscale_serve.sh   # or keep :8080 open on Tailscale"
  echo "  3) From this host: curl -I ${DASHBOARD}/   # must work over Tailscale"
fi
echo "  4) sudo systemctl enable --now $UNIT_ENABLE"
echo "  5) journalctl -u $UNIT_ENABLE -f"
echo "  6) https://amrslam.tech/control/  → bot should go Live within a few seconds"
echo
echo "Disable ROS-launched bridge to avoid a double process:"
echo "  ros2 launch … enable_amr_bridge:=false"
