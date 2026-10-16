#!/usr/bin/env bash
# AMR telemetry bridge setup for the HARDWARE bot (Raspberry Pi 5, ROS 2 Jazzy).
# NOT for AMR4. Does everything except typing your secrets.
# Usage:  bash setup_amr_bridge.sh          (run as your normal user, not root)
set -euo pipefail

REPO="$HOME/Desktop/ros2_ws/src/Ros_lidar_bot"
WS="$HOME/Desktop/ros2_ws"
BRANCH="hardware"
ENV_FILE="/etc/amr-bridge.env"
VENV="$REPO/amr-bridge/.venv"
SUPABASE_URL="https://khucmrlbstdlbxcuboub.supabase.co"

say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m!! %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31mxx %s\033[0m\n' "$*"; exit 1; }

[ "$(id -u)" -ne 0 ] || die "Run as your normal user (not root / not with sudo). The script calls sudo only where needed."
[ -d /opt/ros/jazzy ] || die "ROS 2 Jazzy not found at /opt/ros/jazzy. This script is for the hardware bot (Pi 5, Jazzy) only."
[ -d "$REPO/.git" ] || die "Repo not found at $REPO"

say "1/8 Checking the Pi can reach Supabase (IITD Wi-Fi can block things)"
code="$(curl -s -m 10 -o /dev/null -w '%{http_code}' "$SUPABASE_URL/auth/v1/health" -H "apikey: x" || true)"
if [ -n "$code" ] && [ "$code" != "000" ]; then
  echo "HTTPS to Supabase works."
else
  warn "Cannot reach $SUPABASE_URL over HTTPS. Check Wi-Fi/firewall (try a phone hotspot). Continuing anyway."
fi

say "2/8 Updating the repo to branch '$BRANCH' (saving a rollback commit first)"
cd "$REPO"
# First run only. A re-run is already on $BRANCH, so overwriting would lose the original commit and branch.
if [ ! -f "$HOME/amr_bridge_rollback_commit.txt" ]; then
  git rev-parse --short HEAD > "$HOME/amr_bridge_rollback_commit.txt"
  git rev-parse --abbrev-ref HEAD > "$HOME/amr_bridge_rollback_branch.txt"
fi
echo "Rollback commit: $(cat "$HOME/amr_bridge_rollback_commit.txt") (in ~/amr_bridge_rollback_commit.txt)"
if [ -f "$HOME/amr_bridge_rollback_branch.txt" ]; then
  echo "Rollback branch: $(cat "$HOME/amr_bridge_rollback_branch.txt") (in ~/amr_bridge_rollback_branch.txt)"
fi
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  die "You have uncommitted changes in $REPO. Commit or stash them first (git stash), then re-run. Nothing was changed."
fi
git fetch origin
git checkout "$BRANCH"
git pull --ff-only origin "$BRANCH"
[ -d amr-bridge/amr_telemetry_bridge ] || die "amr-bridge/ not found on branch $BRANCH. Is the bridge merged?"
git log --oneline -3

say "3/8 Installing system packages (sudo password may be asked)"
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-aiohttp git curl python3-colcon-common-extensions python3-rosdep

say "4/8 ROS dependencies (rosdep)"
# shellcheck disable=SC1091
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [ ! -d /etc/ros/rosdep/sources.list.d ]; then sudo rosdep init || true; fi
rosdep update || warn "rosdep update failed (network?). Continuing."
rosdep install --from-paths "$REPO" --ignore-src -r -y || warn "rosdep reported problems. Continuing."

say "5/8 Python virtual environment for the bridge (Ubuntu 24.04 blocks system-wide pip)"
cd "$REPO/amr-bridge"
[ -d "$VENV" ] || python3 -m venv "$VENV"
"$VENV/bin/pip" install -U pip
"$VENV/bin/pip" install -r requirements.txt
"$VENV/bin/pip" freeze | grep -Ei '^(supabase|httpx|websockets|realtime|postgrest)' | tee "$HOME/amr_bridge_pip_versions.txt" || true
# bridge.py imports httpx, and supabase (create_client) when the hub starts.
# It does not import websockets or realtime itself.
"$VENV/bin/python" -c "import sys; sys.path.insert(0,'.'); import httpx; from supabase import create_client; import amr_telemetry_bridge.bridge; print('bridge import OK')" \
  || die "The bridge failed to import. See the error above."

say "6/8 Creating the secrets file $ENV_FILE (owned by YOU, not root; never overwritten if it exists)"
if [ -f "$ENV_FILE" ]; then
  echo "$ENV_FILE already exists, leaving it untouched."
  sudo chown "$USER:$USER" "$ENV_FILE"; sudo chmod 600 "$ENV_FILE"
else
  sudo install -m 600 -o "$USER" -g "$USER" /dev/null "$ENV_FILE"
  cat > "$ENV_FILE" <<ENVEOF
# amr_telemetry_bridge settings for the HARDWARE bot. Keep this file private.
AMR_BOT_ID=hardware-bot
AMR_SUPABASE_URL=$SUPABASE_URL
# >>> YOU FILL THESE TWO <<<
# Publishable key (starts with sb_publishable_...): Supabase dashboard > Project Settings > API Keys. NOT the service_role / secret key.
AMR_SUPABASE_KEY=
# Bridge token: the plaintext from the website owner's private amr-bridge/.env.pi
AMR_BRIDGE_TOKEN=
# Local robot services
AMR_DASHBOARD_URL=http://127.0.0.1:8080
# Joystick endpoint (leave this line out for a telemetry + nudge-only test)
AMR_GAMEPAD_URL=http://127.0.0.1:8765
# Optional: public camera URL (empty = bridge uses the local /camera.mjpg)
# AMR_CAMERA_URL=
# Do NOT ever add AMR_SUPABASE_SERVICE_KEY on the robot.
ENVEOF
fi
ls -l "$ENV_FILE"

say "7/8 Building the ROS package"
cd "$WS"
colcon build --packages-select Ros_lidar_bot --symlink-install
set +u; source "$WS/install/setup.bash"; set -u

say "8/8 Writing the launcher ~/run_robot_with_bridge.sh"
cat > "$HOME/run_robot_with_bridge.sh" <<'RUNEOF'
#!/usr/bin/env bash
# Starts the hardware robot + the cloud bridge. Refuses to start if secrets are missing.
set -euo pipefail
ENV_FILE=/etc/amr-bridge.env
REPO="$HOME/Desktop/ros2_ws/src/Ros_lidar_bot"
for k in AMR_SUPABASE_KEY AMR_BRIDGE_TOKEN; do
  # || true: a missing key makes grep exit 1, which would abort under set -e/pipefail.
  v="$(grep -E "^$k=" "$ENV_FILE" | head -1 | cut -d= -f2- || true)"
  v="${v%$'\r'}"
  v="${v#\"}"; v="${v%\"}"
  v="${v#\'}"; v="${v%\'}"
  [ -n "$v" ] || { echo "Missing $k in $ENV_FILE. Add it first (nano $ENV_FILE)."; exit 1; }
done
echo "SAFETY: anyone with the public Supabase key can currently drive the bot via the bridge (review finding C1)."
echo "Keep the wheels lifted / the bot supervised. To run WITHOUT the bridge: add  enable_amr_bridge:=false"
read -r -p "Type 'yes' to continue: " a; [ "$a" = "yes" ] || exit 1
set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/Desktop/ros2_ws/install/setup.bash"; set -u
exec ros2 launch Ros_lidar_bot launch_robot.launch.py \
  amr_bot_id:=hardware-bot \
  amr_bridge_python:="$REPO/amr-bridge/.venv/bin/python" "$@"
RUNEOF
chmod +x "$HOME/run_robot_with_bridge.sh"

cat <<DONE

Setup finished. Your only manual step:
  1) nano $ENV_FILE      -> fill AMR_SUPABASE_KEY and AMR_BRIDGE_TOKEN (save with Ctrl+O, exit Ctrl+X)
  2) ~/run_robot_with_bridge.sh
Then check: ps aux | grep amr_telemetry_bridge
            amrslam.tech/control -> pick "Hardware bot" -> online in ~3 s
Rollback: git -C $REPO checkout "\$(cat ~/amr_bridge_rollback_branch.txt)"
DONE
