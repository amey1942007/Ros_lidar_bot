"""ROS 2 launch: start the cloud telemetry bridge beside the robot dashboard.

Include from Ros_lidar_bot bringup (after the dashboard is up), or run:

    ros2 launch Ros_lidar_bot amr_bridge.launch.py bot_id:=hardware-bot

Env file (required on the Pi): /etc/amr-bridge.env — see amr-bridge/.env.pi.example

Precedence (launch arg wins over the env file when the arg is non-empty):
  bot_id    -> AMR_BOT_ID
  dashboard -> AMR_DASHBOARD_URL
  gamepad   -> AMR_GAMEPAD_URL, except none/off/- which unsets it
               (joy then falls back to /api/nudge).
File values override the process environment. AMR_SUPABASE_URL,
AMR_SUPABASE_KEY (publishable), and AMR_BRIDGE_TOKEN are required.
AMR_SUPABASE_SERVICE_KEY is never passed to the bridge. Private channels
also need AMR_BRIDGE_EMAIL and AMR_BRIDGE_PASSWORD; without them the legacy
public channel still runs when AMR_PUBLIC_CHANNEL is on (the default).
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _load_env_file(path: Path) -> tuple[dict[str, str], str | None]:
    """Return (vars, error). A missing file is ({}, None). Read failures do not raise."""
    out: dict[str, str] = {}
    try:
        if not path.is_file():
            return out, None
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        return {}, f"cannot read {path}: {exc}"
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("export "):
            line = line[len("export "):].strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, _, v = line.partition("=")
        out[k.strip()] = v.strip().strip('"').strip("'")
    return out, None


def _missing_required(env: dict[str, str]) -> list[str]:
    """URL, publishable key, and bridge token. The service key does not count."""
    missing: list[str] = []
    if not env.get("AMR_SUPABASE_URL"):
        missing.append("AMR_SUPABASE_URL")
    if not env.get("AMR_SUPABASE_KEY"):
        missing.append("AMR_SUPABASE_KEY")
    if not env.get("AMR_BRIDGE_TOKEN"):
        missing.append("AMR_BRIDGE_TOKEN")
    return missing


def _resolve_bridge_root() -> Path | None:
    """Locate vendored amr-bridge so `python -m amr_telemetry_bridge` can import."""
    env_root = os.environ.get("AMR_BRIDGE_ROOT", "").strip()
    candidates: list[Path] = []
    if env_root:
        candidates.append(Path(env_root).expanduser())
    try:
        share = Path(get_package_share_directory("Ros_lidar_bot"))
        candidates.append(share / "amr-bridge")
    except Exception:
        pass
    # Source tree: <repo>/launch/amr_bridge.launch.py → <repo>/amr-bridge
    candidates.append(Path(__file__).resolve().parents[1] / "amr-bridge")
    for cand in candidates:
        try:
            if (cand / "amr_telemetry_bridge").is_dir():
                return cand
        except OSError:
            continue
    return None


def _python_candidate_ok(path: Path) -> bool:
    try:
        return path.is_file() and os.access(path, os.X_OK)
    except OSError:
        return False


def _resolve_python(arg: str, bridge_root: Path | None) -> tuple[str, str | None]:
    """Pick the bridge interpreter.

    An explicit arg (anything other than "" or "auto") wins, with ~ expanded.
    Otherwise the first executable candidate is used. The returned venv path
    is never resolve()'d: a venv python is a symlink to the system interpreter,
    and resolving it drops the venv.
    Returns (executable, note). note is set only when falling back to python3.
    """
    raw = (arg or "").strip()
    if raw and raw != "auto":
        return str(Path(raw).expanduser()), None

    candidates: list[Path] = []
    env_py = os.environ.get("AMR_BRIDGE_PYTHON", "").strip()
    if env_py:
        candidates.append(Path(env_py).expanduser())
    if bridge_root is not None:
        candidates.append(bridge_root / ".venv" / "bin" / "python")
    candidates.append(
        Path(__file__).resolve().parents[1] / "amr-bridge" / ".venv" / "bin" / "python"
    )
    try:
        share = Path(get_package_share_directory("Ros_lidar_bot"))
        candidates.append(
            share.parents[3]
            / "src"
            / "Ros_lidar_bot"
            / "amr-bridge"
            / ".venv"
            / "bin"
            / "python"
        )
    except Exception:
        pass

    for cand in candidates:
        if _python_candidate_ok(cand):
            # absolute() keeps the venv symlink; resolve() would not.
            return str(cand.absolute()), None
    return "python3", "[amr_bridge] no venv python found; using python3"


# Process variables the bridge may need. Everything else (ROS PYTHONPATH,
# PYTHONHOME, AMENT_*, secrets that are not AMR_*) stays out. ExecuteProcess
# env= replaces the environment; additional_env= would merge and ignore pops.
_PASSTHROUGH_ENV = (
    "PATH",
    "HOME",
    "USER",
    "LOGNAME",
    "LANG",
    "LC_ALL",
    "LC_CTYPE",
    "TMPDIR",
    "TMP",
    "TEMP",
    "SSL_CERT_FILE",
    "SSL_CERT_DIR",
    "REQUESTS_CA_BUNDLE",
    "CURL_CA_BUNDLE",
)


def _isolated_base_env(file_env: dict[str, str]) -> dict[str, str]:
    env: dict[str, str] = {}
    for key in _PASSTHROUGH_ENV:
        val = os.environ.get(key)
        if val:
            env[key] = val
    for key, val in os.environ.items():
        if key.startswith("AMR_") or key.startswith("NEXT_PUBLIC_"):
            env[key] = val
    env.update(file_env)
    env.pop("PYTHONHOME", None)
    env.pop("PYTHONPATH", None)
    return env


def _skipped(reason: str, fix: str) -> LogInfo:
    return LogInfo(msg=f"[amr_bridge] SKIPPED: {reason}. Fix: {fix}")


def _launch_bridge(context, *args, **kwargs):
    try:
        return _launch_bridge_body(context)
    except Exception as exc:
        return [_skipped(
            f"unexpected error ({exc})",
            "check the message above; the robot launch is left running",
        )]


def _launch_bridge_body(context):
    bot_id = LaunchConfiguration("bot_id").perform(context)
    env_file = LaunchConfiguration("env_file").perform(context)
    dashboard = LaunchConfiguration("dashboard").perform(context)
    gamepad = LaunchConfiguration("gamepad").perform(context)
    python_arg = LaunchConfiguration("python").perform(context)

    file_env, read_error = _load_env_file(Path(env_file).expanduser())
    notes: list[LogInfo] = []
    if read_error:
        # systemd EnvironmentFile= (or a sourced shell) may already have the secrets.
        if _missing_required(os.environ):
            return [_skipped(
                f"cannot read {env_file} ({read_error})",
                f"sudo chown $USER:$USER {env_file}",
            )]
        notes.append(LogInfo(
            msg=f"[amr_bridge] {read_error}; continuing with the process environment",
        ))
        file_env = {}

    # Child environment is only the passthrough keys plus AMR_* / file values.
    # env= replaces the parent environment, so a missing key is really gone.
    env = _isolated_base_env(file_env)
    # bot_id's default is non-empty, so the launch arg overrides AMR_BOT_ID.
    env["AMR_BOT_ID"] = bot_id or env.get("AMR_BOT_ID", "hardware-bot")
    if dashboard:
        env["AMR_DASHBOARD_URL"] = dashboard
    gamepad_key = gamepad.strip().lower()
    if gamepad_key in ("none", "off", "-"):
        env.pop("AMR_GAMEPAD_URL", None)
    elif gamepad.strip():
        env["AMR_GAMEPAD_URL"] = gamepad.strip()

    missing = _missing_required(env)
    if missing:
        return [_skipped(
            f"missing {missing}",
            f"put them in {env_file} (AMR_SUPABASE_URL, AMR_SUPABASE_KEY, "
            "and AMR_BRIDGE_TOKEN). Do not use AMR_SUPABASE_SERVICE_KEY.",
        )]
    if env.get("AMR_SUPABASE_SERVICE_KEY"):
        notes.append(LogInfo(
            msg="[amr_bridge] AMR_SUPABASE_SERVICE_KEY is set and will not be "
                "passed to the bridge. Use AMR_SUPABASE_KEY (publishable key) only.",
        ))
        env.pop("AMR_SUPABASE_SERVICE_KEY", None)

    bridge_root = _resolve_bridge_root()
    if bridge_root is None:
        return [_skipped(
            "amr_telemetry_bridge package not found",
            "set AMR_BRIDGE_ROOT or install amr-bridge under the package share",
        )]

    python_exe, python_note = _resolve_python(python_arg, bridge_root)
    if python_note:
        notes.append(LogInfo(msg=python_note))

    env["PYTHONPATH"] = str(bridge_root)
    env["PYTHONUNBUFFERED"] = "1"

    # Same isolated env the process will get: venv imports, and the bridge
    # package via PYTHONPATH (not the ROS site-packages).
    probe_code = "import supabase, httpx, amr_telemetry_bridge"
    try:
        probe = subprocess.run(
            [python_exe, "-c", probe_code],
            env=env,
            capture_output=True,
            timeout=20,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return notes + [_skipped(
            f"{python_exe} cannot import supabase/httpx/amr_telemetry_bridge ({exc})",
            "create the venv: see amr-bridge/PI_SETUP.md",
        )]
    if probe.returncode != 0:
        err = (probe.stderr or b"").decode("utf-8", "replace").strip().splitlines()
        first = err[0] if err else f"exit {probe.returncode}"
        return notes + [_skipped(
            f"{python_exe} cannot import supabase/httpx/amr_telemetry_bridge ({first})",
            "create the venv: see amr-bridge/PI_SETUP.md",
        )]

    cmd = [python_exe, "-u", "-m", "amr_telemetry_bridge"]
    return notes + [
        LogInfo(msg=f"[amr_bridge] starting bot_id={env['AMR_BOT_ID']} "
                    f"env_file={env_file} root={bridge_root} python={python_exe}"),
        ExecuteProcess(
            cmd=cmd,
            name="amr_telemetry_bridge",
            output="screen",
            env=env,
            respawn=True,
            respawn_delay=3.0,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "bot_id",
            default_value="hardware-bot",
            description="Supabase bot id. Overrides AMR_BOT_ID from the env file "
                        "(default hardware-bot is non-empty, so the file value is ignored).",
        ),
        DeclareLaunchArgument("env_file", default_value="/etc/amr-bridge.env",
                              description="Path to Pi secrets env file"),
        DeclareLaunchArgument(
            "dashboard",
            default_value="http://127.0.0.1:8080",
            description="Local robot_dashboard HTTP base. Overrides AMR_DASHBOARD_URL "
                        "from the env file.",
        ),
        DeclareLaunchArgument(
            "gamepad",
            default_value="http://127.0.0.1:8765",
            description="Local web_gamepad HTTP base. Overrides AMR_GAMEPAD_URL from the "
                        "env file. none, off, or - unsets AMR_GAMEPAD_URL so joy falls "
                        "back to nudge (use this when web_gamepad is not running).",
        ),
        DeclareLaunchArgument(
            "python",
            default_value="auto",
            description="auto = <repo>/amr-bridge/.venv/bin/python if present, else python3; "
                        "or give an explicit interpreter path",
        ),
        OpaqueFunction(function=_launch_bridge),
    ])
