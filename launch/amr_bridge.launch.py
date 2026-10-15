"""ROS 2 launch: start the cloud telemetry bridge beside the robot dashboard.

Include from Ros_lidar_bot bringup (after the dashboard is up), or run:

    ros2 launch Ros_lidar_bot amr_bridge.launch.py bot_id:=hardware-bot

Env file (required on the Pi): /etc/amr-bridge.env — see amr-bridge/.env.pi.example
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _load_env_file(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.is_file():
        return out
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, _, v = line.partition("=")
        out[k.strip()] = v.strip().strip('"').strip("'")
    return out


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
        if (cand / "amr_telemetry_bridge").is_dir():
            return cand
    return None


def _launch_bridge(context, *args, **kwargs):
    bot_id = LaunchConfiguration("bot_id").perform(context)
    env_file = LaunchConfiguration("env_file").perform(context)
    dashboard = LaunchConfiguration("dashboard").perform(context)
    gamepad = LaunchConfiguration("gamepad").perform(context)
    python_exe = LaunchConfiguration("python").perform(context)

    file_env = _load_env_file(Path(env_file).expanduser())
    env = {**os.environ, **file_env}
    env["AMR_BOT_ID"] = bot_id or env.get("AMR_BOT_ID", "hardware-bot")
    if dashboard:
        env["AMR_DASHBOARD_URL"] = dashboard
    if gamepad and not env.get("AMR_GAMEPAD_URL"):
        env["AMR_GAMEPAD_URL"] = gamepad

    missing = [k for k in ("AMR_SUPABASE_URL", "AMR_SUPABASE_KEY", "AMR_BRIDGE_TOKEN") if not env.get(k)]
    # Back-compat: service role still works if set instead of KEY+TOKEN
    if missing and not env.get("AMR_SUPABASE_SERVICE_KEY"):
        return [
            LogInfo(msg=f"[amr_bridge] missing env {missing} — put them in {env_file}"),
        ]

    bridge_root = _resolve_bridge_root()
    if bridge_root is None:
        return [
            LogInfo(msg="[amr_bridge] amr_telemetry_bridge package not found — "
                        "set AMR_BRIDGE_ROOT or install amr-bridge under the package share"),
        ]

    pythonpath = str(bridge_root)
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{pythonpath}{os.pathsep}{existing}" if existing else pythonpath

    cmd = [python_exe, "-m", "amr_telemetry_bridge"]
    return [
        LogInfo(msg=f"[amr_bridge] starting bot_id={env['AMR_BOT_ID']} "
                    f"env_file={env_file} root={bridge_root}"),
        ExecuteProcess(
            cmd=cmd,
            name="amr_telemetry_bridge",
            output="screen",
            additional_env=env,
            respawn=True,
            respawn_delay=3.0,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("bot_id", default_value="hardware-bot",
                              description="amr4 | hardware-bot"),
        DeclareLaunchArgument("env_file", default_value="/etc/amr-bridge.env",
                              description="Path to Pi secrets env file"),
        DeclareLaunchArgument("dashboard", default_value="http://127.0.0.1:8080",
                              description="Local robot_dashboard_node HTTP base"),
        DeclareLaunchArgument("gamepad", default_value="http://127.0.0.1:8765",
                              description="Local web_gamepad_node HTTP base (Xbox /joy)"),
        DeclareLaunchArgument("python", default_value="python3",
                              description="Python used for the bridge venv/system"),
        OpaqueFunction(function=_launch_bridge),
    ])
