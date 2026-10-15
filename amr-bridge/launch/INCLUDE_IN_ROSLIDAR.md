# Include `amr_bridge` in Ros_lidar_bot bringup

Yes — start the cloud bridge from the same launch file as the dashboard.

For the **full agent playbook** (secrets, venv, joy/camera, verify, failure matrix) see  
[`../AGENT_SETUP_AND_INTEGRATION.md`](../AGENT_SETUP_AND_INTEGRATION.md).

## 1. Copy onto the Pi package

From this repo’s `amr-bridge/`:

```bash
# On the Pi, inside the Ros_lidar_bot package share/src:
cp launch/amr_bridge.launch.py   ~/ros2_ws/src/Ros_lidar_bot/launch/
cp -r amr_telemetry_bridge       ~/ros2_ws/src/Ros_lidar_bot/amr_bridge_py/   # or pip install -e amr-bridge
cp scripts/run_bridge.sh         ~/ros2_ws/src/Ros_lidar_bot/scripts/
sudo cp .env.pi                  /etc/amr-bridge.env && sudo chmod 600 /etc/amr-bridge.env
pip3 install -r requirements.txt
```

## 2. Add to your bringup launch

```python
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

bridge = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('Ros_lidar_bot'), 'launch', 'amr_bridge.launch.py')
    ),
    launch_arguments={
        'bot_id': 'amr4',                 # or hardware-bot
        'env_file': '/etc/amr-bridge.env',
        'dashboard': 'http://127.0.0.1:8080',
    }.items(),
)

# Delay so robot_dashboard_node is listening on :8080 first
delayed_bridge = TimerAction(period=5.0, actions=[bridge])
```

Add `delayed_bridge` to the bringup `LaunchDescription` next to the dashboard node.

## 3. Standalone smoke test

```bash
ros2 launch Ros_lidar_bot amr_bridge.launch.py bot_id:=amr4
# or
AMR_ENV_FILE=/etc/amr-bridge.env ./scripts/run_bridge.sh
```

Then open https://amrslam.tech/control/ as `amey@amrslam.tech` (Owner rank).
