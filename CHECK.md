# BNO055 IMU Check

Use this checklist on the Raspberry Pi 5 to confirm that `imu_data_node.py`
reads the BNO055 over I2C and publishes usable data to `/imu` for the EKF.

## 1. Check I2C Is Enabled

Run:

```bash
i2cdetect -y 1
```

Expected BNO055 address:

```text
0x28
```

or:

```text
0x29
```

If no address appears, check wiring, power, ground, SDA, SCL, and whether I2C is
enabled on the Raspberry Pi.

## 2. Install Python BNO055 Library

Run:

```bash
pip3 install adafruit-circuitpython-bno055
```

The node uses:

```python
board.I2C()
adafruit_bno055.BNO055_I2C(...)
```

to read directly from the BNO055 hardware.

## 3. Build The ROS 2 Package

From the workspace root:

```bash
cd ~/Desktop/ros2_ws
colcon build --packages-select Ros_lidar_bot
source install/setup.bash
```

## 4. Run The IMU Node

If the BNO055 address is `0x28`, run:

```bash
ros2 run Ros_lidar_bot imu_data_node.py
```

If the BNO055 address is `0x29`, run:

```bash
ros2 run Ros_lidar_bot imu_data_node.py --ros-args -p i2c_address:=41
```

The node publishes to:

```text
/imu
```

## 5. Check The `/imu` Topic

In another terminal:

```bash
source ~/Desktop/ros2_ws/install/setup.bash
ros2 topic echo /imu
```

You should see a `sensor_msgs/msg/Imu` message with:

```text
orientation
angular_velocity
linear_acceleration
```

For this robot, `imu_data_node.py` publishes yaw orientation, yaw angular
velocity, and linear acceleration for EKF use.

## 6. EKF Settings For Real Hardware

For Raspberry Pi hardware, `config/ekf.yaml` should use:

```yaml
use_sim_time: false
```

Because the node uses BNO055 `linear_acceleration`, gravity is already removed.
Use:

```yaml
imu0_remove_gravitational_acceleration: false
```

The EKF is configured to consume:

```text
yaw orientation
yaw angular velocity
linear acceleration x
```

from `/imu`.

## 7. Quick Debug Commands

List ROS topics:

```bash
ros2 topic list
```

Check `/imu` type:

```bash
ros2 topic type /imu
```

Expected:

```text
sensor_msgs/msg/Imu
```

Check publish rate:

```bash
ros2 topic hz /imu
```

Expected default rate:

```text
about 30 Hz
```

