"""Pure velocity policy for safety_stop_node.

No ROS imports, so the rules can be tested without a graph or hardware.
"""

# Topics the hardware launch wires together. Absolute names: the publishers
# use leading slashes, so a launch remap cannot move them.
CMD_VEL_IN = "/cmd_vel"
CMD_VEL_OUT = "/cmd_vel_safe"
EMERGENCY_STOP_TOPIC = "/emergency_stop"
SAFETY_NODE_NAME = "safety_stop"
DRIVER_NODE_NAME = "driver_node"


def filter_velocity(
    linear_x: float,
    angular_z: float,
    *,
    estop: bool,
    odom_stale: bool,
    blocked_fwd: bool,
    blocked_bwd: bool,
) -> tuple[float, float]:
    """Return ``(linear.x, angular.z)`` after the safety policy.

    E-STOP and a dead wheel-odometry feed zero both axes. They outrank the
    scan gate, so a teleop stick or Nav2 command cannot outvote a stop.
    The scan gate drops forward speed while the front arc is blocked and
    reverse speed while the rear arc is blocked. Rotation is kept so an
    in-place recovery can still turn.
    """
    if estop or odom_stale:
        return 0.0, 0.0
    out_x = float(linear_x)
    if blocked_fwd and out_x > 0.0:
        out_x = 0.0
    elif blocked_bwd and out_x < 0.0:
        out_x = 0.0
    return out_x, float(angular_z)
