#!/usr/bin/env python3
"""Fail if anything other than safety_stop publishes the motor command.

The motor command topic is the geometry_msgs/Twist subscription of
``driver_node`` (launch_robot.launch.py sets it to ``/cmd_vel_safe``).
``safety_stop`` is the only node allowed to publish it. Nav2, teleop, the
dashboard, and the on-demand tools must publish ``/cmd_vel`` instead, and
safety_stop forwards a filtered command.

Run this on the Pi after the stack is up (same ROS domain as the robot):

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 scripts/check_cmd_vel_chain.py

Exit 0 when the chain is intact. Exit 1 when the graph violates it or the
driver/safety nodes are missing. Exit 2 when ROS 2 is not available.
"""

from __future__ import annotations

import argparse
import sys
import time

# Keep these aligned with Ros_lidar_bot.cmd_vel_filter. The script stays
# runnable from a checkout without the package installed.
SAFETY_NODE_NAME = "safety_stop"
DRIVER_NODE_NAME = "driver_node"
CMD_VEL_IN = "/cmd_vel"
TWIST_TYPE = "geometry_msgs/msg/Twist"


def _bare(node_name: str) -> str:
    return node_name.rstrip("/").split("/")[-1]


def _abs_topic(topic: str) -> str:
    if not topic:
        return topic
    return topic if topic.startswith("/") else "/" + topic


def evaluate_chain(
    driver_twist_topics: list[str],
    motor_publishers: list[str],
    safety_subscriptions: list[str] | None = None,
) -> tuple[bool, list[str]]:
    """Decide whether the live graph is allowed to command the motors.

    ``driver_twist_topics`` are the Twist topics ``driver_node`` subscribes
    to. ``motor_publishers`` are the bare node names publishing the single
    motor topic. ``safety_subscriptions`` is optional; when discovery
    returned some, ``/cmd_vel`` must be one of them.
    """
    topics = [_abs_topic(topic) for topic in driver_twist_topics]
    lines: list[str] = []
    if not topics:
        lines.append(
            f"FAIL: /{DRIVER_NODE_NAME} has no {TWIST_TYPE} subscription. "
            "The motor driver is not in this graph, or it is not up yet."
        )
        return False, lines
    if len(topics) != 1:
        lines.append(
            f"FAIL: /{DRIVER_NODE_NAME} subscribes to {len(topics)} Twist "
            f"topics ({', '.join(topics)}). It must consume only the safety "
            "node output."
        )
        return False, lines

    topic = topics[0]
    publishers = sorted({_bare(name) for name in motor_publishers if name})
    ok = True
    if publishers != [SAFETY_NODE_NAME]:
        ok = False
        if SAFETY_NODE_NAME not in publishers:
            shown = ", ".join(publishers) if publishers else "none"
            lines.append(
                f"FAIL: {topic} is not published by /{SAFETY_NODE_NAME} "
                f"(publishers: {shown})."
            )
        extra = [name for name in publishers if name != SAFETY_NODE_NAME]
        if extra:
            lines.append(
                f"FAIL: {topic} has publishers other than /{SAFETY_NODE_NAME}: "
                + ", ".join(f"/{name}" for name in extra)
                + ". Those commands bypass the safety stop."
            )
    else:
        lines.append(
            f"OK: /{DRIVER_NODE_NAME} subscribes only to {topic}, "
            f"published only by /{SAFETY_NODE_NAME}."
        )

    if safety_subscriptions is not None and len(safety_subscriptions) > 0:
        subs = sorted({_abs_topic(item) for item in safety_subscriptions})
        if CMD_VEL_IN not in subs:
            ok = False
            lines.append(
                f"FAIL: /{SAFETY_NODE_NAME} subscriptions {subs} do not "
                f"include {CMD_VEL_IN}, so motion sources miss the filter."
            )
        elif ok:
            lines.append(
                f"OK: /{SAFETY_NODE_NAME} subscribes to {CMD_VEL_IN}."
            )
    return ok, lines


def _node_pairs(node) -> list[tuple[str, str]]:
    return list(node.get_node_names_and_namespaces())


def _matching_nodes(node, bare_name: str) -> list[tuple[str, str]]:
    return [(name, ns) for name, ns in _node_pairs(node) if name == bare_name]


def _twist_topics(entries) -> list[str]:
    topics = []
    for topic, types in entries:
        if TWIST_TYPE in types:
            topics.append(_abs_topic(topic))
    return topics


def read_chain(node) -> dict:
    """One graph snapshot. Empty lists mean 'not visible yet'."""
    drivers = _matching_nodes(node, DRIVER_NODE_NAME)
    safeties = _matching_nodes(node, SAFETY_NODE_NAME)
    driver_topics: list[str] = []
    if len(drivers) == 1:
        name, ns = drivers[0]
        try:
            subs = node.get_subscriber_names_and_types_by_node(name, ns)
        except Exception:
            subs = []
        driver_topics = _twist_topics(subs)

    motor_publishers: list[str] = []
    if len(driver_topics) == 1:
        try:
            infos = node.get_publishers_info_by_topic(driver_topics[0])
        except Exception:
            infos = []
        for info in infos:
            topic_type = getattr(info, "topic_type", TWIST_TYPE)
            if topic_type not in (TWIST_TYPE, ""):
                # A different type on this name is still a publisher and
                # still a bypass. Keep the node name so evaluate fails.
                pass
            motor_publishers.append(_bare(info.node_name))

    safety_subs: list[str] = []
    if len(safeties) == 1:
        name, ns = safeties[0]
        try:
            subs = node.get_subscriber_names_and_types_by_node(name, ns)
        except Exception:
            subs = []
        safety_subs = [_abs_topic(topic) for topic, _types in subs]

    cmd_vel_pubs: list[str] = []
    try:
        for info in node.get_publishers_info_by_topic(CMD_VEL_IN):
            cmd_vel_pubs.append(_bare(info.node_name))
    except Exception:
        pass

    return {
        "driver_count": len(drivers),
        "safety_count": len(safeties),
        "driver_topics": driver_topics,
        "motor_publishers": motor_publishers,
        "safety_subs": safety_subs,
        "cmd_vel_pubs": cmd_vel_pubs,
    }


def inspect_graph(node, timeout_s: float, settle_s: float) -> tuple[bool, list[str]]:
    """Wait until the driver is visible, then settle and evaluate."""
    import rclpy

    deadline = time.monotonic() + timeout_s
    snap = read_chain(node)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        snap = read_chain(node)
        if snap["driver_count"] == 1 and snap["driver_topics"]:
            break
    else:
        if snap["driver_count"] == 0:
            return False, [
                f"FAIL: /{DRIVER_NODE_NAME} was not in the graph within "
                f"{timeout_s:.0f}s. Launch the robot first "
                "(ros2 launch Ros_lidar_bot launch_robot.launch.py)."
            ]
        if snap["driver_count"] > 1:
            return False, [
                f"FAIL: {snap['driver_count']} nodes are named "
                f"{DRIVER_NODE_NAME}. The check does not know which one "
                "drives the motors."
            ]
        return False, [
            f"FAIL: /{DRIVER_NODE_NAME} is up but its Twist subscription "
            "was not visible. Check that cmd_vel_topic is set and the "
            "graph has finished discovery."
        ]

    if snap["driver_count"] > 1:
        return False, [
            f"FAIL: {snap['driver_count']} nodes are named {DRIVER_NODE_NAME}."
        ]

    publishers = set(snap["motor_publishers"])
    cmd_vel_pubs = set(snap["cmd_vel_pubs"])
    safety_subs = list(snap["safety_subs"])
    settle_end = time.monotonic() + settle_s
    while time.monotonic() < settle_end:
        rclpy.spin_once(node, timeout_sec=0.2)
        snap = read_chain(node)
        if snap["driver_count"] != 1 or len(snap["driver_topics"]) != 1:
            continue
        publishers.update(snap["motor_publishers"])
        cmd_vel_pubs.update(snap["cmd_vel_pubs"])
        if snap["safety_subs"]:
            safety_subs = list(snap["safety_subs"])

    # Re-read once more so the topic we judge is the driver's current one.
    snap = read_chain(node)
    topics = snap["driver_topics"] or []
    if len(topics) == 1:
        publishers.update(snap["motor_publishers"])
    ok, lines = evaluate_chain(
        topics,
        sorted(publishers),
        safety_subs if safety_subs else None,
    )
    if snap["safety_count"] == 0 and ok:
        ok = False
        lines.append(
            f"FAIL: /{SAFETY_NODE_NAME} is not in the graph, so the motor "
            "topic publisher cannot be the safety node."
        )
    elif snap["safety_count"] > 1:
        lines.append(
            f"info: {snap['safety_count']} nodes are named {SAFETY_NODE_NAME}."
        )
    if cmd_vel_pubs:
        lines.append(
            "info: "
            + CMD_VEL_IN
            + " publishers (filtered input, not the motor topic): "
            + ", ".join(f"/{name}" for name in sorted(cmd_vel_pubs))
        )
    return ok, lines


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="seconds to wait for driver_node to appear (default 10)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=2.0,
        help="seconds to watch the motor topic for late publishers (default 2)",
    )
    args = parser.parse_args(argv)

    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        print(
            "FAIL: rclpy is not importable. Source ROS 2 Jazzy first "
            "(source /opt/ros/jazzy/setup.bash).",
            file=sys.stderr,
        )
        return 2

    rclpy.init(args=None)
    node = Node("cmd_vel_chain_check")
    try:
        ok, lines = inspect_graph(node, args.timeout, args.settle)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("\n".join(lines))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
