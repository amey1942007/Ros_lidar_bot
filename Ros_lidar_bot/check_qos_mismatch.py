#!/usr/bin/env python3
"""\ncheck_qos_mismatch.py — Diagnostic utility to scan ROS 2 network graph QoS compatibility.

================================================================================
UNDERLYING SYSTEM & DATA FLOW
================================================================================
- Scans all active topics on the DDS global network graph.
- Inspects publisher and subscriber QoS configurations.
- Focuses on identifying two critical mismatch zones that lead to silently dropped packets:
  1. Reliability: Sub expects RELIABLE, Pub is BEST_EFFORT.
  2. Durability: Sub expects TRANSIENT_LOCAL, Pub is VOLATILE.
- Ignores parameter events and system level rosout logging topics.
- Shuts down with exit code 1 if a mismatch is found (useful for integration testing).\n"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy

# Standard ANSI formatting codes for clean visual outputs
COLOR_RESET = "\033[0m"
COLOR_RED   = "\033[31m"
COLOR_GREEN = "\033[32m"
COLOR_YELLOW = "\033[33m"
COLOR_BOLD   = "\033[1m"

def get_rel_str(policy):
    if policy == ReliabilityPolicy.RELIABLE:
        return "RELIABLE"
    elif policy == ReliabilityPolicy.BEST_EFFORT:
        return "BEST_EFFORT"
    return "UNKNOWN"

def get_dur_str(policy):
    if policy == DurabilityPolicy.TRANSIENT_LOCAL:
        return "TRANSIENT_LOCAL"
    elif policy == DurabilityPolicy.VOLATILE:
        return "VOLATILE"
    return "UNKNOWN"

class QoSGraphChecker(Node):
    def __init__(self):
        super().__init__('qos_graph_checker')
        self.get_logger().info("Discovery starting... Waiting 2 seconds for active nodes to register.")

    def run_check(self):
        # Retrieve all active topics
        topic_list = self.get_topic_names_and_types()
        if not topic_list:
            print(f"{COLOR_YELLOW}No active topics found on the ROS 2 network graph.{COLOR_RESET}")
            print("Make sure you have launched your nodes / simulation before running this script.")
            return

        print(f"\n{COLOR_BOLD}Scanning active topics for QoS compatibility...{COLOR_RESET}\n")
        print(f"{'Topic Name':<40} | {'Publisher Node':<25} | {'Subscriber Node':<25} | {'Status/Issues'}")
        print("-" * 115)

        mismatches_found = []
        healthy_links = 0

        for topic_name, topic_types in topic_list:
            # Skip internal framework parameters and logs to ignore background noise
            if any(x in topic_name for x in ['/parameter_events', '/rosout']):
                continue

            try:
                publishers = self.get_publishers_info_by_topic(topic_name)
                subscribers = self.get_subscriptions_info_by_topic(topic_name)
            except Exception as e:
                # Handle race conditions if topics go offline mid-iteration
                continue

            # Check matching if there is at least one active publisher and subscriber on this topic
            if not publishers or not subscribers:
                continue

            for pub in publishers:
                pub_node = f"{pub.node_namespace}/{pub.node_name}".replace("//", "/")
                pub_rel = pub.qos_profile.reliability
                pub_dur = pub.qos_profile.durability

                for sub in subscribers:
                    sub_node = f"{sub.node_namespace}/{sub.node_name}".replace("//", "/")
                    sub_rel = sub.qos_profile.reliability
                    sub_dur = sub.qos_profile.durability

                    # 1. Reliability Mismatch: Subscriber requires RELIABLE, Publisher only provides BEST_EFFORT
                    rel_mismatch = (sub_rel == ReliabilityPolicy.RELIABLE and 
                                    pub_rel == ReliabilityPolicy.BEST_EFFORT)

                    # 2. Durability Mismatch: Subscriber requires TRANSIENT_LOCAL, Publisher only provides VOLATILE
                    dur_mismatch = (sub_dur == DurabilityPolicy.TRANSIENT_LOCAL and 
                                    pub_dur == DurabilityPolicy.VOLATILE)

                    if rel_mismatch or dur_mismatch:
                        issues = []
                        if rel_mismatch:
                            issues.append(f"Reliability (Sub wants RELIABLE, Pub is BEST_EFFORT)")
                        if dur_mismatch:
                            issues.append(f"Durability (Sub wants TRANSIENT_LOCAL, Pub is VOLATILE)")
                        
                        mismatches_found.append({
                            'topic': topic_name,
                            'pub': pub_node,
                            'sub': sub_node,
                            'detail': " & ".join(issues)
                        })
                        print(f"{COLOR_RED}{topic_name:<40} | {pub_node:<25} | {sub_node:<25} | MISMATCH: {' & '.join(issues)}{COLOR_RESET}")
                    else:
                        healthy_links += 1

        print("-" * 115)
        if not mismatches_found:
            print(f"\n{COLOR_GREEN}{COLOR_BOLD}SUCCESS: All {healthy_links} active pub/sub links are fully compatible. No QoS mismatches discovered!{COLOR_RESET}\n")
            sys.exit(0)
        else:
            print(f"\n{COLOR_RED}{COLOR_BOLD}WARNING: Detected {len(mismatches_found)} QoS mismatches on the graph!{COLOR_RESET}")
            print("ROS 2 will drop messages silently between mismatched subscribers and publishers.")
            print("Please synchronize their QoS policies (Reliability & Durability) in code or YAML parameter configurations.\n")
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    checker = QoSGraphChecker()
    # Sleep to allow global discovery info to be received from DDS
    time.sleep(2.0)
    try:
        checker.run_check()
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        # Do not shutdown rclpy here if it was already shutdown in sys.exit()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
