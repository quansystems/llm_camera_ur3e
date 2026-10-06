#!/usr/bin/env python3
"""Open or close the simulated gripper with a constant drive, without feedback."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["open", "close"])
    parser.add_argument("--wait", type=float, default=1.0, help="Wait before returning, in simulation seconds")
    parser.add_argument("--force", type=float, default=5.0, help="Constant drive per finger in newtons (0 < force <= 5)")
    parser.add_argument("--timeout", type=float, default=30.0, help="Wall-clock limit for startup and waiting")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    if not math.isfinite(args.wait) or args.wait <= 0:
        parser.error("--wait must be positive and finite")
    if not math.isfinite(args.force) or not 0 < args.force <= 5.0:
        parser.error("--force must be between 0 and 5 N")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")

    rclpy.init(args=sys.argv)
    node = Node("gripper_control", parameter_overrides=[Parameter("use_sim_time", value=True)])
    publisher = node.create_publisher(Float64MultiArray, "/gripper_controller/commands", 1)
    drive = args.force if args.command == "close" else -args.force
    message = Float64MultiArray(data=[drive, drive])
    exit_code = 0
    try:
        deadline = time.monotonic() + args.timeout
        while publisher.get_subscription_count() == 0 or node.get_clock().now().nanoseconds == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("No gripper controller or simulation clock. Start the workcell and unpause Gazebo.")
            rclpy.spin_once(node, timeout_sec=0.05)
        start = node.get_clock().now().nanoseconds
        while (node.get_clock().now().nanoseconds - start) / 1e9 < args.wait:
            if time.monotonic() >= deadline:
                raise RuntimeError("Wait timed out. Check whether Gazebo is paused.")
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
        # The controller keeps the last command after this program exits.
        # Closing drive stays on during lifting; opening drive holds the open stop.
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f"{args.command}: sent {drive:+.2f} N per finger; waited {args.wait:.2f} simulation seconds")
    except RuntimeError as error:
        node.get_logger().error(str(error))
        exit_code = 1
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
