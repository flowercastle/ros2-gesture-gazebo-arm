#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_joint",
    "left_finger_joint",
    "right_finger_joint",
]


class TestArmController(Node):
    def __init__(self):
        super().__init__("test_arm_controller")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10
        )

        self.poses = [
            [0.0, 0.0, 0.0, 0.0, 0.5, -0.5],
            [0.8, 0.4, -0.5, 0.2, 0.5, -0.5],
            [-0.8, -0.3, 0.6, -0.2, 0.1, -0.1],
            [0.0, 0.0, 0.0, 0.0, 0.5, -0.5],
        ]

    def publish_pose(self, positions, duration_sec=1):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)

        msg.points.append(point)
        self.publisher.publish(msg)

        self.get_logger().info(f"Publish pose: {positions}")


def main(args=None):
    rclpy.init(args=args)

    node = TestArmController()

    time.sleep(2.0)

    try:
        while rclpy.ok():
            for pose in node.poses:
                node.publish_pose(pose, duration_sec=1)
                rclpy.spin_once(node, timeout_sec=0.1)
                time.sleep(2.0)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()