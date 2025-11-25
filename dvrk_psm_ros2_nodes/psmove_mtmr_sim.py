#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler


def clamp(val: float, low: float, high: float) -> float:
    return max(low, min(high, val))


class PSMoveMTMRSim(Node):
    """
    Publishes MTMR/PSMove-like servo commands to a PSM arm.
    - Cartesian poses on <arm>/servo_cp to exercise IK and joint outputs.
    - Jaw angle on <arm>/jaw/servo_jp.
    Keeps motions small and inside conservative limits.
    """

    def __init__(self):
        super().__init__('psmove_mtmr_sim')
        ns = f"/{self.declare_parameter('arm_namespace_prefix', 'PSM1').get_parameter_value().string_value}"
        rate_hz = float(self.declare_parameter('rate_hz', 50.0).get_parameter_value().double_value)
        self.alpha = float(self.declare_parameter('smoothing_alpha', 0.05).get_parameter_value().double_value)

        self.pub_pose = self.create_publisher(PoseStamped, f'{ns}/servo_cp', 10)
        self.pub_jaw = self.create_publisher(JointState, f'{ns}/jaw/servo_jp', 10)

        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.prev_pos = [0.0, -0.02, -0.12]
        self.prev_rpy = [0.0, 0.0, 0.0]
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f'Publishing simulated MTMR commands to {ns} at {rate_hz} Hz (alpha={self.alpha})')

    def tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9 - self.t0

        # Conservative workspace around a nominal pose.
        base = [0.0, -0.02, -0.12]
        amp = [0.015, 0.012, 0.015]  # smaller swings for smoother motion
        limits = [(-0.05, 0.05), (-0.06, 0.02), (-0.16, -0.08)]

        target_pos = [
            clamp(base[0] + amp[0] * math.sin(0.35 * t), *limits[0]),
            clamp(base[1] + amp[1] * math.sin(0.45 * t + 0.5), *limits[1]),
            clamp(base[2] + amp[2] * math.sin(0.55 * t + 1.0), *limits[2]),
        ]

        pos = [
            (1 - self.alpha) * p_prev + self.alpha * p_tgt
            for p_prev, p_tgt in zip(self.prev_pos, target_pos)
        ]
        self.prev_pos = pos

        # Mild orientation oscillation.
        target_rpy = [
            0.12 * math.sin(0.4 * t),
            0.12 * math.sin(0.5 * t + 0.4),
            0.12 * math.sin(0.6 * t + 0.8),
        ]

        rpy = [
            (1 - self.alpha) * r_prev + self.alpha * r_tgt
            for r_prev, r_tgt in zip(self.prev_rpy, target_rpy)
        ]
        self.prev_rpy = rpy
        quat = quaternion_from_euler(*rpy)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.pub_pose.publish(pose)

        jaw = JointState()
        jaw.header.stamp = now.to_msg()
        jaw.position = [clamp(0.25 * (1.0 + math.sin(0.8 * t)), 0.0, 0.6)]
        self.pub_jaw.publish(jaw)


def main():
    rclpy.init()
    rclpy.spin(PSMoveMTMRSim())
    rclpy.shutdown()
