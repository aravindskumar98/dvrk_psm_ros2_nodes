#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy


class PSMJointLevelNode(Node):
    def __init__(self):
        super().__init__('psm_jointlevel_ros2node')

        # Set initial joint position (e.g., 8 cm insertion)
        jp = numpy.zeros(6)
        jp[2] = 0.08  # joint 3 = insertion

        # Full arm
        self.latest_jp = JointState()
        self.latest_jp.position = jp.tolist()
        self.latest_jp.velocity = []
        self.latest_jp.effort = []

        # Jaw 
        self.latest_jaw_jp = JointState()
        self.latest_jaw_jp.position = [0.0]  # single joint for jaw
        self.latest_jaw_jp.velocity = []
        self.latest_jaw_jp.effort = []

        # ROS pub/sub for full arm
        self.pub_setpoint_js = self.create_publisher(JointState, '/PSM1/setpoint_js', 10)
        self.create_subscription(JointState, '/PSM1/servo_jp', self.cb_servo_jp, 10)

        # ROS pub/sub for jaw
        self.create_subscription(JointState, '/PSM1/jaw/servo_jp', self.cb_servo_jaw_jp, 10)
        self.pub_jaw_setpoint = self.create_publisher(JointState, '/PSM1/jaw/setpoint_js', 10)
        self.pub_jaw_measured = self.create_publisher(JointState, '/PSM1/jaw/measured_js', 10)

        # Timer to stream at 500 Hz
        self.timer = self.create_timer(0.002, self.publish_setpoint)

        self.get_logger().info('PSMJointLevelNode initialized and streaming')

    def cb_servo_jp(self, msg):
        self.latest_jp = msg

    def cb_servo_jaw_jp(self, msg):
        if len(msg.position) > 0:
            self.latest_jaw_jp.position = [msg.position[0]]

    def publish_setpoint(self):
        now = self.get_clock().now().to_msg()

        # Stream arm joint setpoint
        self.latest_jp.header.stamp = now
        self.pub_setpoint_js.publish(self.latest_jp)

        # Stream jaw setpoint and measured_js as same (no sensing yet)
        self.latest_jaw_jp.header.stamp = now
        self.pub_jaw_setpoint.publish(self.latest_jaw_jp)
        self.pub_jaw_measured.publish(self.latest_jaw_jp)

def main():
    rclpy.init()
    rclpy.spin(PSMJointLevelNode())
    rclpy.shutdown()
