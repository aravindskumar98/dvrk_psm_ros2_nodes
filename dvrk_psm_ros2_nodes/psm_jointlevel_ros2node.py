#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from cisstRobotPython import robManipulator
import numpy

class PSMJointLevelNode(Node):
    def __init__(self):
        super().__init__('psm_jointlevel_ros2node')

        # Load PSM + tool model
        arm_path = '/home/arav/ros2_ws/src/cisst-saw/sawIntuitiveResearchKit/share/kinematic/PSM.json'
        tool_path = '/home/arav/ros2_ws/src/cisst-saw/sawIntuitiveResearchKit/share/tool/LARGE_NEEDLE_DRIVER_400006.json'

        self.r = robManipulator()
        self.r.LoadRobot(arm_path)
        self.r.LoadRobot(tool_path)  # overrides the arm model with the tool

        # Set initial joint position (e.g., 8 cm insertion)
        jp = numpy.zeros(6)
        jp[2] = 0.08  # joint 3 = insertion

        self.latest_jp = JointState()
        # self.latest_jp.name = self.r.GetJointNames()
        self.latest_jp.position = jp.tolist()
        self.latest_jp.velocity = []
        self.latest_jp.effort = []

        # ROS pub/sub
        self.pub = self.create_publisher(JointState, '/PSM1/setpoint_js', 10)  ## This needs to publish measured_js too
        self.create_subscription(JointState, '/PSM1/servo_jp', self.cb_servo_jp, 10)

        # Timer to stream at 500 Hz
        self.timer = self.create_timer(0.002, self.publish_setpoint)

        self.get_logger().info('PSMJointLevelNode initialized and streaming')

    def cb_servo_jp(self, msg):
        # Replace the latest commanded joint state
        self.latest_jp = msg

    def publish_setpoint(self):
        # Always stream the most recent joint setpoint
        self.latest_jp.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.latest_jp)

def main():
    rclpy.init()
    rclpy.spin(PSMJointLevelNode())
    rclpy.shutdown()

# Add two topics
## / PSM1/jaw/servo_jp - subcriber  --- teleop will directly publish this in joint space
## / PSM1/jaw/setpoint_js - publisher
## / PSM1/jaw/measured_js - publisher

