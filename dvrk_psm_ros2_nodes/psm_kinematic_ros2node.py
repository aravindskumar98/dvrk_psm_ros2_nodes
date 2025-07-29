
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from crtk_msgs.msg import OperatingState, StringStamped

from std_msgs.msg import Header
from cisstRobotPython import robManipulator
from tf_transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np

class PSMKinematicNode(Node):
    def __init__(self):
        super().__init__('psm_kinematic_ros2node')

        # Load robot
        arm_path = '/home/arav/ros2_ws/src/cisst-saw/sawIntuitiveResearchKit/share/kinematic/PSM.json'
        tool_path = '/home/arav/ros2_ws/src/cisst-saw/sawIntuitiveResearchKit/share/tool/LARGE_NEEDLE_DRIVER_400006.json'

        self.robot = robManipulator()
        self.robot.LoadRobot(arm_path)
        self.robot.LoadRobot(tool_path)  # this modifies the existing robot chain

        self.num_joints = len(self.robot.links)
        self.last_joint_position = np.zeros(self.num_joints)

        self.state_sub = self.create_subscription(StringStamped, '/PSM1/state_command', self.state_cb, 10)
        self.state_pub = self.create_publisher(OperatingState, '/PSM1/operating_state', 10)
        self.create_timer(0.05, self.publish_operating_state)

        self.servo_cp_sub = self.create_subscription(PoseStamped, '/PSM1/servo_cp', self.ik_cb, 10)
        self.servo_jp_pub = self.create_publisher(JointState, '/PSM1/servo_jp', 10)

        self.setpoint_js_sub = self.create_subscription(JointState, '/PSM1/setpoint_js', self.fk_cb, 10)
        self.setpoint_cp_pub = self.create_publisher(PoseStamped, '/PSM1/setpoint_cp', 10)
        self.measured_cp_pub = self.create_publisher(PoseStamped, '/PSM1/measured_cp', 10)

    def state_cb(self, msg):
        pass  # ignore

    def publish_operating_state(self): ## Can be made a call back
        msg = OperatingState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = 'ENABLED'
        msg.is_homed = True
        msg.is_busy = False
        self.state_pub.publish(msg)

    def ik_cb(self, msg: PoseStamped):
        cp = np.identity(4)
        cp[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        cp[:3, :3] = quaternion_matrix(q)[:3, :3]

        jp = np.zeros(self.num_joints)
        self.robot.InverseKinematics(jp, cp)
        self.last_joint_position = jp

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f'joint_{i+1}' for i in range(self.num_joints)]
        js.position = list(jp)
        self.servo_jp_pub.publish(js)

    def fk_cb(self, msg: JointState):
        q = np.array(msg.position)
        cp = self.robot.ForwardKinematics(q)
        pos = cp[:3, 3]
        quat = quaternion_from_matrix(cp)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.setpoint_cp_pub.publish(pose)
        # Publish measured Cartesian position
        self.measured_cp_pub.publish(pose)

def main():
    rclpy.init()
    rclpy.spin(PSMKinematicNode())
    rclpy.shutdown()



