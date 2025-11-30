
#!/usr/bin/env python3

import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory

from crtk_msgs.msg import OperatingState, StringStamped
from tf_transformations import quaternion_matrix, quaternion_from_matrix
from cisstRobotPython import robManipulator

from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy

def latched_qos(depth: int = 1) -> QoSProfile:
    qos = QoSProfile(depth=depth)
    qos.history = HistoryPolicy.KEEP_LAST       # keep the last ‘depth’ msgs
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # store them in the RMW
    return qos

# how it works:
# subscribes to servo_cp from MTM
# computes ik to get joint positions
# publishes joint positions to servo_jp of PSM 
# subscribes to setpoint_js from PSM
# computes fk to get cartesian position
# publishes cartesian position to setpoint_cp of MTM

class PSMKinematicNode(Node):
    def __init__(self):
        super().__init__('psm_kinematic_ros2node')

        # Load robot
        arm_pkg  = self.declare_parameter('arm_pkg',  'dvrk_config').get_parameter_value().string_value
        tool_pkg = self.declare_parameter('tool_pkg', 'dvrk_config').get_parameter_value().string_value
        arm_rel  = self.declare_parameter('arm_relpath',  'kinematic/PSM.json').get_parameter_value().string_value
        tool_rel = self.declare_parameter('tool_relpath', 'tool/LARGE_NEEDLE_DRIVER_400006.json').get_parameter_value().string_value ## FIXME: Tool name can be moved to a config file or added to launch.py
        namespace_prefix = f'/{self.declare_parameter('arm_namespace_prefix', 'PSM1').get_parameter_value().string_value}'

        arm_path  = os.path.join(get_package_share_directory(arm_pkg),  arm_rel)
        tool_path = os.path.join(get_package_share_directory(tool_pkg), tool_rel)

        self.get_logger().info(f'[ROS2 Kinematic Node] Using namespace: {namespace_prefix}')
        self.get_logger().info(f'[ROS2 Kinematic Node] Arm model:  {arm_path}')
        self.get_logger().info(f'[ROS2 Kinematic Node] Tool model: {tool_path}')

        self.robot = robManipulator()
        self.robot.LoadRobot(arm_path)
        self.robot.LoadRobot(tool_path)  # this modifies the existing robot chain

        self.num_joints = len(self.robot.links)
        self.last_joint_position = np.zeros(self.num_joints)

        self.state_sub = self.create_subscription(StringStamped, f'{namespace_prefix}/state_command', self.state_cb, 10) # subscriber can stay VOLATILE
        self.state_pub = self.create_publisher(OperatingState, f'{namespace_prefix}/operating_state', latched_qos(1000)) # latched publisher

        self.servo_cp_sub = self.create_subscription(PoseStamped, f'{namespace_prefix}/servo_cp', self.ik_cb, 10)
        self.servo_jp_pub = self.create_publisher(JointState, f'{namespace_prefix}/servo_jp', 10)

        self.setpoint_js_sub = self.create_subscription(JointState, f'{namespace_prefix}/setpoint_js', self.fk_cb, 10)
        self.setpoint_cp_pub = self.create_publisher(PoseStamped, f'{namespace_prefix}/setpoint_cp', 10)
        self.measured_cp_pub = self.create_publisher(PoseStamped, f'{namespace_prefix}/measured_cp', 10)

        self.publish_operating_state()  # Initial state

    def state_cb(self, msg):
        self.publish_operating_state()

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
        js.header.frame_id = 'map'
        js.name = ['Base_Yaw', 'Yaw_Pitch_End', 'Pitch_End_Main_Insert', 'Main_Insert_Tool_Roll', 'Tool_Roll_Tool_Pitch', 'Tool_Yaw_Tool_Pitch' ] # [f'joint_{i+1}' for i in range(self.num_joints)]
        js.position = list(jp)
        self.servo_jp_pub.publish(js)

    def fk_cb(self, msg: JointState):
        q = np.array(msg.position)
        cp = self.robot.ForwardKinematics(q)
        pos = cp[:3, 3]
        quat = quaternion_from_matrix(cp)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
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



