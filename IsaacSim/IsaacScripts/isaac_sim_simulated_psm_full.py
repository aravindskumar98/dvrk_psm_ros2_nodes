import time
import os
from isaacsim import SimulationApp

# Initialize the Isaac Sim app
CONFIG = {"headless": False, "startup_timeout": 120}
simulation_app = SimulationApp(CONFIG)

from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction

# Enable ROS 2 Bridge
enable_extension("isaacsim.ros2.bridge")

print("ROS 2 Bridge enabled. Press Enter to continue...")
input()

simulation_app.update()

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

PUBLISH_RATE_HZ = 30.0
PUBLISH_PERIOD_S = 1.0 / PUBLISH_RATE_HZ
ASSET_PATH = "/home/arav/workspace/isaac-sim-surgical-robotics-challenge/Assets"
USD_NAME = "psm_flat_tuned.usd"

# Main controllable joints (6 DOF) - user only needs to specify these
MAIN_CONTROLLABLE_JOINTS = ['yaw', 'pitch', 'insertion', 'roll', 'wrist_pitch', 'wrist_yaw']

# Mimic joint mapping: mimic_joint -> (main_joint, scale_factor)
# These joints automatically follow their main joint counterparts
MIMIC_JOINT_MAP = {
    'pitch_1': ('pitch', 1.0),
    'pitch_2': ('pitch', 1.0),
    'pitch_3': ('pitch', 1.0),
    'pitch_4': ('pitch', 1.0),
    'pitch_5': ('pitch', 1.0),
    'jaw_mimic_1': ('jaw', 1.0),
    'jaw_mimic_2': ('jaw', 1.0),
}

PSM_PRIM_PATH = "/World/PSM2"

# Control mode: Set to True to use ROS2 subscriber commands, False to use sine wave motion
USE_ROS2_CONTROL = True


class Scene:
    """Manages the simulation world and scene elements."""
    
    def __init__(self):
        self.world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
        self.world.scene.add_default_ground_plane()
    
    def add_robot(self, usd_path, prim_path, name):
        """Load a robot USD and add it to the scene."""
        if not os.path.exists(usd_path):
            raise FileNotFoundError(f"USD file not found: {usd_path}")
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        return self.world.scene.add(Robot(prim_path=prim_path, name=name))
    
    def reset(self):
        """Reset the world (must be called after adding all robots)."""
        self.world.reset()
        print("Scene initialized and physics started.")


class PSMController:
    """Controller for PSM robot with ROS 2 integration.

    Subscribes to servo_jp topics and publishes both measured_js and setpoint_js.
    Architecture: servo_jp (input) -> robot control -> measured_js + setpoint_js (output)
    """
    
    def __init__(self, world, psm_robot, node, name="PSM1", use_ros2_control=True):
        self.world = world
        self.psm = psm_robot
        self.node = node
        self.name = name
        self.use_ros2_control = use_ros2_control
        self.latest_servo_jp = None
        self.latest_jaw_servo_jp = None
        self.start_time = time.time()
        self.last_publish_time = 0.0
        
        # ROS 2 publishers - publish measured_js and setpoint_js
        self.measured_js_pub = node.create_publisher(JointState, f"{name}/measured_js", 10)
        self.setpoint_js_pub = node.create_publisher(JointState, f"{name}/setpoint_js", 10)
        self.jaw_measured_js_pub = node.create_publisher(JointState, f"{name}/jaw/measured_js", 10)
        self.jaw_setpoint_js_pub = node.create_publisher(JointState, f"{name}/jaw/setpoint_js", 10)

        # ROS 2 subscribers - subscribe to servo_jp only
        self.sub_servo_jp = node.create_subscription(
            JointState, f"{name}/servo_jp", self._servo_jp_callback, 10
        )
        self.sub_jaw_servo_jp = node.create_subscription(
            JointState, f"{name}/jaw/servo_jp", self._jaw_servo_jp_callback, 10
        )
        
        # Joint control setup (after world.reset())
        self.controller = self.psm.get_articulation_controller()
        self.dof_map = {name: i for i, name in enumerate(self.psm.dof_names)}

        self.arm_joint_indices = list(range(len(self.psm.dof_names)))

        # Identify main controllable joints and mimic joints
        self.main_joint_indices = [self.dof_map[j] for j in MAIN_CONTROLLABLE_JOINTS if j in self.dof_map]
        self.main_joint_names = [j for j in MAIN_CONTROLLABLE_JOINTS if j in self.dof_map]

        # Detect which mimic joints actually exist in this robot
        self.active_mimic_joints = {
            mimic: main_info for mimic, main_info in MIMIC_JOINT_MAP.items()
            if mimic in self.dof_map
        }

        # If no mimic joints exist, we're working with a simplified robot model
        self.has_mimic_joints = len(self.active_mimic_joints) > 0

        # Jaw state (separate from arm joints, not part of dof_names)
        self.current_jaw_position = 0.0  # 0=closed

        # Setup separate messages for arm and jaw - only expose main controllable joints
        self.measured_js_msg = JointState()
        self.measured_js_msg.name = self.main_joint_names

        self.setpoint_js_msg = JointState()
        self.setpoint_js_msg.name = self.main_joint_names

        self.jaw_measured_js_msg = JointState()
        self.jaw_measured_js_msg.name = ['Jaw_Angle']  # Jaw joint name

        self.jaw_setpoint_js_msg = JointState()
        self.jaw_setpoint_js_msg.name = ['Jaw_Angle']

        self._print_controller_info()

    def _print_controller_info(self):
        """Print controller initialization information including DOFs, topics, and example commands."""
        print(f"\n{'='*60}")
        print(f"{self.name} controller initialized")
        print(f"{'='*60}")
        print(f"\nMain Controllable Joints ({len(self.main_joint_names)} DOF):")
        for i, joint_name in enumerate(self.main_joint_names):
            print(f"  [{i}] {joint_name}")

        if self.has_mimic_joints:
            print(f"\nAll Isaac Sim DOFs ({len(self.psm.dof_names)} total - includes mimic joints):")
            for i, joint_name in enumerate(self.psm.dof_names):
                is_mimic = joint_name in self.active_mimic_joints
                marker = " (mimic)" if is_mimic else ""
                print(f"  [{i}] {joint_name}{marker}")
        else:
            print(f"\nRobot Model: Simplified (no mimic joints detected)")

        print(f"\nJaw: Separate from arm joints (1 DOF gripper, 0=closed, 0.6=open)")

        print(f"\nROS2 Topics Architecture:")
        print(f"  Subscribers (inputs):")
        print(f"    - {self.name}/servo_jp (arm joint commands - only main joints required)")
        print(f"    - {self.name}/jaw/servo_jp (jaw commands, 0=closed, 0.6=open)")
        print(f"  Publishers (outputs at {PUBLISH_RATE_HZ}Hz):")
        print(f"    - {self.name}/measured_js (current arm joint positions - only main joints)")
        print(f"    - {self.name}/setpoint_js (commanded arm joint positions - only main joints)")
        print(f"    - {self.name}/jaw/measured_js (current jaw position)")
        print(f"    - {self.name}/jaw/setpoint_js (commanded jaw position)")

        print(f"\nExample commands:")
        print(f"  # Move arm joints (only specify the {len(self.main_joint_names)} main joints):")
        print(f"  ros2 topic pub --once /{self.name}/servo_jp sensor_msgs/msg/JointState \"{{")
        print(f"    name: {self.main_joint_names},")
        print(f"    position: {[0.0] * len(self.main_joint_names)}")
        print(f"  }}\"")
        print(f"\n  # Control jaw (0=closed, 0.6=open):")
        print(f"  ros2 topic pub --once /{self.name}/jaw/servo_jp sensor_msgs/msg/JointState \"{{")
        print(f"    position: [0.3]")
        print(f"  }}\"")

        print(f"\nControl Mode: {'ROS2 Control' if self.use_ros2_control else 'Sine Wave Motion'}")
        if self.has_mimic_joints:
            print(f"\nNote: Mimic joints are automatically controlled based on their main joint counterparts.")
        print(f"{'='*60}\n")

    def _servo_jp_callback(self, msg):
        """Callback for servo joint position commands - stores command to apply."""
        self.latest_servo_jp = msg

    def _jaw_servo_jp_callback(self, msg):
        """Callback for jaw servo joint position commands - stores command to apply."""
        if len(msg.position) > 0:
            self.latest_jaw_servo_jp = msg
    
    def update(self):
        """Update control and publish state."""
        self._apply_control()
        self._publish_joint_states()
    
    def _apply_control(self):
        """Apply joint control commands from servo_jp topics."""
        joint_positions = [None] * len(self.psm.dof_names)
        has_arm_command = False

        if self.use_ros2_control:
            # Apply arm servo commands
            if self.latest_servo_jp is not None:
                self._apply_joint_command(self.latest_servo_jp, joint_positions)
                self.latest_servo_jp = None
                has_arm_command = True

            # Apply jaw servo commands
            if self.latest_jaw_servo_jp is not None:
                if len(self.latest_jaw_servo_jp.position) > 0:
                    self.current_jaw_position = self.latest_jaw_servo_jp.position[0]
                self.latest_jaw_servo_jp = None

            # Apply arm joint positions to the robot
            if has_arm_command:
                self.controller.apply_action(ArticulationAction(joint_positions=joint_positions))

            # TODO: Apply jaw position using gripper controller


        else:
            # Sine wave motion mode (ignores ROS2 commands)
            elapsed = time.time() - self.start_time
            joint_positions = [
                1.5 * np.sin(elapsed * (0.8 + i * 0.2) + i * np.pi / 4)
                for i in range(len(self.psm.dof_names))
            ]
            self.controller.apply_action(ArticulationAction(joint_positions=joint_positions))
            # Also move jaw in sine wave
            self.current_jaw_position = 0.3 * (1.0 + np.sin(elapsed * 0.5))
    
    def _apply_joint_command(self, msg, joint_positions):
        """Helper to apply joint command to position array and handle mimic joints."""
        # Dictionary to store commanded values for main joints
        commanded_values = {}

        # Apply commanded joint positions
        for i, name in enumerate(msg.name):
            if name in self.dof_map:
                joint_positions[self.dof_map[name]] = msg.position[i]
                commanded_values[name] = msg.position[i]

        # Automatically compute and apply mimic joint positions (if any exist)
        if self.has_mimic_joints:
            for mimic_joint, (main_joint, scale) in self.active_mimic_joints.items():
                if main_joint in commanded_values:
                    joint_positions[self.dof_map[mimic_joint]] = commanded_values[main_joint] * scale
    
    def _publish_joint_states(self):
        """Publish measured_js and setpoint_js at specified rate."""
        current_time = self.world.current_time
        if current_time - self.last_publish_time >= PUBLISH_PERIOD_S:
            now = self.node.get_clock().now().to_msg()
            all_arm_positions = self.psm.get_joint_positions().tolist()

            # Extract only the main controllable joint positions
            main_joint_positions = [all_arm_positions[i] for i in self.main_joint_indices]

            # Publish arm measured_js (current actual positions - only main joints)
            self.measured_js_msg.header.stamp = now
            self.measured_js_msg.position = main_joint_positions
            self.measured_js_pub.publish(self.measured_js_msg)

            # Publish arm setpoint_js (echo of the commanded positions - only main joints)
            # TODO: setpoint should echo the commanded positions, not the current actual positions.
            # Currently both measured_js and setpoint_js publish the same thing (actual positions)
            self.setpoint_js_msg.header.stamp = now
            self.setpoint_js_msg.position = main_joint_positions
            self.setpoint_js_pub.publish(self.setpoint_js_msg)

            # Publish jaw measured_js (current jaw position)
            self.jaw_measured_js_msg.header.stamp = now
            self.jaw_measured_js_msg.position = [self.current_jaw_position]
            self.jaw_measured_js_pub.publish(self.jaw_measured_js_msg)

            # Publish jaw setpoint_js (echo of commanded jaw position)
            self.jaw_setpoint_js_msg.header.stamp = now
            self.jaw_setpoint_js_msg.position = [self.current_jaw_position]
            self.jaw_setpoint_js_pub.publish(self.jaw_setpoint_js_msg)

            self.last_publish_time = current_time


# Main execution
rclpy.init()
node = rclpy.create_node("psm_controller_node")

# Setup scene and robots
scene = Scene()
psm1 = scene.add_robot(
    usd_path=os.path.join(ASSET_PATH, USD_NAME),
    prim_path=PSM_PRIM_PATH,
    name="psm1"
)
scene.reset()

# Create controllers
psm1_controller = PSMController(scene.world, psm1, node, name="PSM1", use_ros2_control=USE_ROS2_CONTROL)

# Main loop
try:
    while simulation_app.is_running():
        scene.world.step(render=True)
        
        if scene.world.is_playing():
            psm1_controller.update()
        
        rclpy.spin_once(node, timeout_sec=0.0)
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    rclpy.shutdown()
    simulation_app.close()