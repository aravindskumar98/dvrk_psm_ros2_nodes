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
USD_NAME = "PSM2ROS2.usd"
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

        # Jaw state (separate from arm joints, not part of dof_names)
        self.current_jaw_position = 0.0  # 0=closed

        # Setup separate messages for arm and jaw
        self.measured_js_msg = JointState()
        self.measured_js_msg.name = [self.psm.dof_names[i] for i in self.arm_joint_indices]

        self.setpoint_js_msg = JointState()
        self.setpoint_js_msg.name = [self.psm.dof_names[i] for i in self.arm_joint_indices]

        self.jaw_measured_js_msg = JointState()
        self.jaw_measured_js_msg.name = ['Jaw_Angle']  # Jaw joint name

        self.jaw_setpoint_js_msg = JointState()
        self.jaw_setpoint_js_msg.name = ['Jaw_Angle']

        self._print_controller_info()

    def _print_controller_info(self):
        """Print controller initialization information including DOFs, topics, and example commands."""
        print(f"\n{'='*60}")
        print(f"{self.name} controller initialized with {len(self.psm.dof_names)} DOFs")
        print(f"{'='*60}")
        print(f"\nArm Joint Names (6 DOF):")
        for i, joint_name in enumerate(self.psm.dof_names):
            print(f"  [{i}] {joint_name}")

        print(f"\nJaw: Separate from arm joints (1 DOF gripper, 0=closed, 0.6=open)")

        print(f"\nROS2 Topics Architecture:")
        print(f"  Subscribers (inputs):")
        print(f"    - {self.name}/servo_jp (arm joint commands)")
        print(f"    - {self.name}/jaw/servo_jp (jaw commands, 0=closed, 0.6=open)")
        print(f"  Publishers (outputs at {PUBLISH_RATE_HZ}Hz):")
        print(f"    - {self.name}/measured_js (current arm joint positions)")
        print(f"    - {self.name}/setpoint_js (commanded arm joint positions)")
        print(f"    - {self.name}/jaw/measured_js (current jaw position)")
        print(f"    - {self.name}/jaw/setpoint_js (commanded jaw position)")

        print(f"\nExample commands:")
        print(f"  # Move arm joints:")
        print(f"  ros2 topic pub --once /{self.name}/servo_jp sensor_msgs/msg/JointState \"{{")
        print(f"    name: {list(self.psm.dof_names)},")
        print(f"    position: {[0.0] * len(self.psm.dof_names)}")
        print(f"  }}\"")
        print(f"\n  # Control jaw (0=closed, 0.6=open):")
        print(f"  ros2 topic pub --once /{self.name}/jaw/servo_jp sensor_msgs/msg/JointState \"{{")
        print(f"    position: [0.3]")
        print(f"  }}\"")

        print(f"\nControl Mode: {'ROS2 Control' if self.use_ros2_control else 'Sine Wave Motion'}")
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
        """Helper to apply joint command to position array."""
        for i, name in enumerate(msg.name):
            if name in self.dof_map:
                joint_positions[self.dof_map[name]] = msg.position[i]
    
    def _publish_joint_states(self):
        """Publish measured_js and setpoint_js at specified rate."""
        current_time = self.world.current_time
        if current_time - self.last_publish_time >= PUBLISH_PERIOD_S:
            now = self.node.get_clock().now().to_msg()
            arm_positions = self.psm.get_joint_positions().tolist()

            # Publish arm measured_js (current actual positions)
            self.measured_js_msg.header.stamp = now
            self.measured_js_msg.position = arm_positions
            self.measured_js_pub.publish(self.measured_js_msg)

            # Publish arm setpoint_js (echo of the commanded positions)
            # TODO: setpoint should echo the commanded positions, not the current actual positions. 
            # Currently both measured_js and setpoint_js publish the same thing (actual positions)
            self.setpoint_js_msg.header.stamp = now
            self.setpoint_js_msg.position = arm_positions
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