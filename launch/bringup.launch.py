"""
Launch file for dVRK PSM ROS 2 C++ nodes

This launch file starts both the kinematic and joint-level nodes
with configurable namespace and parameter file.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for dVRK PSM nodes."""
    
    # Get package share directory
    package_share = get_package_share_directory('dvrk_psm_ros2_nodes')
    default_config = os.path.join(package_share, 'config', 'default.yaml')

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to parameter YAML file for PSM configuration'
    )
    
    arm_namespace_arg = DeclareLaunchArgument(
        'arm_namespace',
        default_value='PSM1',
        description='Namespace prefix for this arm (e.g. PSM1, PSM2)'
    )

    # PSM Kinematic Node
    psm_kinematic_node = Node(
        package='dvrk_psm_ros2_nodes',
        executable='psm_kinematic_ros2node',
        name='psm_kinematic',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'arm_namespace_prefix': LaunchConfiguration('arm_namespace')}
        ],
    )
    
    # PSM Joint Level Node
    psm_jointlevel_node = Node(
        package='dvrk_psm_ros2_nodes',
        executable='psm_jointlevel_ros2node',
        name='psm_jointlevel',
        output='screen',
        parameters=[
            {'arm_namespace_prefix': LaunchConfiguration('arm_namespace')}
        ],
    )

    return LaunchDescription([
        config_arg,
        arm_namespace_arg,
        psm_kinematic_node,
        psm_jointlevel_node,
    ])
