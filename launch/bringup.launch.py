from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dvrk_psm_ros2_nodes',
            executable='psm_kinematic_ros2node',
            name='psm_kinematic',
            output='screen'
        ),
        Node(
            package='dvrk_psm_ros2_nodes',
            executable='psm_jointlevel_ros2node',
            name='psm_jointlevel',
            output='screen'
        ),
    ])
