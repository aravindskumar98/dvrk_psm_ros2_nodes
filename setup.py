from setuptools import setup

package_name = 'dvrk_psm_ros2_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aravind',
    maintainer_email='aravindskumar1998@gmail.com',
    description='ROS 2 nodes for dVRK PSM kinematics and joint-level control using CRTK topics.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'psm_kinematic_ros2node = dvrk_psm_ros2_nodes.psm_kinematic_ros2node:main',
            'psm_jointlevel_ros2node = dvrk_psm_ros2_nodes.psm_jointlevel_ros2node:main',
        ],
    },
)

