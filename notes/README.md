# dVRK PSM ROS2 Quickstart

## Overview
Control a PSM robot in IsaacSim using MTM cartesian commands. The kinematic node converts MTM cartesian commands to PSM joint commands and provides cartesian feedback.

## Prerequisites
Source ROS2 in all three terminals:
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

## Terminal 1: IsaacSim
Launch IsaacSim and load the USD scene with PSM robot and ActionGraph:
```bash
export isaac_sim_package_path=$HOME/isaacsim/_build/linux-x86_64/release/
$isaac_sim_package_path/isaac-sim.sh
```

**Topics**: Subscribes to `/PSM1/servo_jp` and `/PSM1/jaw/servo_jp`, publishes `/PSM1/measured_js`, `/PSM1/setpoint_js`, `/PSM1/jaw/measured_js`, and `/PSM1/jaw/setpoint_js`.

## Terminal 2: dVRK System
Run the dVRK system configuration with MTMR (PSMove controller):
```bash
cd src/cisst-saw/sawIntuitiveResearchKit/share/system/
ros2 run dvrk_robot dvrk_system -j system-MTMR-PSMove-PSM1_from_ROS.json
```

**Topics**: Publishes `{namespace}/state_command` and `{namespace}/servo_cp` (cartesian pose commands from MTM).

## Terminal 3: Kinematic Node
Launch the PSM kinematic node:
```bash
ros2 run dvrk_psm_ros2_nodes psm_kinematic_ros2node \
  --ros-args \
  --params-file $(ros2 pkg prefix dvrk_psm_ros2_nodes)/share/dvrk_psm_ros2_nodes/config/default.yaml \
  -p arm_namespace_prefix:=PSM1
```

### How It Works
- Subscribes to `servo_cp` from MTM (cartesian commands)
- Computes IK to get joint positions
- Publishes joint positions to `servo_jp` of PSM (for IsaacSim)
- Subscribes to `setpoint_js` from PSM (from IsaacSim)
- Computes FK to get cartesian position
- Publishes cartesian position to `setpoint_cp` and `measured_cp` of MTM (for feedback)