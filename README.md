
# dvrk_psm_ros2_nodes

C++ ROS 2 nodes for dVRK PSM kinematics and joint-level control using CRTK topics.

## What this package does
- `psm_kinematic_ros2node`: loads the PSM + tool kinematic chains from JSON (from `dvrk_config` by default). It converts Cartesian servo commands (`<arm>/servo_cp`) into joint targets (`<arm>/servo_jp`) and publishes FK for whatever joint target it last saw (`<arm>/setpoint_cp`, `<arm>/measured_cp`). It also latches an ENABLED operating state on `<arm>/operating_state`.
- `psm_jointlevel_ros2node`: simple joint-level streamer. It seeds the arm with a small insertion, listens for joint servo commands, and republishes them as setpoints at 500 Hz on `<arm>/setpoint_js`. It mirrors jaw setpoints to `jaw/measured_js` (no sensing yet).
- The launch file starts both nodes and wires the namespace via `arm_namespace` (default `PSM1`), so topics look like `/PSM1/servo_cp`, `/PSM1/servo_jp`, `/PSM1/jaw/servo_jp`, etc.

## Dependencies

### ROS 2 Dependencies
- `rclcpp`
- `ament_index_cpp`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `crtk_msgs` (CRTK message definitions)
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`

### cisst Dependencies
The package requires the **cisst** library suite and Qt5:
- `cisstCommon`, `cisstVector`, `cisstNumerical`, `cisstRobot` (provides `robManipulator`)
- Qt5 (Core, Widgets, OpenGL, Xml, XmlPatterns)

### Installing cisst
See the [cisst wiki](https://github.com/jhu-cisst/cisst/wiki) for detailed build instructions. For ROS 2:
```bash
cd ~/ros2_ws/src
git clone https://github.com/jhu-cisst/cisst.git
git clone https://github.com/jhu-cisst/cisst-ros.git
cd ~/ros2_ws
colcon build --packages-select cisst cisst_ros_bridge
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select dvrk_psm_ros2_nodes
source install/setup.bash
```

## Related Packages
- `dvrk` and `cisst-saw`: provide the dVRK ROS tooling and system JSONs (`sawIntuitiveResearchKit` under `cisst-saw`)
- `crtk_msgs`: common message definitions (e.g., `OperatingState`)
- `sawPSMove`: PS Move controller input that feeds the MTM/PSM control stack

## Launch

**Default (PSM1):**
```bash
ros2 launch dvrk_psm_ros2_nodes bringup.launch.py
```

**Switch to PSM2 with different tool/arm:**

```bash
ros2 launch dvrk_psm_ros2_nodes bringup.launch.py \
    arm_namespace:=PSM2 \
    config:=$(ros2 pkg prefix dvrk_psm_ros2_nodes)/share/dvrk_psm_ros2_nodes/config/custom.yaml
```

**Custom config:**
Copy `config/default.yaml` and modify:

* `arm_relpath`, `tool_relpath`: paths to model JSONs 

## Isaac Sim setup

Run Isaac Sim and the dVRK system configuration for MTMR + PS Move + PSM1, then start the PSM kinematics node. Use separate terminals for each block so logs stay readable.

**Prerequisites**: Source ROS2 in all terminals:
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```

**Terminal 1: IsaacSim**

Launch IsaacSim and load the USD scene with PSM robot and ActionGraph:
```bash
export isaac_sim_package_path=$HOME/isaacsim/_build/linux-x86_64/release/
$isaac_sim_package_path/isaac-sim.sh
```
Topics: Subscribes to `/PSM1/servo_jp` and `/PSM1/jaw/servo_jp`, publishes `/PSM1/measured_js`, `/PSM1/setpoint_js`, `/PSM1/jaw/measured_js`, and `/PSM1/jaw/setpoint_js`.

**Terminal 2: dVRK System**

Start the dVRK system bridge with MTMR (PSMove controller):
```bash
cd src/cisst-saw/sawIntuitiveResearchKit/share/system/
ros2 run dvrk_robot dvrk_system -j system-MTMR-PSMove-PSM1_from_ROS.json
```
Topics: Publishes `{namespace}/state_command` and `{namespace}/servo_cp` (cartesian pose commands from MTM).

**Terminal 3: Kinematic Node**

Launch only the kinematics node (Isaac Sim provides joint simulation):
```bash
ros2 run dvrk_psm_ros2_nodes psm_kinematic_ros2node \
  --ros-args \
  --params-file $(ros2 pkg prefix dvrk_psm_ros2_nodes)/share/dvrk_psm_ros2_nodes/config/default.yaml \
  -p arm_namespace_prefix:=PSM1
```

**Notes:**
- The launch file also starts the joint-level node, which is not needed when Isaac Sim is simulating the joints.
- MTM operates in cartesian space, PSM in joint space on the Isaac Sim side.
- The kinematic node bridges them: subscribes to `servo_cp` from MTM, computes IK, publishes `servo_jp` to PSM, then computes FK from `setpoint_js` and publishes `setpoint_cp`/`measured_cp` back to MTM for feedback.

## Mental model / topic flow
- For hardware or simulation: MTM/PS Move (e.g., from `sawPSMove`) publishes Cartesian servo commands on `/PSM1/servo_cp`.
- `psm_kinematic_ros2node` runs IK and republishes joint targets on `/PSM1/servo_jp`; it also computes FK for any incoming `/PSM1/setpoint_js` to keep `/PSM1/setpoint_cp` and `/PSM1/measured_cp` populated.
- If you use the provided launch file, `psm_jointlevel_ros2node` republishes `/PSM1/servo_jp` as `/PSM1/setpoint_js` at 500 Hz and mirrors jaw commands. Use this when you need a simple joint streamer; skip it when Isaac Sim or another simulator handles the joints.
- The dVRK system JSON `system-MTMR-PSMove-PSM1_from_ROS.json` in `cisst-saw/sawIntuitiveResearchKit/share/system` wires the MTMR + PS Move inputs to the PSM topics used here.

## Simulate MTMR/PS Move commands (Deprecated)

**Note:** The Python-based `psmove_mtmr_sim` script has been moved to the `deprecated/` folder. The package now uses C++ implementations.

If you need to test the system, you can still use the deprecated script:

```bash
python3 deprecated/psmove_mtmr_sim.py --ros-args -p arm_namespace_prefix:=PSM1 -p rate_hz:=50.0 -p smoothing_alpha:=0.05
```

What it does:
- Streams `/PSM1/servo_cp` with Cartesian sine wave
- Streams `/PSM1/jaw/servo_jp` with a jaw angle between 0 and 0.6 rad

