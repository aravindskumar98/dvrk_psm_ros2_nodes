# Isaac Sim PSM Script

This Python script replaces the USD ActionGraph approach. It programmatically loads the PSM robot, enables ROS2 bridge, and handles joint control with feedback publishing.

## Setup Environment

Before running the script, configure the terminal to use IsaacSim's ROS2 environment:

```bash
export isaac_sim_package_path=$HOME/isaacsim/_build/linux-x86_64/release/
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# WARNING: Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again potentially leading to conflicts
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/jazzy/lib
```

## Run the Script

```bash
$isaac_sim_package_path/python.sh isaac_sim_simulated_psm.py
```

## What It Does

- Loads the PSM robot USD file into IsaacSim
- Enables the ROS2 bridge extension
- Creates a PSM controller that:
  - **Subscribes** to `/PSM1/servo_jp` (arm joint commands) and `/PSM1/jaw/servo_jp` (jaw commands)
  - **Publishes** at 30Hz:
    - `/PSM1/measured_js` (current arm joint positions)
    - `/PSM1/setpoint_js` (commanded arm joint positions)
    - `/PSM1/jaw/measured_js` (current jaw position, 0=closed, 0.6=open)
    - `/PSM1/jaw/setpoint_js` (commanded jaw position)

## Using with dVRK System

After IsaacSim is running with this script, follow the setup from the main README:

**Terminal 2: dVRK System**
```bash
cd src/cisst-saw/sawIntuitiveResearchKit/share/system/
ros2 run dvrk_robot dvrk_system -j system-MTMR-PSMove-PSM1_from_ROS.json
```

**Terminal 3: Kinematic Node**
```bash
ros2 run dvrk_psm_ros2_nodes psm_kinematic_ros2node \
  --ros-args \
  --params-file $(ros2 pkg prefix dvrk_psm_ros2_nodes)/share/dvrk_psm_ros2_nodes/config/default.yaml \
  -p arm_namespace_prefix:=PSM1
```

## Control Modes

The script has two control modes (set `USE_ROS2_CONTROL` variable):
- **True** (default): Responds to ROS2 servo_jp commands
- **False**: Runs autonomous sine wave motion for testing

## Configuration

Key variables in the script:
- `ASSET_PATH`: Path to PSM USD files
- `USD_NAME`: USD filename (default: `PSM2ROS2.usd`)
- `PSM_PRIM_PATH`: Prim path in the scene (default: `/World/PSM2`)
- `PUBLISH_RATE_HZ`: Joint state publishing rate (default: 30 Hz)




