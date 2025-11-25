
# dvrk_psm_ros2_nodes

ROS 2 nodes for dVRK PSM kinematics and joint-level control.

## What this package does
- `psm_kinematic_ros2node`: loads the PSM + tool kinematic chains from JSON (from `dvrk_config` by default). It converts Cartesian servo commands (`<arm>/servo_cp`) into joint targets (`<arm>/servo_jp`) and publishes FK for whatever joint target it last saw (`<arm>/setpoint_cp`, `<arm>/measured_cp`). It also latches an ENABLED operating state on `<arm>/operating_state`.
- `psm_jointlevel_ros2node`: simple joint-level streamer. It seeds the arm with a small insertion, listens for joint servo commands, and republishes them as setpoints at 500 Hz on `<arm>/setpoint_js`. It mirrors jaw setpoints to `jaw/measured_js` (no sensing yet).
- The launch file starts both nodes and wires the namespace via `arm_namespace` (default `PSM1`), so topics look like `/PSM1/servo_cp`, `/PSM1/servo_jp`, `/PSM1/jaw/servo_jp`, etc.

Related workspace packages (for context):
- `dvrk` and `cisst-saw`: provide the dVRK ROS tooling and system JSONs (`sawIntuitiveResearchKit` under `cisst-saw`).
- `crtk`: common message definitions (e.g., `OperatingState`).
- `sawPSMove`: PS Move controller input that feeds the MTM/PSM control stack.

## Launch

**Default (PSM1):**
```bash
ros2 launch dvrk_psm_ros2_nodes bringup.launch.py
````

**Switch to PSM2 with different tool/arm:**

```bash
ros2 launch dvrk_psm_ros2_nodes bringup.launch.py arm_namespace:=PSM2 config:=$(ros2 pkg prefix dvrk_psm_ros2_nodes)/share/dvrk_psm_ros2_nodes/config/custom.yaml
```

**Custom config:**
Copy `config/default.yaml` and modify:

* `arm_relpath`, `tool_relpath`: paths to model JSONs 

## Isaac Sim setup

Run Isaac Sim and the dVRK system configuration for MTMR + PS Move + PSM1, then start the PSM kinematics node. Use separate terminals for each block so logs stay readable:

```bash
# Terminal 1: launch Isaac Sim
$isaac_sim_package_path/isaac-sim.sh

# Terminal 2: start the dVRK system bridge
cd src/cisst-saw/sawIntuitiveResearchKit/share/system/
# (use the devel branch if needed)
ros2 run dvrk_robot dvrk_system -j system-MTMR-PSMove-PSM1_from_ROS.json

# Terminal 3: observe joint state feedback
ros2 topic echo /PSM1/measured_js

# Terminal 4: launch only the kinematics node; Isaac Sim provides joint simulation
ros2 run dvrk_psm_ros2_nodes psm_kinematic_ros2node \
  --ros-args \
  --params-file $(ros2 pkg prefix dvrk_psm_ros2_nodes)/share/dvrk_psm_ros2_nodes/config/default.yaml \
  -p arm_namespace_prefix:=PSM1
```

Notes:
- The launch file also starts the joint-level node, which is not needed when Isaac Sim is simulating the joints.
- MTM is in cartesian space, PSM is in joint space on the Isaac Sim side.

## Mental model / topic flow
- For hardware or simulation: MTM/PS Move (e.g., from `sawPSMove`) publishes Cartesian servo commands on `/PSM1/servo_cp`.
- `psm_kinematic_ros2node` runs IK and republishes joint targets on `/PSM1/servo_jp`; it also computes FK for any incoming `/PSM1/setpoint_js` to keep `/PSM1/setpoint_cp` and `/PSM1/measured_cp` populated.
- If you use the provided launch file, `psm_jointlevel_ros2node` republishes `/PSM1/servo_jp` as `/PSM1/setpoint_js` at 500 Hz and mirrors jaw commands. Use this when you need a simple joint streamer; skip it when Isaac Sim or another simulator handles the joints.
- The dVRK system JSON `system-MTMR-PSMove-PSM1_from_ROS.json` in `cisst-saw/sawIntuitiveResearchKit/share/system` wires the MTMR + PS Move inputs to the PSM topics used here.
