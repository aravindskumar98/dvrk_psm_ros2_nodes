
# dvrk_psm_ros2_nodes

ROS 2 nodes for dVRK PSM kinematics and joint-level control.

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

