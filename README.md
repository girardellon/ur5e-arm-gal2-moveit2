# UR5e Arm + Contactile GAL2 MoveIt2 (ROS 2 Humble)

This repository provides a complete ROS 2 Humble and MoveIt2 integration stack for controlling a Universal Robots UR5e manipulator equipped with a Contactile GAL2 two-finger angular gripper.

The project includes:

- A combined URDF description of UR5e + GAL2
- A MoveIt2 configuration (SRDF, OMPL, kinematics, joint limits, controllers)
- A ROS 2 driver for the Contactile GAL2 gripper
- A MoveIt-compatible gripper action bridge
- A high-level execution node for coordinated arm + gripper actions
- RViz integration for planning and execution

The goal is to provide a consistent motion planning and execution framework for the UR5e arm while exposing the GAL2 gripper through a clean, MoveIt-compatible control interface.

---

## 1. Repository Structure

```
ur5e-arm-gal2-moveit2/
├── contactile_gripper_ros2/
├── contactile_gripper_moveit_bridge/
├── ur5e_gripper_description/
├── ur5e_gripper_moveit_config/
├── ur5e_gal2_actions/
├── README.md
└── .gitignore
```

### 1.1 contactile_gripper_ros2

ROS 2 driver package wrapping the Contactile C++ library.

Provides:

- Gripper node executable
- Launch file for starting the driver
- Service interfaces for:
  - Serial port configuration
  - Gripper command execution

This package manages low-level communication with the GAL2 hardware via the vendor-provided static library.

---

### 1.2 contactile_gripper_moveit_bridge

MoveIt integration layer that:

- Exposes a gripper action interface compatible with MoveIt
- Publishes aggregated joint states when required
- Translates MoveIt action requests into gripper driver service calls

This bridge decouples MoveIt from vendor-specific interfaces and provides a standardized action-based API.

---

### 1.3 ur5e_gripper_description

Robot description package containing the combined URDF model of:

- UR5e manipulator
- Attached Contactile GAL2 gripper

The model is used by:

- robot_state_publisher
- MoveIt robot model loader
- RViz visualization

---

### 1.4 ur5e_gripper_moveit_config

MoveIt2 configuration package including:

- SRDF (planning groups and end-effector definitions)
- OMPL planning pipeline configuration
- KDL kinematics configuration
- Joint limits
- Controller configuration
- Bringup launch file

The configuration defines planning groups for:

- UR5e arm
- GAL2 gripper
- Optional combined arm + gripper group

---

### 1.5 ur5e_gal2_actions

High-level execution node for coordinated arm and gripper behavior.

Provides:

- Parameterized action sequences
- Coordinated motion execution (arm + gripper)
- Configurable motion parameters via YAML

This package enables structured task-level control beyond manual RViz planning.

---

## 2. System Architecture

Typical runtime structure:

```
UR5e Hardware
  │
  ├── ur_robot_driver (ros2_control)
  │       ├── Joint states (/joint_states)
  │       └── Arm trajectory action (FollowJointTrajectory)
  │
  └── Contactile GAL2
          └── contactile_gripper_ros2 (driver node)
                  └── contactile_gripper_moveit_bridge
                          └── MoveIt2 (move_group) + RViz
                                  └── ur5e_gal2_actions
```

MoveIt2 performs motion planning and trajectory execution for the UR5e arm.

The gripper is controlled through an action bridge that translates MoveIt commands into driver service calls.

The `ur5e_gal2_actions` node provides higher-level coordinated behavior.

---

## 3. Requirements

### Software

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt2
- ur_robot_driver
- ur_description
- kdl_kinematics_plugin
- C++17 compatible compiler

### Hardware

- Universal Robots UR5e
- Contactile GAL2 gripper
- Ethernet connection to UR controller
- Serial access to gripper device

---

## 4. Build Instructions

From your ROS 2 workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To build only the packages in this repository:

```bash
colcon build --packages-select \
  contactile_gripper_ros2 \
  contactile_gripper_moveit_bridge \
  ur5e_gripper_description \
  ur5e_gripper_moveit_config \
  ur5e_gal2_actions
```

---

## 5. Execution

The system is typically started in separate terminals.

---

### 5.1 Start the Contactile gripper driver

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch contactile_gripper_ros2 GRIPPER_ROS2.launch.py
```

If required, configure the serial port:

```bash
ros2 service call /contactile_gripper_port_set contactile_gripper_ros2/srv/GripperSerialPort "{port: '/dev/contactile_gripper'}"
```

---

### 5.2 Start UR driver + MoveIt2 + RViz

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ur5e_gripper_moveit_config ur5e_gripper_moveit_bringup.launch.py
```

This bringup should:

- Start ur_robot_driver
- Start move_group
- Launch RViz

Ensure the UR robot IP and bringup parameters match your setup.

---

### 5.3 Start the MoveIt gripper bridge (if not included in bringup)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run contactile_gripper_moveit_bridge contactile_gripper_action_bridge
```

Verify the action interface:

```bash
ros2 action list
```

---

### 5.4 Start the high-level action node

```bash
ros2 launch ur5e_gal2_actions ur5e_gal2_actions.launch.py
```

This node executes coordinated arm and gripper behaviors based on the configured parameters.

---

## 6. MoveIt Configuration Overview

Planning:

- Planning pipeline: OMPL
- Planner: RRTConnect
- Kinematics: KDL
- Arm group: UR5e manipulator
- Gripper group: GAL2
- Optional combined group: arm + gripper

Execution:

- Arm controller: FollowJointTrajectory via UR driver
- Gripper control: action interface provided by bridge

---

## 7. Validation and Debugging

### Verify arm controller availability

```bash
ros2 action list -t | grep follow_joint_trajectory
```

### Verify joint states

```bash
ros2 topic echo /joint_states --once
```

### Verify gripper services

```bash
ros2 service list | grep contactile
```

### Verify gripper action bridge

```bash
ros2 action list | grep gripper
```

---

## 8. Known Limitations

- No simulation-only configuration included
- No preconfigured RViz visualization file
- Gripper impedance or force control not implemented
- No dynamic collision object integration
- Vendor static library included directly in the source tree
