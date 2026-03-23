# Module Decomposition

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          OPERATOR SIDE                                   │
│  PICO VR (arm tracking)  |  MANUS Gloves (fingers)  |  Foot Paddle     │
└──────┬───────────────────────────┬───────────────────────────┬───────────┘
       │                           │                           │
┌──────▼───────────────────────────▼───────────────────────────▼───────────┐
│                          INPUT LAYER                                     │
│                                                                          │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │ pico_input   │  │ manus_input │  │ foot_paddle  │  │ camera_input  │  │
│  │              │  │             │  │ _input       │  │ (D435i × 2)   │  │
│  │ Absolute map │  │ 21-pt keypts│  │ USB HID      │  │ RGB-D streams │  │
│  │ + One-Euro   │  │ /hand_input │  │ /paddle_state│  │ /cam_ee/...   │  │
│  │ /arm_target  │  │             │  │              │  │ /cam_global/..│  │
│  └──────┬───────┘  └──────┬──────┘  └──────┬───────┘  └──────┬────────┘  │
└─────────┼─────────────────┼────────────────┼─────────────────┼───────────┘
          │                 │                │                 │
┌─────────▼─────────────────▼────────────────▼─────────────────▼───────────┐
│                          CONTROL LAYER                                    │
│                                                                           │
│  ┌──────────────────────────────────────────────────────────────────┐     │
│  │                    teleop_manager (central coordinator)          │     │
│  │                                                                  │     │
│  │  ┌──────────┐  ┌──────────────┐  ┌───────────────┐             │     │
│  │  │ Absolute │  │  Collision   │  │  Paddle State │             │     │
│  │  │ Mapping  │  │  Manager     │  │  Machine      │             │     │
│  │  │ + Offset │  │  (FCL)       │  │               │             │     │
│  │  └────┬─────┘  └──────┬───────┘  └───────┬───────┘             │     │
│  │       │               │                   │                     │     │
│  │       └───────────────┼───────────────────┘                     │     │
│  │                       ▼                                         │     │
│  │              Virtual Target → Safe Projection                   │     │
│  └──────────────────────┬──────────────────────────────────────────┘     │
│                         │                                                 │
│  ┌──────────────┐  ┌────▼─────────┐                                      │
│  │ wujihand     │  │ franka       │                                      │
│  │ _controller  │  │ _controller  │                                      │
│  │ (100Hz)      │  │ (100Hz)      │                                      │
│  │ retarget     │  │ IK solve     │                                      │
│  └──────┬───────┘  └──────┬───────┘                                      │
└─────────┼─────────────────┼──────────────────────────────────────────────┘
          │                 │
┌─────────▼─────────────────▼──────────────────────────────────────────────┐
│                          OUTPUT LAYER                                     │
│                                                                           │
│  ┌──────────────┐  ┌──────────────┐                                      │
│  │ wujihand     │  │ franka       │                                      │
│  │ _driver      │  │ _ros2        │                                      │
│  │ (1000Hz C++) │  │ (1000Hz FCI) │                                      │
│  └──────┬───────┘  └──────┬───────┘                                      │
└─────────┼─────────────────┼──────────────────────────────────────────────┘
          │                 │
          ▼                 ▼
     Wuji Hand (R)    Franka FR3 × 2
```

## Cross-Cutting Concerns

```
┌────────────────────────────────────────────────┐
│  data_recorder        │  teleop_visualization  │
│  - sync all streams   │  - RViz2 3D view       │
│  - episode management │  - camera feeds        │
│  - export for VTLA    │  - status dashboard    │
├────────────────────────┤                        │
│  teleop_calibration   │  franka_description    │
│  - PICO↔Robot mapping │  - dual-arm URDF       │
│  - sim↔real alignment │  - collision capsules  │
└────────────────────────────────────────────────┘
```

## Module Details

### M-01: pico_input (ADAPT from existing)

| Item | Detail |
|------|--------|
| **Source** | Adapt from `input_devices/pico_input` |
| **Change** | Incremental → absolute mapping + offset compensation |
| **Reuse** | One-Euro filter, data source abstraction, PICO SDK interface |
| **Input** | PICO SDK (XRoboToolkit) |
| **Output** | `/left_arm_target_pose`, `/right_arm_target_pose` (PoseStamped) |
| **Config** | Calibration matrix, filter params, scale factor |

### M-02: manus_input (REUSE as-is)

| Item | Detail |
|------|--------|
| **Source** | `input_devices/manus_input` |
| **Change** | None |
| **Input** | MANUS SDK (USB dongle) |
| **Output** | `/hand_input` (Float32MultiArray, 126 values) |

### M-03: foot_paddle_input (NEW)

| Item | Detail |
|------|--------|
| **Purpose** | Read FS2019_USB2 foot paddle via Linux HID |
| **Output** | `/paddle_state` (std_msgs/Bool) |
| **Frequency** | >= 100Hz |
| **Notes** | Simple USB HID device, likely shows as /dev/input/eventX |

### M-04: camera_input (NEW)

| Item | Detail |
|------|--------|
| **Purpose** | D435i × 2 RGB-D streaming |
| **Output** | `/cam_ee/{color,depth,pointcloud}`, `/cam_global/{color,depth,pointcloud}` |
| **Dependency** | `realsense2_camera` ROS2 package |
| **Config** | Serial numbers, resolution, FPS |

### M-05: franka_controller (NEW - CRITICAL)

| Item | Detail |
|------|--------|
| **Purpose** | Dual FR3 arm control |
| **Input** | Safe target poses from teleop_manager |
| **Output** | Joint commands to franka_ros2 |
| **IK** | MoveIt2 or libfranka built-in |
| **Frequency** | 100Hz command, 1000Hz FCI internal |
| **Dependency** | `franka_ros2`, `libfranka` |
| **Config** | Robot IPs, joint limits, tool params |

### M-06: wujihand_controller (REUSE with minor adapt)

| Item | Detail |
|------|--------|
| **Source** | `controller/wujihand_node.py` |
| **Change** | Remove tianji_arm_node entry point from setup.py |
| **Input** | `/hand_input` (from manus_input) |
| **Output** | Joint commands to wujihandros2 driver |

### M-07: collision_manager (NEW - CRITICAL)

| Item | Detail |
|------|--------|
| **Purpose** | Real-time collision detection + safe projection |
| **Algorithm** | FCL distance query with capsule geometry |
| **Input** | Current joint states (all), virtual target poses |
| **Output** | `/collision_state`, safe projected poses |
| **Frequency** | 100Hz (< 1ms per check) |
| **Collision Pairs** | Left arm ↔ Right arm, Hand ↔ own arm, All ↔ table/env |
| **Dependency** | `python-fcl` or `hpp-fcl` |

### M-08: teleop_manager (NEW - CRITICAL)

| Item | Detail |
|------|--------|
| **Purpose** | Central coordinator: mapping + safety + paddle logic |
| **Subsystems** | Absolute mapping, collision manager, paddle state machine |
| **Logic** | See unified state handling in franka_teleop_plan.md |

```python
# Pseudocode
virtual_target = absolute_map(human) + offset

if paddle_off:
    freeze, record offset on resume
elif collision(virtual_target):
    robot = project_to_nearest_safe(virtual_target)
else:
    robot = virtual_target
```

### M-09: data_recorder (NEW)

| Item | Detail |
|------|--------|
| **Purpose** | Synchronized multi-stream recording |
| **Streams** | All streams from PRD section 2.3.1 |
| **Format** | HDF5 (primary) + rosbag2 (backup) |
| **Features** | Episode management, metadata, paddle-based segmentation |
| **Interface** | Service: `/recorder/start`, `/recorder/stop` |

### M-10: teleop_calibration (NEW)

| Item | Detail |
|------|--------|
| **Purpose** | PICO ↔ Robot coordinate mapping calibration |
| **Method** | Guided calibration procedure (match N poses) |
| **Output** | T_calibration matrix saved to YAML |
| **Also** | Sim-to-real calibration support (P2) |

### M-11: teleop_visualization (ADAPT from monitor)

| Item | Detail |
|------|--------|
| **Source** | Adapt from `wuji_teleop_monitor` |
| **Features** | RViz2 3D, camera feeds, status dashboard, recording indicator |
| **Framework** | RViz2 + rqt panels or PyQt5 |

### M-12: franka_description (NEW)

| Item | Detail |
|------|--------|
| **Purpose** | Dual FR3 URDF + Wuji Hand combined model |
| **Contents** | URDF/xacro, collision capsules, visual meshes |
| **Source** | Franka official URDF + wujihand_urdf |
| **Config** | Arm spacing (60cm), mount orientation |

### M-13: teleop_bringup (ADAPT)

| Item | Detail |
|------|--------|
| **Source** | Adapt from `wuji_teleop_bringup` |
| **Launch files** | Full system, arm-only, hand-only, calibration, recording |

## Dependency Graph (Build Order)

```
Level 0 (no deps):     franka_description, wujihand_urdf, foot_paddle_input
Level 1:               manus_input, camera_input, pico_input
Level 2:               wujihand_output, franka_controller, collision_manager
Level 3:               wujihand_controller, teleop_manager
Level 4:               data_recorder, teleop_visualization
Level 5:               teleop_bringup (launch files)
```

## File Count Estimate

| Module | Files (approx) |
|--------|----------------|
| Reused (manus, wujihand, retarget, urdf) | ~50 (existing) |
| franka_controller | ~10 |
| collision_manager | ~8 |
| teleop_manager | ~6 |
| data_recorder | ~8 |
| foot_paddle_input | ~4 |
| camera_input | ~4 |
| teleop_calibration | ~6 |
| teleop_visualization | ~10 |
| franka_description | ~5 |
| teleop_bringup | ~8 |
| **Total new** | **~70** |
