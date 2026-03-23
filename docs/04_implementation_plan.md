# Implementation Plan

## Phase Overview

```
Phase 1: Foundation (Week 1-2)
  ├── Franka basic control
  ├── Foot paddle driver
  └── URDF + collision geometry

Phase 2: Teleoperation Core (Week 3-4)
  ├── PICO absolute mapping
  ├── Teleop manager + paddle logic
  └── Hand pipeline (MANUS → Wuji)

Phase 3: Safety (Week 5)
  ├── Collision detection
  └── Safe projection

Phase 4: Data & Viz (Week 6-7)
  ├── Data recorder
  ├── Visualization
  └── Camera integration

Phase 5: Integration & Polish (Week 8)
  ├── Full system launch
  ├── Calibration tools
  └── Testing & tuning
```

## Phase 1: Foundation

### 1.1 Franka FR3 Basic Control

**Goal**: Move both arms via ROS2

```bash
sudo apt install ros-humble-franka-ros2 ros-humble-moveit
```

Tasks:
- [ ] Install franka_ros2, configure dual-arm Ethernet
- [ ] Create `franka_description` package (dual-arm URDF with 60cm offset)
- [ ] Verify `franka_ros2` can control both arms independently
- [ ] Test Cartesian impedance control mode (preferred for teleop)
- [ ] Measure control loop latency

**Deliverable**: Both FR3 arms moving to commanded poses via ROS2 topic

### 1.2 Foot Paddle Driver

**Goal**: Read FS2019_USB2 as ROS2 topic

Tasks:
- [ ] Identify HID device: `ls /dev/input/event*`, `evtest`
- [ ] Write `foot_paddle_input` node (read `/dev/input/eventX`)
- [ ] Publish `/paddle_state` (Bool) at 100Hz
- [ ] Test latency (press → topic < 5ms)

**Deliverable**: `/paddle_state` topic active, responsive

### 1.3 URDF + Collision Geometry

**Goal**: Combined model for RViz and collision

Tasks:
- [ ] Get FR3 official URDF from franka_ros2
- [ ] Create dual-arm xacro (parameterized spacing)
- [ ] Attach Wuji Hand URDF to right arm EE
- [ ] Define collision capsules for each link (simplified geometry)
- [ ] Visualize in RViz2

**Deliverable**: Full dual-arm + hand model visible in RViz2

## Phase 2: Teleoperation Core

### 2.1 PICO Absolute Mapping

**Goal**: PICO tracking → Franka target poses (absolute mode)

Tasks:
- [ ] Adapt `pico_input` incremental controller → absolute mapping
- [ ] Implement calibration procedure (record T_calibration)
- [ ] Add scale factor (configurable 0.3~0.5)
- [ ] Keep One-Euro filter for smoothing
- [ ] Publish `/left_arm_target_pose`, `/right_arm_target_pose`

**Deliverable**: PICO hand motion → smooth Franka target poses

### 2.2 Teleop Manager

**Goal**: Central coordinator with paddle + offset logic

Tasks:
- [ ] Implement teleop_manager node
- [ ] Paddle state machine: ACTIVE / FROZEN / RESUMING
- [ ] Offset compensation on re-press (no jump)
- [ ] Forward safe targets to franka_controller
- [ ] Publish `/teleop_status`

```
State Machine:
  FROZEN ──(paddle press)──► RESUMING ──(offset computed)──► ACTIVE
    ▲                                                          │
    └──────────────────(paddle release)────────────────────────┘
```

**Deliverable**: Smooth freeze/resume with no trajectory discontinuity

### 2.3 Hand Pipeline

**Goal**: MANUS → Wuji Hand (right only)

Tasks:
- [ ] Verify manus_input works (may need MANUS SDK install)
- [ ] Verify wujihand_controller works (needs wujihandcpp SDK)
- [ ] Replace EMA filter with One-Euro filter in retarget config
- [ ] Test end-to-end: glove → topic → retarget → hand

**Deliverable**: Right hand follows MANUS glove input

## Phase 3: Safety

### 3.1 Collision Detection

**Goal**: Real-time collision checking at 100Hz

Tasks:
- [ ] Install `python-fcl` or `hpp-fcl`
- [ ] Create `collision_manager` node
- [ ] Load capsule geometry from config
- [ ] Implement collision pair checking (arm↔arm, hand↔arm, all↔table)
- [ ] Publish `/collision_state`
- [ ] Benchmark: verify < 1ms per check

**Deliverable**: Collision detection running at 100Hz with collision state published

### 3.2 Safe Projection

**Goal**: Project virtual target to nearest safe point

Tasks:
- [ ] Implement `project_to_nearest_safe()` using FCL distance query
- [ ] Implement boundary sliding (not freeze) behavior
- [ ] Integrate with teleop_manager
- [ ] Test: move hand into collision zone → arm slides along boundary

**Deliverable**: Arms never collide, smooth boundary sliding works

## Phase 4: Data & Visualization

### 4.1 Data Recorder

**Goal**: Synchronized multi-stream recording

Tasks:
- [ ] Create `data_recorder` node
- [ ] Subscribe to all data streams (see PRD 2.3.1)
- [ ] Implement timestamp synchronization (use ROS2 message timestamps)
- [ ] HDF5 export with episode structure
- [ ] rosbag2 recording as backup
- [ ] Service interface: `/recorder/start`, `/recorder/stop`
- [ ] Episode metadata (task description, duration, frame count)
- [ ] Paddle-based valid/invalid segment labeling

HDF5 structure:
```
episode_0001.hdf5
├── /timestamp                    # (N,) float64
├── /franka_left/joint_pos        # (N, 7) float64
├── /franka_left/joint_vel        # (N, 7) float64
├── /franka_left/joint_torque     # (N, 7) float64
├── /franka_left/ee_pose          # (N, 4, 4) float64
├── /franka_right/...             # same structure
├── /wuji_hand_right/joint_pos    # (N, 20) float32
├── /cam_ee/color                 # (M, H, W, 3) uint8
├── /cam_ee/depth                 # (M, H, W) uint16
├── /cam_ee/timestamp             # (M,) float64
├── /cam_global/color             # (M, H, W, 3) uint8
├── /cam_global/depth             # (M, H, W) uint16
├── /cam_global/timestamp         # (M,) float64
├── /pico/left_wrist              # (N, 7) float64 [pos+quat]
├── /pico/right_wrist             # (N, 7) float64
├── /manus/hand_input             # (N, 126) float32
├── /paddle_state                 # (N,) bool
├── /collision_state              # (N,) bool
└── /metadata                     # JSON attrs
```

**Deliverable**: Complete episode recording with all streams, HDF5 export

### 4.2 Camera Integration

**Goal**: D435i × 2 streaming

Tasks:
- [ ] Install `realsense-ros`
- [ ] Configure dual D435i (serial-based)
- [ ] Verify RGB + depth streaming at 30Hz
- [ ] Add to data_recorder subscription list

**Deliverable**: Both cameras streaming and recorded

### 4.3 Visualization

**Goal**: Unified monitoring interface

Tasks:
- [ ] Adapt `wuji_teleop_monitor` for new system
- [ ] RViz2 config: dual-arm model + collision capsules + camera frustums
- [ ] Status panel: connection states, frequencies, paddle, recording
- [ ] Camera feed display (EE + global)
- [ ] Collision visualization (green=safe, red=collision zone)

**Deliverable**: One-screen monitoring of entire system

## Phase 5: Integration & Polish

### 5.1 Full System Launch

Tasks:
- [ ] Create `teleop_bringup` launch files
  - `full_teleop.launch.py` — everything
  - `arm_only.launch.py` — arms without hand
  - `calibration.launch.py` — calibration mode
  - `recording.launch.py` — recording mode
- [ ] Test full startup sequence
- [ ] Test graceful shutdown (all hardware safe-stopped)

### 5.2 Calibration Tools

Tasks:
- [ ] Implement guided calibration CLI
- [ ] Save/load calibration YAML
- [ ] Verify calibration accuracy (< 5mm error)

### 5.3 Testing & Tuning

Tasks:
- [ ] End-to-end latency measurement (PICO → Franka motion)
- [ ] Filter parameter tuning (One-Euro beta, min_cutoff)
- [ ] Scale factor optimization for target task
- [ ] Collision margin tuning
- [ ] 30-minute continuous operation test
- [ ] Data quality validation (load in VTLA pipeline, verify)

## Risk Register

| Risk | Impact | Mitigation |
|------|--------|------------|
| franka_ros2 dual-arm latency | High | Test early (Phase 1), fallback to libfranka direct |
| FCL too slow with full geometry | Medium | Use simplified capsules, benchmark in Phase 3 |
| PICO WiFi latency/jitter | Medium | One-Euro filter absorbs, test range limits |
| Wuji Hand SDK unavailable | Medium | Hand pipeline blocked, arm pipeline independent |
| FS2019 no Linux driver | Low | Simple HID, worst case use evdev directly |
| D435i USB bandwidth (2 cameras) | Low | Use separate USB controllers, reduce resolution |
