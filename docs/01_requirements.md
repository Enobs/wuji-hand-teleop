# Franka Dual-Arm Teleoperation System - Requirements Document (PRD)

## 1. Project Overview

### 1.1 Mission
Build a ROS2-based dual-arm teleoperation system for **real-time remote operation** and **imitation learning data collection**, targeting tabletop manipulation tasks. Data feeds into world models (VTLA, DreamTac).

### 1.2 Hardware Configuration

| Device | Spec | Qty | Status |
|--------|------|-----|--------|
| Franka FR3 | 7-DOF robot arm | 2 (parallel mount, ~60cm apart) | Available |
| Wuji Hand | 20-DOF dexterous hand | 1 right (left pending) | Right available |
| MANUS Gloves | Finger tracking | 1 pair | Available |
| PICO VR | 6-DoF arm tracking (HMD + controllers) | 1 | Available |
| Foot Paddle | FS2019_USB2 safety switch | 1 | Available |
| Intel RealSense D435i | RGB-D camera | 2 (1x EE, 1x global) | Available |

### 1.3 Physical Setup

```
         Global Camera (D435i)
              ↓ (top-down or angled)
┌──────────────────────────────┐
│          Tabletop            │
│                              │
│   ┌─────┐  60cm  ┌─────┐    │
│   │FR3-L│◄──────►│FR3-R│    │
│   │     │        │     │    │
│   │  EE │        │  EE │    │
│   │  +cam│       │+Wuji│    │
│   └─────┘        └─────┘    │
│                              │
└──────────────────────────────┘

Operator: PICO (head) + MANUS (hands) + Foot Paddle
```

Note: EE camera on one arm, Wuji Hand on right arm. Left hand hardware pending.

## 2. Functional Requirements

### 2.1 Core: Real-Time Teleoperation

| ID | Requirement | Priority |
|----|-------------|----------|
| F-01 | Absolute pose mapping (PICO → Franka EE) with calibration | P0 |
| F-02 | Hand retargeting (MANUS → Wuji Hand, 21-point → 20-joint) | P0 |
| F-03 | Foot paddle safety: release = freeze, re-press = seamless resume (offset compensation) | P0 |
| F-04 | Dual-arm independent IK solving | P0 |
| F-05 | Control frequency >= 100Hz | P0 |
| F-06 | Scale factor configurable (default 0.3~0.5) for precision | P1 |
| F-07 | One-Euro adaptive filter on arm tracking (smooth + responsive) | P1 |

### 2.2 Core: Safety & Collision

| ID | Requirement | Priority |
|----|-------------|----------|
| S-01 | Self-collision detection: left arm vs right arm | P0 |
| S-02 | Self-collision detection: hand vs own arm | P0 |
| S-03 | Collision detection: arms/hands vs tabletop/environment | P0 |
| S-04 | Safe projection: virtual target → nearest collision-free reachable point | P0 |
| S-05 | Boundary sliding (not freeze) when in collision zone | P1 |
| S-06 | Capsule-based collision geometry (not mesh) for real-time performance | P1 |
| S-07 | FCL distance query < 1ms @ 100Hz | P1 |
| S-08 | Joint limit protection | P0 |
| S-09 | Singularity avoidance | P1 |

### 2.3 Core: Data Collection

| ID | Requirement | Priority |
|----|-------------|----------|
| D-01 | Synchronized recording of ALL data streams (see 2.3.1) | P0 |
| D-02 | Continuous trajectory (no clutch breaks in valid segments) | P0 |
| D-03 | Paddle state recorded (valid/invalid segment labeling) | P0 |
| D-04 | Timestamps aligned across all streams | P0 |
| D-05 | Export format compatible with VTLA/DreamTac training | P0 |
| D-06 | Recording start/stop via GUI or service call | P1 |
| D-07 | Episode management (auto-numbering, metadata) | P1 |

#### 2.3.1 Data Streams to Record

| Stream | Source | Frequency | Format |
|--------|--------|-----------|--------|
| Franka joint positions (L/R) | FR3 × 2 | >= 100Hz | 7 × float64 each |
| Franka joint velocities (L/R) | FR3 × 2 | >= 100Hz | 7 × float64 each |
| Franka joint torques (L/R) | FR3 × 2 | >= 100Hz | 7 × float64 each |
| Franka EE pose (L/R) | FR3 × 2 | >= 100Hz | 4×4 matrix |
| Wuji Hand joint angles | Right hand | >= 100Hz | 20 × float32 |
| Wuji Hand joint state (feedback) | Right hand | 1000Hz | 20 × float32 |
| EE Camera RGB-D | D435i on EE | 30Hz | 640×480 or 848×480 |
| Global Camera RGB-D | D435i global | 30Hz | 640×480 or 848×480 |
| PICO raw tracking | HMD + controllers | 90Hz | 6-DoF poses |
| MANUS raw keypoints | Gloves | 120Hz | 21×3 per hand |
| Foot paddle state | FS2019_USB2 | >= 100Hz | bool |
| Collision state | FCL | >= 100Hz | bool + distance |

### 2.4 Visualization & Monitoring

| ID | Requirement | Priority |
|----|-------------|----------|
| V-01 | Real-time 3D visualization (RViz2 or custom) | P0 |
| V-02 | Dual-arm + hand model rendering | P0 |
| V-03 | Collision geometry overlay (capsules) | P1 |
| V-04 | Camera feeds display (EE + global) | P1 |
| V-05 | System status dashboard (connection, frequency, latency) | P1 |
| V-06 | Recording status indicator | P1 |

### 2.5 Simulation

| ID | Requirement | Priority |
|----|-------------|----------|
| SIM-01 | MuJoCo/Isaac Sim digital twin | P2 |
| SIM-02 | Sim-to-real calibration support | P2 |
| SIM-03 | Replay recorded episodes in sim | P2 |

## 3. Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NF-01 | Control loop latency (PICO → Franka command) | < 10ms |
| NF-02 | Collision check latency | < 1ms |
| NF-03 | System uptime per session | > 2 hours continuous |
| NF-04 | Data loss during recording | 0% (no dropped frames) |
| NF-05 | Platform | Ubuntu 22.04, ROS2 Humble |
| NF-06 | Franka control interface | franka_ros2 (FCI) |

## 4. Constraints

| Constraint | Detail |
|------------|--------|
| Left Wuji Hand not available | Right hand only for now, left hand interface reserved |
| FR3 dual-arm parallel mount | ~60cm spacing, same orientation |
| Foot paddle USB | FS2019_USB2, need Linux driver/HID interface |
| Network | PICO via WiFi, Franka via Ethernet (realtime kernel) |

## 5. Success Criteria

1. Operator can teleoperate dual-arm + right hand to complete tabletop pick-and-place
2. All data streams recorded synchronously with < 1ms timestamp drift
3. No collision between arms during operation (collision detection active)
4. Paddle freeze/resume works without trajectory discontinuity
5. Recorded data loadable by VTLA/DreamTac training pipeline
6. System runs stable for > 30 minutes per session

## 6. Reusable from wuji-hand-teleop

| Module | Reuse | Notes |
|--------|-------|-------|
| `manus_input` | Direct | MANUS → /hand_input |
| `wujihand_output` | Direct | Retarget + hand control |
| `wuji_retargeting` | Direct (source in repo) | Editable |
| `wujihandros2` | Direct (submodule) | C++ driver |
| `controller/wujihand_node` | Direct | Hand controller |
| `pico_input` (incremental controller) | Adapt | Change to absolute mapping |
| `pico_input` (One-Euro filter) | Reuse | Adaptive filtering |
| `wujihand_urdf` | Direct | Hand 3D model |
| `wuji_teleop_monitor` | Adapt | Add new status panels |

## 7. New Modules Needed

| Module | Purpose |
|--------|---------|
| `franka_output` | FR3 IK + control via franka_ros2 |
| `collision_manager` | FCL-based collision detection + safe projection |
| `data_recorder` | Synchronized multi-stream recording |
| `foot_paddle_input` | FS2019_USB2 HID driver |
| `teleop_calibration` | PICO-to-Franka absolute mapping calibration |
| `teleop_visualization` | Unified GUI (RViz2 + status + camera) |
| `franka_description` | FR3 dual-arm URDF + collision capsules |
| `teleop_bringup` | New launch files |
