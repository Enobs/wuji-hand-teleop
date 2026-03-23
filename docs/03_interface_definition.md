# Interface Definition - ROS2 Topics, Services, TF

## 1. Topic Interface

### 1.1 Input Topics

| Topic | Type | Publisher | Subscriber | Hz | QoS |
|-------|------|-----------|------------|-----|-----|
| `/hand_input` | Float32MultiArray | manus_input | wujihand_controller | 120 | BEST_EFFORT/1 |
| `/paddle_state` | std_msgs/Bool | foot_paddle_input | teleop_manager | 100 | RELIABLE/1 |
| `/cam_ee/color/image_raw` | sensor_msgs/Image | camera_input | data_recorder, viz | 30 | BEST_EFFORT/1 |
| `/cam_ee/depth/image_rect_raw` | sensor_msgs/Image | camera_input | data_recorder | 30 | BEST_EFFORT/1 |
| `/cam_global/color/image_raw` | sensor_msgs/Image | camera_input | data_recorder, viz | 30 | BEST_EFFORT/1 |
| `/cam_global/depth/image_rect_raw` | sensor_msgs/Image | camera_input | data_recorder | 30 | BEST_EFFORT/1 |

### 1.2 Control Topics

| Topic | Type | Publisher | Subscriber | Hz | QoS |
|-------|------|-----------|------------|-----|-----|
| `/left_arm_target_pose` | PoseStamped | teleop_manager | franka_controller | 100 | BEST_EFFORT/1 |
| `/right_arm_target_pose` | PoseStamped | teleop_manager | franka_controller | 100 | BEST_EFFORT/1 |
| `/left_arm_safe_pose` | PoseStamped | collision_manager | franka_controller | 100 | BEST_EFFORT/1 |
| `/right_arm_safe_pose` | PoseStamped | collision_manager | franka_controller | 100 | BEST_EFFORT/1 |
| `/wuji_hand/right/joint_command` | JointState | wujihand_controller | wujihand_driver | 100 | BEST_EFFORT/1 |

### 1.3 State/Feedback Topics

| Topic | Type | Publisher | Subscriber | Hz | QoS |
|-------|------|-----------|------------|-----|-----|
| `/franka_left/joint_states` | JointState | franka_ros2 | data_recorder, viz | 100+ | BEST_EFFORT/1 |
| `/franka_right/joint_states` | JointState | franka_ros2 | data_recorder, viz | 100+ | BEST_EFFORT/1 |
| `/franka_left/ee_pose` | PoseStamped | franka_controller | data_recorder | 100 | BEST_EFFORT/1 |
| `/franka_right/ee_pose` | PoseStamped | franka_controller | data_recorder | 100 | BEST_EFFORT/1 |
| `/wuji_hand/right/joint_states` | JointState | wujihand_driver | data_recorder | 1000 | BEST_EFFORT/1 |
| `/collision_state` | custom/CollisionState | collision_manager | data_recorder, viz | 100 | RELIABLE/1 |
| `/teleop_status` | custom/TeleopStatus | teleop_manager | viz, data_recorder | 10 | RELIABLE/1 |

### 1.4 PICO Raw Topics (internal)

| Topic | Type | Publisher | Subscriber | Hz |
|-------|------|-----------|------------|-----|
| `/pico/left_wrist` | PoseStamped | pico_input | teleop_manager | 90 |
| `/pico/right_wrist` | PoseStamped | pico_input | teleop_manager | 90 |
| `/pico/hmd` | PoseStamped | pico_input | data_recorder | 90 |

## 2. Service Interface

| Service | Type | Server | Purpose |
|---------|------|--------|---------|
| `/teleop/calibrate` | Trigger | teleop_calibration | Start calibration procedure |
| `/teleop/save_calibration` | Trigger | teleop_calibration | Save calibration to YAML |
| `/recorder/start` | Trigger | data_recorder | Start recording episode |
| `/recorder/stop` | Trigger | data_recorder | Stop and save episode |
| `/wuji_hand/switch_mode` | SetBool | wujihand_controller | TELEOP/INFERENCE mode |
| `/collision/enable` | SetBool | collision_manager | Enable/disable checking |
| `/pico_input/init` | Trigger | pico_input | Initialize tracking origin |

## 3. TF Tree

```
world (fixed, robot base)
│
├── franka_left_base (static: Y = +0.03m from world center)
│   ├── franka_left_link0 ... link7 (franka_ros2)
│   │   └── franka_left_ee
│   │       ├── cam_ee_link (static: camera mount offset)
│   │       └── wuji_hand_right_base (static: hand mount) [on right arm]
│   │           └── wuji_hand joints...
│   └── franka_left_collision_capsules (for FCL)
│
├── franka_right_base (static: Y = -0.03m from world center)
│   ├── franka_right_link0 ... link7 (franka_ros2)
│   │   └── franka_right_ee
│   └── franka_right_collision_capsules
│
├── table_surface (static: Z offset)
│
└── cam_global_link (static: global camera mount)
```

Note: 60cm spacing → each arm offset ±30mm (±0.03m) from center along Y axis.

## 4. Custom Message Definitions

### 4.1 CollisionState.msg

```
std_msgs/Header header
bool is_colliding                    # any collision detected
float32 min_distance                 # closest pair distance (m)
string closest_pair_a                # link name A
string closest_pair_b                # link name B
geometry_msgs/Point nearest_safe     # nearest collision-free point
```

### 4.2 TeleopStatus.msg

```
std_msgs/Header header
string mode                          # "teleop" / "idle" / "recording"
bool paddle_pressed                  # foot paddle state
bool left_arm_tracking               # PICO tracking valid
bool right_arm_tracking
bool hand_tracking                   # MANUS tracking valid
bool collision_active                # collision detection enabled
bool is_colliding                    # currently in collision
uint32 episode_number                # current recording episode
float32 recording_duration           # seconds since recording start
```

### 4.3 EpisodeMetadata.msg

```
std_msgs/Header header
uint32 episode_id
string task_description
float32 duration
uint32 num_frames
string[] data_streams                # list of recorded topics
string export_path
```

## 5. Configuration Files

| Config | Package | Contents |
|--------|---------|----------|
| `franka_dual_arm.yaml` | franka_controller | Robot IPs, joint limits, tool params |
| `collision_config.yaml` | collision_manager | Capsule geometry, safety margins, collision pairs |
| `calibration.yaml` | teleop_calibration | T_calibration, scale, filter params |
| `recording_config.yaml` | data_recorder | Streams to record, format, export path |
| `camera_config.yaml` | camera_input | D435i serials, resolution, FPS |
| `pico_input.yaml` | pico_input | Filter params, tracker mapping |
| `wujihand_ik.yaml` | wujihand_output | Hand serial, retarget config |

## 6. Data Flow Summary

```
PICO ──90Hz──► pico_input ──90Hz──► teleop_manager ──100Hz──► collision_manager
                                          │                          │
                                          │ virtual_target    safe_target
                                          │                          │
                                          ▼                          ▼
                                    paddle logic ◄── foot_paddle ── franka_controller ──► FR3
                                          │                                     │
                                          │                              joint_states
                                          ▼                                     │
MANUS ─120Hz─► manus_input ──► wujihand_controller ──► wujihand_driver ──► Wuji Hand
                                                                               │
D435i×2 ─────► camera_input ──────────────────┐                          joint_states
                                               │                               │
                                               ▼                               ▼
                                         data_recorder ◄───────────────── ALL streams
                                               │
                                               ▼
                                          HDF5 / rosbag2
```
