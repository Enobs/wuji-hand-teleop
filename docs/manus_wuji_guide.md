# MANUS Glove → Wuji Hand 遥操作指南

## 系统要求

### 操作系统 & ROS2

- Ubuntu 22.04 LTS
- ROS2 Humble
- Conda environment `ros2_teleoperation` (Python 3.10)

### 系统依赖

```bash
sudo apt install g++-13 libncursesw5-dev libusb-1.0-0-dev -y
# g++-13: wujihandcpp 需要 C++20 <format> 支持
# ncurses: MANUS SDK 依赖
# libusb: Wuji Hand USB 通讯
```

如果 g++-13 不可用:
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt update && sudo apt install g++-13 -y
```

### Python 依赖

```bash
conda create -n ros2_teleoperation python=3.10 -y
conda activate ros2_teleoperation
pip install cmake --upgrade          # wujihandcpp 需要 cmake >= 3.24
pip install empy==3.3.4 lark catkin_pkg  # ROS2 消息编译 (empy 必须 3.x)
pip install scipy numpy              # retarget 依赖
pip install wujihandpy               # Wuji Hand Python SDK
```

### USB 权限 (udev rules)

```bash
# MANUS Dongle
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="3325", MODE="0666", GROUP="plugdev"' | \
    sudo tee /etc/udev/rules.d/99-manus.rules

# Wuji Hand
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/95-wujihand.rules

# 生效
sudo udevadm control --reload-rules && sudo udevadm trigger
```

添加规则后需要**拔插设备**才会生效。

### 编译

```bash
cd ~/Documents/Preh/Teleoperation/wuji-hand-teleop

# 1. wuji_retargeting (pip install, 不走 colcon)
cd src/wuji_retargeting && pip install -e . && cd ../..
touch src/wuji_retargeting/COLCON_IGNORE

# 2. wujihandcpp (C++ SDK)
cd src/wujihandpy
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
make -j$(nproc)
cd ../../..
touch src/wujihandpy/COLCON_IGNORE

# 3. ROS2 packages
export WUJIHANDPY_SOURCE_DIR=$HOME/Documents/Preh/Teleoperation/wuji-hand-teleop/src/wujihandpy
colcon build --symlink-install --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
source install/setup.bash
```

### Shebang 修复

colcon build 后 entry point 的 shebang 可能指向系统 Python。需要修复:

```bash
CONDA_PY=#!/home/yinan/miniconda3/envs/ros2_teleoperation/bin/python3

sed -i "1s|.*|$CONDA_PY|" install/controller/lib/controller/wujihand_controller
sed -i "1s|.*|$CONDA_PY|" install/manus_input_py/lib/manus_input_py/manus_input
```

验证:
```bash
head -1 install/controller/lib/controller/wujihand_controller
# 应该是: #!/home/yinan/miniconda3/envs/ros2_teleoperation/bin/python3
```

用了 `--symlink-install` 的话，修复一次即可，不需要每次 build 后重做。

## 硬件准备

| 设备 | 连接方式 | 说明 |
|------|---------|------|
| MANUS Quantum Metaglove | USB Dongle | 手套开机后自动配对 |
| MANUS USB Dongle | USB-A 插电脑 | 需要 udev 权限 |
| Wuji Hand (右手) | USB-C 插电脑 | 序列号: 307B37583333 |

## 校准 MANUS 手套

首次使用或更换手套后需要校准。Linux 原生校准，不需要 Windows MANUS Core。

```bash
# 确保没有其他 manus 节点在跑
ros2 run manus_ros2 manus_calibration
```

**三步校准:**
1. **手放平面** — 手放在水平非金属表面，手指合拢，拇指向外伸 → 摆好后按 Enter
2. **握拳** — 举起手，所有手指握成拳头，拇指靠侧面 → 摆好后按 Enter
3. **开合手指** — 用指尖反复触碰掌根 → 按 Enter 后持续做动作直到完成

校准文件保存在: `install/manus_ros2/share/manus_ros2/calibration/RightQuantumMetaglove.mcal`

**注意:**
- 项目自带的 `LeftMetaglovePro.mcal` / `RightMetaglovePro.mcal` 是给 MetaglovePro 的，Quantum Metaglove 不兼容
- 不校准也能用，数据正常输出，只是精度差

## 快速启动

### 1. 环境准备

```bash
cd ~/Documents/Preh/Teleoperation/wuji-hand-teleop
source /opt/ros/humble/setup.bash
conda activate ros2_teleoperation
source install/setup.bash
```

### 2. 检查设备

```bash
lsusb | grep -i manus                              # MANUS Dongle
lsusb -v -d 0483:2000 2>/dev/null | grep iSerial   # Wuji Hand 序列号
```

### 3. 启动节点 (4 个终端)

```bash
# T1: MANUS 驱动 (C++ 节点)
ros2 run manus_ros2 manus_data_publisher

# T2: MANUS → MediaPipe 21点转换
ros2 run manus_input_py manus_input

# T3: Retarget 控制器 (100Hz)
ros2 run controller wujihand_controller

# T4: Wuji Hand 硬件驱动
ros2 run wujihand_driver wujihand_driver_node \
    --ros-args -p serial_number:="307B37583333" -r __ns:=/right_hand
```

**启动顺序**: T1 → T2 → T3 → T4 (等前一个无报错再启动下一个)

### 4. 验证

```bash
# MANUS 原始数据 (应该有 25 个 nodes)
ros2 topic echo /manus_glove_0 --qos-reliability best_effort --once

# MediaPipe 关键点 (63 个 float = 21点 x 3D)
ros2 topic echo /hand_input --once

# Retarget 输出 (20 个关节角)
ros2 topic echo /wuji_hand/right/joint_command --once

# Wuji Hand 反馈
ros2 topic echo /right_hand/joint_states --once

# 频率检查
ros2 topic hz /hand_input                        # 应该 ~120Hz
ros2 topic hz /wuji_hand/right/joint_command      # 应该 ~100Hz
```

## 数据流

```
MANUS Quantum Metaglove (120Hz, USB Dongle)
    |
    v
manus_data_publisher (C++)
    | /manus_glove_0 (ManusGlove, 25 nodes, 关节四元数)
    | 自动加载 RightQuantumMetaglove.mcal
    v
manus_input (Python)
    | node ID 映射: 25 Quantum nodes → 21 MediaPipe points (跳过 IP 关节)
    | /hand_input (Float32MultiArray, 63 values = 21 x 3D)
    v
wujihand_controller (Python, 100Hz)
    | wuji_retargeting.Retargeter (nlopt + pinocchio, 指尖位置 IK)
    | EMA 滤波 (alpha=0.3, 截止 ~5.7Hz)
    | /wuji_hand/right/joint_command (JointState, 20 joints)
    v
wujihand_driver (C++, 1000Hz)
    | wujihandcpp SDK → USB CDC
    | 固件: 1阶 IIR LP 10Hz @ 16kHz → PID (Kp=5, Ki=0.08, Kd=150) → 电机
    v
Wuji Hand 硬件
```

### Quantum Metaglove Node ID 映射

Quantum 有 25 个 node (每指 5 个含 IP 关节), MediaPipe 格式只有 21 个 (每指 4 个):

```
Quantum (25 nodes):
  0=Wrist
  1-4=Thumb (MCP, PIP, DIP, TIP)
  5-9=Index (MCP, PIP, IP, DIP, TIP)
  10-14=Middle, 15-19=Ring, 20-24=Pinky (同上)

MediaPipe (21 points):
  0=Wrist, 1-4=Thumb, 5-8=Index, 9-12=Middle, 13-16=Ring, 17-20=Pinky

映射 (跳过 IP 关节):
  MEDIAPIPE_TO_MANUS = (
      0,                    # WRIST
      1, 2, 3, 4,          # THUMB
      5, 6, 8, 9,          # INDEX: skip IP(7)
      10, 11, 13, 14,      # MIDDLE: skip IP(12)
      15, 16, 18, 19,      # RING: skip IP(17)
      20, 21, 23, 24,      # PINKY: skip IP(22)
  )
```

## 配置文件

| 配置 | 路径 | 说明 |
|------|------|------|
| MANUS 手套 ID | `src/input_devices/manus_input/manus_input_py/config/manus_input.yaml` | 手套序列号映射 |
| Retarget 参数 | `src/output_devices/wujihand_output/config/retarget_manus_right.yaml` | IK 权重、滤波、scaling |
| Wuji Hand 序列号 | 启动参数 `-p serial_number:="307B37583333"` | USB 序列号 |

### Wuji Hand 驱动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_number` | `""` | USB 设备序列号 |
| `publish_rate` | `1000.0` | 状态发布频率 (Hz) |
| `filter_cutoff_freq` | `10.0` | 固件侧 RT_POS LP 截止频率 (Hz, 覆写固件默认 3Hz) |
| `effort_limit` | `-1.0` | 电流限制 (A), <0 = 固件默认 (1.5A), 最大 3.5A |

## 常见问题

### MANUS 相关

| 问题 | 原因 | 解决 |
|------|------|------|
| `No compatible license found` | USB Dongle 权限不足 | 检查 udev 规则，拔插 Dongle |
| `glove family does not match` | 校准文件型号不对 (Pro vs Quantum) | 用 `manus_calibration` 重新校准 |
| 手套连接但无数据 | Dongle 未配对 | 手套关机重开，等 LED 变绿 |
| `ros2 topic echo` 收不到数据 | QoS 不匹配 | 加 `--qos-reliability best_effort` |
| Retarget 输出全卡在上限 | Node ID 映射错误 | 检查 MEDIAPIPE_TO_MANUS 常量 |

### Wuji Hand 相关

| 问题 | 原因 | 解决 |
|------|------|------|
| `No device found` / `ERROR_ACCESS` | USB 权限 / 未插线 | 检查 udev 规则，拔插手 |
| 序列号不对 | 换了一只手 | `lsusb -v -d 0483:2000 2>/dev/null \| grep iSerial` |
| 手指动但无法完全握拳 | retarget scaling 太小 | 调 `retarget_manus_right.yaml` 中 `segment_scaling` |
| 关节方向不自然 (有的内凹有的外拉) | IK 多解问题 | 已知问题，需要加关节耦合约束 |
| 邻指振荡 | 机械耦合 (固件确认预期行为) | 正常现象，无法消除 |

### Python 环境相关

| 问题 | 原因 | 解决 |
|------|------|------|
| `No module named 'wuji_retargeting'` | Shebang 指向系统 Python | 修复 shebang (见上方编译章节) |
| `No module named 'scipy'` | 同上 | 同上 |
| `No module named 'em'` | conda 缺 empy | `pip install empy==3.3.4` |
| `CMake 3.24 required` | cmake 版本太低 | `pip install cmake --upgrade` |
| `fatal error: format` | g++ 版本太低 | 安装 g++-13 |

### 快速诊断命令

```bash
# 1. 检查设备
lsusb | grep -i manus
lsusb -v -d 0483:2000 2>/dev/null | grep iSerial

# 2. 检查 shebang
head -1 install/controller/lib/controller/wujihand_controller
head -1 install/manus_input_py/lib/manus_input_py/manus_input

# 3. 检查频率
ros2 topic hz /hand_input
ros2 topic hz /wuji_hand/right/joint_command

# 4. 检查 node ID 映射
python3 -c "
import rclpy
from rclpy.qos import qos_profile_sensor_data
from manus_ros2_msgs.msg import ManusGlove
rclpy.init()
node = rclpy.create_node('debug')
def cb(msg):
    for n in msg.raw_nodes:
        print(f'  id={n.node_id:2d}  chain={n.chain_type:10s}  joint={n.joint_type}')
    rclpy.shutdown()
node.create_subscription(ManusGlove, '/manus_glove_0', cb, qos_profile_sensor_data)
rclpy.spin(node)
"
```

## 当前配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 手 | 右手 only | `include_right_hand: true, include_left_hand: false` |
| EMA 滤波 | `lp_alpha: 1.0` (关闭) | 调试模式，正式使用建议 0.3 |
| Wuji Hand 序列号 | `307B37583333` | 启动参数指定 |
| 固件滤波截止频率 | 10Hz | SDK SDO 0x05:19 覆写默认 3Hz |
