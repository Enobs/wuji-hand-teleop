# Setup Guide - MANUS → Wuji Hand Teleoperation

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- Conda environment `ros2_teleoperation` (Python 3.10)

## 1. Environment Setup

### 1.1 Conda Environment

```bash
conda create -n ros2_teleoperation python=3.10 -y
```

### 1.2 ~/.bashrc Configuration

**顺序很重要**：先 source ROS2，再 activate conda（conda 覆盖 PATH，确保 python3 指向 conda）

```bash
# ROS2 base (只 source 一次！)
source /opt/ros/humble/setup.bash

# Conda (放在 ROS2 之后)
conda activate ros2_teleoperation

# Workspace overlay
source ~/Documents/Preh/Teleoperation/wuji-hand-teleop/install/setup.bash 2>/dev/null
```

**常见坑**：如果 `source /opt/ros/humble/setup.bash` 出现两次（conda 前后各一次），会把系统 Python 路径放回前面，导致 shebang 问题。

### 1.3 Python Dependencies

```bash
pip install cmake --upgrade    # wujihandcpp 需要 cmake >= 3.24
pip install empy==3.3.4 lark catkin_pkg  # ROS2 消息编译需要 (empy 必须 3.x)
pip install scipy numpy        # retarget 依赖
pip install wujihandpy          # Wuji Hand Python SDK
```

### 1.4 System Dependencies

```bash
sudo apt install g++-13 libncursesw5-dev libusb-1.0-0-dev -y
# g++-13: wujihandcpp 需要 C++20 <format> 支持
# ncurses: MANUS SDK 依赖
# libusb: Wuji Hand USB 通讯
```

如果 g++-13 不可用：
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt update
sudo apt install g++-13 -y
```

## 2. USB Permissions (udev rules)

### MANUS Gloves (VID: 3325)
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="3325", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-manus.rules
```

### Wuji Hand (VID: 0483)
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"' | sudo tee /etc/udev/rules.d/95-wujihand.rules
```

### Apply rules
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**注意**：添加规则后需要**拔插设备**才会生效。

## 3. Build

### 3.1 wuji_retargeting (pip install, 不走 colcon)

```bash
cd ~/Documents/Preh/Teleoperation/wuji-hand-teleop/src/wuji_retargeting
pip install -e .
```

源码目录需要 `COLCON_IGNORE` 文件防止 colcon 扫描：
```bash
touch src/wuji_retargeting/COLCON_IGNORE
```

### 3.2 wujihandcpp (C++ SDK, 从源码编译)

```bash
cd ~/Documents/Preh/Teleoperation/wuji-hand-teleop/src/wujihandpy
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
make -j$(nproc)
```

Python binding 编译可能报错 — 没关系，只要 `build/wujihandcpp/libwujihandcpp.a` 生成了就行。

`wujihandpy` 源码目录也需要 COLCON_IGNORE：
```bash
touch src/wujihandpy/COLCON_IGNORE
```

### 3.3 ROS2 Packages (colcon)

```bash
cd ~/Documents/Preh/Teleoperation/wuji-hand-teleop
export WUJIHANDPY_SOURCE_DIR=$HOME/Documents/Preh/Teleoperation/wuji-hand-teleop/src/wujihandpy
colcon build --symlink-install --cmake-args -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
source install/setup.bash
```

## 4. Known Issue: Python Shebang

### 问题

colcon build 生成的 entry point 脚本 shebang 可能指向 `/usr/bin/python3`（系统 Python）而不是 conda Python。这导致 `ros2 run` 时找不到 conda 里装的包（如 `wuji_retargeting`、`scipy`）。

### 原因

`source /opt/ros/humble/setup.bash` 会修改 PATH，把 `/usr/bin` 放到 conda 前面。colcon build 时如果 `which python3` 返回系统 Python，shebang 就指向它。

### 解决方案

**方案 A：修复 ~/.bashrc 顺序**（推荐）

确保 conda activate 在 ROS2 source 之后，且 ROS2 只 source 一次。

**方案 B：手动修改 shebang**（快速修复）

```bash
# 修复 controller
sed -i '1s|.*|#!/home/yinan/miniconda3/envs/ros2_teleoperation/bin/python3|' \
    install/controller/lib/controller/wujihand_controller

# 修复 manus_input
sed -i '1s|.*|#!/home/yinan/miniconda3/envs/ros2_teleoperation/bin/python3|' \
    install/manus_input_py/lib/manus_input_py/manus_input
```

**注意**：每次 `colcon build` 后可能需要重新修复。

**方案 C：设置 PYTHONPATH**

如果 egg-link 找不到包：
```bash
export PYTHONPATH=$HOME/Documents/Preh/Teleoperation/wuji-hand-teleop/build/manus_input_py:$PYTHONPATH
```

### 验证

```bash
# 确认 python3 指向 conda
which python3
# 应该是: /home/yinan/miniconda3/envs/ros2_teleoperation/bin/python3

# 确认 shebang
head -1 install/controller/lib/controller/wujihand_controller
# 应该是: #!/home/yinan/miniconda3/envs/ros2_teleoperation/bin/python3
```

## 5. MANUS Glove Calibration

### 校准工具（Linux 原生，不需要 Windows MANUS Core）

```bash
# 确保没有其他 manus 节点在跑
ros2 run manus_ros2 manus_calibration
```

### 校准流程（3 步）

1. **手放平面** — 手放在水平非金属表面，手指合拢，拇指向外伸 → 摆好后按 Enter
2. **握拳** — 举起手，所有手指握成拳头，拇指靠侧面 → 摆好后按 Enter
3. **开合手指** — 用指尖反复触碰掌根 → 按 Enter 后持续做动作直到完成

### 校准文件

- 保存位置：`install/manus_ros2/share/manus_ros2/calibration/RightQuantumMetaglove.mcal`
- 源码目录备份：`src/input_devices/manus_input/manus_ros2/calibration/`
- `manus_data_publisher` 启动时自动按优先级加载：Quantum > Pro > generic

### 注意

- 项目自带的 `LeftMetaglovePro.mcal` / `RightMetaglovePro.mcal` 是给 MetaglovePro 的，Quantum Metaglove 不兼容（会报 "glove family does not match"）
- 不校准也能用，数据正常输出，只是精度差

## 6. MANUS Node ID Mapping

### Quantum Metaglove vs MetaglovePro

**两种手套的 node ID 布局不同！** 映射表在 `manus_input_node.py` 的 `MEDIAPIPE_TO_MANUS` 常量。

Quantum Metaglove 有 25 个 node（每指 5 个含 IP 关节），MediaPipe 格式只有 21 个（每指 4 个，无 IP）：

```
Quantum node layout (25 nodes):
  0=Hand(wrist)
  1-4=Thumb (MCP, PIP, DIP, TIP)
  5-9=Index (MCP, PIP, IP, DIP, TIP)
  10-14=Middle (MCP, PIP, IP, DIP, TIP)
  15-19=Ring (MCP, PIP, IP, DIP, TIP)
  20-24=Pinky (MCP, PIP, IP, DIP, TIP)

MediaPipe format (21 points):
  0=Wrist, 1-4=Thumb, 5-8=Index, 9-12=Middle, 13-16=Ring, 17-20=Pinky
```

转换时跳过 IP 关节：
```python
MEDIAPIPE_TO_MANUS = (
    0,                    # WRIST
    1, 2, 3, 4,          # THUMB: MCP, PIP, DIP, TIP
    5, 6, 8, 9,          # INDEX: skip IP(7)
    10, 11, 13, 14,      # MIDDLE: skip IP(12)
    15, 16, 18, 19,      # RING: skip IP(17)
    20, 21, 23, 24,      # PINKY: skip IP(22)
)
```

### 如何诊断映射问题

```bash
python3 -c "
import rclpy
from rclpy.qos import qos_profile_sensor_data
from manus_ros2_msgs.msg import ManusGlove
rclpy.init()
node = rclpy.create_node('debug_nodes')
def cb(msg):
    for n in msg.raw_nodes:
        print(f'  id={n.node_id:2d}  chain={n.chain_type:10s}  joint={n.joint_type}')
    rclpy.shutdown()
node.create_subscription(ManusGlove, '/manus_glove_0', cb, qos_profile_sensor_data)
rclpy.spin(node)
"
```

## 7. Running the System

### 查看设备

```bash
# MANUS 手套 + dongle
lsusb | grep -i manus

# Wuji Hand
lsusb -v -d 0483:2000 2>/dev/null | grep iSerial
```

### 启动顺序（4 个终端）

```bash
# T1: MANUS 驱动 (C++ 节点, 不受 Python shebang 影响)
ros2 run manus_ros2 manus_data_publisher

# T2: MANUS → MediaPipe 转换
ros2 run manus_input_py manus_input

# T3: Controller (retarget → joint commands)
ros2 run controller wujihand_controller

# T4: Wuji Hand 硬件驱动
ros2 run wujihand_driver wujihand_driver_node \
    --ros-args -p serial_number:="307B37583333" -r __ns:=/right_hand
```

### 验证

```bash
# 看 MANUS 原始数据
ros2 topic echo /manus_glove_0 --qos-reliability best_effort --once

# 看 MediaPipe 关键点
ros2 topic echo /hand_input --once

# 看 retarget 输出的关节角度
ros2 topic echo /wuji_hand/right/joint_command --once

# 看 Wuji Hand 反馈
ros2 topic echo /right_hand/joint_states --once
```

### 当前配置

| Config | Value | File |
|--------|-------|------|
| 右手手套 only | `include_right_hand: true, include_left_hand: false` | `manus_input.yaml` |
| EMA 滤波关闭 | `lp_alpha: 1.0` (debug) | `retarget_manus_right.yaml` |
| Wuji Hand 序列号 | `307B37583333` | 启动参数 |

## 8. Data Flow

```
MANUS Quantum Metaglove (120Hz, USB)
    ↓
manus_data_publisher (C++, Integrated SDK)
    ↓ /manus_glove_0 (ManusGlove msg, 25 nodes)
    ↓ + 自动加载 RightQuantumMetaglove.mcal
    ↓
manus_input (Python)
    ↓ node ID 映射 (25 nodes → 21 MediaPipe points, skip IP joints)
    ↓ /hand_input (Float32MultiArray, 63 values = 21 × 3D)
    ↓
wujihand_controller (Python, 100Hz timer)
    ↓ wuji_retargeting.Retargeter (nlopt + pinocchio)
    ↓ /wuji_hand/right/joint_command (JointState, 20 joints)
    ↓
wujihand_driver (C++, 1000Hz)
    ↓ wujihandcpp SDK → USB CDC
    ↓
Wuji Hand Hardware (firmware PID @ 16kHz)
```

## 9. Troubleshooting

| 问题 | 原因 | 解决 |
|------|------|------|
| `No module named 'wuji_retargeting'` | shebang 指向系统 Python | 修复 shebang (见 Section 4) |
| `PackageNotFoundError: manus-input-py` | shebang + egg-link 问题 | 修复 shebang + 设 PYTHONPATH |
| `No compatible license found` | MANUS dongle USB 权限 | 检查 udev 规则，拔插 dongle |
| `glove family does not match` | 校准文件型号不对 | 用 `manus_calibration` 重新校准 |
| `ERROR_ACCESS` (Wuji Hand) | USB 权限 | 检查 udev 规则，拔插手 |
| `No module named 'em'` | conda 缺 empy | `pip install empy==3.3.4` |
| `CMake 3.24 required` | cmake 版本太低 | `pip install cmake --upgrade` |
| `fatal error: format` | g++ 版本太低 | 安装 g++-13 |
| retarget 输出全卡在上限 | node ID 映射错误 | 检查 MEDIAPIPE_TO_MANUS 常量 |
| ros2 topic echo 收不到数据 | QoS 不匹配 | 加 `--qos-reliability best_effort` |
