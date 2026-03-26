# Franka FR3 双臂 + PICO VR 遥操作指南

## 概述

通过 PICO VR 手柄控制虚拟 Franka FR3 双臂，在 RViz2 中实时可视化。使用 pink + pinocchio 做逆运动学，FCL mesh 碰撞检测 + 二分安全投影。

## 系统要求

- Ubuntu 22.04 LTS, ROS2 Humble
- Conda environment `ros2_teleoperation` (Python 3.10)
- PICO 4/4 Ultra 头显 + 手柄 (同一 WiFi)

### Python 依赖

```bash
conda activate ros2_teleoperation
pip install pin-pink         # pinocchio + pink IK
pip install quadprog         # QP solver
pip install scipy numpy
```

**注意**: `pin-pink` 只能装在 conda 环境里，系统 Python 的 pinocchio (ROS2 自带) 与 numpy 版本有冲突。

### 编译

```bash
cd ~/Documents/Preh/Teleoperation/wuji-hand-teleop

# 编译 franka 相关包
colcon build --packages-select franka_description franka_ik --symlink-install
source install/setup.bash

# 修复 shebang (一次性)
CONDA_PY="#!/home/yinan/miniconda3/envs/ros2_teleoperation/bin/python3"
sed -i "1s|.*|$CONDA_PY|" install/franka_ik/lib/franka_ik/franka_ik_node
sed -i "1s|.*|$CONDA_PY|" install/franka_ik/lib/franka_ik/franka_pico_input_node
```

## 架构

```
PICO 手柄 (WebXR, 144Hz)
    | WiFi (WebSocket/HTTPS)
    v
pico_controller_server.py (aiohttp)
    | /tmp/pico_data.json (文件 IO)
    v
franka_pico_input_node
    | 校准 + 坐标变换 + 相对旋转
    | /left_arm_target_pose  (PoseStamped)
    | /right_arm_target_pose (PoseStamped)
    v
franka_ik_node (pink + pinocchio, 100Hz)
    | IK → candidate q
    | FCL mesh 碰撞检测 (3ms)
    | 碰撞时 bisect 回退到安全位置
    | /joint_states (14 joints)
    v
robot_state_publisher → TF → RViz2
```

### 包结构

| 包 | 用途 |
|---|------|
| `franka_description` | FR3 双臂 URDF + collision meshes |
| `franka_ik` | IK 节点 + PICO 输入节点 + launch |
| `collision_manager` | FCL mesh 碰撞检测器 |

### URDF

`fr3_dual.urdf` — 双臂配置，世界原点居中:
- 左臂 base: Y = +0.3m
- 右臂 base: Y = -0.3m
- 间距: 600mm

重新生成: `python3 src/franka_description/robots/fr3/generate_dual_arm.py`

## 快速启动

### 1. 启动 PICO 服务器

```bash
cd ~/Documents/Preh/Teleoperation/wuji-hand-teleop/franka_teleop-master
python pico_teleop/pico_controller_server.py
```

服务器打印 `HTTPS 模式: https://192.168.x.x:8012`

### 2. PICO 连接

1. PICO 和电脑在同一 WiFi
2. PICO 浏览器打开 `https://<电脑IP>:8012`
3. 接受证书警告 → 页面显示 "WebSocket 已连接"
4. 点 **Enter VR**

验证连接:

```bash
cat /tmp/pico_data.json | grep connected
# 应该显示: "connected": true
```

### 3. 启动 Franka RViz

```bash
cd ~/Documents/Preh/Teleoperation/wuji-hand-teleop
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch franka_ik franka_pico_viz.launch.py data_source_type:=webxr
```

3 秒后自动校准，或按**两个 trigger 同时按**重新校准。

### Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `data_source_type` | `live` | `live` (Motion Tracker) / `recorded` (回放) / `webxr` (PICO 手柄) |
| `enable_input` | `true` | `false` 关闭 PICO 输入 (用于测试) |
| `playback_speed` | `1.0` | 录制回放速度倍率 |

## 坐标系

### PICO WebXR (实测)

```
X = forward (前)
Y = right (右)        ← 注意: 不是标准 WebXR 的 Y=up
Z = up (上)
```

### Franka World

```
X = forward (前)
Y = left (左)
Z = up (上)
```

### 变换矩阵

```python
# PICO → Franka (仅 Y 轴翻转)
R_pico2world = [[ 1,  0,  0],   # Franka_X = +PICO_X
                [ 0,  1,  0],   # Franka_Y = +PICO_Y
                [ 0,  0,  1]]   # Franka_Z = +PICO_Z
```

## 控制模式

### 位置: 绝对映射

```
target_pos = init_ee + scale * R @ (pico_current - pico_calib)
```

- 校准时记录 PICO 手柄位置
- 之后的位移 1:1 映射到机器人末端
- `scale` 参数可调节灵敏度 (默认 1.0)

### 旋转: 相对叠加

```
delta_rot = pico_current_rot @ pico_calib_rot⁻¹
target_rot = delta_rot @ home_ee_rot
```

- 校准时记录手柄朝向
- 之后的旋转变化叠加到 FR3 home 末端朝向上
- 避免了绝对朝向导致的 180° 翻转问题

### 按钮

| 按钮 | 功能 |
|------|------|
| 两个 trigger 同时按 | 重新校准 |
| grip (任一手) | 暂停/恢复 |

## IK 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `pos_cost` | 1.0 | 位置跟踪权重 |
| `ori_cost` | 0.5 | 朝向跟踪权重 |
| `posture_cost` | 0.1 | 姿态正则化 (防止 null space 抖动) |
| `max_vel` | 2.0 rad/s | 关节速度限制 (平滑输出) |
| `safety_margin` | 0.03m | FCL 碰撞安全裕度 |
| `bisect_iters` | 8 | 碰撞时二分搜索迭代次数 |
| `solver` | quadprog | QP 求解器 |

### 碰撞检测

- 使用 FCL mesh (STL 文件), 不是胶囊体
- 单次检测 ~3ms
- 同臂相邻 3 个 link 排除 (无需检测)
- 跨臂 link0/link1 排除 (固定不动)
- 碰撞时 binary search `pin.interpolate()` 找最近安全位置

## 测试

### 无 PICO 测试 (录制回放)

```bash
ROS_DOMAIN_ID=42 ros2 launch franka_ik franka_pico_viz.launch.py data_source_type:=recorded
```

### IK 单元测试

```bash
# 启动 (不启动 PICO 输入)
ROS_DOMAIN_ID=42 ros2 launch franka_ik franka_pico_viz.launch.py enable_input:=false

# 静态目标 (home 位置)
ROS_DOMAIN_ID=42 python3 src/franka_ik/test/test_ik_targets.py static

# 正弦运动 (右臂前后 ±10cm)
ROS_DOMAIN_ID=42 python3 src/franka_ik/test/test_ik_targets.py sine

# 圆周运动 (右臂 XZ 平面)
ROS_DOMAIN_ID=42 python3 src/franka_ik/test/test_ik_targets.py circle

# 逐轴运动 (5s 切换 X/Y/Z)
ROS_DOMAIN_ID=42 python3 src/franka_ik/test/test_ik_targets.py axes
```

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| `pinocchio` import 报 `_ARRAY_API not found` | numpy 版本与 pinocchio 编译版本不匹配 | `pip install "numpy<2"` 然后重启 |
| `No module named 'pink'` | shebang 指向系统 Python | 修复 shebang (见编译章节) |
| RViz 只显示 base, 关节没动 | `franka_ik_node` 崩了 | 检查终端报错, 通常是 pinocchio/numpy 问题 |
| PICO WebSocket 不连接 | 自签名证书 | 确保用 `https://`, 接受证书警告 |
| PICO 连接但 `connected: false` | 进入 VR 后 WebSocket 断开 | 重新打开页面, 确认 "WebSocket 已连接" 后再点 Enter VR |
| 手臂到限位/翻转 | `ori_cost` 过高 + 180° 歧义 | 使用相对旋转模式 (已默认) |
| 手臂抖动 | null space 自由度太多 | 增大 `posture_cost` 或增大 `ori_cost` |
| 方向反了 | 坐标变换矩阵不对 | 检查 `_R_PICO2WORLD`, 用 test axes 模式逐轴验证 |
| 多台机器 ROS2 冲突 | DDS 广播 | 使用 `ROS_DOMAIN_ID=42` 隔离 |

## 文件清单

| 文件 | 用途 |
|------|------|
| `src/franka_ik/franka_ik/franka_ik_node.py` | IK + 碰撞检测主节点 |
| `src/franka_ik/franka_ik/franka_pico_input_node.py` | PICO 输入 + 校准 + 坐标变换 |
| `src/franka_ik/franka_ik/webxr_data_source.py` | WebXR JSON 数据源 |
| `src/franka_ik/launch/franka_pico_viz.launch.py` | 主 launch 文件 |
| `src/franka_ik/test/test_ik_targets.py` | IK 测试脚本 |
| `src/franka_description/robots/fr3/fr3_dual.urdf` | 双臂 URDF |
| `src/franka_description/robots/fr3/generate_dual_arm.py` | URDF 生成脚本 |
| `franka_teleop-master/pico_teleop/pico_controller_server.py` | PICO WebXR 服务器 |
