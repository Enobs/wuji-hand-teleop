# Franka + Wuji Hand 双臂遥操作方案

## 硬件配置

| 设备 | 用途 |
|------|------|
| Franka 双臂 | 机械臂执行 |
| Wuji Hand 双手 | 灵巧手执行 |
| MANUS 手套 | 手指输入 |
| PICO VR | 手臂追踪输入 |
| Foot Paddle | 安全开关 |

## 整体架构

```
PICO (连续追踪)
  ↓ 绝对映射 + offset
Franka 双臂 IK (独立解)  →  碰撞检测 → 安全投影 → 执行

MANUS (连续追踪)
  ↓ retarget (已有，直接复用)
Wuji Hand 双手 → 执行
```

## 可复用模块（来自 wuji-hand-teleop）

| 模块 | 说明 |
|------|------|
| `manus_input` | MANUS 手套驱动 → `/hand_input` |
| `pico_input` | PICO VR 追踪 → TF wrist 帧 |
| `wujihand_output` | Wuji Hand IK 重定向 |
| `controller/wujihand_node` | 灵巧手控制器 |
| `wuji_retargeting` (外部包) | 手指运动重定向算法 |

## 需要新建的模块

| 模块 | 说明 |
|------|------|
| `franka_output` | Franka IK + 控制，订阅 TF wrist → MoveIt2 IK → libfranka |
| launch 文件 | 替换 tianji_output 为 franka_output |

## 映射方案：绝对映射 + 缩放

不使用增量 + clutch，因为需要采集连续精细操作数据用于模仿学习。

### 标定

```python
# 一次性标定：人站好（自然站立，手在胸前），机械臂在工作空间中心
T_calibration = robot_pose × human_pose⁻¹
```

- 标定姿势选自然舒适的，不用伸直手臂
- 机械臂初始姿态选关节中位附近，远离奇异点，各方向余量最大
- 标定结果写入配置文件，不需要每次重新标定

### 运行时映射

```python
robot_target = T_calibration × scale × human_current_pose
```

| 参数 | 建议值 | 说明 |
|------|--------|------|
| scale | 0.3~0.5 | 人手 30cm = 机械臂 10~15cm，放大精度 |
| 控制频率 | ≥100Hz | 精细动作需要高频 |
| 滤波 | 轻度低通 2~5Hz | 去手抖，不能太重 |
| 录制频率 | 跟控制频率一致 | 保证时间关系完整 |

## Foot Paddle 安全机制

```python
if paddle_off:
    robot_last_stop = robot_current       # 记住停的位置
    # 机械臂保持不动，不录制
elif paddle_just_pressed:
    offset = robot_last_stop - absolute_map(human_current)  # 消除跳变
    # 之后: robot_target = absolute_map(human) + offset
else:
    robot_target = absolute_map(human_current) + offset
    record(robot_target, timestamp)       # 录制有效数据
```

- 松开 paddle → 机械臂停住，人可以休息/调整
- 重踩 paddle → 计算 offset 衔接，无跳变
- 训练数据按 paddle 状态切割：踩住=有效段，松开=丢弃

## 碰撞检测与安全投影

### 架构：虚拟目标 + 最近安全投影

```python
# 虚拟目标始终跟踪人手（不管能不能执行）
virtual_target = absolute_map(human_current) + offset

if not collision(virtual_target) and reachable(virtual_target):
    robot_target = virtual_target                              # 正常跟随
else:
    robot_target = project_to_nearest_safe(virtual_target)     # 投影到最近安全点
```

### 安全投影数学表达

```
robot_target = argmin ‖x - virtual_target‖
               s.t.  collision_free(x)
                     reachable(x)
```

### 实现策略：简单几何约束为主 + FCL 兜底

1. **快速几何约束**（<0.01ms）：双臂最小间距、关节限位、工作空间边界
2. **FCL 距离查询**（~0.5ms）：精确碰撞检测 + 最近安全点

### 碰撞几何体：胶囊体包络

不用 URDF 精细 mesh，用简化胶囊体：

```
精细 mesh (渲染用):    每个 link 500 三角面  → 慢
简化包络 (碰撞用):    每个 link 一个胶囊体  → 快 100 倍
```

双臂双手约 50+ 个胶囊体，BVH 加速后整体 <1ms @ 100Hz。

### FCL 性能参考

| 操作 | 耗时 | 说明 |
|------|------|------|
| 碰撞检测（是/否） | 0.01~0.1ms | 最快 |
| 距离查询（最近点） | 0.1~1ms | 返回距离+最近点 |
| 连续碰撞检测 CCD | 1~5ms | 轨迹碰撞 |

### 碰撞恢复行为

- 人手移回安全区域时，虚拟目标自然回到安全区域，机械臂直接跟上，无跳变
- 碰撞区内横向移动时，机械臂沿碰撞边界滑动，不冻住

## 三种状态统一处理

```python
virtual_target = absolute_map(human) + offset

if paddle_off:
    freeze, record offset on resume
elif collision(virtual_target):
    robot = project_to_nearest_safe(virtual_target)   # 贴边滑
else:
    robot = virtual_target                             # 正常跟
```

## 依赖

| 依赖 | 用途 |
|------|------|
| franka_ros2 | Franka 官方 ROS2 驱动 (libfranka) |
| MoveIt2 | IK 求解 + 运动规划 |
| FCL (python-fcl) | 碰撞检测 |
| wuji_retargeting | 手指重定向（已有） |
| wujihandcpp / wujihandros2 | Wuji Hand 驱动（已有） |
