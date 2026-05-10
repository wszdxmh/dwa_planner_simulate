# DWA 局部规划器仿真平台

基于 MATLAB 的 DWA (Dynamic Window Approach) 与 Lattice 局部路径规划仿真平台，集成 A\* 全局路径规划 + 贝塞尔平滑 + 纯追踪预瞄 + 双局部规划器对比，支持静态/动态障碍物、多场景测试和参数敏感度分析。

## 快速开始

```matlab
% 1. 打开 main.m, 选择场景和规划器
scene_name = 'cluttered';  % 可选: empty | simple | cluttered | corridor | dynamic
planner_type = 'dwa';      % 可选: 'dwa' | 'lattice'

% 2. 运行
main
```

仿真窗口会实时显示机器人运动、激光雷达点云、规划器预测轨迹束和最优轨迹。关闭窗口或到达目标后自动输出性能指标和汇总图表。

## 架构

```
                         ┌─ DWA (速度空间采样) ────┐
地图 → A* → waypoints → 贝塞尔 → 纯追踪预瞄 ┤                        ├→ 差速轮控制
                         └─ Lattice (位姿空间采样) ┘
                                      ↑
                                激光雷达感知
```

- **A\***: 8邻域搜索，障碍物膨胀代价图，欧氏距离启发函数
- **贝塞尔平滑**: 三次贝塞尔曲线插值，使全局路径平滑可跟踪
- **纯追踪预瞄**: 在平滑路径上寻找机器人前方 `lookahead_dist` 处的点作为局部目标
- **DWA**: 动态速度窗口内采样 (v, ω)，圆弧前向模拟轨迹，5项代价函数选最优 — 紧贴路径、保守安全
- **Lattice**: 空间格点采样终点位姿 (x, y, θ)，五次多项式生成光滑轨迹 — 速度快、绕行灵活

## 目录结构

```
dwa_planner_simulate/
├── main.m                        入口脚本
├── dwa_params.m                  参数集中定义
├── setup_environment.m           地图 + 障碍物场景生成
│
├── planner/
│   ├── astar_planner.m           A* 全局路径规划
│   ├── dwa_core.m                DWA 局部规划核心
│   ├── lattice_core.m            Lattice 局部规划核心
│   ├── quintic_trajectory.m      五次多项式轨迹生成
│   ├── evaluate_cost.m           5项代价评估 (两规划器共用)
│   ├── generate_trajectory.m     轨迹前向模拟 (DWA)
│   └── compute_dynamic_window.m  动态速度窗口 (DWA)
│
├── robot/
│   ├── robot_kinematics.m        差速轮运动学
│   └── sensor_raycast.m          DDA 射线投射激光雷达
│
├── simulation/
│   ├── run_simulation.m          主仿真循环
│   └── step_dynamic_obstacles.m  动态障碍物更新
│
├── visualization/
│   └── animate_frame.m           逐帧实时渲染 (含地图/激光/轨迹/机器人)
│
├── metrics/
│   ├── compute_metrics.m         性能指标计算
│   └── plot_summary.m            仿真后汇总图表
│
├── tuning/
│   └── param_sweep.m             参数批量扫描 + 敏感度分析
│
└── utils/
    ├── bezier_smooth.m           贝塞尔曲线平滑
    └── normalize_angle.m         角度归一化 [-π, π]
```

## 场景

| 场景 | 说明 | 特点 |
|------|------|------|
| `empty` | 空地 | 无静态障碍物，验证基本跟踪 |
| `simple` | 简单 | 3个矩形障碍物 |
| `cluttered` | 密集 | 8个矩形障碍物，考验避障 |
| `corridor` | 走廊 | 狭窄通道，考验通过性 |
| `dynamic` | 动态 | 静态障碍物 + 1个移动障碍物 |

在 `main.m` 中通过 `scene_name` 变量切换。

## 关键参数

所有参数集中在 `dwa_params.m` 中定义：

### 机器人

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot.max_velocity` | 0.4 m/s | 最大线速度 |
| `robot.min_velocity` | 0.05 m/s | 最小线速度 |
| `robot.max_omega` | 2.0 rad/s | 最大角速度 |
| `robot.radius` | 0.175 m | 机器人半径 |
| `robot.wheel_separation` | 0.30 m | 差速轮轮距 |

### DWA 规划

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `dwa.sim_time` | 2.0 s | 前向模拟时长 |
| `dwa.dt` | 0.2 s | 模拟步长 |
| `dwa.velocity_res` | 0.05 m/s | 速度采样分辨率 |
| `dwa.omega_res` | 0.05 rad/s | 角速度采样分辨率 |

### 代价权重

| 权重 | 默认值 | 说明 |
|------|--------|------|
| `costs.to_goal` | 10.0 | 目标距离代价 |
| `costs.heading` | 25.0 | 航向对齐代价 |
| `costs.speed` | 2.5 | 速度偏好代价 |
| `costs.obstacle` | 5.5 | 障碍物安全代价 |
| `costs.path` | 0.4 | 路径偏离代价 |

代价函数: `J = w₁·dist_to_goal + w₂·heading + w₃·speed + w₄·obstacle + w₅·path`

### 障碍物与规划

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `planner.lookahead_dist` | 0.5 m | 纯追踪预瞄距离 |
| `planner.obstacle_inflation` | 0.35 m | 障碍物膨胀半径 |
| `planner.waypoint_tolerance` | 0.5 m | waypoint 切换阈值 |
| `sim.goal_tolerance` | 0.3 m | 到达目标判定距离 |

### Lattice 规划

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `lattice.longitudinal_dists` | [0.8, 1.5, 2.0] m | 纵向采样距离 |
| `lattice.num_lateral` | 7 | 横向采样数 |
| `lattice.lateral_range` | 0.9 m | 横向偏移范围 ±0.9m |
| `lattice.num_headings` | 3 | 终点朝向采样数 |
| `lattice.heading_range` | π/6 | 朝向偏移范围 ±30° |
| `lattice.n_samples` | 25 | 每条轨迹离散点数 |
| `lattice.path_weight` | 5.0 | 路径贴合权重 |

### 障碍物代价

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `obstacle.safe_distance` | 0.5 m | 安全距离，超过后代价为 0 |
| `obstacle.decay_factor` | 5.0 | 指数衰减系数（越大越陡） |

## 可视化图例

| 符号 | 含义 |
|------|------|
| 灰色虚线 | A\* 原始路径点 |
| 青色实线 | 贝塞尔平滑路径 |
| 绿色菱形 ◇ | 纯追踪预瞄点 (局部目标) |
| 红色点 | 激光雷达点云 |
| 浅绿轨迹 | 全部预测轨迹束 (DWA圆弧 / Lattice多项式曲线) |
| 品红轨迹 | 最优轨迹 |
| 绿色圆 + 黑线 | 机器人 (朝向指示线) |
| 绿色实线 | 机器人历史轨迹 |
| 红色星号 \* | 全局目标点 |

## 性能指标

仿真结束后自动输出:
- 是否成功到达目标
- 最终距离目标误差
- 仿真帧数与总时间
- 平均 DWA 规划耗时
- 机器人行驶路径总长度

`metrics/compute_metrics.m` 额外提供: 路径平滑度、最小安全距离、速度统计等。

## 参数扫描

```matlab
tuning/param_sweep.m
```

支持 DWA 代价权重和 Lattice 采样参数批量扫描，生成敏感度分析图。通过 `planner_type` 切换扫描对象。

## 兼容性

- MATLAB R2020b 及以上
- 无需额外工具箱 (仅使用基础 MATLAB 功能)

## 参考

- Fox, D., Burgard, W., & Thrun, S. (1997). The Dynamic Window Approach to Collision Avoidance. *IEEE Robotics & Automation Magazine*.
- 原始 DWA 算法文件 `dwa_planner.m` 基于 amslabtech/dwa_planner (ROS C++ 实现) 的代价函数设计

## License

MIT
