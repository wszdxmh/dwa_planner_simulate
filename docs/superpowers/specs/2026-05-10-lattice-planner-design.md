# Lattice Planner — 设计规格说明书

## 背景

在当前 DWA 局部规划器仿真平台中新增 **Lattice Planner** 作为对比算法。两个规划器共用 A*/贝塞尔/预瞄等上下游模块，仅在局部规划环节切换，便于公平对比。

## 用户需求摘要

- 对比方式：同一场景分别运行（`main.m` 中选择 planner_type）
- 轨迹生成：五次多项式（quintic polynomial）
- 采样网格：中等密度（3层纵向 × 7条横向 × 3个朝向 ≈ 189条候选）

## 架构

```
                         ┌─ DWA (dwa_core.m) ────┐
A* → Bezier → PurePursuit ┤                        ├→ Control → Robot
                         └─ Lattice (lattice_core.m) ┘
```

两个规划器**共用**上游（A* / 贝塞尔平滑 / 纯追踪预瞄）和下游（运控 / 感知 / 可视化），只在局部规划这一步切换。

## 新增文件

### `planner/lattice_core.m`
Lattice 主逻辑：
1. 在机器人前方采样终点位姿网格（纵向 × 横向 × 朝向）
2. 对每个候选终点调用 `quintic_trajectory` 生成轨迹
3. 碰撞检测 + 5项代价评估（复用 `evaluate_cost` 的结构）
4. 在可行轨迹中选代价最小者，返回最优轨迹及对应控制指令

接口：
```
[best_v, best_w, best_traj, min_cost, all_trajs] = lattice_core(state, laser_pts, target_xy, p, global_path)
```
与 `dwa_core` 完全一致的输入输出签名，方便 `run_simulation` 中切换。

### `planner/quintic_trajectory.m`

五次多项式轨迹生成。给定起始/终止状态，求解 6×6 线性方程组得到 x(t) 和 y(t) 的系数，再离散化为轨迹点序列。

接口：
```
traj = quintic_trajectory(state, end_pose, T, n_samples)
% state     = [x, y, θ, v, ω]  起始状态
% end_pose  = [x, y, θ, v, ω]  终止状态
% T         = 轨迹时长 (s)
% n_samples = 离散点数
% traj      = [Nx3] (x, y, θ)
```

## 采样网格参数

```matlab
p.lattice.num_longitudinal = 3;     % 纵向采样层数
p.lattice.longitudinal_dists = [1.0, 2.0, 3.0];  % m
p.lattice.num_lateral = 7;          % 横向采样数
p.lattice.lateral_range = 0.9;      % 横向偏移 [-0.9, 0.9] m
p.lattice.num_headings = 3;         % 朝向采样数
p.lattice.heading_range = pi/6;     % 朝向偏移 ±30°
p.lattice.target_velocity = 0.3;    % 终点期望线速度 m/s
p.lattice.n_samples = 50;           % 每条轨迹离散点数
```

候选总数：3 × 7 × 3 = 63 个终止状态，每个生成 1 条轨迹 = 63 条候选（实际纵向层和横向偏移在 world frame 中组合）。

## 修改文件

| 文件 | 改动内容 |
|------|----------|
| `main.m` | 新增 `planner_type = 'lattice'` 选项，传入 `run_simulation` |
| `dwa_params.m` | 新增 `p.lattice.*` 参数组 |
| `run_simulation.m` | 接收 `planner_type` 参数，在规划步骤 `if strcmp(planner_type, 'lattice')` 分支调用 `lattice_core` |
| `param_sweep.m` | 扩展支持 Lattice 参数扫描 |
| `metrics/compute_metrics.m` | 将 `planner_type` 记入结果结构体 |
| `metrics/plot_summary.m` | 标题显示 planner_type，便于对比截图 |
| `visualization/animate_frame.m` | 轻量适配：Lattice 轨迹也有候选束 + 最优轨迹，渲染逻辑与 DWA 相同 |

## 五次多项式求解细节

对 x(t) 和 y(t) 分别求解 6 个系数：

```
x(t)  = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
x'(t) = a1 + 2a2·t + 3a3·t² + 4a4·t³ + 5a5·t⁴
x''(t)= 2a2 + 6a3·t + 12a4·t² + 20a5·t³

边界条件 (t=0):  x(0)=x0, x'(0)=v0·cos(θ0), x''(0)=0
边界条件 (t=T):  x(T)=xf, x'(T)=vf·cos(θf), x''(T)=0
```

代入得 6×6 线性系统，闭式矩阵求逆或 `A\b` 求解。y(t) 同理（y' 用 sin）。

轨迹时间 T = `dist(起点, 终点) / p.lattice.target_velocity`，钳位在 [1.0, 3.0]s。

## 控制指令提取

Lattice 不直接输出 (v, ω)，而是输出最优轨迹的第一个步的等效控制：

```
v_cmd = hypot(traj(2,1)-traj(1,1), traj(2,2)-traj(1,2)) / dt
w_cmd = (traj(2,3) - traj(1,3)) / dt
```

## 对比维度

仿真结束后输出：
- 路径长度、到达时间、成功率
- 路径平滑度、最小安全距离
- 平均/最大规划耗时
- 轨迹形状视觉对比（汇总图的轨迹子图）

## 不做什么

- 不修改 DWA 现有代码逻辑
- 不新增场景
- 不同屏实时对比（选的是分次运行对比）
- 不做 Frenet 坐标系的 Lattice（保持与 DWA 相同的 world frame 输入，公平对比）
