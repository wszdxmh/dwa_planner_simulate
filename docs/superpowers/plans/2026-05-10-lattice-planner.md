# Lattice Planner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Lattice local planner alongside the existing DWA planner, with planner_type switching in main.m for comparison.

**Architecture:** Two new planner files (`quintic_trajectory.m`, `lattice_core.m`) plug into the existing pipeline at the same point as DWA. `run_simulation.m` branches on `planner_type`. Existing visualization, metrics, and tuning modules adapt with minimal changes — all share the same trajectory rendering and cost evaluation infrastructure.

**Tech Stack:** MATLAB (R2020b+), no additional toolboxes

---

### Task 1: Create quintic polynomial trajectory generator

**Files:**
- Create: `planner/quintic_trajectory.m`

- [ ] **Step 1: Write `quintic_trajectory.m`**

```matlab
function traj = quintic_trajectory(state, end_pose, T, n_samples)
% Generate a quintic polynomial trajectory between start and end states.
% Solves 3x3 linear system for x(t) and y(t) coefficients with boundary
% conditions: position, velocity, and zero acceleration at both ends.
%   state     = [x, y, theta, v, omega]  start state
%   end_pose  = [x, y, theta, v, omega]  end state
%   T         = trajectory duration (s)
%   n_samples = number of discrete points
% Returns:
%   traj = [Nx3] (x, y, theta) trajectory points

    x0 = state(1); y0 = state(2); th0 = state(3);
    v0 = state(4);
    xf = end_pose(1); yf = end_pose(2); thf = end_pose(3);
    vf = end_pose(4);

    % --- x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 ---
    a0 = x0;
    a1 = v0 * cos(th0);
    a2 = 0;

    A = [T^3,   T^4,    T^5;
         3*T^2, 4*T^3,  5*T^4;
         6*T,   12*T^2, 20*T^3];

    bx = [xf - a0 - a1*T;
          vf*cos(thf) - a1;
          0];
    ax = A \ bx;

    % --- y(t) = b0 + b1*t + b2*t^2 + b3*t^3 + b4*t^4 + b5*t^5 ---
    b0 = y0;
    b1 = v0 * sin(th0);
    b2 = 0;

    by = [yf - b0 - b1*T;
          vf*sin(thf) - b1;
          0];
    ay = A \ by;

    % --- Discretize ---
    traj = zeros(n_samples, 3);
    dt = T / n_samples;
    for i = 1:n_samples
        t = i * dt;
        t2 = t * t;
        t3 = t2 * t;
        t4 = t3 * t;
        t5 = t4 * t;

        x_t  = a0 + a1*t + a2*t2 + ax(1)*t3 + ax(2)*t4 + ax(3)*t5;
        y_t  = b0 + b1*t + b2*t2 + ay(1)*t3 + ay(2)*t4 + ay(3)*t5;

        xp_t = a1 + 2*a2*t + 3*ax(1)*t2 + 4*ax(2)*t3 + 5*ax(3)*t4;
        yp_t = b1 + 2*b2*t + 3*ay(1)*t2 + 4*ay(2)*t3 + 5*ay(3)*t4;

        traj(i, :) = [x_t, y_t, atan2(yp_t, xp_t)];
    end
end
```

- [ ] **Step 2: Quick smoke test in MATLAB**

```matlab
% In MATLAB console from project root:
addpath(genpath(pwd));
state = [1, 1, 0, 0.3, 0];
end_pose = [2, 1.5, pi/6, 0.3, 0];
traj = quintic_trajectory(state, end_pose, 2.0, 50);
% Verify: traj(1,:) near [1,1,0], traj(end,:) near end_pose
plot(traj(:,1), traj(:,2), 'b.-'); axis equal; grid on;
```

- [ ] **Step 3: Commit**

```bash
git add planner/quintic_trajectory.m
git commit -m "feat: add quintic polynomial trajectory generator for Lattice planner"
```

---

### Task 2: Create Lattice planner core

**Files:**
- Create: `planner/lattice_core.m`

- [ ] **Step 1: Write `lattice_core.m`**

```matlab
function [best_v, best_w, best_traj, min_cost, all_trajs] = lattice_core(state, laser_pts, target_xy, p, global_path)
% Lattice planner: sample end states on a spatial grid, generate quintic
% polynomial trajectories, evaluate costs, return best.
% Same interface as dwa_core for drop-in switching.
%   state       = [x, y, theta, v, omega]
%   laser_pts   = [Mx2] obstacle points in world frame
%   target_xy   = [x, y] local target (lookahead point)
%   p           = parameter struct (includes p.lattice.*)
%   global_path = [Nx2] smoothed global path

    if nargin < 5, global_path = []; end

    robot_pose = state(1:3);

    % Build sampling grid in robot frame
    long_dists = p.lattice.longitudinal_dists;
    lat_offsets = linspace(-p.lattice.lateral_range, p.lattice.lateral_range, p.lattice.num_lateral);
    heading_offsets = linspace(-p.lattice.heading_range, p.lattice.heading_range, p.lattice.num_headings);

    n_candidates = length(long_dists) * length(lat_offsets) * length(heading_offsets);
    all_trajs = cell(n_candidates, 1);
    all_vw = zeros(n_candidates, 2);
    all_weighted_costs = inf(n_candidates, 1);

    idx = 0;
    for li = 1:length(long_dists)
        long_d = long_dists(li);
        for lai = 1:length(lat_offsets)
            lat_d = lat_offsets(lai);
            for hi = 1:length(heading_offsets)
                h_off = heading_offsets(hi);
                idx = idx + 1;

                th = robot_pose(3);
                end_x = robot_pose(1) + long_d * cos(th) - lat_d * sin(th);
                end_y = robot_pose(2) + long_d * sin(th) + lat_d * cos(th);
                end_th = normalize_angle(th + h_off);
                end_pose = [end_x, end_y, end_th, p.lattice.target_velocity, 0];

                dist_to_end = hypot(end_x - robot_pose(1), end_y - robot_pose(2));
                T = max(1.0, min(3.0, dist_to_end / max(p.lattice.target_velocity, 0.1)));

                traj = quintic_trajectory(state, end_pose, T, p.lattice.n_samples);
                all_trajs{idx} = traj;

                dt = T / p.lattice.n_samples;
                v_eq = hypot(traj(2,1)-traj(1,1), traj(2,2)-traj(1,2)) / dt;
                w_eq = (traj(2,3) - traj(1,3)) / dt;
                all_vw(idx, :) = [v_eq, w_eq];

                [costs, infeasible] = evaluate_cost(traj, v_eq, laser_pts, target_xy, robot_pose, p, global_path);
                if ~infeasible
                    all_weighted_costs(idx) = p.costs.to_goal * costs.to_goal ...
                                            + p.costs.heading * costs.heading ...
                                            + p.costs.speed   * costs.speed ...
                                            + p.costs.obstacle * costs.obstacle ...
                                            + p.costs.path    * costs.path;
                end
            end
        end
    end

    [min_cost, best_idx] = min(all_weighted_costs);
    if isinf(min_cost)
        best_v = 0; best_w = 0; best_traj = [];
    else
        best_v = all_vw(best_idx, 1);
        best_w = all_vw(best_idx, 2);
        best_traj = all_trajs{best_idx};
    end
end
```

- [ ] **Step 2: Quick smoke test in MATLAB**

```matlab
% From project root:
addpath(genpath(pwd));
p = dwa_params();
state = [5, 5, 0, 0.3, 0];
laser_pts = [5.5, 5.5; 6, 5];
target_xy = [8, 8];
[best_v, best_w, best_traj, min_cost, all_trajs] = lattice_core(state, laser_pts, target_xy, p, []);
fprintf('Best: v=%.2f, w=%.2f, cost=%.2f, n_trajs=%d\n', best_v, best_w, min_cost, length(all_trajs));
```

- [ ] **Step 3: Commit**

```bash
git add planner/lattice_core.m
git commit -m "feat: add Lattice planner core with spatial grid sampling"
```

---

### Task 3: Add Lattice parameters to dwa_params.m

**Files:**
- Modify: `dwa_params.m`

- [ ] **Step 1: Add lattice parameter section**

Insert before the final `end` line in `dwa_params.m`, after the existing `p.sim.traj_display_stride` line:

```matlab
    %% Lattice规划参数
    p.lattice.num_longitudinal = 3;
    p.lattice.longitudinal_dists = [1.0, 2.0, 3.0];
    p.lattice.num_lateral = 7;
    p.lattice.lateral_range = 0.9;
    p.lattice.num_headings = 3;
    p.lattice.heading_range = pi/6;
    p.lattice.target_velocity = 0.30;
    p.lattice.n_samples = 50;
```

- [ ] **Step 2: Commit**

```bash
git add dwa_params.m
git commit -m "feat: add Lattice planner parameter group to dwa_params"
```

---

### Task 4: Add planner_type switching to main.m and run_simulation.m

**Files:**
- Modify: `main.m`
- Modify: `simulation/run_simulation.m`

- [ ] **Step 1: Update `main.m` — add planner_type selection**

Change the section after `scene_name` to include `planner_type`:

```matlab
% 选择场景: 'empty' | 'simple' | 'cluttered' | 'corridor' | 'dynamic'
scene_name = 'cluttered';

% 选择规划器: 'dwa' | 'lattice'
planner_type = 'dwa';
```

And update the `run_simulation` call:

```matlab
result = run_simulation(p, env, do_vis, planner_type);
```

And update the status print after simulation:

```matlab
fprintf('=== 仿真结果 [%s] ===\n', planner_type);
```

- [ ] **Step 2: Update `run_simulation.m` — accept planner_type and branch**

Change function signature (line 1):

```matlab
function result = run_simulation(p, env, do_visualize, planner_type)
```

Add default value handling right after the function signature, before `addpath`:

```matlab
    if nargin < 4
        planner_type = 'dwa';
    end
```

Find the DWA planning call in the main loop (the block with `dwa_core`). Replace:

```matlab
        [v_cmd, w_cmd, best_traj, min_cost, all_trajs] = dwa_core(state, laser_pts, lookahead_pt, p, smoothed_path);
```

With:

```matlab
        if strcmp(planner_type, 'lattice')
            [v_cmd, w_cmd, best_traj, min_cost, all_trajs] = lattice_core(state, laser_pts, lookahead_pt, p, smoothed_path);
        else
            [v_cmd, w_cmd, best_traj, min_cost, all_trajs] = dwa_core(state, laser_pts, lookahead_pt, p, smoothed_path);
        end
```

- [ ] **Step 3: Smoke test both planners**

```matlab
% Run DWA (existing behavior should still work):
main  % with planner_type = 'dwa'

% Run Lattice:
% Change planner_type = 'lattice' in main.m, then:
main
```

- [ ] **Step 4: Commit**

```bash
git add main.m simulation/run_simulation.m
git commit -m "feat: add planner_type switching (dwa/lattice) to main and simulation loop"
```

---

### Task 5: Adapt visualization to show planner type

**Files:**
- Modify: `visualization/animate_frame.m`

- [ ] **Step 1: Add planner_type parameter to animate_frame**

Change function signature (line 1):

```matlab
function keep_running = animate_frame(p, env, state, laser_pts, all_trajs, best_traj, ...
        lookahead_pt, waypoints, smoothed_path, wp_idx, frame_num, planner_type)
```

Add default:

```matlab
    if nargin < 12, planner_type = 'dwa'; end
```

Update the title line to include planner type. Find the line:

```matlab
    title_str = sprintf('Frame: %d | v=%.2fm/s  w=%.2frad/s | 预瞄→绿◇ | V_R=%.2f V_L=%.2f', ...
```

Replace with:

```matlab
    title_str = sprintf('[%s] Frame: %d | v=%.2fm/s  w=%.2frad/s | 预瞄→绿◇ | V_R=%.2f V_L=%.2f', ...
        upper(planner_type), ...
```

- [ ] **Step 2: Update run_simulation.m call to animate_frame**

In `run_simulation.m`, find the `animate_frame` call and add `planner_type` as the last argument:

```matlab
            keep_running = animate_frame(p, env, state, laser_pts, all_trajs, best_traj, ...
                lookahead_pt, waypoints, smoothed_path, current_wp_idx, t, planner_type);
```

- [ ] **Step 3: Commit**

```bash
git add visualization/animate_frame.m simulation/run_simulation.m
git commit -m "feat: show planner type in visualization window title"
```

---

### Task 6: Record planner_type in metrics

**Files:**
- Modify: `metrics/compute_metrics.m`
- Modify: `metrics/plot_summary.m`

- [ ] **Step 1: Add planner_type field to compute_metrics.m**

After the line `m = struct();` add:

```matlab
    if isfield(result, 'planner_type')
        m.planner_type = result.planner_type;
    else
        m.planner_type = 'dwa';
    end
```

- [ ] **Step 2: Store planner_type in run_simulation.m result**

In `run_simulation.m`, in the `pack_result` subfunction, after setting `result.success`:

```matlab
    result.planner_type = planner_type;
```

- [ ] **Step 3: Update plot_summary.m title**

Change the `sgtitle` line from:

```matlab
    sgtitle(result.env.name);
```

To:

```matlab
    if isfield(result, 'planner_type')
        sgtitle(sprintf('%s [%s]', result.env.name, upper(result.planner_type)));
    else
        sgtitle(result.env.name);
    end
```

- [ ] **Step 4: Commit**

```bash
git add metrics/compute_metrics.m metrics/plot_summary.m simulation/run_simulation.m
git commit -m "feat: record and display planner_type in metrics and summary plots"
```

---

### Task 7: Extend param_sweep to support Lattice

**Files:**
- Modify: `tuning/param_sweep.m`

- [ ] **Step 1: Add planner_type and lattice sweep parameters**

After the line `scene_name = 'simple';` add:

```matlab
planner_type = 'dwa';  % 'dwa' | 'lattice'
```

After the sweep_params definition, add lattice-specific sweep params:

```matlab
lattice_sweep_params = {
    'longitudinal', [1.0, 2.0, 3.0];
    'lateral_range', [0.3, 0.6, 0.9, 1.2];
    'heading_range', [pi/12, pi/6, pi/4];
    'target_vel', [0.1, 0.2, 0.3, 0.4];
};
```

Update the `run_simulation` call to pass `planner_type`:

```matlab
        result = run_simulation(p, env, do_vis, planner_type);
```

- [ ] **Step 2: Add lattice parameter sweep logic**

After the existing DWA sweep loop, add a conditional lattice sweep section:

```matlab
if strcmp(planner_type, 'lattice')
    param_names = {'lattice.longitudinal', 'lattice.lateral_range', 'lattice.heading_range', 'lattice.target_vel'};
    sweep_params = lattice_sweep_params;
    param_labels = {'longitudinal', 'lateral\_range', 'heading\_range', 'target\_vel'};
else
    param_names = {'costs.to_goal', 'costs.speed', 'costs.obstacle', 'costs.heading'};
    param_labels = {'to\_goal', 'speed', 'obstacle', 'heading'};
end
```

And update the parameter-setting switch to handle lattice params:

```matlab
        if strcmp(planner_type, 'lattice')
            switch pi
                case 1, p.lattice.longitudinal_dists = [vals(vi), vals(vi)*2, vals(vi)*3];
                case 2, p.lattice.lateral_range = vals(vi);
                case 3, p.lattice.heading_range = vals(vi);
                case 4, p.lattice.target_velocity = vals(vi);
            end
        else
            switch pi
                case 1, p.costs.to_goal = vals(vi);
                case 2, p.costs.speed = vals(vi);
                case 3, p.costs.obstacle = vals(vi);
                case 4, p.costs.heading = vals(vi);
            end
        end
```

- [ ] **Step 3: Commit**

```bash
git add tuning/param_sweep.m
git commit -m "feat: extend param_sweep to support Lattice planner parameters"
```

---

### Task 8: End-to-end verification

- [ ] **Step 1: Run DWA on all 5 scenes, verify no regressions**

```matlab
% In main.m, set planner_type = 'dwa'
% Run each scene: empty, simple, cluttered, corridor, dynamic
% Verify: all complete without errors, metrics print as before
```

- [ ] **Step 2: Run Lattice on all 5 scenes**

```matlab
% In main.m, set planner_type = 'lattice'
% Run each scene
% Verify: all complete, trajectories are smooth curves (not arcs),
% planning time is higher than DWA but within reason (<200ms)
```

- [ ] **Step 3: Compare DWA vs Lattice on 'cluttered' scene**

```matlab
% Run DWA: planner_type = 'dwa', scene = 'cluttered'
% Save result_dwa = result; save('dwa_result.mat', 'result_dwa');
% Run Lattice: planner_type = 'lattice', scene = 'cluttered'
% Save result_lattice = result; save('lattice_result.mat', 'result_lattice');
% Compare: both summary plots open, visually check trajectory differences
```

- [ ] **Step 4: Commit any final tweaks**

```bash
git add -A
git commit -m "chore: final adjustments after end-to-end verification"
```
