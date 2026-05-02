function [best_v, best_w, best_traj, min_cost, all_trajs] = dwa_core(state, laser_pts, target_xy, p, global_path)
% DWA 局部规划器核心 (优化版：不可行排除 + ω=0采样 + 路径代价)
%   state       = [x, y, theta, v, omega] 当前状态
%   laser_pts   = [Mx2] 障碍物点世界坐标
%   target_xy   = [x, y] 局部目标 (预瞄点)
%   p           = 参数结构体
%   global_path = [Nx2] 全局平滑路径 (用于路径代价)
% Returns:
%   best_v, best_w = 最优控制指令
%   best_traj      = [Nx3] 最优轨迹
%   min_cost       = 最优代价 (原始代价加权和)
%   all_trajs      = cell array, 每条轨迹 [Nx3]

    if nargin < 5, global_path = []; end

    % 计算动态窗口
    [v_min, v_max, w_min, w_max] = compute_dynamic_window(state, p);

    % 速度采样
    v_samples = v_min:p.dwa.velocity_res:v_max;
    w_samples = w_min:p.dwa.omega_res:w_max;

    if isempty(v_samples), v_samples = v_min; end
    if isempty(w_samples), w_samples = w_min; end

    % 预估候选数量
    est_total = length(v_samples) * length(w_samples) + length(v_samples);
    all_trajs = cell(est_total, 1);
    all_costs = cell(est_total, 1);
    all_infeasible = false(est_total, 1);
    all_vw = zeros(est_total, 2);

    robot_pose = state(1:3);
    idx = 0;

    for vi = 1:length(v_samples)
        v = v_samples(vi);
        for wi = 1:length(w_samples)
            w = w_samples(wi);
            idx = idx + 1;

            traj = generate_trajectory(state, v, w, p);
            all_trajs{idx} = traj;
            all_vw(idx, :) = [v, w];
            [all_costs{idx}, all_infeasible(idx)] = ...
                evaluate_cost(traj, v, laser_pts, target_xy, robot_pose, p, global_path);
        end

        % 优化3: 当动态窗口包含0时，总是追加 ω=0 候选 (偏好直行)
        if w_min < 0 && 0 < w_max
            idx = idx + 1;
            w = 0;
            traj = generate_trajectory(state, v, w, p);
            all_trajs{idx} = traj;
            all_vw(idx, :) = [v, w];
            [all_costs{idx}, all_infeasible(idx)] = ...
                evaluate_cost(traj, v, laser_pts, target_xy, robot_pose, p, global_path);
        end
    end

    all_trajs = all_trajs(1:idx);
    all_costs = all_costs(1:idx);
    all_infeasible = all_infeasible(1:idx);

    % 在可行轨迹中选最小加权代价 (原始代价直接加权)
    min_cost = inf;
    best_v = 0;
    best_w = 0;
    best_traj = [];
    best_idx = 0;

    for i = 1:idx
        if all_infeasible(i)
            continue;
        end
        c = all_costs{i};
        weighted = p.costs.to_goal * c.to_goal ...
                 + p.costs.heading * c.heading ...
                 + p.costs.speed   * c.speed ...
                 + p.costs.obstacle * c.obstacle ...
                 + p.costs.path    * c.path;

        if weighted < min_cost
            min_cost = weighted;
            best_traj = all_trajs{i};
            best_idx = i;
        end
    end

    if best_idx > 0
        best_v = all_vw(best_idx, 1);
        best_w = all_vw(best_idx, 2);
    end
end
