function [costs, infeasible] = evaluate_cost(traj, candidate_v, laser_pts, target_xy, robot_pose, p, global_path)
% 评估单条轨迹，返回各项原始代价 (供归一化用) + 不可行标记
%   costs.to_goal, costs.heading, costs.speed, costs.obstacle, costs.path
%   infeasible = true 表示该轨迹会碰撞，不可行

    % 默认
    costs = struct('to_goal', 0, 'heading', 0, 'speed', 0, 'obstacle', 0, 'path', 0);
    infeasible = false;

    % 1. 目标距离代价
    costs.to_goal = hypot(traj(end,1) - target_xy(1), traj(end,2) - target_xy(2));

    % 2. 航向代价 (归一化到 [0, 1])
    dir_to_target = target_xy - robot_pose(1:2);
    norm_t = norm(dir_to_target);
    dir_traj_end = traj(end, 1:2) - robot_pose(1:2);
    norm_e = norm(dir_traj_end);
    if norm_t > 1e-9 && norm_e > 1e-9
        cos_h = dot(dir_to_target, dir_traj_end) / (norm_t * norm_e);
        cos_h = max(-1, min(1, cos_h));
        costs.heading = acos(cos_h) / pi;
    end

    % 3. 速度代价
    costs.speed = abs(p.robot.target_velocity - candidate_v);

    % 4. 障碍物代价
    obs_min = inf;
    n_laser = size(laser_pts, 1);
    if n_laser > 0
        dists_to_robot = hypot(laser_pts(:,1) - robot_pose(1), laser_pts(:,2) - robot_pose(2));
        nearby = dists_to_robot < p.obstacle.filter_radius;
        if any(nearby)
            near_pts = laser_pts(nearby, :);
            for k = 1:size(traj, 1)
                dx = traj(k,1) - near_pts(:,1);
                dy = traj(k,2) - near_pts(:,2);
                d_min = min(hypot(dx, dy));
                obs_min = min(obs_min, d_min);
            end
        end
    end

    % 不可行判定：轨迹点距离障碍物小于机器人半径
    if obs_min < p.robot.radius
        infeasible = true;
        costs.obstacle = inf;
    elseif isinf(obs_min)
        costs.obstacle = 0;
    elseif obs_min < p.obstacle.safe_distance
        % 指数衰减：从半径处≈1 衰减到安全距离处≈0
        costs.obstacle = exp(-p.obstacle.decay_factor * (obs_min - p.robot.radius));
    else
        costs.obstacle = 0;
    end

    % 5. 路径偏离代价：轨迹终点到全局平滑路径的最短距离
    if nargin >= 7 && ~isempty(global_path) && size(global_path, 1) > 1
        dists = hypot(global_path(:,1) - traj(end,1), global_path(:,2) - traj(end,2));
        costs.path = min(dists);
    end
end
