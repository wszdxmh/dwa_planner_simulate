function result = run_simulation(p, env, do_visualize)
% 主仿真循环: A*全局规划 + 贝塞尔平滑 + 纯追踪预瞄 + DWA局部跟踪
%   result = 仿真结果结构体

    addpath(genpath(fileparts(mfilename('fullpath'))));

    %% A* 全局路径规划
    waypoints = [];
    if p.planner.astar_enabled
        base_map = env.map;
        [waypoints, found] = astar_planner(base_map, env.robot_start(1:2), env.target, p);
        if ~found
            warning('A*未找到路径，直接使用终点作为目标');
            waypoints = [env.robot_start(1:2); env.target];
        end
    else
        waypoints = [env.robot_start(1:2); env.target];
    end

    %% 贝塞尔曲线平滑全局路径
    smoothed_path = bezier_smooth(waypoints, 20);

    % 同步env字段到p.env (供sensor_raycast等函数使用)
    p.env.resolution = env.resolution;
    p.env.origin_x = env.origin(1);
    p.env.origin_y = env.origin(2);

    %% 初始化
    state = [env.robot_start(1:3), 0.0, 0.0];  % [x, y, theta, v, omega]
    n_frames = p.sim.max_frames;

    % 预分配日志
    log.time = zeros(n_frames, 1);
    log.pose = zeros(n_frames, 3);
    log.velocity = zeros(n_frames, 2);
    log.cmd_velocity = zeros(n_frames, 2);
    log.wheel_vel = zeros(n_frames, 2);
    log.waypoint_idx = zeros(n_frames, 1);
    log.plan_time = zeros(n_frames, 1);
    log.min_cost = zeros(n_frames, 1);
    log.cost_components = zeros(n_frames, 4);
    log.best_traj_cell = cell(n_frames, 1);
    log.all_trajs_cell = cell(n_frames, 1);
    log.laser_pts_cell = cell(n_frames, 1);
    log.lookahead_pt = zeros(n_frames, 2);
    log.dist_to_goal = zeros(n_frames, 1);

    current_wp_idx = 1;
    stuck_counter = 0;
    last_pos = state(1:2);

    %% 主循环
    total_start = tic;

    for t = 1:n_frames
        log.time(t) = (t - 1) * p.sim.control_dt;
        log.pose(t, :) = state(1:3);
        log.velocity(t, :) = state(4:5);

        % --- 纯追踪预瞄：找平滑路径上机器前方50cm的点 ---
        lookahead_pt = compute_lookahead(state(1:2), state(3), smoothed_path, p.planner.lookahead_dist);
        log.lookahead_pt(t, :) = lookahead_pt;

        % --- waypoint切换：距当前waypoint足够近 ---
        if current_wp_idx > size(waypoints, 1)
            current_wp_idx = size(waypoints, 1);
        end
        current_wp = waypoints(current_wp_idx, :);
        dist_to_wp = hypot(state(1) - current_wp(1), state(2) - current_wp(2));
        if dist_to_wp < p.planner.waypoint_tolerance && current_wp_idx < size(waypoints, 1)
            current_wp_idx = current_wp_idx + 1;
        end
        log.waypoint_idx(t) = current_wp_idx;

        log.dist_to_goal(t) = hypot(env.target(1) - state(1), env.target(2) - state(2));

        % 检查是否到达目标
        if log.dist_to_goal(t) < p.sim.goal_tolerance
            fprintf('到达目标! 帧: %d, 时间: %.2fs\n', t, log.time(t));
            result = pack_result(log, waypoints, smoothed_path, p, env, t);
            return;
        end

        % 感知：模拟激光雷达
        [laser_pts, ~] = sensor_raycast(state(1:3), env.map, p);
        log.laser_pts_cell{t} = laser_pts;

        % 规划：DWA局部规划 (目标=预瞄点)
        tic_plan = tic;
        [v_cmd, w_cmd, best_traj, min_cost, all_trajs] = dwa_core(state, laser_pts, lookahead_pt, p, smoothed_path);
        log.plan_time(t) = toc(tic_plan);
        log.min_cost(t) = min_cost;
        log.best_traj_cell{t} = best_traj;
        log.all_trajs_cell{t} = all_trajs;

        log.cmd_velocity(t, :) = [v_cmd, w_cmd];

        % 执行控制 (差速轮)
        [state, wheel_v] = robot_kinematics(state, v_cmd, w_cmd, p.sim.control_dt, p);
        state(3) = normalize_angle(state(3));
        log.wheel_vel(t, :) = wheel_v;

        % 更新动态障碍物
        env = step_dynamic_obstacles(env, p.sim.control_dt, p);

        % 卡住检测 → 触发A*重规划 (接近目标时抑制，避免终点附近误触发)
        near_goal = log.dist_to_goal(t) < 1.0;
        if ~near_goal && hypot(state(1) - last_pos(1), state(2) - last_pos(2)) < 0.01
            stuck_counter = stuck_counter + 1;
            if stuck_counter > 30
                fprintf('卡住检测 (帧 %d)，触发A*重规划...\n', t);
                [new_wp, found] = astar_planner(env.map, state(1:2), env.target, p);
                if found && size(new_wp,1) > 1
                    waypoints = new_wp;
                    smoothed_path = bezier_smooth(waypoints, 20);
                    current_wp_idx = 1;
                    fprintf('  新路径: %d 个waypoint\n', size(waypoints,1));
                else
                    fprintf('  重规划失败，直接瞄准终点\n');
                    waypoints = [state(1:2); env.target];
                    smoothed_path = waypoints;
                    current_wp_idx = 1;
                end
                stuck_counter = 0;
            end
        else
            stuck_counter = 0;
        end
        last_pos = state(1:2);

        % 可视化
        if do_visualize
            keep_running = animate_frame(p, env, state, laser_pts, all_trajs, best_traj, ...
                lookahead_pt, waypoints, smoothed_path, current_wp_idx, t);
            if ~keep_running
                fprintf('用户关闭了可视化窗口 (帧 %d)\n', t);
                result = pack_result(log, waypoints, smoothed_path, p, env, t);
                return;
            end
        end

        if mod(t, 50) == 0
            fprintf('帧: %d/%d, 距离目标: %.2fm, 规划耗时: %.1fms\n', ...
                t, n_frames, log.dist_to_goal(t), log.plan_time(t)*1000);
        end
    end

    fprintf('达到最大帧数 %d, 未到达目标。最终距离: %.2fm\n', n_frames, log.dist_to_goal(end));
    result = pack_result(log, waypoints, smoothed_path, p, env, n_frames);
end

function lookahead_pt = compute_lookahead(robot_xy, robot_th, path, lookahead_dist)
% 纯追踪预瞄：找path上从最近点算起，距离机器人 > lookahead_dist 的第一个点
% 优先在机器人前进方向搜索

    n_pts = size(path, 1);
    if n_pts < 1
        lookahead_pt = robot_xy + [cos(robot_th), sin(robot_th)] * lookahead_dist;
        return;
    end

    % 找路径上距离机器人最近的点索引
    dists = hypot(path(:,1) - robot_xy(1), path(:,2) - robot_xy(2));
    [~, nearest_idx] = min(dists);

    % 从最近点向前搜索，直到距离 > lookahead_dist
    lookahead_pt = path(end, :);
    for i = nearest_idx:n_pts
        d = hypot(path(i,1) - robot_xy(1), path(i,2) - robot_xy(2));
        if d >= lookahead_dist
            lookahead_pt = path(i, :);
            return;
        end
    end

    % 路径终点也不够远 → 锁定终点，不再延伸
end

function result = pack_result(log, waypoints, smoothed_path, p, env, n_frames)
    result.log = struct();
    fns = fieldnames(log);
    for i = 1:length(fns)
        fn = fns{i};
        if iscell(log.(fn))
            result.log.(fn) = log.(fn)(1:n_frames);
        else
            result.log.(fn) = log.(fn)(1:n_frames, :);
        end
    end
    result.waypoints = waypoints;
    result.smoothed_path = smoothed_path;
    result.params = p;
    result.env = env;
    result.n_frames = n_frames;
    result.success = result.log.dist_to_goal(end) < p.sim.goal_tolerance;
end
