function m = compute_metrics(result)
% 从仿真结果计算性能指标

    n = result.n_frames;
    pose = result.log.pose(1:n, :);
    vel = result.log.velocity(1:n, :);
    cmd_vel = result.log.cmd_velocity(1:n, :);
    plan_t = result.log.plan_time(1:n);

    m = struct();
    m.success = result.success;
    m.n_frames = n;
    m.total_time = result.log.time(n);

    % 路径长度 (累积位移)
    dx = diff(pose(:,1));
    dy = diff(pose(:,2));
    m.path_length = sum(sqrt(dx.^2 + dy.^2));

    % 目标误差
    m.goal_error = result.log.dist_to_goal(n);

    % 到达时间 (如果成功)
    if m.success
        m.time_to_goal = result.log.time(n);
    else
        m.time_to_goal = NaN;
    end

    % 平均速度
    m.avg_velocity = mean(vel(:,1));

    % 路径平滑度 (航向变化绝对值和)
    m.smoothness = sum(abs(diff(pose(:,3))));

    % 最小障碍物距离 (从激光数据)
    m.min_clearance = inf;
    for i = 1:n
        if ~isempty(result.log.laser_pts_cell{i})
            pts = result.log.laser_pts_cell{i};
            dists = hypot(pts(:,1) - pose(i,1), pts(:,2) - pose(i,2));
            m.min_clearance = min(m.min_clearance, min(dists));
        end
    end
    if isinf(m.min_clearance), m.min_clearance = NaN; end

    % 规划时间统计
    m.avg_plan_time = mean(plan_t);
    m.max_plan_time = max(plan_t);
    m.min_plan_time = min(plan_t);

    % 平均线速度和角速度
    m.avg_linear_vel = mean(vel(:,1));
    m.avg_angular_vel = mean(abs(vel(:,2)));
end
