function plot_summary(result)
% 仿真后汇总绘图

    n = result.n_frames;
    time = result.log.time(1:n);
    pose = result.log.pose(1:n, :);
    vel = result.log.velocity(1:n, :);
    cmd_vel = result.log.cmd_velocity(1:n, :);
    plan_t = result.log.plan_time(1:n);
    cost = result.log.min_cost(1:n);
    dist = result.log.dist_to_goal(1:n);
    wp_idx = result.log.waypoint_idx(1:n);

    figure('Name', 'Simulation Summary', 'NumberTitle', 'off', 'Position', [100, 100, 900, 700]);

    % 1. 机器人轨迹 (地图 + A*路径 + 贝塞尔平滑 + 实际路径)
    subplot(2, 3, 1);
    env = result.env;
    map_vis = double(env.map);
    map_vis(map_vis == 0) = 0.95;
    map_vis(map_vis == 1) = 0.3;
    imagesc(env.bounds(1:2), env.bounds(3:4), map_vis);
    colormap(gca, gray); hold on;
    if ~isempty(result.waypoints)
        plot(result.waypoints(:,1), result.waypoints(:,2), 'Color', [0.6 0.6 0.6], ...
            'LineStyle', '--', 'LineWidth', 1, 'DisplayName', 'A* 原始路径');
    end
    if isfield(result, 'smoothed_path') && ~isempty(result.smoothed_path)
        plot(result.smoothed_path(:,1), result.smoothed_path(:,2), 'c-', 'LineWidth', 2, ...
            'DisplayName', '贝塞尔平滑路径');
    end
    plot(pose(:,1), pose(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'DWA 实际路径');
    plot(pose(1,1), pose(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    plot(env.target(1), env.target(2), 'r*', 'MarkerSize', 12, 'DisplayName', '终点');
    axis equal; xlim(env.bounds(1:2)); ylim(env.bounds(3:4));
    xlabel('X (m)'); ylabel('Y (m)');
    title('机器人轨迹');
    legend('Location', 'best');
    grid on; set(gca, 'YDir', 'normal');

    % 2. 距离目标变化
    subplot(2, 3, 2);
    plot(time, dist, 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('距离目标 (m)');
    title('距离目标变化');
    grid on;

    % 3. 速度曲线
    subplot(2, 3, 3);
    yyaxis left;
    plot(time, vel(:,1), 'b-', 'LineWidth', 1, 'DisplayName', '实际v');
    hold on;
    plot(time, cmd_vel(:,1), 'b--', 'LineWidth', 1, 'DisplayName', '指令v');
    ylabel('线速度 (m/s)');
    yyaxis right;
    plot(time, vel(:,2), 'r-', 'LineWidth', 1, 'DisplayName', '实际ω');
    hold on;
    plot(time, cmd_vel(:,2), 'r--', 'LineWidth', 1, 'DisplayName', '指令ω');
    ylabel('角速度 (rad/s)');
    xlabel('时间 (s)');
    title('速度曲线');
    legend('Location', 'best');
    grid on;

    % 4. 规划耗时
    subplot(2, 3, 4);
    plot(time, plan_t*1000, 'k-', 'LineWidth', 1);
    xlabel('时间 (s)'); ylabel('规划时间 (ms)');
    title(sprintf('DWA规划耗时 (均值: %.1fms)', mean(plan_t)*1000));
    grid on;

    % 5. 代价变化
    subplot(2, 3, 5);
    plot(time, cost, 'm-', 'LineWidth', 1);
    xlabel('时间 (s)'); ylabel('最小代价');
    title('DWA代价变化');
    grid on;

    % 6. waypoint 索引
    subplot(2, 3, 6);
    stairs(time, wp_idx, 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('Waypoint 索引');
    title(sprintf('Waypoint 跟踪进度 (%d/%d)', wp_idx(end), size(result.waypoints,1)));
    grid on;

    if isfield(result, 'planner_type')
        sgtitle(sprintf('%s [%s]', result.env.name, upper(result.planner_type)));
    else
        sgtitle(result.env.name);
    end
end
