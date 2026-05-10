function keep_running = animate_frame(p, env, state, laser_pts, all_trajs, best_traj, ...
        lookahead_pt, waypoints, smoothed_path, wp_idx, frame_num, planner_type)
% 渲染单帧，返回 keep_running = false 表示用户关闭了窗口
    if nargin < 12, planner_type = 'dwa'; end
    persistent fig_handle ax_handle trail_x trail_y;

    % 检查用户是否关闭了窗口 (通过root appdata通信)
    if isappdata(0, 'dwa_fig_closed') && getappdata(0, 'dwa_fig_closed')
        keep_running = false;
        return;
    end

    if isempty(fig_handle) || ~isvalid(fig_handle)
        fig_handle = figure('Name', 'DWA Planner Simulation', 'NumberTitle', 'off', ...
            'Position', [100, 100, 800, 700]);
        ax_handle = axes('Parent', fig_handle);
        trail_x = [];
        trail_y = [];
        setappdata(0, 'dwa_fig_closed', false);
        set(fig_handle, 'CloseRequestFcn', @(src,~) on_fig_close(src));
    end

    cla(ax_handle);

    % --- 地图 ---
    map_vis = double(env.map);
    map_vis(map_vis == 0) = 0.95;
    map_vis(map_vis == 1) = 0.3;
    imagesc(ax_handle, env.bounds(1:2), env.bounds(3:4), map_vis);
    colormap(ax_handle, gray);
    hold(ax_handle, 'on');

    % --- A*原始路径 (灰色虚线) ---
    if ~isempty(waypoints) && size(waypoints, 1) > 1
        plot(ax_handle, waypoints(:,1), waypoints(:,2), 'Color', [0.6 0.6 0.6], ...
            'LineStyle', '--', 'LineWidth', 1);
        plot(ax_handle, waypoints(:,1), waypoints(:,2), 'ko', 'MarkerSize', 3, ...
            'MarkerFaceColor', [0.5 0.5 0.5]);
    end

    % --- 贝塞尔平滑路径 (青色实线) ---
    if ~isempty(smoothed_path) && size(smoothed_path, 1) > 1
        plot(ax_handle, smoothed_path(:,1), smoothed_path(:,2), 'c-', 'LineWidth', 2);
    end

    % --- 预瞄点 (绿色菱形) — DWA跟踪目标 ---
    plot(ax_handle, lookahead_pt(1), lookahead_pt(2), 'gd', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);

    % --- 激光点 ---
    valid = hypot(laser_pts(:,1) - state(1), laser_pts(:,2) - state(2)) > 0.18;
    if any(valid)
        plot(ax_handle, laser_pts(valid,1), laser_pts(valid,2), '.r', 'MarkerSize', 2);
    end

    % --- 全部DWA预测轨迹 (浅绿色，隔条绘制加速) ---
    if ~isempty(all_trajs)
        stride = max(1, p.sim.traj_display_stride);
        for i = 1:stride:length(all_trajs)
            if ~isempty(all_trajs{i})
                plot(ax_handle, all_trajs{i}(:,1), all_trajs{i}(:,2), ...
                    'Color', [0.2 0.8 0.2 0.3], 'LineWidth', 0.5);
            end
        end
    end

    % --- 最优DWA轨迹 (品红色) ---
    if ~isempty(best_traj)
        plot(ax_handle, best_traj(:,1), best_traj(:,2), 'm.', 'MarkerSize', 10);
        plot(ax_handle, best_traj(:,1), best_traj(:,2), 'm-', 'LineWidth', 2);
    end

    % --- 机器人 (圆形 + 轮子指示) ---
    rx = state(1); ry = state(2); rth = state(3);
    r_radius = p.robot.radius;
    d = p.robot.wheel_separation;

    % 机器人身体 (绿色圆)
    theta_circle = linspace(0, 2*pi, 40);
    circle_x = rx + r_radius * cos(theta_circle);
    circle_y = ry + r_radius * sin(theta_circle);
    patch(ax_handle, circle_x, circle_y, 'g', 'EdgeColor', 'k', 'LineWidth', 1.5, ...
        'FaceAlpha', 0.7);

    % 朝向指示线
    head_x = rx + r_radius * 1.3 * cos(rth);
    head_y = ry + r_radius * 1.3 * sin(rth);
    plot(ax_handle, [rx, head_x], [ry, head_y], 'k-', 'LineWidth', 2);

    % 左右轮子指示
    wl_x = rx - (d/2) * sin(rth);
    wl_y = ry + (d/2) * cos(rth);
    wr_x = rx + (d/2) * sin(rth);
    wr_y = ry - (d/2) * cos(rth);
    wheel_len = r_radius * 0.6;
    plot(ax_handle, [wl_x - wheel_len*cos(rth), wl_x + wheel_len*cos(rth)], ...
        [wl_y - wheel_len*sin(rth), wl_y + wheel_len*sin(rth)], 'k-', 'LineWidth', 2.5);
    plot(ax_handle, [wr_x - wheel_len*cos(rth), wr_x + wheel_len*cos(rth)], ...
        [wr_y - wheel_len*sin(rth), wr_y + wheel_len*sin(rth)], 'k-', 'LineWidth', 2.5);

    % --- 终点 (红色星号) ---
    plot(ax_handle, env.target(1), env.target(2), 'r*', 'MarkerSize', 14, 'LineWidth', 2);

    % --- 机器人历史轨迹 (persistent数组，不受cla影响) ---
    trail_x = [trail_x, state(1)];
    trail_y = [trail_y, state(2)];
    plot(ax_handle, trail_x, trail_y, 'g-', 'LineWidth', 1.5);

    % --- 图例与标题 ---
    title_str = sprintf('[%s] Frame: %d | v=%.2fm/s  w=%.2frad/s | 预瞄→绿◇ | V_R=%.2f V_L=%.2f', ...
        upper(planner_type), ...
        frame_num, state(4), state(5), ...
        state(4) + state(5)*d/2, state(4) - state(5)*d/2);
    title(ax_handle, title_str);

    axis(ax_handle, 'equal');
    xlim(ax_handle, env.bounds(1:2));
    ylim(ax_handle, env.bounds(3:4));
    xlabel(ax_handle, 'X (m)');
    ylabel(ax_handle, 'Y (m)');
    grid(ax_handle, 'on');
    set(ax_handle, 'YDir', 'normal');

    drawnow;
    if isfield(p.sim, 'pause_time') && p.sim.pause_time > 0
        pause(p.sim.pause_time);
    end
    keep_running = true;
end

function on_fig_close(src)
% 用户点击关闭按钮时的回调 — 设置root flag并删除figure
    setappdata(0, 'dwa_fig_closed', true);
    delete(src);
end
