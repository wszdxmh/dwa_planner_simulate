function env = setup_environment(name, p)
% 构建仿真环境和地图
%   name = 'empty' | 'simple' | 'cluttered' | 'corridor' | 'dynamic'
%   p    = 参数结构体
% Returns:
%   env  = 结构体包含 map, static_obs, dynamic_obs, robot_start, target, bounds

    env = struct();
    env.name = name;
    env.bounds = [0, p.env.world_size, 0, p.env.world_size];

    res = p.env.resolution;
    world_size = p.env.world_size;
    grid_dim = round(world_size / res);
    env.map = zeros(grid_dim, grid_dim);

    % 默认起点和终点
    env.robot_start = [1.0, 1.0, 45 * pi / 180];
    env.target = [9.0, 9.0];
    env.dynamic_obs = [];

    switch lower(name)
        case 'empty'
            % 无静态障碍物
            env.robot_start = [1.0, 1.0, 0];
            env.target = [9.0, 9.0];

        case 'simple'
            obstacles = [
                3.0, 4.0, 3.0, 4.0;
                6.0, 7.0, 5.0, 6.0;
                3.0, 4.0, 7.0, 8.0;
            ];
            env = rasterize_obstacles(env, obstacles, res);

        case 'cluttered'
            obstacles = [
                2.0, 3.0, 2.0, 3.0;
                5.0, 6.0, 2.5, 3.5;
                7.0, 8.0, 2.0, 3.0;
                2.5, 3.5, 5.0, 6.0;
                5.5, 6.5, 5.5, 6.5;
                3.0, 4.0, 7.5, 8.5;
                7.0, 8.5, 7.0, 8.0;
                1.5, 2.5, 7.0, 8.0;
            ];
            env = rasterize_obstacles(env, obstacles, res);

        case 'corridor'
            % 走廊场景：两堵墙中间有通道
            % 左墙 (上方)
            map_g = zeros(grid_dim, grid_dim);
            wall_left_top = round(6.5 / res);
            wall_right_bottom = round(10 / res);
            % 填充上半部分左墙
            row_start = round(1 / res);
            row_end = round(4 / res);
            for r = row_start:row_end
                for c = 1:wall_left_top
                    if r <= grid_dim && c <= grid_dim
                        map_g(r, c) = 1;
                    end
                end
            end
            for r = row_end+1:wall_right_bottom
                for c = wall_left_top-5:wall_left_top
                    if r <= grid_dim && c <= grid_dim
                        map_g(r, c) = 1;
                    end
                end
            end
            % 右墙
            wall_right_start = round(3.5 / res);
            for r = 1:round(6 / res)
                for c = wall_right_start:grid_dim
                    if r <= grid_dim && c <= grid_dim
                        map_g(r, c) = 1;
                    end
                end
            end
            for r = round(6/res):grid_dim
                for c = wall_right_start-5:grid_dim
                    if r <= grid_dim && c <= grid_dim
                        map_g(r, c) = 1;
                    end
                end
            end
            env.map = map_g;
            env.robot_start = [1.5, 5.0, 0];
            env.target = [6.0, 9.0];

        case 'dynamic'
            obstacles = [
                3.0, 4.0, 3.0, 4.0;
                6.0, 7.0, 5.0, 6.0;
            ];
            env = rasterize_obstacles(env, obstacles, res);

            % 动态障碍物：沿直线来回移动
            env.dynamic_obs = struct(...
                'pos', [5.0, 7.0], ...
                'radius', 0.3, ...
                'waypoints', [5.0, 1.0; 5.0, 7.0; 8.0, 7.0; 8.0, 1.0], ...
                'current_wp', 1, ...
                'speed', 0.2);

        otherwise
            error('未知场景: %s. 可选: empty, simple, cluttered, corridor, dynamic', name);
    end

    % 增加地图边界墙
    env.map(1, :) = 1;
    env.map(end, :) = 1;
    env.map(:, 1) = 1;
    env.map(:, end) = 1;

    env.origin = [p.env.origin_x, p.env.origin_y];
    env.resolution = res;
end

function env = rasterize_obstacles(env, obstacles, res)
    grid_dim = size(env.map, 1);
    for o = 1:size(obstacles, 1)
        gx_min = max(1, round(obstacles(o, 1) / res));
        gx_max = min(grid_dim, round(obstacles(o, 2) / res));
        gy_min = max(1, round(obstacles(o, 3) / res));
        gy_max = min(grid_dim, round(obstacles(o, 4) / res));
        env.map(gy_min:gy_max, gx_min:gx_max) = 1;
    end
    env.static_obs_rects = obstacles;
end
