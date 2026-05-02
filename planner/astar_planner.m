function [waypoints, found] = astar_planner(map, start_xy, goal_xy, p)
% A* 全局路径规划 (8邻域)
%   map        = 二值占据栅格 (H x W), 1=占据, 0=自由
%   start_xy   = [x, y] 起点世界坐标
%   goal_xy    = [x, y] 终点世界坐标
%   p          = 参数结构体
% Returns:
%   waypoints  = [Mx2] 世界坐标下的路径点序列 (含起点)
%   found      = true/false 是否找到路径

    res = p.env.resolution;
    orig_x = p.env.origin_x;
    orig_y = p.env.origin_y;
    map_h = size(map, 1);
    map_w = size(map, 2);

    % 世界坐标 → 栅格坐标
    start_gx = round((start_xy(1) - orig_x) / res) + 1;
    start_gy = round((start_xy(2) - orig_y) / res) + 1;
    goal_gx = round((goal_xy(1) - orig_x) / res) + 1;
    goal_gy = round((goal_xy(2) - orig_y) / res) + 1;

    % 边界裁剪
    start_gx = max(1, min(map_w, start_gx));
    start_gy = max(1, min(map_h, start_gy));
    goal_gx = max(1, min(map_w, goal_gx));
    goal_gy = max(1, min(map_h, goal_gy));

    % 起点或终点在障碍物上，膨胀搜索
    if map(start_gy, start_gx) == 1
        [start_gx, start_gy] = find_nearest_free(map, start_gx, start_gy, map_w, map_h);
    end
    if map(goal_gy, goal_gx) == 1
        [goal_gx, goal_gy] = find_nearest_free(map, goal_gx, goal_gy, map_w, map_h);
    end

    % 8邻域: 上下左右 + 对角线
    dx = [ 0,  1,  1,  1,  0, -1, -1, -1];
    dy = [-1, -1,  0,  1,  1,  1,  0, -1];
    costs = [1, sqrt(2), 1, sqrt(2), 1, sqrt(2), 1, sqrt(2)];

    % 障碍物膨胀代价图：离障碍物越近代价越高
    cost_map = zeros(map_h, map_w);
    if isfield(p.planner, 'obstacle_inflation') && p.planner.obstacle_inflation > 0
        inflate_cells = ceil(p.planner.obstacle_inflation / res);
        [kx, ky] = meshgrid(-inflate_cells:inflate_cells, -inflate_cells:inflate_cells);
        kernel = max(0, 1 - sqrt(kx.^2 + ky.^2) / inflate_cells);
        kernel = kernel / max(kernel(:));
        cost_map = p.planner.inflation_weight * conv2(double(map), kernel, 'same');
    end

    % Open set (优先队列简化版：用数组+循环找最小)
    % 每个元素: [gx, gy, g_cost, parent_idx]
    open_list = zeros(map_h * map_w, 4);
    open_count = 0;

    % 已探索节点 (closed set): visited(gy, gx) = g_cost
    visited = inf(map_h, map_w);
    parent = zeros(map_h * map_w, 2);  % 父节点索引
    parent_idx_map = zeros(map_h, map_w);  % 栅格 → open_list索引

    % 初始化起点
    h_start = hypot(goal_gx - start_gx, goal_gy - start_gy);
    open_count = 1;
    open_list(1, :) = [start_gx, start_gy, 0, h_start];
    visited(start_gy, start_gx) = 0;

    found = false;
    goal_node_linear = 0;

    while open_count > 0
        % 找 f = g + h 最小的节点
        min_idx = 1;
        min_f = open_list(1, 3) + open_list(1, 4);
        for k = 2:open_count
            f = open_list(k, 3) + open_list(k, 4);
            if f < min_f
                min_f = f;
                min_idx = k;
            end
        end

        % 取出当前节点
        curr_gx = open_list(min_idx, 1);
        curr_gy = open_list(min_idx, 2);
        curr_g = open_list(min_idx, 3);

        % 到达目标
        if curr_gx == goal_gx && curr_gy == goal_gy
            goal_node_linear = (curr_gy - 1) * map_w + curr_gx;
            found = true;
            break;
        end

        % 从open list移除 (用最后元素替换)
        open_list(min_idx, :) = open_list(open_count, :);
        open_count = open_count - 1;

        % 探索邻域
        for d = 1:8
            nx = curr_gx + dx(d);
            ny = curr_gy + dy(d);

            if nx < 1 || nx > map_w || ny < 1 || ny > map_h
                continue;
            end
            if map(ny, nx) == 1
                continue;
            end

            penalty = cost_map(ny, nx);
            new_g = curr_g + costs(d) * (1 + penalty);
            if new_g < visited(ny, nx)
                visited(ny, nx) = new_g;
                linear_idx = (ny - 1) * map_w + nx;
                parent(linear_idx, :) = [curr_gx, curr_gy];

                h = hypot(goal_gx - nx, goal_gy - ny);
                open_count = open_count + 1;
                if open_count > size(open_list, 1)
                    open_list = [open_list; zeros(map_h * map_w, 4)];
                end
                open_list(open_count, :) = [nx, ny, new_g, h];
            end
        end
    end

    if ~found
        waypoints = [start_xy; goal_xy];
        return;
    end

    % 回溯路径
    path_g = [];
    cx = goal_gx;
    cy = goal_gy;
    while true
        path_g = [cx, cy; path_g];
        if cx == start_gx && cy == start_gy
            break;
        end
        linear = (cy - 1) * map_w + cx;
        cx = parent(linear, 1);
        cy = parent(linear, 2);
    end

    % 转换回世界坐标
    waypoints = zeros(size(path_g, 1), 2);
    for i = 1:size(path_g, 1)
        waypoints(i, 1) = orig_x + (path_g(i, 1) - 1) * res;
        waypoints(i, 2) = orig_y + (path_g(i, 2) - 1) * res;
    end

    % 路径平滑：剔除共线点
    if size(waypoints, 1) > 2
        keep = true(size(waypoints, 1), 1);
        for i = 2:size(waypoints, 1) - 1
            d1 = waypoints(i, :) - waypoints(i - 1, :);
            d2 = waypoints(i + 1, :) - waypoints(i, :);
            if abs(d1(1)*d2(2) - d1(2)*d2(1)) < 1e-6 && dot(d1, d2) > 0
                keep(i) = false;
            end
        end
        waypoints = waypoints(keep, :);
    end
end

function [fx, fy] = find_nearest_free(map, x, y, w, h)
    for r = 1:max(w, h)
        for dx = -r:r
            for dy = -r:r
                if abs(dx) ~= r && abs(dy) ~= r
                    continue;
                end
                nx = x + dx; ny = y + dy;
                if nx >= 1 && nx <= w && ny >= 1 && ny <= h && map(ny, nx) == 0
                    fx = nx; fy = ny;
                    return;
                end
            end
        end
    end
    fx = x; fy = y;
end
