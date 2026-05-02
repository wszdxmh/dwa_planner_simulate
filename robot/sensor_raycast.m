function [laser_pts_world, ranges] = sensor_raycast(robot_pose, map, p)
% 在栅格地图上模拟360束2D激光雷达
%   robot_pose = [x, y, theta] (rad)
%   map        = 二值占据栅格 (H x W), 1=占据, 0=自由
%   p          = 参数结构体 (来自 dwa_params)
% Returns:
%   laser_pts_world = Nx2 世界坐标下的击中点
%   ranges          = Nx1 距离 (m)

    n_beams = p.sensor.num_beams;
    max_range = p.sensor.max_range;
    min_range = p.sensor.min_range;

    map_h = size(map, 1);
    map_w = size(map, 2);
    res = p.env.resolution;
    origin_x = p.env.origin_x;
    origin_y = p.env.origin_y;

    % 激光在世界坐标系中的原点
    px = robot_pose(1);
    py = robot_pose(2);
    ptheta = robot_pose(3);
    sensor_wx = px + p.sensor.offset_x * cos(ptheta) - p.sensor.offset_y * sin(ptheta);
    sensor_wy = py + p.sensor.offset_x * sin(ptheta) + p.sensor.offset_y * cos(ptheta);
    sensor_angle_offset = ptheta + p.sensor.offset_theta;

    ranges = zeros(n_beams, 1);
    laser_pts_world = zeros(n_beams, 2);

    for j = 1:n_beams
        beam_angle = sensor_angle_offset + (j - 1) * pi / 180;
        dx = cos(beam_angle);
        dy = sin(beam_angle);

        % DDA 射线投射
        dist = cast_ray(sensor_wx, sensor_wy, dx, dy, map, map_h, map_w, res, origin_x, origin_y, max_range);

        % 添加比例噪声
        dist = dist * (1 + 0.02 * randn());
        dist = max(min_range, min(max_range, dist));

        ranges(j) = dist;
        if dist < max_range
            laser_pts_world(j, :) = [sensor_wx + dx * dist, sensor_wy + dy * dist];
        else
            laser_pts_world(j, :) = [sensor_wx + dx * max_range, sensor_wy + dy * max_range];
        end
    end
end

function dist = cast_ray(ox, oy, dx, dy, map, map_h, map_w, res, orig_x, orig_y, max_range)
% DDA射线投射，返回首次击中距离

    % 起点所在的栅格坐标
    gx = round((ox - orig_x) / res) + 1;
    gy = round((oy - orig_y) / res) + 1;

    % 检查起点是否出界
    if gx < 1 || gx > map_w || gy < 1 || gy > map_h
        dist = max_range;
        return;
    end

    % DDA步进参数
    step_x = sign(dx);
    step_y = sign(dy);
    if step_x == 0, step_x = 1; end
    if step_y == 0, step_y = 1; end

    % 到下一个栅格边界的距离
    if dx > 0
        t_delta_x = res / abs(dx);
        next_boundary_x = orig_x + (gx) * res;
        t_max_x = (next_boundary_x - ox) / dx;
    else
        t_delta_x = res / abs(dx);
        next_boundary_x = orig_x + (gx - 1) * res;
        t_max_x = (next_boundary_x - ox) / dx;
    end

    if dy > 0
        t_delta_y = res / abs(dy);
        next_boundary_y = orig_y + (gy) * res;
        t_max_y = (next_boundary_y - oy) / dy;
    else
        t_delta_y = res / abs(dy);
        next_boundary_y = orig_y + (gy - 1) * res;
        t_max_y = (next_boundary_y - oy) / dy;
    end

    dist = 0;
    while dist < max_range
        if t_max_x < t_max_y
            dist = t_max_x;
            gx = gx + step_x;
            t_max_x = t_max_x + t_delta_x;
        else
            dist = t_max_y;
            gy = gy + step_y;
            t_max_y = t_max_y + t_delta_y;
        end

        % 出界检查
        if gx < 1 || gx > map_w || gy < 1 || gy > map_h
            dist = max_range;
            return;
        end

        % 击中检查
        if map(gy, gx) == 1
            return;
        end
    end
    dist = max_range;
end
