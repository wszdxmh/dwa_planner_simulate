function env = step_dynamic_obstacles(env, dt, p)
% 更新动态障碍物位置
    if isempty(env.dynamic_obs) || ~isfield(env.dynamic_obs, 'waypoints')
        return;
    end

    for d = 1:length(env.dynamic_obs)
        obs = env.dynamic_obs(d);
        target_wp = obs.waypoints(obs.current_wp, :);
        dir_to = target_wp - obs.pos;
        dist = norm(dir_to);

        if dist < 0.1
            % 切换到下一个waypoint (循环)
            obs.current_wp = mod(obs.current_wp, size(obs.waypoints, 1)) + 1;
            target_wp = obs.waypoints(obs.current_wp, :);
            dir_to = target_wp - obs.pos;
            dist = norm(dir_to);
        end

        if dist > 0
            move_dist = min(obs.speed * dt, dist);
            obs.pos = obs.pos + dir_to / dist * move_dist;
        end

        % 栅格化动态障碍物到地图
        res = p.env.resolution;
        grid_dim = size(env.map, 1);
        cx = round(obs.pos(1) / res);
        cy = round(obs.pos(2) / res);
        r = ceil(obs.radius / res);
        for dx = -r:r
            for dy = -r:r
                gx = cx + dx;
                gy = cy + dy;
                if gx >= 1 && gx <= grid_dim && gy >= 1 && gy <= grid_dim
                    if hypot(dx * res, dy * res) <= obs.radius
                        env.map(gy, gx) = 1;
                    end
                end
            end
        end

        env.dynamic_obs(d) = obs;
    end
end
