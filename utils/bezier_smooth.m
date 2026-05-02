function smoothed = bezier_smooth(waypoints, n_samples_per_seg)
% 对A*路径点进行三次贝塞尔曲线平滑
%   waypoints = [Mx2] 原始路径点
%   n_samples_per_seg = 每段采样点数 (默认20)
% Returns:
%   smoothed = [Nx2] 平滑后的密集路径点

    if nargin < 2
        n_samples_per_seg = 20;
    end

    if size(waypoints, 1) < 3
        smoothed = waypoints;
        return;
    end

    n = size(waypoints, 1);
    smoothed = [];

    for i = 1:n-1
        p0 = waypoints(i, :);
        p3 = waypoints(min(i+1, n), :);

        % 计算切线方向
        if i == 1
            t0 = waypoints(2, :) - waypoints(1, :);
        else
            t0 = waypoints(i+1, :) - waypoints(i-1, :);
        end
        if i == n-1
            t3 = waypoints(n, :) - waypoints(n-1, :);
        else
            t3 = waypoints(i+2, :) - waypoints(i, :);
        end

        % 归一化切线并用弦长缩放
        chord = norm(p3 - p0);
        if norm(t0) > 0, t0 = t0 / norm(t0) * chord * 0.4; end
        if norm(t3) > 0, t3 = t3 / norm(t3) * chord * 0.4; end

        p1 = p0 + t0;
        p2 = p3 - t3;

        % 生成贝塞尔点
        for j = 0:n_samples_per_seg
            t = j / n_samples_per_seg;
            % 三次贝塞尔: B(t) = (1-t)^3*P0 + 3(1-t)^2*t*P1 + 3(1-t)*t^2*P2 + t^3*P3
            pt = (1-t)^3 * p0 + 3*(1-t)^2*t * p1 + 3*(1-t)*t^2 * p2 + t^3 * p3;
            smoothed = [smoothed; pt];
        end
    end
end
