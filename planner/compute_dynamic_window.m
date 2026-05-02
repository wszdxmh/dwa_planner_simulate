function [v_min, v_max, omega_min, omega_max] = compute_dynamic_window(state, p)
% 计算动态速度窗口
%   受加速度限制 + 全局速度限制

    v_curr = state(4);
    omega_curr = state(5);

    % 加速度限制窗口
    v_min = max(v_curr - p.robot.max_acceleration * p.dwa.dt, p.robot.min_velocity);
    v_max = min(v_curr + p.robot.max_acceleration * p.dwa.dt, p.robot.max_velocity);
    omega_min = max(omega_curr - p.robot.max_ang_accel * p.dwa.dt, -p.robot.max_omega);
    omega_max = min(omega_curr + p.robot.max_ang_accel * p.dwa.dt, p.robot.max_omega);

    % 确保 min <= max
    if v_min > v_max, v_min = v_max; end
    if omega_min > omega_max, omega_min = omega_max; end
end
