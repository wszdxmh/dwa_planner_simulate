function traj = generate_trajectory(state, v, omega, p)
% 前向模拟一条轨迹
%   state = [x, y, theta, v_curr, omega_curr]
%   v, omega = 候选控制指令
%   p = 参数结构体
%   traj = [Nx3] (x, y, theta) 轨迹点序列

    n_steps = ceil(p.dwa.sim_time / p.dwa.dt) + 1;
    traj = zeros(n_steps, 3);

    s = state;
    for t = 1:n_steps
        s(1) = s(1) + v * cos(s(3)) * p.dwa.dt;
        s(2) = s(2) + v * sin(s(3)) * p.dwa.dt;
        s(3) = s(3) + omega * p.dwa.dt;
        traj(t, :) = [s(1), s(2), s(3)];
    end
end
