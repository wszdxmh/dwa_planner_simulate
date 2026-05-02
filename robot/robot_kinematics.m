function [state_next, wheel_vel] = robot_kinematics(state, v, omega, dt, p)
% 差速轮运动学模型
%   state      = [x, y, theta, v, omega] (当前状态)
%   v, omega   = 控制指令 (线速度 m/s, 角速度 rad/s)
%   dt         = 时间步长
%   p          = 参数结构体 (含 wheel_separation)
% Returns:
%   state_next = 下一状态
%   wheel_vel  = [v_R, v_L] 左右轮速度

    if nargin < 5 || ~isfield(p.robot, 'wheel_separation')
        d = 0.3;  % 默认轮距 0.3m
    else
        d = p.robot.wheel_separation;
    end

    % 差速轮速度
    v_R = v + omega * d / 2;
    v_L = v - omega * d / 2;
    wheel_vel = [v_R, v_L];

    % 位置更新 (与独轮车一致，因为(v,ω)是等效控制量)
    state_next = state;
    state_next(1) = state(1) + v * cos(state(3)) * dt;
    state_next(2) = state(2) + v * sin(state(3)) * dt;
    state_next(3) = state_next(3) + omega * dt;
    state_next(4) = v;
    state_next(5) = omega;
end
