function traj = quintic_trajectory(state, end_pose, T, n_samples)
% Generate a quintic polynomial trajectory between start and end states.
% Solves 3x3 linear system for x(t) and y(t) coefficients with boundary
% conditions: position, velocity, and zero acceleration at both ends.
%   state     = [x, y, theta, v, omega]  start state
%   end_pose  = [x, y, theta, v, omega]  end state
%   T         = trajectory duration (s)
%   n_samples = number of discrete points
% Returns:
%   traj = [Nx3] (x, y, theta) trajectory points

    x0 = state(1); y0 = state(2); th0 = state(3);
    v0 = state(4);
    xf = end_pose(1); yf = end_pose(2); thf = end_pose(3);
    vf = end_pose(4);

    % --- x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 ---
    a0 = x0;
    a1 = v0 * cos(th0);
    a2 = 0;

    A = [T^3,   T^4,    T^5;
         3*T^2, 4*T^3,  5*T^4;
         6*T,   12*T^2, 20*T^3];

    bx = [xf - a0 - a1*T;
          vf*cos(thf) - a1;
          0];
    ax = A \ bx;

    % --- y(t) = b0 + b1*t + b2*t^2 + b3*t^3 + b4*t^4 + b5*t^5 ---
    b0 = y0;
    b1 = v0 * sin(th0);
    b2 = 0;

    by = [yf - b0 - b1*T;
          vf*sin(thf) - b1;
          0];
    ay = A \ by;

    % --- Discretize ---
    traj = zeros(n_samples, 3);
    dt = T / n_samples;
    for i = 1:n_samples
        t = i * dt;
        t2 = t * t;
        t3 = t2 * t;
        t4 = t3 * t;
        t5 = t4 * t;

        x_t  = a0 + a1*t + a2*t2 + ax(1)*t3 + ax(2)*t4 + ax(3)*t5;
        y_t  = b0 + b1*t + b2*t2 + ay(1)*t3 + ay(2)*t4 + ay(3)*t5;

        xp_t = a1 + 2*a2*t + 3*ax(1)*t2 + 4*ax(2)*t3 + 5*ax(3)*t4;
        yp_t = b1 + 2*b2*t + 3*ay(1)*t2 + 4*ay(2)*t3 + 5*ay(3)*t4;

        traj(i, :) = [x_t, y_t, atan2(yp_t, xp_t)];
    end
end
