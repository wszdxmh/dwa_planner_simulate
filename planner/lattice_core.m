function [best_v, best_w, best_traj, min_cost, all_trajs] = lattice_core(state, laser_pts, target_xy, p, global_path)
% Lattice planner: sample end states on a spatial grid, generate quintic
% polynomial trajectories, evaluate costs, return best.
% Same interface as dwa_core for drop-in switching.
%   state       = [x, y, theta, v, omega]
%   laser_pts   = [Mx2] obstacle points in world frame
%   target_xy   = [x, y] local target (lookahead point)
%   p           = parameter struct (includes p.lattice.*)
%   global_path = [Nx2] smoothed global path

    if nargin < 5, global_path = []; end

    robot_pose = state(1:3);

    % Build sampling grid in robot frame
    long_dists = p.lattice.longitudinal_dists;
    lat_base = linspace(-p.lattice.lateral_range, p.lattice.lateral_range, p.lattice.num_lateral);
    heading_offsets = linspace(-p.lattice.heading_range, p.lattice.heading_range, p.lattice.num_headings);

    % Bias lateral sampling toward global path to improve path tracking
    lat_bias = 0;
    if ~isempty(global_path) && size(global_path, 1) > 1
        dists = hypot(global_path(:,1) - robot_pose(1), global_path(:,2) - robot_pose(2));
        [~, nearest_idx] = min(dists);
        nearest_pt = global_path(nearest_idx, :);
        % Lateral offset in robot frame: >0 means path is to robot's left
        vec = nearest_pt - robot_pose(1:2);
        lat_bias = -sin(robot_pose(3)) * vec(1) + cos(robot_pose(3)) * vec(2);
        lat_bias = max(-p.lattice.lateral_range * 0.8, min(p.lattice.lateral_range * 0.8, lat_bias));
    end
    lat_offsets = lat_base + lat_bias;

    n_candidates = length(long_dists) * length(lat_offsets) * length(heading_offsets);
    all_trajs = cell(n_candidates, 1);
    all_vw = zeros(n_candidates, 2);
    all_weighted_costs = inf(n_candidates, 1);

    idx = 0;
    for li = 1:length(long_dists)
        long_d = long_dists(li);
        for lai = 1:length(lat_offsets)
            lat_d = lat_offsets(lai);
            for hi = 1:length(heading_offsets)
                h_off = heading_offsets(hi);
                idx = idx + 1;

                th = robot_pose(3);
                end_x = robot_pose(1) + long_d * cos(th) - lat_d * sin(th);
                end_y = robot_pose(2) + long_d * sin(th) + lat_d * cos(th);
                end_th = normalize_angle(th + h_off);
                end_pose = [end_x, end_y, end_th, p.lattice.target_velocity, 0];

                dist_to_end = hypot(end_x - robot_pose(1), end_y - robot_pose(2));
                T = max(1.0, min(3.0, dist_to_end / max(p.lattice.target_velocity, 0.1)));

                traj = quintic_trajectory(state, end_pose, T, p.lattice.n_samples);
                all_trajs{idx} = traj;

                dt = T / p.lattice.n_samples;
                v_eq = hypot(traj(2,1)-traj(1,1), traj(2,2)-traj(1,2)) / dt;
                w_eq = (traj(2,3) - traj(1,3)) / dt;
                all_vw(idx, :) = [v_eq, w_eq];

                [costs, infeasible] = evaluate_cost(traj, v_eq, laser_pts, target_xy, robot_pose, p, global_path);
                if ~infeasible
                    all_weighted_costs(idx) = p.costs.to_goal * costs.to_goal ...
                                            + p.costs.heading * costs.heading ...
                                            + p.costs.speed   * costs.speed ...
                                            + p.costs.obstacle * costs.obstacle ...
                                            + p.costs.path    * costs.path;
                end
            end
        end
    end

    [min_cost, best_idx] = min(all_weighted_costs);
    if isinf(min_cost)
        best_v = 0; best_w = 0; best_traj = [];
    else
        best_v = all_vw(best_idx, 1);
        best_w = all_vw(best_idx, 2);
        best_traj = all_trajs{best_idx};
    end
end
