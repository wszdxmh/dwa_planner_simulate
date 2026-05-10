function p = dwa_params()
% DWA规划器参数集中定义
% 所有模块均通过该结构体获取参数

    %% 机器人运动学参数
    p.robot.max_velocity      = 0.4;
    p.robot.min_velocity      = 0.05;
    p.robot.max_omega         = 2.0;
    p.robot.max_acceleration  = 2.0;
    p.robot.max_ang_accel     = 4.0;
    p.robot.target_velocity   = 0.30;
    p.robot.radius            = 0.175;    % 机器人半径 (m)
    p.robot.wheel_separation  = 0.30;    % 差速轮轮距 (m)

    %% DWA规划参数
    p.dwa.sim_time            = 2.0;
    p.dwa.dt                  = 0.2;
    p.dwa.mhz                 = 10.0;
    p.dwa.velocity_res        = 0.05;
    p.dwa.omega_res           = 0.05;

    %% 代价函数权重
    p.costs.to_goal           = 10.0;
    p.costs.speed             = 2.5;
    p.costs.obstacle          = 5.5;
    p.costs.heading           = 25.0;
    p.costs.path              = 0.4;     % 路径偏离代价 (启用)

    %% 激光雷达参数
    p.sensor.num_beams        = 360;
    p.sensor.max_range        = 8.0;
    p.sensor.min_range        = 0.17;
    p.sensor.offset_x         = 0.0628;
    p.sensor.offset_y         = 0.0;
    p.sensor.offset_theta     = pi/2;

    %% 障碍物检测参数
    p.obstacle.filter_radius  = 1.5;    % 障碍物检测半径 (m), 太大导致无谓减速
    p.obstacle.robot_radius   = 0.17;
    p.obstacle.safe_distance  = 0.5;    % 安全距离, 超过此距离障碍物代价为0
    p.obstacle.decay_factor   = 5.0;    % 指数衰减系数 (越大越陡)

    %% 仿真参数
    p.sim.max_frames          = 500;
    p.sim.control_dt          = 0.25;
    p.sim.goal_tolerance      = 0.3;

    %% 环境参数 (由 setup_environment 填充)
    p.env.resolution          = 0.05;
    p.env.world_size          = 10.0;
    p.env.origin_x            = 0.0;
    p.env.origin_y            = 0.0;

    %% waypoint 切换参数
    p.planner.waypoint_tolerance = 0.5;  % 到达waypoint范围内切换下一个
    p.planner.astar_enabled   = true;     % 启用A*全局规划
    p.planner.lookahead_dist  = 0.5;      % 纯追踪预瞄距离 (m)
    p.planner.obstacle_inflation = 0.35;   % 障碍物膨胀半径 (m)
    p.planner.inflation_weight = 10.0;     % 障碍物代价权重

    %% 仿真速度控制
    p.sim.pause_time          = 0.0025;     % 帧间暂停 (s), 越小越快
    p.sim.traj_display_stride = 1;        % 轨迹束显示间隔 (1=全部, 越大越快)

    %% Lattice规划参数
    p.lattice.num_longitudinal = 3;
    p.lattice.longitudinal_dists = [1.0, 2.0, 3.0];
    p.lattice.num_lateral = 7;
    p.lattice.lateral_range = 0.9;
    p.lattice.num_headings = 3;
    p.lattice.heading_range = pi/6;
    p.lattice.target_velocity = 0.30;
    p.lattice.n_samples = 50;
end
