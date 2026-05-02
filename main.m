%% main.m — DWA局部规划器仿真平台入口
% 先运行A*全局路径规划，再使用DWA局部规划器跟踪全局路径
clear; clc; close all;

% 添加所有子目录到路径
addpath(genpath(fileparts(mfilename('fullpath'))));

%% 配置
p = dwa_params();

% 选择场景: 'empty' | 'simple' | 'cluttered' | 'corridor' | 'dynamic'
scene_name = 'cluttered';

%% 构建环境和全局路径
fprintf('=== 场景: %s ===\n', scene_name);
env = setup_environment(scene_name, p);

%% 运行仿真
do_vis = true;  % 是否可视化
if isappdata(0, 'dwa_fig_closed'), rmappdata(0, 'dwa_fig_closed'); end
result = run_simulation(p, env, do_vis);

%% 后处理评估
fprintf('\n=== 仿真结果 ===\n');
if result.success, status_str = '是'; else, status_str = '否'; end
fprintf('是否成功: %s\n', status_str);
fprintf('最终距离目标: %.3f m\n', result.log.dist_to_goal(end));
fprintf('仿真帧数: %d\n', result.n_frames);
fprintf('平均规划时间: %.1f ms\n', mean(result.log.plan_time(1:result.n_frames)) * 1000);
fprintf('路径长度: %.2f m\n', sum(sqrt(diff(result.log.pose(1:result.n_frames,1)).^2 + diff(result.log.pose(1:result.n_frames,2)).^2)));

%% 汇总绘图
plot_summary(result);
