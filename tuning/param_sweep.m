%% param_sweep.m — DWA参数批量扫描与敏感度分析
% 对关键代价权重进行扫描，评估不同参数组合下的性能
clear; clc;
addpath(genpath(fileparts(mfilename('fullpath'))));

%% 扫描参数定义
sweep_params = {
    'to_goal',  [2, 5, 10, 15, 20, 30];
    'speed',    [1, 2, 3, 5, 8];
    'obstacle', [2, 4, 6, 8, 12];
    'heading',  [5, 10, 20, 30, 50];
};

param_names = {'costs.to_goal', 'costs.speed', 'costs.obstacle', 'costs.heading'};

%% 选择要扫描的参数
fprintf('=== DWA 参数敏感度分析 ===\n');
fprintf('扫描参数: to_goal, speed, obstacle, heading\n\n');

scene_name = 'simple';  % 使用simple场景
do_vis = false;

% 基准参数
p_base = dwa_params();
env = setup_environment(scene_name, p_base);

results = cell(length(param_names), 1);

for pi = 1:length(param_names)
    vals = sweep_params{pi, 2};
    n_vals = length(vals);

    fprintf('扫描 %s: %d 个值\n', param_names{pi}, n_vals);

    metrics_arr = struct();

    for vi = 1:n_vals
        p = p_base;
        % 动态设置参数
        switch pi
            case 1, p.costs.to_goal = vals(vi);
            case 2, p.costs.speed = vals(vi);
            case 3, p.costs.obstacle = vals(vi);
            case 4, p.costs.heading = vals(vi);
        end

        env = setup_environment(scene_name, p);
        result = run_simulation(p, env, do_vis);
        m = compute_metrics(result);

        metrics_arr(vi).value = vals(vi);
        metrics_arr(vi).success = m.success;
        metrics_arr(vi).path_length = m.path_length;
        metrics_arr(vi).time_to_goal = m.time_to_goal;
        metrics_arr(vi).goal_error = m.goal_error;
        metrics_arr(vi).smoothness = m.smoothness;
        metrics_arr(vi).min_clearance = m.min_clearance;
        metrics_arr(vi).avg_plan_time = m.avg_plan_time;

        fprintf('  值=%.1f: 成功=%d, 路径=%.1fm, 误差=%.2fm, 平滑度=%.0f, 安全距离=%.2fm\n', ...
            vals(vi), m.success, m.path_length, m.goal_error, m.smoothness, m.min_clearance);
    end

    results{pi} = metrics_arr;
end

%% 敏感度绘图
figure('Name', 'Parameter Sensitivity', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

metric_labels = {'路径长度 (m)', '目标误差 (m)', '路径平滑度', '最小安全距离 (m)', '平均规划时间 (ms)'};
metric_fields = {'path_length', 'goal_error', 'smoothness', 'min_clearance', 'avg_plan_time'};
param_labels = {'to\_goal', 'speed', 'obstacle', 'heading'};
x_label_str = {'代价权重', '代价权重', '代价权重', '代价权重'};

for mi = 1:length(metric_fields)
    subplot(2, 3, mi);
    hold on;
    colors = {'r', 'g', 'b', 'm'};
    for pi = 1:length(param_names)
        vals = [results{pi}.value];
        metric_vals = [results{pi}.(metric_fields{mi})];
        % 过滤NaN
        valid = ~isnan(metric_vals) & isfinite(metric_vals);
        if any(valid)
            plot(vals(valid), metric_vals(valid), ['o-', colors{pi}], ...
                'LineWidth', 1.5, 'DisplayName', param_labels{pi});
        end
    end
    xlabel('参数值');
    ylabel(metric_labels{mi});
    title(metric_labels{mi});
    legend('Location', 'best');
    grid on;
end

subplot(2, 3, 6);
hold on;
for pi = 1:length(param_names)
    vals = [results{pi}.value];
    success_vals = [results{pi}.success];
    plot(vals, success_vals, ['s-', colors{pi}], 'LineWidth', 1.5, 'DisplayName', param_labels{pi});
end
xlabel('参数值');
ylabel('成功率');
title('成功率');
legend('Location', 'best');
grid on;

sgtitle(sprintf('DWA参数敏感度分析 - 场景: %s', scene_name));

fprintf('\n分析完成。查看图表窗口。\n');
