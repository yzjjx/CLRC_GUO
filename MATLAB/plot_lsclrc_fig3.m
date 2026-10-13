function plot_lsclrc_fig3(csv_file, output_dir, save_outputs)
%% Guo LS-CLRC 七轴实机实验绘图
% 在 MATLAB 中打开本文件并点击 Run，或在本目录执行 plot_lsclrc_fig3。
% 默认读取 ../build/lsclrc_fig3.csv；路径相对于本文件，不依赖工作目录。
% MATLAB R2018b 或更新版本，无需额外工具箱。
% 实机真实参数未知，W_hat 仅表示参数估计，不表示真实参数误差。

script_dir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(csv_file)
    csv_file = fullfile(script_dir, '..', 'build', 'lsclrc_fig3.csv');
end
if nargin < 2 || isempty(output_dir)
    output_dir = fullfile(script_dir, 'figures');
end
if nargin < 3
    save_outputs = true;                   % false：仅显示图，不导出文件
end

%% 读取并检查数据
required = [{'t'}, column_names('qc', 7), column_names('q', 7), ...
    column_names('dq', 7), column_names('qd', 7), ...
    column_names('dqd', 7), column_names('tau_cmd', 7), ...
    column_names('tau_total', 7), column_names('tau_measured', 7), ...
    column_names('W_hat', 64), {'sigma_min', 'ie_detected', ...
    'information_rank', 'period_ms', 'callback_us', 'command_success_rate'}];
optional = {'best_sigma', 'history_update_active', 'pinv_rank', ...
    'using_frozen_history', 'least_squares_term_norm', 'control_dt_ms', ...
    'control_age_ms', 'control_us', 'control_valid', 'control_update', ...
    'prediction_error_norm', 'direct_learning_term_norm', ...
    'history_learning_term_norm', 'parameter_update_norm'};
data = read_lsclrc_csv(csv_file, required, optional);
t = data.t;
qc = data{:, column_names('qc', 7)};
q = data{:, column_names('q', 7)};
qd = data{:, column_names('qd', 7)};
dq = data{:, column_names('dq', 7)};
dqd = data{:, column_names('dqd', 7)};
tau_cmd = data{:, column_names('tau_cmd', 7)};
tau_total = data{:, column_names('tau_total', 7)};
tau_measured = data{:, column_names('tau_measured', 7)};
W_hat = data{:, column_names('W_hat', 64)};
% 导出端记录 tau_total = tau_cmd + 当前 g_Franka；用两列之差还原重力。
gravity_franka = tau_total - tau_cmd;
% 新 CSV 记录实际学习状态；旧 CSV 保留采集时的 ie_detected 状态。
history_active = data.ie_detected;
if ismember('history_update_active', data.Properties.VariableNames)
    history_active = data.history_update_active;
end
has_best_sigma = ismember('best_sigma', data.Properties.VariableNames);
has_pinv_rank = ismember('pinv_rank', data.Properties.VariableNames);
has_prediction_error = ismember('prediction_error_norm', data.Properties.VariableNames);
has_async = all(ismember({'control_dt_ms', 'control_age_ms', 'control_us', ...
    'control_valid', 'control_update'}, data.Properties.VariableNames));
has_learning_terms = all(ismember({'direct_learning_term_norm', ...
    'history_learning_term_norm'}, data.Properties.VariableNames));
has_parameter_update = ismember('parameter_update_norm', data.Properties.VariableNames);
valid_result = true(size(t));
if has_async
    valid_result = data.control_valid ~= 0;
end
norm_columns = intersect({'direct_learning_term_norm', 'history_learning_term_norm', ...
    'parameter_update_norm', 'least_squares_term_norm'}, data.Properties.VariableNames);
for k = 1:numel(norm_columns)
    if any(data.(norm_columns{k}) < 0)
        error('LSCLRC:InvalidData', '%s 必须为非负数值。', norm_columns{k});
    end
end
if has_prediction_error && any(data.prediction_error_norm < 0)
    error('LSCLRC:InvalidData', 'prediction_error_norm 必须为非负数值。');
end
e = qd - q;                              % 与控制器的误差定义一致
duration = t(end) - t(1);
joint_colors = lines(7);
figures = gobjects(7 + has_prediction_error, 1);
file_names = {'position_tracking', 'tracking_error', 'velocity_tracking', ...
    'joint_torque', 'parameter_estimates', 'learning_diagnostics', 'timing'};
if has_prediction_error
    file_names{end + 1} = 'torque_prediction_error';
else
    warning('LSCLRC:MissingPredictionError', ...
        'CSV 缺少 prediction_error_norm，跳过力矩预测误差图。');
end

%% 1. 实际位置、二阶模型参考与分段位置指令
figures(1) = new_figure('Position tracking', [1200, 900]);
for j = 1:7
    ax = subplot(4, 2, j);
    plot(t, q(:, j), '-', 'Color', joint_colors(j, :), 'LineWidth', 1.2);
    hold on;
    plot(t, qd(:, j), 'k--', 'LineWidth', 1.2);
    stairs(t, qc(:, j), ':', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 0.9);
    title(sprintf('Joint %d', j));
    format_axes(ax, t, 'q (rad)');
    if j == 1
        legend({'Measured q', 'Reference q_d', 'Command q_c'}, 'Location', 'best');
    end
end
sgtitle('Guo LS-CLRC position tracking');

%% 2. 跟踪误差（rad；不对漏包位置补造样本）
figures(2) = new_figure('Tracking error', [1200, 900]);
for j = 1:7
    ax = subplot(4, 2, j);
    plot(t, e(:, j), 'Color', joint_colors(j, :), 'LineWidth', 1.1);
    hold on;
    plot([t(1), t(end)], [0, 0], 'k:', 'HandleVisibility', 'off');
    title(sprintf('Joint %d', j));
    format_axes(ax, t, 'e (rad)');
end
sgtitle('Tracking error e = q_d - q');

%% 3. 实际速度与参考速度
figures(3) = new_figure('Velocity tracking', [1200, 900]);
for j = 1:7
    ax = subplot(4, 2, j);
    plot(t, dq(:, j), 'Color', joint_colors(j, :), 'LineWidth', 1.0);
    hold on;
    plot(t, dqd(:, j), 'k--', 'LineWidth', 1.2);
    title(sprintf('Joint %d', j));
    format_axes(ax, t, 'dq (rad/s)');
    if j == 1
        legend({'Measured dq', 'Reference dq_d'}, 'Location', 'best');
    end
end
sgtitle('Guo LS-CLRC velocity tracking');

%% 4. 力矩：总力矩、测量力矩与接口指令分开标注
% tau_total = tau_cmd + gravity；tau_measured 为发送本周期新指令前的状态。
% 保留各列原始时间戳，不把测量力矩与新指令之差解释为控制误差。
figures(4) = new_figure('Joint torque', [1200, 900]);
for j = 1:7
    ax = subplot(4, 2, j);
    plot(t, tau_total(:, j), 'Color', joint_colors(j, :), 'LineWidth', 1.0);
    hold on;
    plot(t, tau_measured(:, j), 'k--', 'LineWidth', 0.8);
    plot(t, tau_cmd(:, j), ':', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 0.8);
    plot(t, gravity_franka(:, j), '-.', 'Color', [0.1, 0.6, 0.2], 'LineWidth', 0.8);
    title(sprintf('Joint %d', j));
    format_axes(ax, t, 'Torque (N m)');
    if j == 1
        legend({'Total command', 'Measured torque', 'FCI command', ...
            'Franka gravity (total - FCI)'}, 'Location', 'best');
    end
end
sgtitle('Joint torques: FCI command = total command - Franka gravity');

%% 5. 64 维参数估计，按惯性参数和摩擦参数分组
groups = {1:15, 16:30, 31:43, 44:50, 51:57, 58:64};
group_titles = {'Inertial parameters 1-15', 'Inertial parameters 16-30', ...
    'Inertial parameters 31-43', 'Viscous friction (joints 1-7)', ...
    'Coulomb friction (joints 1-7)', 'Torque bias (joints 1-7)'};
figures(5) = new_figure('Parameter estimates', [1400, 1000]);
for k = 1:numel(groups)
    indices = groups{k};
    ax = subplot(3, 2, k);
    colororder_local = lines(numel(indices));
    set(ax, 'ColorOrder', colororder_local);
    hold(ax, 'on');
    if has_async
        stairs(t, W_hat(:, indices), 'LineWidth', 1.0);
    else
        plot(t, W_hat(:, indices), 'LineWidth', 1.0);
    end
    title(group_titles{k});
    format_axes(ax, t, 'Parameter estimate');
    legend(column_names('W_hat', indices, true), 'Interpreter', 'none', ...
        'Location', 'best', 'NumColumns', 3, 'FontSize', 7);
end
sgtitle('Estimated parameters W_hat (true parameters unavailable)');

%% 6. 当前窗与已保存历史批、学习状态及参数变化（激活不代表满足 IE）
learning_rows = 4 + has_learning_terms + has_parameter_update;
figures(6) = new_figure('Learning diagnostics', [1200, min(1050, 200 * learning_rows)]);
ax = subplot(learning_rows, 1, 1);
stairs(t, valid_diagnostic(data.sigma_min, valid_result), 'b-', 'LineWidth', 1.1);
hold on;
if has_best_sigma
    stairs(t, valid_diagnostic(data.best_sigma, valid_result), 'r--', 'LineWidth', 1.1);
    legend({'Current history window sigma min', 'Stored batch excitation metric'}, 'Location', 'best');
else
    legend({'Current history window'}, 'Location', 'best');
end
format_axes(ax, t, 'sigma min');
ax = subplot(learning_rows, 1, 2);
stairs(t, valid_diagnostic(history_active, valid_result), 'LineWidth', 1.1);
format_axes(ax, t, 'Composite active');
ylim([-0.1, 1.1]);
yticks([0, 1]);
ax = subplot(learning_rows, 1, 3);
stairs(t, valid_diagnostic(data.information_rank, valid_result), 'LineWidth', 1.1);
if has_pinv_rank
    hold on;
    stairs(t, valid_diagnostic(data.pinv_rank, valid_result), '--', 'LineWidth', 1.1);
    legend({'Reported information rank', 'Active pseudoinverse rank'}, 'Location', 'best');
end
format_axes(ax, t, 'Rank');
ylim([-1, 65]);
% IE已记录时information_rank报告保存批；关闭冻结时它可能与学习矩阵不同。
% pinv_rank使用绝对截断阈值，历史门控关闭时为零。
ax = subplot(learning_rows, 1, 4);
stairs(t, sqrt(sum(W_hat.^2, 2)), 'LineWidth', 1.1);
hold on;
stairs(t, sqrt(sum((W_hat - W_hat(1, :)).^2, 2)), '--', 'LineWidth', 1.1);
format_axes(ax, t, 'Euclidean norm');
legend({'||W_hat||_2', '||W_hat - W_hat(0)||_2'}, ...
    'Interpreter', 'none', 'Location', 'best');
if has_learning_terms
    ax = subplot(learning_rows, 1, 5);
    stairs(t, valid_diagnostic(data.history_learning_term_norm, valid_result), ...
        'b-', 'LineWidth', 1.1);
    hold on;
    stairs(t, valid_diagnostic(data.direct_learning_term_norm, valid_result), ...
        'r--', 'LineWidth', 1.1);
    format_axes(ax, t, 'Learning term norm');
    legend({'Composite term ||Kw Theta^+ epsilon||', 'Recorded direct term'}, ...
        'Interpreter', 'none', 'Location', 'best');
    if any(valid_result) && all(data.direct_learning_term_norm(valid_result) == 0)
        title('Recorded direct term is zero; composite learning retained');
    else
        title('Learning terms as recorded in this CSV');
    end
end
if has_parameter_update
    ax = subplot(learning_rows, 1, 5 + has_learning_terms);
    stairs(t, valid_diagnostic(data.parameter_update_norm, valid_result), 'LineWidth', 1.1);
    format_axes(ax, t, '||Delta W|| / dt');
    title('Last completed calculation: parameter update rate (includes projection)');
end
sgtitle('Learning diagnostics (norms mix parameter units)');

%% 7. 控制周期、回调计算耗时与通信成功率
timing_rows = 3 + 3 * has_async;
figures(7) = new_figure('Timing and communication', [1200, min(1000, 180 * timing_rows)]);
ax = subplot(timing_rows, 1, 1);
plot(t, data.period_ms, 'LineWidth', 1.0);
hold on;
plot([t(1), t(end)], [1, 1], 'k--');
format_axes(ax, t, 'Period (ms)');
legend({'Actual period (first = 0)', 'Nominal period'}, 'Location', 'best');
ax = subplot(timing_rows, 1, 2);
plot(t, data.callback_us, 'LineWidth', 1.0);
hold on;
plot([t(1), t(end)], [1000, 1000], 'r--');
format_axes(ax, t, 'Compute time (us)');
legend({'Callback body', 'Nominal 1 ms budget'}, 'Location', 'best');
ax = subplot(timing_rows, 1, 3);
plot(t, 100 * data.command_success_rate, 'LineWidth', 1.0);
format_axes(ax, t, 'Success rate (%)');
ylim([0, 101]);
if has_async
    ax = subplot(timing_rows, 1, 4);
    stairs(t, valid_diagnostic(data.control_dt_ms, valid_result), 'LineWidth', 1.0);
    hold on;
    stairs(t, valid_diagnostic(data.control_age_ms, valid_result), '--', 'LineWidth', 1.0);
    plot([t(1), t(end)], [1, 1], 'k:');
    format_axes(ax, t, 'Time (ms)');
    legend({'Calculation sample interval', 'Result age at send', '1 kHz target interval'}, ...
        'Location', 'best');
    ax = subplot(timing_rows, 1, 5);
    stairs(t, valid_diagnostic(data.control_us, valid_result), 'LineWidth', 1.0);
    hold on;
    plot([t(1), t(end)], [1000, 1000], 'r--');
    format_axes(ax, t, 'Calculation (us)');
    legend({'Last completed calculation', '1 kHz target budget'}, 'Location', 'best');
    ax = subplot(timing_rows, 1, 6);
    % 0：启动静止力矩；1：本周期保持旧结果；2：本周期收到新结果。
    result_state = double(valid_result) + double(valid_result & data.control_update ~= 0);
    stairs(t, result_state, 'LineWidth', 1.0);
    format_axes(ax, t, 'Result state');
    ylim([-0.2, 2.2]);
    yticks([0, 1, 2]);
    yticklabels({'Startup hold', 'Previous result', 'New result'});
    title('Send continues at 1 kHz while waiting for a calculation result');
end
sgtitle('Timing and communication (compute time excludes network and postprocessing)');

%% 8. 历史力矩预测误差范数（使用控制器记录的更新前参数）
% epsilon = y_use - Theta_use * W_hat，是 64 维历史残差。
% 各回归列的单位不同，其范数不能标为七轴瞬时力矩误差（N m）。
if has_prediction_error
    prediction_error_norm = data.prediction_error_norm;
    if has_async
        % 首个计算结果到达前没有预测误差；不把占位零值画成有效结果。
        prediction_error_norm(data.control_valid == 0) = NaN;
    end
    figures(8) = new_figure('Torque prediction error', [1100, 500]);
    ax = axes('Parent', figures(8));
    if has_async
        stairs(t, prediction_error_norm, 'b-', 'LineWidth', 1.1);
    else
        plot(t, prediction_error_norm, 'b-', 'LineWidth', 1.1);
    end
    hold on;
    plot([t(1), t(end)], [0, 0], 'k:', 'HandleVisibility', 'off');
    format_axes(ax, t, '||\epsilon||_2');
    legend({'||y_{use} - \Theta_{use} W_{hat}||_2'}, 'Location', 'best');
    title('Historical torque prediction error norm');
end

%% 统计与导出
% 时间加权 RMSE 使用实际时间积分，适用于记录中存在非 1 ms 周期的情况。
rmse_rad = sqrt(trapz(t, e.^2, 1) / duration).';
max_abs_error_rad = max(abs(e), [], 1).';
final_error_rad = e(end, :).';
joint = (1:7).';
tracking_summary = table(joint, rmse_rad, max_abs_error_rad, final_error_rad);
fprintf('\n数据：%s\n', csv_file);
fprintf('样本数：%d；时间区间：[%.3f, %.3f] s\n', numel(t), t(1), t(end));
disp(tracking_summary);
fprintf('最大周期：%.3f ms；周期 > 1 ms 的回调数：%d\n', ...
    max(data.period_ms(2:end)), sum(data.period_ms(2:end) > 1));
fprintf('最大回调计算耗时：%.3f us\n', max(data.callback_us));
fprintf('末次通信成功率：%.2f%%\n', 100 * data.command_success_rate(end));
if has_async
    % 只统计发送端可见的更新；计算线程中间结果可能未单独记录。
    update_count = sum(valid_result & data.control_update ~= 0);
    received_hz = sum(valid_result(2:end) & data.control_update(2:end) ~= 0) / duration;
    held = valid_result & data.control_update == 0;
    fprintf('收到新计算结果的周期数：%d；记录区间内接收更新频率：%.2f Hz（目标1000 Hz）。\n', ...
        update_count, received_hz);
    fprintf('启动保持周期数：%d；旧结果保持周期数：%d；最长连续旧结果保持：%.3f ms。\n', ...
        sum(~valid_result), sum(held), 1000 * longest_hold(t, held));
    if any(valid_result)
        fprintf('最大计算耗时：%.3f us；超过1 ms的已接收计算结果数：%d；最大结果年龄：%.3f ms。\n', ...
            max(data.control_us(valid_result)), ...
            sum(valid_result & data.control_update ~= 0 & data.control_us > 1000), ...
            max(data.control_age_ms(valid_result)));
    else
        fprintf('记录中尚未收到计算结果，发送端全程使用启动保持数据。\n');
    end
end
if has_learning_terms && any(valid_result)
    if all(data.direct_learning_term_norm(valid_result) == 0)
        fprintf('直接学习项在本次记录中恒为0；仅保留复合学习项。\n');
    else
        fprintf('本CSV包含非零直接学习项，按采集时的原值绘图。\n');
    end
end
first_history = find(valid_result & history_active ~= 0, 1);
if isempty(first_history)
    fprintf('本次记录中历史学习未激活；最大 sigma_min = %.6g。\n', max(data.sigma_min));
else
    fprintf('历史学习首次激活时间：%.3f s（激活不代表满足完整参数 IE）。\n', t(first_history));
end

if has_best_sigma
    fprintf('末次已保存历史批的激励指标：%.6g。\n', data.best_sigma(end));
end
if save_outputs
    if ~isfolder(output_dir)
        mkdir(output_dir);
    end
    for k = 1:numel(figures)
        set(figures(k), 'PaperPositionMode', 'auto');
        print(figures(k), fullfile(output_dir, [file_names{k}, '.png']), '-dpng', '-r200');
        print(figures(k), fullfile(output_dir, [file_names{k}, '.pdf']), '-dpdf', '-bestfit');
    end
    writetable(tracking_summary, fullfile(output_dir, 'tracking_summary.csv'));
    fprintf('图片和误差统计已保存至：%s\n', output_dir);
end

end  % plot_lsclrc_fig3

%% 本地辅助函数
function values = valid_diagnostic(values, valid_result)
    % 首个完成结果之前的诊断为占位数据，不画成有效的零值。
    values = double(values);
    values(~valid_result) = NaN;
end

function seconds = longest_hold(t, held)
    % 每个样本状态作用于下一记录区间；末尾只计至最后样本，不外推。
    edges = diff([false; held(1:end-1); false]);
    starts = find(edges == 1);
    ends = find(edges == -1);
    seconds = 0;
    if ~isempty(starts)
        seconds = max(t(ends) - t(starts));
    end
end

function names = column_names(prefix, count_or_indices, use_indices)
    if nargin < 3 || ~use_indices
        indices = 1:count_or_indices;
    else
        indices = count_or_indices;
    end
    names = arrayfun(@(j) sprintf('%s%d', prefix, j), indices, 'UniformOutput', false);
end

function fig = new_figure(name, dimensions)
    fig = figure('Name', name, 'NumberTitle', 'off', 'Color', 'w', ...
        'Position', [60, 60, dimensions]);
end

function format_axes(ax, t, label)
    grid(ax, 'on');
    box(ax, 'on');
    xlim(ax, [t(1), t(end)]);
    xlabel(ax, 'Time (s)');
    ylabel(ax, label);
    set(ax, 'FontName', 'Arial', 'FontSize', 10, 'LineWidth', 0.8);
end
