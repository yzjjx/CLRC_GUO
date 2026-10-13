function plot_lsclrc_repeat(csv_file, output_dir, save_outputs)
% 十轮、每轮 10 s 实机采集后绘图，沿用单轮实验的八组图。
% 显示复合学习项、直接学习零值、参数更新率和1000 Hz计算结果保持状态。
% 用法：plot_lsclrc_repeat
%       plot_lsclrc_repeat('/实际路径/lsclrc_repeat.csv')
%       plot_lsclrc_repeat(csv_file, output_dir, false)  % 仅显示
script_dir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(csv_file)
    csv_file = fullfile(script_dir, '..', 'build', 'lsclrc_repeat.csv');
end
if nargin < 2 || isempty(output_dir)
    output_dir = fullfile(script_dir, 'figures_repeat');
end
if nargin < 3
    save_outputs = true;
end
plot_lsclrc_fig3(csv_file, output_dir, save_outputs);
end
