function data = read_lsclrc_csv(csv_file, required, optional)
%READ_LSCLRC_CSV 检查绘图列，跳过末尾缺失行，保留对中间坏数据的严格检查。
% 不修改源 CSV，不插值、不用零值代替缺失数据。
if nargin < 3
    optional = {};
end
if ~isfile(csv_file)
    error('LSCLRC:MissingFile', '找不到数据文件：%s', csv_file);
end
data = readtable(csv_file, 'Delimiter', ',');
missing = setdiff(required, data.Properties.VariableNames);
if ~isempty(missing)
    error('LSCLRC:MissingColumns', 'CSV 缺少列：%s', strjoin(missing, ', '));
end
checked = [required, intersect(optional, data.Properties.VariableNames)];
for k = 1:numel(checked)
    column = data.(checked{k});
    if ~(isnumeric(column) || islogical(column)) || size(column, 2) ~= 1
        error('LSCLRC:InvalidColumn', ...
            'CSV 列 %s 未读成数值列，请检查分隔符和该列内容。文件：%s', ...
            checked{k}, csv_file);
    end
end
values = data{:, checked};

% 写出中断或尚未写完时，最后一行的缺失字段会被 readtable 填成 NaN。
% 只允许跳过连续的末尾缺失行；Inf 及中间的坏数据仍必须报错。
missing_rows = any(isnan(values), 2) & ~any(isinf(values), 2);
last_keep = find(~missing_rows, 1, 'last');
if isempty(last_keep)
    error('LSCLRC:InvalidData', ...
        'CSV 没有完整的有限数值样本，可能尚未写完。文件：%s', csv_file);
end
if last_keep < height(data)
    discarded = height(data) - last_keep;
    bad_columns = checked(any(isnan(values(last_keep + 1:end, :)), 1));
    shown_columns = bad_columns(1:min(8, numel(bad_columns)));
    warning('LSCLRC:IncompleteTail', ...
        ['CSV 末尾第 %d～%d 行存在缺失数值，已跳过 %d 行。', ...
         '涉及 %d 列（例如 %s）。请确认文件已写完。文件：%s'], ...
        last_keep + 2, height(data) + 1, discarded, numel(bad_columns), ...
        strjoin(shown_columns, ', '), csv_file);
    data = data(1:last_keep, :);
    values = values(1:last_keep, :);
end
invalid = ~isfinite(values);
if any(invalid(:))
    [rows, columns] = find(invalid);
    locations = sortrows([rows, columns], [1, 2]);
    examples = cell(1, min(5, size(locations, 1)));
    for k = 1:numel(examples)
        examples{k} = sprintf('第 %d 行/%s', ...
            locations(k, 1) + 1, checked{locations(k, 2)});
    end
    error('LSCLRC:InvalidData', ...
        'CSV 存在 %d 个 NaN/Inf：%s。请检查这些数据。文件：%s', ...
        nnz(invalid), strjoin(examples, ', '), csv_file);
end
if height(data) < 2
    error('LSCLRC:InvalidTime', 'CSV 至少需要两个完整样本。文件：%s', csv_file);
end
bad_interval = find(diff(data.t) <= 0, 1);
if ~isempty(bad_interval)
    error('LSCLRC:InvalidTime', ...
        'CSV 第 %d～%d 行的时间 t 非递增（%.12g → %.12g）。文件：%s', ...
        bad_interval + 1, bad_interval + 2, ...
        data.t(bad_interval), data.t(bad_interval + 1), csv_file);
end
end
