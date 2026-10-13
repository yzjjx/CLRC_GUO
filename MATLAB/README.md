# Guo LS-CLRC 实验绘图

MATLAB R2018b 或更新版本，无需额外工具箱。进入本目录后运行下列函数，也可以打开函数文件点击 Run：

```matlab
plot_lsclrc_fig3      % build/lsclrc_fig3.csv，一轮10 s
plot_lsclrc_repeat    % build/lsclrc_repeat.csv，十轮共100 s
```

默认输入路径相对于函数文件，不依赖 MATLAB 当前工作目录。默认分别导出至 `MATLAB/figures/` 和 `MATLAB/figures_repeat/`，包含八组 PNG（200 dpi）、PDF 和 `tracking_summary.csv`。可指定输入、输出及是否导出：

```matlab
plot_lsclrc_fig3('/实际路径/lsclrc_fig3.csv', 'figures_one', true)
plot_lsclrc_repeat('/实际路径/lsclrc_repeat.csv', 'figures_ten', false)
```

八组图：位置跟踪、位置误差、速度跟踪、关节力矩、64维参数估计、学习诊断、时序与通信、力矩预测误差。十轮使用同一绘图函数，横轴覆盖全部采集时间，轮间不重置参数。异步参数与学习诊断采用阶梯线，显示计算结果之间的保持行为；首个结果前保留名义参数曲线，隐藏尚未有效的学习诊断。

学习诊断图增加 `history_learning_term_norm`（复合学习项）和 `direct_learning_term_norm`（直接学习项），新采集数据的直接项恒为零，旧数据的非零直接项仍按原值显示。`parameter_update_norm`显示最近完成计算的实际参数变化量除以计算区间，包含参数投影；异步保持期间显示上次计算值，不将它误作本发送周期发生的新更新。旧CSV缺少这些可选列时跳过相应子图。

时序图显示发送周期、回调耗时、通信成功率、实际计算采样间隔、结果年龄、计算耗时，以及“启动保持 / 旧结果保持 / 收到新结果”三种状态。1 ms / 1000 us参考线对应1000 Hz目标，仅用于比较，不是结果过期停止门限。终端输出接收更新频率、保持周期数、最长连续保持时间和超过1 ms的已接收计算结果数；接收频率只计发送端看到的更新，不能推断未单独记录的中间计算结果。保持时长按实际记录时间计算，末尾只计至最后样本。

力矩预测误差图读取 `prediction_error_norm`，绘制历史残差 `epsilon = y_use - Theta_use * W_hat` 的二范数，导出为 `torque_prediction_error.png/.pdf`。该残差使用本次学习更新前的参数，当前窗或冻结批与控制器的选择一致；64维分量单位不同，范数不标为 N·m。十轮图按发送时间显示最新结果，计算结果之间用阶梯线保持，`control_valid=0` 时不绘制。旧CSV缺少此列时给出警告，仍生成原有七组图。

误差定义为 `e = qd - q`。RMSE按实际采样时间积分，另导出最大绝对误差和末次误差，单位为rad。脚本不重采样。

读取 CSV 时会自动跳过连续的末尾缺失行，并告警给出行号、数量及涉及的列；这适用于文件尚未写完或写出中断导致的末尾缺字段。不会修改原 CSV、插值或用零填充。统计和横轴只覆盖剩余的完整样本；应等终端显示 `Saved ... samples` 后再绘图。少于两个完整样本、时间非递增、必需列缺失、非数值列、中间出现 NaN 或任何 Inf 仍会报错，并提供具体行号或列名。

可在 MATLAB/ 中运行 `runtests('test_read_lsclrc_csv.m')`，检查完整输入、末尾缺行、中间 NaN、Inf、缺列和非递增时间的处理。

`tau_total`是含重力的总力矩指令，`tau_cmd`是发送至Franka的接口力矩，`tau_measured`是发送本次新指令前的测量。力矩图还显示 `g_Franka = tau_total - tau_cmd`，由导出端的转换关系还原当前Franka重力；内置重力补偿保持开启，发送只减一次重力，辨识输入仍加回重力。这条曲线不是额外发送的补偿指令。参数包含不同单位的分量，范数仅用于观察数值变化；真实参数未知，参数曲线稳定不能独立证明参数收敛。

直接学习项已禁用，新采集CSV中的 `direct_learning_term_norm`恒为零；参数仅由通过门控的复合学习项更新。旧CSV仍按其中实际记录的数值绘制。

`history_update_active`表示历史项通过时间门控、激励门控且存在非零信息。当前默认要求实际学习矩阵的最小特征值达到 `history_activation_threshold=1e-2`，IE保存阈值也为 `1e-2`；冻结后检查保存批，未冻结时检查当前窗。旧CSV或显式将激励门槛设为零时，仍可能有低秩历史参与学习，激活本身不能证明完整IE。`sigma_min`来自当前在线窗口。`best_sigma`在在线IE模式表示保存时的最小特征值；预计算批模式下，沿用原实现，表示大于IE阈值的最小正特征值，没有该方向时为零；此正特征值不能替代激励门控所需的最小特征值。

`information_rank`在IE已记录时报告保存批的秩，否则在历史就绪时报告当前窗的秩；其相对阈值为`1e-8*max(1,norm(Theta,2))`。`pinv_rank`报告实际历史学习矩阵按绝对`pinv_tolerance`保留的方向数，门控关闭时为零。关闭冻结或采用不同阈值时，两者可能不同。

十轮CSV中，`control_t`是计算采用的状态时间，`control_dt_ms`是实际计算采样间隔，`control_age_ms`是发送时结果延迟，`control_us`是计算线程耗时。`callback_us`仅表示Franka发送回调主体耗时。首个结果前`control_valid=0`，参数为名义初值；两次计算结果之间参数记录保持上一值。`tau_learning1`至`tau_learning7`是计算采样区间内的平均总力矩。

脚本沿用参考项目的CSV列名，补充最小二乘诊断；不包含仿真器或实机真实参数。
