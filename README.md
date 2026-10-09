# Guo 2020：Panda 最小二乘复合学习控制器

本目录将 `controllers/lsclrc2020_step.m` 转为 C++，控制律和学习律依据 Guo 等人的论文 *Composite learning control of robotic systems: A least squares modulated approach*，Automatica 111 (2020), 108612。文件结构、缩进、类接口及 Panda 回归模型沿用 `CLRC_2018`。

这里只包含控制器与所需动力学模型，没有仿真入口、机器人通信代码或 CMake 文件。外部程序提供关节状态、参考信号和控制时间，并接收总关节力矩。依赖 C++17 和 Eigen3。

## 文件结构

```text
CLRC_GUO/
├── include/
│   ├── LSCLRC_types.h       类型、配置、诊断量
│   ├── LSCLRC_control.h     控制器接口
│   ├── History_window.h     历史窗口接口
│   └── Panda_regressor.h    Panda 回归矩阵接口
├── src/
│   ├── LSCLRC_config.cpp    默认参数及配置检查
│   ├── LSCLRC_control.cpp   控制、滤波、最小二乘学习与投影
│   ├── History_window.cpp   环形缓存与历史积分
│   ├── Panda_regressor.cpp  跟踪回归矩阵
│   └── generated/
│       ├── Panda_inverse_regressor.cpp  MATLAB 数值核
│       └── Panda_parameters.cpp         与回归列配对的名义参数
└── README.md
```

所有类型和类位于 `clrc_guo` 命名空间。普通控制逻辑与自动生成的模型表达式分别存放。模型保持 `CLRC_2018` 的 64 列顺序：43 个惯性基参数、7 个黏性摩擦参数、7 个库仑摩擦参数、7 个力矩偏置参数；这是当前 Panda 模型的参数化，不是所有七轴机器人的通用参数个数。

## 公式与执行顺序

以 `Y = Phi'` 对应论文记号。误差、虚拟速度及控制律为：

```text
e1 = q_ref - q
e2 = dq_ref - dq + K1*e1
v  = dq_ref + K1*e1
dv = ddq_ref + K1*(dq_ref - dq)
tau_raw = e1 + K2*e2 + Y(q,dq,v,dv)*W_hat           论文式(5)
```

历史信息和参数更新为：

```text
Theta = sum(dt * Yf' * Yf)                        论文式(9)～(10)的离散积分
y     = sum(dt * Yf' * tau_f)
epsilon = y_use - Theta_use*W_hat
least_squares_term = pinv(Theta_use, tol)*epsilon
direction = Y'*e2 + Kw*least_squares_term
W_dot = Gamma*Proj(direction)                    论文式(12)
W_next = W_hat + dt*W_dot
```

每周期先用更新前的参数计算力矩，再进行学习；`parameters()` 和诊断中的 `W_hat` 返回更新后的参数。与 `CLRC_2018` 相比，这里的控制律增加 `e1`，历史学习项改为 `Kw*Theta_use^+*epsilon`，不是仅调整增益。

滤波与给定 MATLAB 实现一致：首个样本只初始化速度和力矩；从第二个样本起，用 `(dq_k-dq_previous)/dt` 估算刚结束区间的加速度，将逆动力学回归矩阵与上一周期施加的力矩按 `exp(-alpha*dt)` 共同滤波。因此这里保留的是 MATLAB 中的速度差分实现，未改写为论文推导中的免加速度滤波形式。

在线窗口满后移除最旧样本并加入新样本。IE 检查使用当前窗口 `Theta` 的最小特征值：满窗口且 `sigma_min >= ie_threshold` 时保存历史批。`freeze_on_ie=true` 时，后续学习使用保存批及缓存伪逆，在线窗口仍更新以供诊断；关闭冻结时继续使用当前在线窗口，并在激励改善时更新保存批。

`require_full_history_window=true` 时，历史学习还需同时满足窗口已满和 `t >= tau_d`。历史学习不要求先检测到 IE：未检测到 IE 的秩亏窗口仍可通过伪逆提供部分方向的信息，但不能据此宣称全部参数收敛。

对称信息矩阵使用特征分解计算 Moore–Penrose 伪逆。仅保留绝对值大于 `pinv_tolerance` 的特征值，对应 MATLAB `pinv(Theta,tol)` 的绝对截断阈值；没有添加正则化项。`pinv_rank` 报告实际伪逆保留的方向数；原 MATLAB 的 `information_rank` 使用另一诊断阈值 `1e-8*max(1,norm(Theta,2))`，两者可能不同。

参数投影保持 MATLAB 顺序：先将更新方向投影到参数球边界的切向或内部，再左乘 `Gamma`，最后对 Euler 更新结果执行范数限制。默认增益均为正对角矩阵；代码配置检查允许一般对称正定矩阵，不等于对任意增益或额外饱和情况作出论文收敛保证。

## 默认配置

默认值取自当前 `make_default_config('franka','tuned').lsclrc2020`。

| 配置 | 默认值 |
|---|---|
| `dt` | 0.001 s |
| `K1` | 6 × I7 |
| `K2` | diag(15,15,15,15,6,6,3.75) |
| `Gamma` | 0.3 × I64 |
| `Kw` | 2.5 × I64 |
| `alpha` | 5 |
| `tau_d` | 3 s |
| `ie_threshold` | 1e-6 |
| `pinv_tolerance` | 1e-9，绝对阈值 |
| `parameter_bound` | 10 |
| `initial_W_hat` | 零向量 |
| `freeze_on_ie` | true |
| `require_full_history_window` | true |
| `torque_limit` | 各关节正无穷，与 MATLAB 默认一致 |
| `torque_rate_limit` | 各关节正无穷，默认关闭额外限速 |

可将 `initial_W_hat` 设置为 `PandaRegressor::nominal_parameters()`。此向量是模型名义值，不表示实物真值；修改初值后需要保证其范数不超过 `parameter_bound`。

需要提前使用部分历史时，将 `require_full_history_window` 设为 `false`。需要外部历史批时，设置 `use_precomputed_batch=true`、`batch_H`、`batch_B`、`publication_time`。预计算批在发布时间记录为 IE 事件，不重新要求其满秩；它只有在 `freeze_on_ie=true` 时成为实际学习使用的批，满窗口门控仍然生效。这些行为与 MATLAB 分支一致。

## 接入方式

```cpp
#include "LSCLRC_control.h"

clrc_guo::PandaRegressor dynamics;
clrc_guo::LSCLRCConfig config;
clrc_guo::PandaLSCLRC controller(dynamics, config);

// 下面的 t、q、dq、q_target 由外部程序提供。
// t 从0开始，每次调用递增config.dt；dynamics需比controller活得更久。
clrc_guo::JointReference reference;
reference.q = q_target;
reference.dq.setZero();
reference.ddq.setZero();
const clrc_guo::JointVector tau = controller.compute_control(t, q, dq, reference);
const clrc_guo::LSCLRCDiagnostic& diagnostic = controller.diagnostics();
```

镇定控制使用固定 `reference.q`，参考速度、加速度置零。轨迹跟踪由调用方提供同一轨迹对应的 `q`、`dq`、`ddq`，控制器中不生成轨迹。

输出为包含重力的总关节力矩。若执行接口自动叠加重力补偿，需要由调用方按接口定义转换指令，避免重复补偿。执行端若再次改变指令，应在本次调用后、下次调用前，用 `set_applied_torque(tau_applied)` 回写与辨识模型一致的最终总输入力矩，确保滤波力矩与速度差分对应同一区间。

`torque_rate_limit` 是沿用控制器接口形式添加的可选保护，单位 N·m/s，默认关闭；开启后第一周期以零力矩为起点，也可在首次调用前用 `set_applied_torque()` 提供初始输入。输入需满足配置的幅值限制。重新开始实验时调用 `reset()`，它同时清空滤波、历史、IE 状态和参数估计，并恢复配置中的初值。

此版本每周期求解 64×64 信息矩阵的特征值，未进行实时性能优化。

## 已完成的检查

检查了头文件依赖、命名空间、四空格缩进及文件组成；两个生成模型文件的数值表达式与 `CLRC_2018` 保持一致。用 Python 执行 C++ 数值核中的标量表达式，与 MATLAB 原函数对比 12 组输入，最大绝对差约为 `3.55e-15`。

另外用独立数值实现核对 MATLAB `lsclrc2020_step.m` 的六组、共 180 次更新，覆盖在线满窗口/部分窗口、冻结/不冻结、预计算批、秩亏伪逆、投影及幅值限幅，最大绝对差约为 `1.72e-9`。这些检查验证的是表达式与更新逻辑，没有编译或执行 C++ 二进制，也没有连接仿真/实物；接入后的 Eigen 执行结果和控制周期性能仍需在实际运行环境验证。
