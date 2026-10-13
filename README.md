# Guo 2020：Panda 最小二乘复合学习控制器

本目录将 `controllers/lsclrc2020_step.m` 转为 C++，控制律和学习律依据 Guo 等人的论文 *Composite learning control of robotic systems: A least squares modulated approach*，Automatica 111 (2020), 108612。文件结构、缩进、类接口及 Panda 回归模型沿用 `CLRC_2018`。

包含控制器、Panda 动力学模型、CMake 构建、一轮/十轮 Franka 实验入口、异步计算及 MATLAB 数据绘图。控制器库依赖 C++17 和 Eigen3；实机入口额外依赖 libfranka。实验指令沿用 `20261009_CLRC_2018` 的分段脉冲和二阶参考模型，不表示复现 Guo 论文中的实验轨迹。

## 文件结构

```text
CLRC_GUO/
├── include/
│   ├── LSCLRC_types.h       类型、配置、诊断量
│   ├── LSCLRC_control.h     控制器接口
│   ├── LSCLRC_async.h       无锁队列、计算线程与区间力矩对齐
│   ├── Fig3_experiment.h    实验参考模型、轮次和检查
│   ├── Franka_startup.h     机器人状态诊断与通信Reflex启动恢复
│   ├── History_window.h     历史窗口接口
│   └── Panda_regressor.h    Panda 回归矩阵接口
├── src/
│   ├── lsclrc_fig3.cpp      一轮/十轮实机回调与CSV记录
│   ├── LSCLRC_config.cpp    默认参数及配置检查
│   ├── LSCLRC_control.cpp   控制、滤波、最小二乘学习与投影
│   ├── History_window.cpp   环形缓存与历史积分
│   ├── Panda_regressor.cpp  跟踪回归矩阵
│   └── generated/
│       ├── Panda_inverse_regressor.cpp  MATLAB 数值核
│       └── Panda_parameters.cpp         与回归列配对的名义参数
├── MATLAB/
│   ├── plot_lsclrc_fig3.m   一轮数据绘图
│   ├── plot_lsclrc_repeat.m 十轮数据绘图
│   └── README.md
├── tests/                  时序、异步和Guo学习律测试
├── CMakeLists.txt
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
direct_term = Phi * zeros(7,7) * e2 = 0
direction = Kw*least_squares_term                仅保留复合学习项
W_dot = Gamma*Proj(direction)                    论文式(12)去掉直接学习项
W_next = W_hat + dt*W_dot
```

每周期先用更新前的参数计算力矩，再进行学习；`parameters()` 和诊断中的 `W_hat` 返回更新后的参数。与 `CLRC_2018` 相比，这里的控制律增加 `e1`，历史学习项改为 `Kw*Theta_use^+*epsilon`，不是仅调整增益。

当前直接学习项固定为零，不计算 `Y'*e2`，跟踪误差仍用于反馈控制。复合学习项保留原来的最小二乘调制、增益、门控和投影；`K1`、`K2`、`Gamma`、`Kw`保持原值。诊断及CSV中的 `direct_learning_term_norm`恒为零。历史学习未激活时，参数估计保持上一值，即使跟踪误差非零也不更新。

滤波与给定 MATLAB 实现一致：首个样本只初始化速度和力矩；从第二个样本起，用 `(dq_k-dq_previous)/dt` 估算刚结束区间的加速度，将逆动力学回归矩阵与上一周期施加的力矩按 `exp(-alpha*dt)` 共同滤波。因此这里保留的是 MATLAB 中的速度差分实现，未改写为论文推导中的免加速度滤波形式。

在线窗口按实际区间时长积分；超过 `tau_d` 时移除过期区间，窗边界落在旧区间内部时仅减去过期部分。不整除名义周期的窗长也按秒数处理，不再四舍五入为固定样本数。IE 检查使用当前窗口 `Theta` 的最小特征值：满窗口且 `sigma_min >= ie_threshold` 时保存历史批。`freeze_on_ie=true` 时，后续学习使用保存批及缓存伪逆，在线窗口仍更新以供诊断；关闭冻结时继续使用当前在线窗口，并在激励改善时更新保存批。

历史学习额外要求实际使用矩阵 `Theta_use` 的最小特征值不小于 `history_activation_threshold`，默认 `1e-2`。冻结时检查保存批，未冻结时检查当前在线窗口；低于门槛时历史学习项和 `pinv_rank` 为零，参数保持上一值。`require_full_history_window=true` 时，还需同时满足窗口已满和 `t >= tau_d`；关闭它只跳过时间门控，不跳过激励门控。

这是针对当前病态历史批新增的保守门控；IE 保存阈值也从 `1e-6` 提高为 `1e-2`。当前 CSV 的最大 `sigma_min` 约为 `1.34e-6`，不会达到新门槛。门槛增大可能使历史学习长期不激活，不表示全部参数收敛或实机跟踪已验证改善。若需恢复原 MATLAB 的低秩历史学习与 IE 设置，显式设置 `history_activation_threshold=0`、`ie_threshold=1e-6`。

对称信息矩阵使用特征分解计算 Moore–Penrose 伪逆。仅保留绝对值大于 `pinv_tolerance` 的特征值，对应 MATLAB `pinv(Theta,tol)` 的绝对截断阈值；没有添加正则化项。`pinv_rank` 报告实际伪逆保留的方向数；原 MATLAB 的 `information_rank` 使用另一诊断阈值 `1e-8*max(1,norm(Theta,2))`，两者可能不同。

参数投影保持 MATLAB 顺序：先将更新方向投影到参数球边界的切向或内部，再左乘 `Gamma`，最后对 Euler 更新结果执行范数限制。默认增益均为正对角矩阵；代码配置检查允许一般对称正定矩阵，不等于对任意增益或额外饱和情况作出论文收敛保证。

## 默认配置

学习增益默认值取自当前 `make_default_config('franka','tuned').lsclrc2020`；力矩幅值默认增加Panda的87/12 N·m限制。需要原MATLAB不限制幅值的调用行为时，可在构造控制器前显式将 `torque_limit` 设为正无穷。

| 配置 | 默认值 |
|---|---|
| `dt` | 0.001 s |
| `K1` | 6 × I7 |
| `K2` | diag(15,15,15,15,6,6,3.75) |
| `Gamma` | 0.3 × I64 |
| `Kw` | 2.5 × I64 |
| `alpha` | 5 |
| `tau_d` | 3 s |
| `ie_threshold` | 1e-2，IE 保存阈值 |
| `history_activation_threshold` | 1e-2，实际学习矩阵最小特征值门槛 |
| `pinv_tolerance` | 1e-9，绝对阈值 |
| `parameter_bound` | 10 |
| `initial_W_hat` | 零向量 |
| `freeze_on_ie` | true |
| `require_full_history_window` | true |
| `torque_limit` | 七轴上限 `[87,87,87,87,12,12,12]` N·m |
| `torque_rate_limit` | 各关节正无穷，默认关闭额外限速 |

可将 `initial_W_hat` 设置为 `PandaRegressor::nominal_parameters()`。此向量是模型名义值，不表示实物真值；修改初值后需要保证其范数不超过 `parameter_bound`。

需要提前使用历史时，将 `require_full_history_window` 设为 `false`，仍需达到激励门槛。需要外部历史批时，设置 `use_precomputed_batch=true`、`batch_H`、`batch_B`、`publication_time`。预计算批沿用 MATLAB 的发布时间 IE 事件；它只有在 `freeze_on_ie=true` 时成为实际学习使用的批。是否参与学习还需检查该批最小特征值和时间门控，发布事件及 `best_sigma` 中的最小正特征值不能绕过这些检查。

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

此版本每次计算求解 64×64 信息矩阵的特征值，未冻结且历史就绪时还要求伪逆。一轮入口在1 kHz回调中同步计算；十轮入口将发送与计算分开。离线测试耗时不能证明实机实时性。

## 构建与运行

在项目根目录构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

没有libfranka、只需要控制器库与离线测试时：

```bash
cmake -S . -B build_core -DBUILD_FRANKA_EXPERIMENTS=OFF
cmake --build build_core -j2
ctest --test-dir build_core --output-on-failure
```

一轮10 s和十轮100 s实机入口：

```bash
./build/lsclrc_fig3 172.16.0.2 build/lsclrc_fig3.csv
./build/lsclrc_repeat 172.16.0.2 build/lsclrc_repeat.csv
./build/lsclrc_repeat 172.16.0.2 build/lsclrc_repeat.csv 1000
```

`--help`只打印用法，不连接机器人。十轮的可选第三参数是计算频率，范围100～1000 Hz，默认1000 Hz；Franka发送保持1 kHz。默认每1 ms提交一次计算状态；计算线程未及时完成时，发送端非阻塞地沿用上一次完整的力矩、参数和诊断结果，直到收到新结果。程序不自动移动至某个预设姿态。启动时打印模式及当前/上次运动错误；仅纯通信错误导致的Reflex允许尝试一次恢复，恢复后重新读取状态并检查Idle。启动中心取首帧测量位置；每轮1～2 s施加负脉冲，5～6 s施加正脉冲，七轴幅值为 `[0.15,0.20,0.10,0.10,0.08,0.08,0.08]` rad，由 `src/lsclrc_fig3.cpp` 的 `AMPLITUDE` 修改。二阶参考模型满足 `ddq_ref=36*(qc-q_ref)-12*dq_ref`，跨切换点分段使用解析解。

轮间保留参考、滤波、历史批和参数估计。实机入口设置名义参数初值并采用力矩幅值 `[87,87,87,87,12,12,12]` N·m和变化率800 N·m/s；控制器库也默认采用相同的幅值上限；变化率仍由实机入口显式设置。Franka内置重力补偿保持开启；发送前只减一次当前Franka模型重力，`tau_cmd = tau_total - g_Franka`，再约束接口/总力矩及每次发送相对 `tau_J_d` 的变化量。辨识输入使用 `tau_J_d + g_Franka`，同步回写最终指令时也加回同一重力。漏包不会放宽发送端的1 ms变化量限制。libfranka额外低通与限速关闭，避免学习输入与已记录指令不一致；`robot.control`的`false`参数只关闭额外限速，不关闭重力补偿。

沿用参考项目的启动检查：Idle状态、静止速度不超过0.05 rad/s、关节限位余量0.10 rad（含脉冲幅值）、名义静止力矩与Franka重力的最大轴差不超过3 N·m。跟踪误差超过0.25 rad、同步回调间隔超过5 ms或异步发送间隔超过10 ms时停止。异步入口不再因结果年龄停止；仍拒绝非法或来自未来的结果时间戳。保留机器人已有碰撞保护设置。

十轮用单生产者/单消费者无锁队列传递状态和结果；计算线程跳过积压状态时，使用实际时间差，以及发送端累计总力矩的差值除以时间差。它不会补造漏掉的测量帧。计算线程使用FIFO优先级10，运行实机程序的系统需具备实时调度权限。输入队列满时本次提交跳过并计数，发送端继续使用上次结果。首个计算结果前使用预检查得到的名义静止力矩及参数初值；保持期间不重复推进参数学习，恢复计算时按实际区间更新。1000 Hz是提交目标，实际完成频率取决于计算耗时。计算线程异常仍会停止实验。一轮入口沿用参考项目的同步计算方式。

## 可变采样间隔

原四参数调用继续使用 `config.dt`。同步漏包及异步计算可显式指定刚结束区间的时长和平均力矩：

```cpp
JointVector tau = controller.compute_control(t, q, dq, reference,
    actual_dt, &average_total_torque);
```

`actual_dt`须介于 `config.dt` 和 `tau_d` 之间，并与相邻时间戳差一致；未提供平均力矩时使用上次回写的输入。滤波、速度差分、历史积分、参数Euler更新及控制器内部变化率限制均使用这个实际间隔。`tau_d`须不小于名义周期。最小二乘历史项仍是 `Kw*Theta_use^+*epsilon`，未加入CLRC-2018的梯度学习或积分分步算法。

## CSV与MATLAB

CSV在控制结束后写入，回调只填充预分配内存。实验异常也会保存已完成的样本；没有样本时保留原CSV。同一路径已有数据会在本次有样本时覆盖。

共同列包含 `t`、七轴 `qc/q/dq/qd/dqd/ddqd`、`tau_cmd/tau_total/tau_measured`、`W_hat1`～`W_hat64`、轮次、轮内时间、通信/时序，以及学习诊断。`parameter_update_norm`是实际离散参数变化量除以计算间隔，包含投影与球裁剪；`pinv_rank`、`using_frozen_history`、`least_squares_term_norm`分别记录伪逆方向数、冻结选择和最小二乘项范数。十轮额外记录计算时间、间隔、结果年龄、更新标记和区间平均总力矩。

MATLAB进入 `MATLAB/` 后运行：

```matlab
plot_lsclrc_fig3
plot_lsclrc_repeat
```

默认读取上述 `build/` CSV，输出八组PNG/PDF（含历史力矩预测误差范数）和时间加权误差统计。可以传入自定义路径，详见 [MATLAB/README.md](MATLAB/README.md)。

## 验证

CTest覆盖实际区间滤波和加速度、历史窗部分区间裁剪、漏包跨脉冲边界、轮次终点、Guo控制律、秩亏伪逆、最小二乘调制、冻结/门控、参数投影与配置拒绝。异步测试覆盖并发队列、工作线程异常、首个结果前的名义力矩保持、连续20个发送周期没有新结果时的数据保持、恢复计算后的累计力矩对齐，以及100 s、1 kHz发送/1 kHz计算的十轮离线回放；同步测试覆盖10 s含漏周期的回放。离线回放提供合成测量，只检查程序行为，不代表闭环仿真或实机跟踪性能。

当前版本已完成Release构建及4/4项测试。新增通信恢复测试使用机器人替身，验证仅纯通信Reflex恢复一次、恢复后重新检查Idle、拒绝混合错误/用户停止/运动状态，以及恢复失败时不重试。另调用实际CSV导出函数检查同步146列、异步159列的样例，列名唯一、数据行与表头对齐，包含全部MATLAB必需列。生成模型的数值表达式保留原有版本。当前验证不连接机器人；环境未安装MATLAB，绘图脚本尚未实际执行。

## TCP断连和非Idle状态

`TCP connection got interrupted`来自libfranka的TCP socket错误检查。此次保存于 `build/lsclrc_repeat.csv` 的记录结束于17.531 s：最大发送回调76.743 us、最大计算耗时1358.736 us、最大结果年龄8 ms，均未触发程序对应的停止门限。最大总力矩逐轴约 `[4.714,13.121,6.325,22.163,2.013,2.612,1.191]` N·m，低于87/12限幅。这些数据不足以确定机器人端停止原因；仅修改幅值或放宽程序时间门限不能修复已中断的TCP连接。

新版本分别报告Franka网络异常、控制异常和本程序检查失败。控制异常优先采用SDK异常日志里的最终状态；TCP/UDP异常结束旧控制后关闭坏连接，尝试一次新连接，仅调用 `readOnce()` 获取停止状态，不恢复或续跑旧实验。实验CSV之外，失败且有采样时保存 `csv_path.status.txt`，包含异常、最后接收状态及重连诊断结果。没有采样时保留已有CSV和诊断文件。

只查看当前机器人状态，不发送实验力矩或执行恢复：

```bash
./build/lsclrc_repeat --status 172.16.0.2
```

输出含 `robot_mode`、`current_errors`、`last_motion_errors` 和通信成功率。新实验启动时若模式为Reflex，且当前错误仅含 `communication_constraints_violation`（或当前错误为空、上次运动错误仅含此项），检查静止及关节余量后调用一次 `automaticErrorRecovery()`，重新读取状态再执行完整启动检查。存在碰撞、力矩不连续、用户停止、运动/引导状态或未知错误时，打印具体错误并退出；这类错误需按Desk显示处理。

通信异常仍需检查FCI会话、连接链路和实时/网络抖动，官方说明见 [Franka故障排查](https://frankarobotics.github.io/docs/troubleshooting.html)。本次代码检查和测试不连接机器人。
