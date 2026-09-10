# POP 控制的 CNC 多目标轨迹优化

[![MATLAB](https://img.shields.io/badge/MATLAB-R2021b%2B-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/1resister/POP_MOO_Trajectory?style=social)](https://github.com/1resister/POP_MOO_Trajectory/stargazers)

> **English summary:** An open-source MATLAB/CasADi framework for time–vibration Pareto optimization of CNC toolpaths with sixth-order POP control, dynamic contour tolerances, mechanical resonance suppression, and reproducible validation outputs.

本项目将 CNC 刀具轨迹生成直接构造成最优控制问题。项目借鉴论文 *Digital Thread Enabled Time Optimal Trajectory Generation for Machining Toolpaths* 中全局轨迹优化、顺序刀位线区域、直接多重打靶、CasADi 和 IPOPT 的思想，并在此基础上进行了以下扩展：

- 将控制输入从 jerk 提升为 POP；
- 运动状态包含位置、速度、加速度、jerk、snap 和 crackle；
- 对 velocity、acceleration、jerk、snap、crackle 和 POP 全部施加硬约束；
- 使用随角点距离连续变化的几何误差管道；
- 加入直线核心区域的单轴运动保持目标；
- 支持每个轴最多两个二阶机械谐振模态；
- 使用“最短时间 + epsilon 约束二阶段优化”，而不是简单的时间加权和；
- 对所有最终结果进行独立动力学、几何、频域和约束验证。

![时间–振动 Pareto 前沿](output/figures/17_pareto_front.png)

## 1. 默认研究对象

默认刀具路径是边长 20 mm 的闭合二维正方形：

```text
(0, 0)
→ (20, 0)
→ (20, 20)
→ (0, 20)
→ (0, 0)
```

起点和最终终点完全静止，中间三个角点不设置停车条件。角点速度、圆角半径和实际切角轨迹均由最优控制问题自动确定。

程序的路径模块采用通用 polyline 数据结构，并提供 CSV 三轴 CL 点加载接口，方便继续扩展到任意二维或三维离散刀位点。

## 2. 单位和 POP 高阶动力学

项目统一使用：

- 长度：mm
- 时间：s
- 速度：mm/s
- 加速度：mm/s²
- jerk：mm/s³
- snap：mm/s⁴
- crackle：mm/s⁵
- POP：mm/s⁶

每个运动轴的状态为：

```text
x = [p, v, a, jerk, snap, crackle]ᵀ
```

控制输入为：

```text
u = POP = d(crackle)/dt = d⁶p/dt⁶
```

POP 是真正的优化控制输入，不是 jerk 或 snap 的别名。

## 3. 精确 ZOH 离散

插补周期固定为：

```text
Ts = 0.001 s
```

每个 1 ms 区间内 POP 保持常值。程序通过增广矩阵指数计算精确零阶保持离散模型：

```text
M = expm([A B; 0 0] Ts)
Xd(k+1) = Ad Xd(k) + Bd U(k)
```

没有使用 Euler 近似。`exact_pop_discretization.m` 还会构造基于阶乘系数的解析积分矩阵，单元测试验证解析形式和矩阵指数形式达到机器精度一致。

总时间严格满足：

```text
T = N × Ts
```

其中 `N` 是外层搜索确定的正整数，而不是 IPOPT 中的连续变量。

## 4. 变量缩放

由于位置约为 20 mm，而 POP 上限达到 `2e11 mm/s⁶`，直接使用物理量优化会导致严重数值病态。

CasADi 内部使用以下尺度进行归一化：

```text
position = 20 mm
velocity = 500 mm/s
acceleration = 10000 mm/s²
jerk = 200000 mm/s³
snap = 7e6 mm/s⁴
crackle = 2e11 mm/s⁵
POP = 2e11 mm/s⁶
```

归一化变量通常位于 `[-1,1]` 附近，最终输出再恢复为物理单位。

## 5. 动态几何误差管道

直线核心区域的允许误差为 0.05 mm，角点允许误差为 0.10 mm，过渡长度为角点前后各 2 mm。

容差函数为：

```text
xi = clip(1 - d_corner/L_transition, 0, 1)
h  = 3 xi² - 2 xi³
epsilon = epsilon_line + (epsilon_corner-epsilon_line) h
```

因此：

- 距离角点不小于 2 mm 时，`epsilon = 0.05 mm`；
- 位于角点时，`epsilon = 0.10 mm`；
- 中间区域使用 smoothstep 连续过渡，不存在误差阶跃。

角点误差从 0.05 mm 放宽到 0.10 mm，是为了让优化器利用允许的几何误差形成连续高速转向，而不是预先指定圆角、半径或角点速度。

## 6. 分段几何约束和防止走捷径

对每一条理想 CL 线段，程序建立：

- 切向进度 `tau`；
- 法向误差 `normal_error`；
- 动态容差约束；
- 切向进度单调约束；
- 切向速度非负约束；
- 有限的角点切向重叠区。

相邻线段的公共 shooting node 同时受到两条误差管道约束，因此位置及全部高阶状态连续，但不强制通过数学角点。

最终验证不会只采用 NLP 内部的法向误差，而是对每一个插补点重新计算到整条有限 polyline 的真实最短欧氏距离，以防止跳边、穿越对角线或错误的线段顺序。

## 7. Direct Multiple Shooting

每个 1 ms 区间拥有一个分段常值 POP 控制量，每个插补节点拥有完整状态变量。相邻节点通过精确离散模型连接：

```text
X(k+1) = Ad X(k) + Bd U(k)
```

所有中间节点均参与：

- 几何误差约束；
- velocity、acceleration、jerk、snap 和 crackle 硬约束；
- POP 硬约束；
- 模态动力学；
- 线段顺序和前进方向约束。

## 8. 初始可行轨迹

程序不会向 IPOPT 提供随机或全零初值。

`generate_high_order_smoothstep.m` 通过求解 12 个边界方程自动生成 11 次多项式，满足位置端点条件以及 1～5 阶导数在两端全部为零。

随后将 smoothstep 形状的 POP 投影到精确离散可达空间，使每段初值严格满足离散动力学和终端状态。该初值会在中间角点临时停车，仅用于建立第一条保守可行轨迹；正式 OCP 中没有中间停车约束。

## 9. 两阶段时间优化

### Stage A：最短整数周期

Stage A 的主目标是寻找满足全部硬约束的最小整数周期数 `Nmin`。

搜索过程包括：

1. 生成确定性的慢速可行轨迹；
2. 使用 warm start 进行时间压缩 continuation；
3. 在可行和不可行时间之间进行整数二分；
4. 分别搜索每一条线段的整数区间数 `N_i`；
5. 进行逐段减 1 和小范围区间重分配检查。

`Nvec` 中各线段的区间数不要求相等。IPOPT 每次只求解固定 `Nvec` 的连续最优控制问题。

### Stage B：时间–振动多目标优化

本项目的两个 Pareto 目标现在明确为：

```text
目标 1：minimize T，其中 T = N × Ts
目标 2：minimize J_vibration
```

搜索采用时间 epsilon 范围：

```text
Nmin <= N <= floor((1 + time_slack) Nmin)
```

对该范围内的每一个整数加工时间 `N`，程序固定相同的时间、`Nvec` 和硬约束，求解两条轨迹：

1. 无谐振抑制参考解，用于公平计算目标频带 PSD 基准；
2. 最小振动解，完整最小化二阶模态能量与峰值、X/Y 加速度 RMS 与峰值、X/Y jerk RMS 与峰值，以及目标频带 PSD 能量。

每个固定时间只保留其最小振动解作为 Pareto 候选，再执行非支配筛选。主 Pareto 图的横轴是实际加工时间 `T [s]`，纵轴是完整振动目标 `J_vibration`；直线保持不再是 Pareto 坐标，只保留为无抑制参考目标和可选硬约束。

三个代表解定义为：

- 最短时间：Pareto 前沿上 `T` 最小的振动优化解；
- 最小振动：Pareto 前沿上 `J_vibration` 最小的解；
- knee：先把时间和振动分别归一化到 `[0,1]`，再选择除两个端点外距离理想点 `(0,0)` 最近的非支配解。

逐整数时间成对优化由以下参数控制：

```matlab
cfg.optimization.time_sweep_enable = true;
cfg.optimization.time_sweep_lambda_vibration = 1.0; % 时间–振动 Pareto 必须为 1
cfg.optimization.time_sweep_resume = true; % 逐组保存并从检查点恢复
cfg.optimization.time_sweep_boundary_cpu_multiplier = 2.0; % 最短时间边界求解时限倍率
```

`time_sweep_lambda_vibration` 必须保持 `1.0`，以确保每一个固定时间的候选点确实最小化完整振动目标。若希望改变“振动”的定义，应调整 `cfg.objective.*` 下各组成项权重，而不是把该值改小。每一对解都会独立验证 50 Hz PSD 带能量必须满足配置的最低下降比例。扫描过程逐组写入 `pareto_time_sweep_checkpoint.mat`，最短时间边界点默认允许使用普通单次求解两倍的 CPU 时间。

## 10. 直线保持目标

在远离角点的区域，程序惩罚法向速度：

```text
J_straight = Ts Σ w_line (v_perp/Vmax)²
```

其中 `w_line` 在线段中心接近 1，在角点附近平滑下降到 0，使另一运动轴可以自然参与转向。

默认使用软目标：

```matlab
cfg.straightness.hard_enable = false;
```

如需在直线核心区域严格要求 `v_perp = 0`，设置：

```matlab
cfg.straightness.hard_enable = true;
```

## 11. 二阶机械谐振模型

每个轴最多支持两个二阶模态。加速度首先使用 `Amax` 归一化：

```text
r = a/Amax
qdot  = qv
qvdot = -wn²q - 2 zeta wn qv + K wn² r
wn = 2 pi f
```

POP 链和全部启用的机械模态会组成同一个连续增广系统，再统一通过矩阵指数进行精确 ZOH 离散。模态状态不会在角点处重置。

优化目标同时包含归一化模态能量、模态 epigraph 峰值，以及 X/Y 轴加速度和 jerk 的归一化 RMS 与峰值：

```text
E = mean(q² + (qdot/wn)²)
RMS_a,d  = rms(a_d/Amax),       Peak_a,d = max|a_d|/Amax
RMS_j,d  = rms(jerk_d/Jmax),    Peak_j,d = max|jerk_d|/Jmax
B(f0)    = integral PSD_q(f) df,  f in [f0-bandwidth, f0+bandwidth]

J_vibration = w_energy ΣE + w_modal_peak Σq_peak²
            + w_a_rms Σ RMS_a,d + w_a_peak Σ Peak_a,d
            + w_j_rms Σ RMS_j,d + w_j_peak Σ Peak_j,d
            + w_band B(f0)/B_no_suppression(f0)
其中 d ∈ {X,Y}
```

默认 `w_energy = 1.0`，加速度 RMS 和 jerk RMS 权重为 `0.2`，模态峰值、加速度峰值和 jerk 峰值权重为 `0.1`，显式谐振频带权重 `w_band = 1.0`。频带权重可通过 `cfg.objective.resonance_band_weight` 调整。所有轴向指标都除以对应硬约束上限，频带能量则除以无抑制基准，因此可作为无量纲量共同参与优化。

当 `cfg.objective.resonance_band_hard_enable = true` 时，每个振动感知 Pareto 解还必须满足：

```text
B(f0) <= (1-r_min) B_no_suppression(f0)
```

其中 `r_min = cfg.objective.resonance_band_min_reduction`，默认值 `0.05` 表示实际 Welch PSD 带积分至少下降 5%。项目将 Welch PSD 的 Hamming 窗频率投影等价写入 IPOPT，并在独立验证阶段再次计算实际 PSD；任一检查未通过，该解都不会标记为可行。

## 12. 修改谐振频率和启用第二模态

当前默认第一模态为 50 Hz：

```matlab
cfg.resonance.mode(1).enable = true;
cfg.resonance.mode(1).frequency = 50;
cfg.resonance.mode(1).zeta = 0.02;
cfg.resonance.mode(1).gain = 1.0;
```

直接修改 `frequency` 即可研究其他谐振频率。

当前默认配置已经是 50 Hz，直接运行即可。若希望把 50 Hz 结果保存到独立目录，避免覆盖其他频率的历史结果，可使用：

```matlab
cfg = trajectory_config();
cfg.resonance.mode(1).frequency = 50;
cfg.frequency.bandwidth = 1.0;              % 统计 49～51 Hz
cfg.frequency.maximum_plot_frequency = 100;
cfg.objective.resonance_band_weight = 1.0;  % 可调频带目标权重
cfg.objective.resonance_band_hard_enable = true;
cfg.objective.resonance_band_min_reduction = 0.05; % 至少下降 5%

output50 = fullfile(cfg.project_root,'output_50Hz');
cfg.output.figures = fullfile(output50,'figures');
cfg.output.csv = fullfile(output50,'csv');
cfg.output.mat = fullfile(output50,'mat');

results50 = run_square_pop_moo(cfg);
```

改变谐振频率后应完整调用 `run_square_pop_moo(cfg)`，不能复用采用其他频率动力学模型的 Stage A 检查点。图题、PSD 频率标线、比较表和轨迹文件名会自动使用当前配置频率；上述算例的抑制轨迹文件名为 `trajectory_50Hz_resonance.csv`。

启用第二模态：

```matlab
cfg.resonance.mode(2).enable = true;
cfg.resonance.mode(2).frequency = 35;
cfg.resonance.mode(2).zeta = 0.02;
cfg.resonance.mode(2).gain = 1.0;
```

双模态增广动力学和可选直线硬约束均已通过构建测试。

## 13. 三组 Pareto 代表解与对照

完整运行会生成：

1. `Minimum Time Pareto`：时间–振动前沿上加工时间最短的振动优化解；
2. `Time-Vibration Knee`：归一化时间–振动空间中距离理想点最近的内部非支配解；
3. `Minimum Vibration`：时间–振动前沿上完整振动目标最小的解。

此外，程序会为 knee 所在的同一加工时间输出 `No Resonance Suppression` 对照解，并为每一个 Pareto 时间点保留一组有抑制/无抑制配对结果。

“无谐振抑制”并不表示忽略机械系统。该轨迹在优化完成后仍会输入同一个 50 Hz、阻尼比 0.02、增益 1 的模态模型，以便进行公平比较。

## 14. 独立验证

`validate_solution.m` 会独立重新计算：

- 精确离散动力学残差；
- 起点和终点全部状态；
- 首尾 POP；
- velocity、acceleration、jerk、snap、crackle 和 POP 峰值；
- 整体真实轮廓误差；
- 直线核心最大误差；
- 各角点最大弓高；
- 采样周期一致性；
- 分段顺序、进度单调性和反向运动；
- NaN 和 Inf。

只有求解器状态满足要求并且独立验证通过时，轨迹才会被标记为可行。若 IPOPT 达到 CPU 时间限制，但当前迭代点通过全部验证，则状态记为 `solver_timeout_but_feasible`，不会直接丢弃。

## 15. FFT 和 PSD

验证阶段对 X/Y 加速度、X/Y jerk 和 X/Y 模态响应计算 PSD。采样频率固定为 1000 Hz，并计算谐振频率 ±1 Hz 范围内的带能量。

如果 Signal Processing Toolbox 可用，程序使用 `pwelch`；否则自动使用基于 MATLAB `fft` 的单边 PSD 估计器。

## 16. 软件依赖

- MATLAB R2021b 或更高版本；
- CasADi MATLAB interface；
- IPOPT；
- Signal Processing Toolbox 可选。

启动时会检查 `casadi.Opti`。如果 CasADi 不存在，程序会明确报错：

```text
CasADi MATLAB interface is required.
```

程序不会静默切换到其他优化算法。

## 17. 参数设置表

项目提供一份完整的中文参数设置表，共 91 项配置，覆盖项目路径、插补与刀具路径、几何容差、运动学硬约束、端点边界、归一化尺度、Stage A、Pareto 扫描、谐振模型、振动目标、IPOPT、独立验证以及绘图输出。表中不仅给出当前值和单位，还说明了参数调大、调小、启用或禁用后的主要影响。

- `output/csv/parameter_settings.xlsx`：格式化的中文查看版，带筛选、冻结标题行、参数分组和修改建议；
- `output/csv/parameter_settings.csv`：机器可读版，每次完整运行在汇总阶段自动写出；
- `build_parameter_settings_table.m`：根据传入的 `cfg` 构造参数表；
- `write_parameter_settings.m`：不运行轨迹优化，单独刷新参数 CSV。

最常修改的参数如下：

| 目的 | MATLAB 参数 | 默认值 | 中文说明 |
| --- | --- | ---: | --- |
| 修改插补周期 | `cfg.Ts` | `0.001 s` | 决定离散时间分辨率和问题规模；必须与实际控制周期一致。 |
| 修改正方形尺寸 | `cfg.path.square_side` | `20 mm` | 内置正方形路径的边长。 |
| 修改直线容差 | `cfg.geometry.line_tolerance` | `0.05 mm` | 直线核心区域允许的最大轮廓误差。 |
| 修改角点容差 | `cfg.geometry.corner_tolerance` | `0.10 mm` | 角点允许的最大轮廓误差；增大后通常可提高角点速度。 |
| 修改过渡长度 | `cfg.geometry.transition_length` | `2 mm` | 动态容差在角点前后平滑变化的距离。 |
| 修改运动上限 | `cfg.limits.Vmax` ～ `POPMax` | 见参数表 | 分别限制速度、加速度、jerk、snap、crackle 和 POP。 |
| 修改近最短时间范围 | `cfg.optimization.time_slack` | `0.02` | Stage B 最多允许比严格最短时间增加 2%。 |
| 增加 Pareto 解 | `cfg.optimization.time_slack` | `0.02` | 增大时间上限可加入更多整数时间点；`Ts=1 ms` 时每增加 1 ms 就多一个候选点。 |
| 启用逐时间成对扫描 | `cfg.optimization.time_sweep_enable` | `true` | 对每个整数时间都生成无抑制和有抑制解。 |
| 固定时间下最小化振动 | `cfg.optimization.time_sweep_lambda_vibration` | `1.0` | 时间–振动 Pareto 必须保持 `1.0`；振动内部权重由 `cfg.objective.*` 调节。 |
| 修改目标谐振频率 | `cfg.resonance.mode(1).frequency` | `50 Hz` | 同时决定第一模态固有频率和默认 PSD 抑制中心。 |
| 修改 PSD 积分带宽 | `cfg.frequency.bandwidth` | `1 Hz` | 半带宽；默认统计 49～51 Hz。 |
| 调整频带抑振强度 | `cfg.objective.resonance_band_weight` | `1.0` | 越大越强调降低目标频带 PSD 能量。 |
| 保证最低 PSD 降幅 | `cfg.objective.resonance_band_min_reduction` | `0.05` | 硬约束开启时要求至少下降 5%。 |
| 调整 X/Y 加速度指标 | `acceleration_rms_weight`、`acceleration_peak_weight` | `0.2`、`0.1` | 控制加速度 RMS 与峰值在振动目标中的比重。 |
| 调整 X/Y jerk 指标 | `jerk_rms_weight`、`jerk_peak_weight` | `0.2`、`0.1` | 控制 jerk RMS 与峰值在振动目标中的比重。 |
| 延长单次求解时间 | `cfg.solver.ipopt.max_cpu_time` | `180 s` | 遇到 `Maximum_CpuTime_Exceeded` 时可适当增大；逐时间扫描会从最后迭代点自动延时重试一次。 |
| 调整超时重试倍率 | `cfg.optimization.time_sweep_boundary_cpu_multiplier` | `2.0` | 同时用于最短时间边界和逐时间扫描的自动重试；较慢电脑可设为 `3～4`。 |
| 修改图片清晰度 | `cfg.plot.resolution` | `180 dpi` | 只影响 PNG 导出清晰度和文件大小。 |

归一化尺度 `cfg.scale.*` 默认由路径尺寸和运动学上限自动派生，不建议单独修改。验证容差 `cfg.validation.*` 用于独立验收，不能为了让失败解通过而随意放宽。

修改参数并立即导出对应的中文 CSV 参数表：

```matlab
cfg = trajectory_config();
cfg.resonance.mode(1).frequency = 50;
cfg.optimization.time_slack = 0.05; % 扩大时间范围，得到更多时间–振动点
cfg.objective.resonance_band_weight = 2.0;
cfg.objective.resonance_band_min_reduction = 0.10;
parameter_settings = write_parameter_settings(cfg);
```

之后再运行优化：

```matlab
results = run_square_pop_moo(cfg);
```

## 18. 运行方法

在 MATLAB 中执行：

```matlab
cd('E:/Desktop/codex_test4/POP_MOO_Trajectory')
addpath(genpath(pwd))
results = run_square_pop_moo();
```

如需修改参数：

```matlab
cfg = trajectory_config();
cfg.resonance.mode(1).frequency = 12;
cfg.straightness.hard_enable = true;
results = run_square_pop_moo(cfg);
```

由于每一个整数候选时间都对应一次独立的 warm-started NLP，完整 Stage A 和 Pareto 扫描可能运行数分钟。

如果 Stage A 已经完成并生成 `output/mat/stage_a_latest_feasible.mat`，但后续汇总或绘图阶段中断，可以跳过 Stage A 并从最近可行解恢复：

```matlab
results = resume_square_pop_moo();
```

恢复入口会先用当前配置重新独立验证检查点；路径或参数不兼容时会拒绝继续。

如果基础 Pareto 前沿已经保存在 `output/mat/results.mat`，只需继续或重新生成逐加工时间的成对振动优化、PSD 和 FFT 输出，可执行：

```matlab
results = resume_pareto_time_sweep();
```

该入口会读取 `pareto_time_sweep_checkpoint.mat` 并自动跳过已经完成且配置兼容的时间点。

## 19. 当前默认算例结果

```text
目标谐振频率 = 50 Hz
Tmin = 0.576 s
Nmin = 576
Nvec = [158, 128, 132, 158]

Pareto 时间范围 = 0.576～0.587 s
时间上限相对 Tmin 增加 = 1.9097%
Pareto 目标 = 加工时间 T 与完整振动目标 J_vibration
最短时间 Pareto 解：T = 0.576 s，J_vibration = 1.4286784
时间–振动 knee：N = 577，T = 0.577 s
knee 相对 Tmin 增加 = 0.1736%
knee J_vibration = 0.8180376
knee 50 Hz 带能量 = 9.3457267e-8
knee 50 Hz 带能量下降 = 99.6425%
最小振动解：N = 587，T = 0.587 s，J_vibration = 0.7855045
最小振动解 50 Hz 带能量 = 1.8820348e-8
最小振动解 50 Hz 带能量下降 = 99.8483%

逐时间成对扫描 = N 576～587，共 12 组
12 组无抑制解均可行 = 是
12 组有抑制解均可行 = 是
严格最短时间 0.576 s 带能量下降 = 39.971%
其余 0.577～0.587 s 带能量下降 = 99.642%～99.848%
```

三条代表轨迹和 12 组成对时间扫描轨迹均通过配置数值容差下的独立验证，中间三个角点没有停车。当前显式 50 Hz 频带目标在优化器内直接约束 Welch PSD 带积分；时间–振动前沿见 `pareto_data.csv`，每个时间点的频带指标见 `pareto_resonance_band_metrics.csv`，完整成对结果见 `pareto_time_sweep_summary.csv`。

## 20. 输出文件

### `output/csv`

- `trajectory_min_time.csv`：最短时间轨迹；
- `trajectory_no_resonance.csv`：无谐振惩罚轨迹；
- `trajectory_<频率>Hz_resonance.csv`：Pareto 前沿的 knee 折中轨迹，例如 50 Hz 对应 `trajectory_50Hz_resonance.csv`；
- `trajectory_minimum_vibration.csv`：Pareto 前沿中的最小振动轨迹；
- `comparison_table.csv`：三组方案的完整指标对比；
- `corner_analysis.csv`：四个角点的进入时间、速度、误差及高阶状态；
- `parameter_settings.xlsx`：91 项默认配置的格式化中文参数设置表；
- `parameter_settings.csv`：当前运行配置的中文参数快照，可用 `write_parameter_settings(cfg)` 单独刷新；
- `pareto_data.csv`：时间–振动 Pareto 主表，包含 `N`、实际时间、完整振动目标、非支配标记、归一化坐标、理想点距离以及最短时间/knee/最小振动标记。
- `pareto_resonance_band_metrics.csv`：每个时间–振动候选解在当前谐振频率 ±带宽内的 X/Y/总带能量，以及相对同一时间无抑制解的实际下降率。
- `pareto_time_sweep_summary.csv`：从 `Nmin` 到 `Nmax` 的每个成功整数加工时间对应的无抑制/有抑制成对结果、目标频带能量、降幅、轮廓误差和可行性状态。
- `pareto_time_sweep_skipped.csv`：自动延时重试和频带约束渐进求解后仍未获得可行成对解的离散时间点及失败原因；这些点不会混入 Pareto 前沿。

默认尝试在 `Nmin:Nmax` 的每个整数时间上生成一个最小振动候选点。当前 `Ts=1 ms`、`time_slack=0.02`，最多尝试 `N=576:587` 共 12 个候选点。若某个时间点在自动延时重试和频带约束渐进求解后仍不可行，程序会记录并剔除该点，继续用其余可行点构建 Pareto 前沿。需要更多 Pareto 点时应增大 `cfg.optimization.time_slack`；`cfg.optimization.pareto_weights` 仅为旧版直线–振动扫描保留，主流程不再使用。

轨迹 CSV 每一行对应一个 1 ms 插补节点，包含位置、全部运动学导数、POP、轮廓误差、允许误差、误差利用率和模态响应。

### `output/mat`

`results.mat` 保存配置、路径、Stage A 严格最短时间参考、时间–振动 Pareto 的最短时间/knee/最小振动轨迹、12 组成对时间扫描轨迹、非支配标志和代表解索引，以及对比表、频域结果、搜索历史和测试结果。目录中还包含 Stage A、Stage B、最近可行解和 `pareto_time_sweep_checkpoint.mat` 检查点，便于恢复长时间计算。

### `output/figures`

自动生成 17 张科研风格 PNG，包括路径、角点、动态容差、误差利用率、全部运动学状态、POP、模态响应、PSD、约束利用率和 Pareto 前沿。其中 `01_toolpath_comparison.png` 与 `02_four_corner_zooms.png` 对比最短时间、knee 和最小振动三条代表轨迹；`17_pareto_front.png` 的横轴为加工时间、纵轴为完整振动目标，并标出这三个代表解。

所有刀具轨迹图和角点放大图均显示动态轮廓容差带：直线核心区域允许误差为 0.05 mm，角点处放宽至 0.10 mm，并在角点前后 2 mm 范围内平滑过渡。

此外，三个代表解分别输出到独立文件夹：

- `minimum_time`：最短时间解；
- `knee`：Pareto 前沿 knee 折中解；
- `minimum_vibration`：Pareto 前沿最小振动解。

每个文件夹都包含该解单独的刀具路径、四角点放大图、轮廓误差、速度至 POP 的全部运动学状态、横向速度、模态响应、频谱、约束利用率、Pareto 位置图和 `solution_summary.csv` 指标摘要。

`resonance_comparison` 文件夹专门逐项对比“无谐振抑制”和目标频率抑制（knee）。其中包含路径、四角点、轮廓误差、位置至 POP、横向速度、模态响应、加速度/jerk/模态 PSD、约束利用率、关键指标、振动目标七项贡献对比图、X/Y 轴加速度与 jerk 的 RMS/峰值对比图，以及全 Pareto 前沿的真实谐振带能量图，共 21 张 PNG；同时输出 `comparison_summary.csv` 与 `suppression_reduction.csv`。

`pareto_time_sweep` 文件夹保存所有整数加工时间上的成对振动优化。每个 `N_xxxx_T_xxxs` 子文件夹包含：

- `01_psd_comparison.png`：X/Y 加速度、jerk 和模态响应的无抑制/有抑制 PSD 对比；
- `02_fft_spectrum_comparison.png`：同一组六个信号的单边加窗 FFT 幅值频谱对比；
- `trajectory_no_suppression.csv` 与 `trajectory_resonance_suppression.csv`：该时间点的两条完整轨迹；
- `comparison_summary.csv`：轨迹、带能量、降幅、轮廓误差和验证结果；
- `frequency_summary.csv`：六个信号在目标频率处的 PSD、FFT 幅值及抑制前后降幅。

文件夹根目录的 `00_pareto_time_sweep_summary.png` 汇总所有加工时间的无抑制/有抑制带能量和实际下降率。由于默认范围为每个 1 ms 整数时间点都求一对 NLP，完整运行时间会比原来的单时间 Pareto 扫描明显增加。

## 21. 开源许可证

本项目采用 [MIT License](LICENSE)。欢迎在研究与工程项目中使用、修改和分享；如果本项目对你有帮助，也欢迎引用、提交 Issue 或点亮 Star。
