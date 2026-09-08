# POP 控制的 CNC 多目标轨迹优化

本项目将 CNC 刀具轨迹生成直接构造成最优控制问题。项目借鉴论文 *Digital Thread Enabled Time Optimal Trajectory Generation for Machining Toolpaths* 中全局轨迹优化、顺序刀位线区域、直接多重打靶、CasADi 和 IPOPT 的思想，并在此基础上进行了以下扩展：

- 将控制输入从 jerk 提升为 POP；
- 运动状态包含位置、速度、加速度、jerk、snap 和 crackle；
- 对 velocity、acceleration、jerk、snap、crackle 和 POP 全部施加硬约束；
- 使用随角点距离连续变化的几何误差管道；
- 加入直线核心区域的单轴运动保持目标；
- 支持每个轴最多两个二阶机械谐振模态；
- 使用“最短时间 + epsilon 约束二阶段优化”，而不是简单的时间加权和；
- 对所有最终结果进行独立动力学、几何、频域和约束验证。

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

### Stage B：近最短时间多目标优化

Stage B 使用时间 epsilon 约束：

```text
Nmin <= N <= floor(1.02 Nmin)
```

在该整数时间范围内优化直线保持、二阶模态振动能量和模态峰值，并扫描振动权重生成 Pareto 数据。时间不作为可以被其他目标无限补偿的普通加权项。

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

优化目标使用归一化模态能量和 epigraph 峰值：

```text
E = mean(q² + (qdot/wn)²)
J_vibration = w_energy ΣE + w_peak Σq_peak²
```

FFT 不直接放入 IPOPT，而是在最终验证阶段使用 PSD 计算指定谐振频带的能量。

## 12. 修改谐振频率和启用第二模态

默认第一模态为 10 Hz：

```matlab
cfg.resonance.mode(1).enable = true;
cfg.resonance.mode(1).frequency = 10;
cfg.resonance.mode(1).zeta = 0.02;
cfg.resonance.mode(1).gain = 1.0;
```

直接修改 `frequency` 即可研究其他谐振频率。

启用第二模态：

```matlab
cfg.resonance.mode(2).enable = true;
cfg.resonance.mode(2).frequency = 35;
cfg.resonance.mode(2).zeta = 0.02;
cfg.resonance.mode(2).gain = 1.0;
```

双模态增广动力学和可选直线硬约束均已通过构建测试。

## 13. 三组默认实验

完整运行会生成：

1. `Minimum Time`：Stage A 最短整数时间解；
2. `No Resonance Suppression`：近最短时间，只优化直线保持，不惩罚谐振；
3. `10 Hz Resonance Suppression`：在相同时间、几何和运动学限制下加入 10 Hz 模态目标。

“无谐振抑制”并不表示忽略机械系统。该轨迹在优化完成后仍会输入同一个 10 Hz、阻尼比 0.02、增益 1 的模态模型，以便进行公平比较。

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

## 17. 运行方法

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

## 18. 当前默认算例结果

```text
Tmin = 0.576 s
Nmin = 576
Nvec = [158, 128, 132, 158]

Stage B 时间 = 0.587 s
相对 Tmin 增加 = 1.9097%
Knee 权重 lambda_vib = 0.60
10 Hz 模态带能量下降 = 28.1550%
```

三条轨迹均通过配置数值容差下的独立验证，中间三个角点没有停车。

## 19. 输出文件

### `output/csv`

- `trajectory_min_time.csv`：最短时间轨迹；
- `trajectory_no_resonance.csv`：无谐振惩罚轨迹；
- `trajectory_10Hz_resonance.csv`：Pareto 前沿的 knee 折中轨迹；
- `trajectory_minimum_vibration.csv`：Pareto 前沿中的最小振动轨迹；
- `comparison_table.csv`：三组方案的完整指标对比；
- `corner_analysis.csv`：四个角点的进入时间、速度、误差及高阶状态；
- `pareto_data.csv`：二阶段 Pareto 扫描数据。

默认扫描 11 个 Pareto 权重点，即 `lambda_vib = 0:0.1:1`。如需更密集的前沿，可在 `trajectory_config.m` 中调整 `cfg.optimization.pareto_weights`，例如设为 `0:0.05:1` 得到 21 个权重点。

轨迹 CSV 每一行对应一个 1 ms 插补节点，包含位置、全部运动学导数、POP、轮廓误差、允许误差、误差利用率和模态响应。

### `output/mat`

`results.mat` 保存配置、路径、最短时间、无谐振惩罚、knee 和最小振动轨迹，以及对比表、Pareto 数据、频域结果、搜索历史和测试结果。目录中还包含 Stage A、Stage B 和最近可行解检查点，便于恢复长时间计算。

### `output/figures`

自动生成 17 张科研风格 PNG，包括路径、角点、动态容差、误差利用率、全部运动学状态、POP、模态响应、PSD、约束利用率和 Pareto 前沿。其中 `01_toolpath_comparison.png` 与 `02_four_corner_zooms.png` 对比最短时间、knee 和最小振动三条代表轨迹，`17_pareto_front.png` 标出这三个代表解。

所有刀具轨迹图和角点放大图均显示动态轮廓容差带：直线核心区域允许误差为 0.05 mm，角点处放宽至 0.10 mm，并在角点前后 2 mm 范围内平滑过渡。

此外，三个代表解分别输出到独立文件夹：

- `minimum_time`：最短时间解；
- `knee`：Pareto 前沿 knee 折中解；
- `minimum_vibration`：Pareto 前沿最小振动解。

每个文件夹都包含该解单独的刀具路径、四角点放大图、轮廓误差、速度至 POP 的全部运动学状态、横向速度、模态响应、频谱、约束利用率、Pareto 位置图和 `solution_summary.csv` 指标摘要。

`resonance_comparison` 文件夹专门逐项对比“无谐振抑制”和“10 Hz 抑制（knee）”。其中包含路径、四角点、轮廓误差、位置至 POP、横向速度、10 Hz 模态响应、加速度/jerk/模态 PSD、约束利用率和关键指标对比图，同时输出 `comparison_summary.csv` 与 `suppression_reduction.csv`。
