下面是一份可以直接交给 Claude 执行的 **MATLAB 代码任务书（按师兄最新要求）**。重点落实你刚补充的三点：


1. **删掉 “add small yaw to break symmetry” 注释**，yaw 初值与全过程固定为 0，不用它“加 rank”；
2. **arm_ik 允许用 Robotics System Toolbox**；
3. **platform-only 允许 roll/pitch（yaw=0）**，并且 roll/pitch 主要通过 **滑轨锚点高度 $h_k$** 的重构来实现；平台-only 的 IK 要有一个明确的优化目标（质心稳定性或索力平滑性）。


# A. 总体目标与系统假设


## A1. 目标


在 2m 立方框架内，对给定目标末端点/轨迹 $p^{ee}_{O,\text{target}}(t)$，完成三种模式的可达性判定与规划：


- **Platform-only**：仅由柔索系统/滑轨锚点重构驱动动平台（平台 yaw=0，roll/pitch 可变），机械臂保持固定姿态 $q_a=q_{a,\text{fix}}$。输出平台位姿 $q_p(t)$、锚点高度 $h(t)$、以及相应的索长 $l(t)$ 与（可选）索力 $T(t)$。
- **Arm-only**：平台位姿固定 $q_p=q_{p,\text{fix}}$，机械臂按普通串联臂 IK 求 $q_a(t)$ 达到目标点。
- **Cooperative**：平台 + 机械臂协同。需要分层/优化框架（可引入 DLS），并能在目标点处综合考虑“平台动得少/臂动得少/索力平滑/锚点重构代价”。

## A2. 无重力与微扰


- 默认 **无重力**：$g=0$。
- 允许一个 **重力方向微扰**（仅 z 向）：用来打破上下索完全对称，从而产生上下索索力差异。
建议用一个参数：

$$
f_{z,\epsilon}\in[0.5, 5]\text{ N} \quad(\text{量级先小后大试})
$$

外载荷扳手（平台任务空间）记为 $W$，仅含 $F_z$（以及可选极小 $M_x,M_y$）。

## A3. 平台 yaw 不纳入平台自由度


- 平台状态不含 yaw，**全过程 yaw=0**，由机械臂更方便地承担 z 轴旋转（若未来有姿态任务再在 cooperative 中做）。


# B. 关键建模约定（务必统一）


## B1. 坐标系与姿态


- 基座系 $\{O\}$：框架底面中心为原点，z 竖直向上。
- 平台系 $\{p\}$：平台几何中心为原点，$Z_p$ 指向平台上表面法向。
- 欧拉角顺序：代码中既定为 **Z→Y→X**（即 yaw–pitch–roll）。
但 **平台 yaw 固定为 0**，平台旋转只保留：

$$
R_{Op}(\phi,\theta)=R_y(\theta)R_x(\phi)
$$

其中 $\phi=\text{roll},\theta=\text{pitch}$。

## B2. 新平台状态向量（5DOF）


$$
q_p=\begin{bmatrix}x&y&z&\phi&\theta\end{bmatrix}^T
$$


yaw 不出现在任何求解变量里（不是“初值设 0”那种软处理，而是**变量维度里就没有**）。



# C. 代码重构任务清单（文件级）


## C1. 清理错误注释与旧逻辑


- 在 `HCDR_config.m` 或任何相关文件中，**删除**：


`add small yaw to break symmetry`


任何暗示“用 yaw 破对称提升 rank”的注释/逻辑分支
- 初始化 yaw 相关字段仍可保留为 0（便于与旧接口兼容），但 **不得参与求解**。


## C2. 新增/重写模块建议（建议开新分支）


### 1) HCDR_config_v2.m


新增配置字段（最少这些）：


- `config.platform.dof = 5; config.platform.yaw_fixed = 0;`
- `config.microg.enabled = true; config.microg.fz_eps = ...;`
- `config.platform_only.objective = 'tension_smooth' or 'com_stable' or 'hybrid';`
- 约束：


`config.limits.platform_xyz`、`config.limits.platform_rp`（roll/pitch 上限）


`config.limits.slider_h_minmax`（每条滑轨高度范围）


`config.limits.tension_minmax`
- 优化权重：


`w_com, w_rp, w_h, w_Tsmooth, w_Tpair, w_step`


### 2) HCDR_kinematics_5d.m（平台+索几何，输出 5D 结构矩阵）


必须提供以下接口（函数名可改，但接口要清晰）：


#### (a) 平台旋转（唯一入口）


```matlab
function R = R_platform(roll, pitch)
R = Ry(pitch) * Rx(roll); % yaw fixed 0
end

```

#### (b) 给定 $q_p,h$ 计算索几何


输入：


- `q_p = [x;y;z;roll;pitch]`
- `h = [h1;h2;h3;h4]`（四条竖棱滑块高度，upper/lower 的 z 坐标在此基础上加/减 d/2）

输出（至少）：


- `L (8x1)`：索长
- `U (3x8)`：索单位向量（从平台连接点指向锚点）
- `r_O (3x8)`：平台连接点在 O 系坐标（attachment）
- `a_O (3x8)`：锚点在 O 系坐标（anchor）
- `A5 (5x8)`：**5D 结构矩阵**

结构矩阵定义建议：


$$
A_5(:,i)=
\begin{bmatrix}
u_i\\
(r_i \times u_i)_x\\
(r_i \times u_i)_y
\end{bmatrix}
$$


即只保留 $(F_x,F_y,F_z,M_x,M_y)$，丢弃 $M_z$。


#### (c) 5D 构型检查


- `rank(A5) == 5` 为“平台 5D 任务空间满秩”
- 输出 `sigma_min, cond(A5*A5')` 作为病态指标
- 输出“几何越界原因”（平台越界、h 越界、索长越界等）


### 3) HCDR_statics_5d.m（无重力 + 微扰 + 张力分配）


给定 $A5$ 与外载 $W5$，求张力 $T\in\mathbb{R}^8$：


约束：


$$
A_5 T + W_5 = 0,\qquad T_{\min}\le T\le T_{\max}
$$


建议实现两个求解器：


1. **可行性 LP**（判断有无解、并给出 margin）
2. **最优化 QP**（在可行域内选“更平滑/更均衡”的张力）

QP 目标（建议给三种可选）：


- 张力方差最小：$\min \|T-\bar T\mathbf 1\|^2$
- 与上一步张力差最小（轨迹平滑）：$\min \|T-T_{prev}\|^2$
- 上下索成对平衡：设同一棱的上下索为 (2k-1,2k)，惩罚差：

$$
\min \sum_k (T_{2k-1}-T_{2k})^2
$$

> 注意：你们希望通过微扰产生上下索差异，所以“成对差”惩罚权重不要过大，建议作为次级/软目标。



### 4) HCDR_arm.m（机械臂模块，允许用 Robotics System Toolbox）


要求：


- 用 `rigidBodyTree` 搭好机械臂模型（DH 或 URDF 均可）
- 实现：


`arm_fk(q_a)`：输出末端点在平台系的位置


`arm_ik(p_p_target, q_seed)`：用 `inverseKinematics` 求解

接口建议：


```matlab
function [q_sol, info] = arm_ik_rbtree(robot, p_p_target, q_seed, weights)
% 只关心位置可将 orientation 权重设很小或用目标姿态恒等
end

```


# D. 三模式算法任务书（最核心）


## D1. Platform-only（允许 roll/pitch，通过锚点高度 h 重构）


### D1.1 问题定义


- 机械臂固定：$q_a=q_{a,\text{fix}}$
- 平台变量：$q_p=[x,y,z,\phi,\theta]^T$
- 锚点高度：$h=[h_1,h_2,h_3,h_4]^T$

**硬约束 1：末端点到达目标**
末端在 O 系：


$$
p_O^{ee}=p_O^p + R_{Op}(\phi,\theta)\left(p_p^{base}+p_p^{ee}(q_{a,\text{fix}})\right)
$$


约束：


$$
p_O^{ee}=p_{O,\text{target}}^{ee}
$$


这是 3 个等式约束。


**硬约束 2：构型与机构限位**


- $x,y,z$ 在允许范围
- $\phi,\theta$ 在允许范围
- $h_k\in[h_{min},h_{max}]$
- 索长/几何不得穿越结构（先做基本碰撞/越界门控即可）

**可选硬约束 3：张力可行性（微扰下）**
对候选 $q_p,h$，构造：


$$
W_5 = [0,0,-f_{z,\epsilon},0,0]^T
$$


调用 `statics_5d` 判断是否存在 $T$ 满足平衡与张力界。


### D1.2 优化变量与维度


你有两种做法，推荐 **外层优化 $q_p,h$ + 内层 QP 求张力**：


- 外层变量：$z = [q_p; h]\in\mathbb{R}^9$
- 内层：给定 $z$，解 QP 得 $T^\*(z)$

外层目标函数写成 $J(z)$，内部需要返回“最优张力”和“可行性”。


### D1.3 平台-only 的合理优化目标（按你要求给两类）


你说目标可以是“质心稳定性”或“索力平滑性”，这里给可落地的数学形式（并允许 hybrid）：


#### 方案 A：质心稳定性（无重力下的“稳定”定义）


无重力时谈“质心稳定”更像是 **把平台几何中心/末端工作点维持在框架中心附近 + 避免过大倾斜**，以降低锚点重构需求、提升数值鲁棒性。建议：


$$
J_{com} = w_{xy}\|(x,y)\|^2 + w_{rp}(\phi^2+\theta^2)+w_h\|h-h_{ref}\|^2
$$


- $h_{ref}$ 可取中位高度，或上一步高度（轨迹平滑）。

#### 方案 B：索力平滑性（推荐优先用这个）


通过内层 QP 得到 $T^\*(z)$，外层目标用：


$$
J_T = w_{var}\|T^\*-\bar T^\*\mathbf 1\|^2 + w_{\Delta}\|T^\*-T_{prev}\|^2 + w_{pair}\sum_k (T_{2k-1}^\*-T_{2k}^\*)^2
$$


- 若是单点 IK，则 `T_prev` 可设为某个“舒适张力”初始化（比如全在中值）。
- 若是轨迹，则 `T_prev` 就是上一时刻最优张力，实现“索力平滑”。

#### 方案 C：Hybrid（建议作为默认）


$$
J = J_{com}+J_T
$$


并给 `config.platform_only.objective='hybrid'` 切换。


### D1.4 求解器建议（必须工程可跑）


外层优化推荐用 `fmincon`（带等式约束 + box 约束）：


- 决策变量：`z=[x;y;z;roll;pitch;h1;h2;h3;h4]`
- 等式约束：`ceq = p_ee(z) - p_target`（3×1）
- 不等式约束：限位都可做成 `lb/ub`；若要加“张力可行”可用 penalty 或外层过滤

**关键实现策略（避免难收敛）**：


1. **先给解析初值**

- 先假设 roll=pitch=0，算 $p_O^p$ 的显式初值：

$$
p_O^p = p_{O,\text{target}}^{ee} - (p_p^{base}+p_p^{ee}(q_{a,\text{fix}}))
$$
- h 初值用 `h_ref`。
2. **采用两阶段**

- Stage-1：只优化 `h`（4 维），roll/pitch 先由几何/平衡自然形成（或轻微允许）
- Stage-2：全量优化 `z`（9 维），加上张力平滑目标
3. **尺度化（很重要）**

- 位置 m，角度 rad，高度 m，张力 N
- 在 `fmincon` 里对不同量纲做缩放，否则会出现“角度基本不动/高度乱跳”的病态行为。

### D1.5 platform-only 输出


对每个目标点输出结构体：


- `sol.q_p, sol.h, sol.l, sol.T`
- `sol.rankA5, sol.condA5, sol.feasibleStatics`
- `sol.obj_breakdown`（com项、T项分别多少，便于调参）


## D2. Arm-only（平台固定，机械臂普通 IK）


- 输入：`p_O_target`, `q_p_fix`, `q_seed`
- 变换到平台系：

$$
p_p^{target} = R_{Op}^T(p_O^{target}-p_O^p)-p_p^{base}
$$


- 调用 Robotics System Toolbox：


`inverseKinematics` 只需位置精度，orientation 权重可很小或保持固定姿态
- 输出：`q_a_sol` + 可达性（是否满足关节限位/碰撞简化门控）


## D3. Cooperative（平台 + 机械臂协同，分层优化，可引入 DLS）


协同模式建议从“最小可跑版本”开始，再逐步加复杂约束。


### D3.1 变量


$$
x = [\Delta q_p(5);\Delta q_a(6)]\in\mathbb{R}^{11}
$$


### D3.2 主任务（末端位置误差）


$$
e = p_{O,\text{target}}^{ee}-p_O^{ee}(q_p,q_a)
$$


线性化：


$$
J = [J_p\;\;J_a] \in \mathbb{R}^{3\times 11}
$$


用 DLS 稳定：


$$
\min_x \|Jx-e\|^2+\lambda^2\|x\|^2
$$


### D3.3 次级目标


- 平台动少：$\|\Delta q_p\|^2$
- 臂动少：$\|\Delta q_a\|^2$
- 锚点重构少：$\|h-h_{prev}\|^2$（可在外层更新 h 或把 h 当慢变量）

### D3.4 索力门控（可作为层级约束）


每步更新后，用 5D statics 检查张力可行；不可行则：


- 增大 $\lambda$ 或缩小步长
- 或在 QP 中加入“远离不可行域”的 soft penalty


# E. 主入口与测试脚本（Claude 必须交付）


## E1. 统一入口


```matlab
function result = plan_to_target(p_target, mode, state0, config)
% mode: 'platform' | 'arm' | 'coop'
% state0 contains q_p0, h0, q_a0, T0
% result: feasible, solution, report
end

```

## E2. 单点测试 test_modes_v2.m


至少覆盖三类点：


1. **arm-only 可达** / platform-only 不可达
2. **platform-only 可达** / arm-only 不可达
3. 两者都可达，cooperative 代价更低（比如平台倾角更小或索力更平滑）

输出要打印：


- 模式可行性
- 误差 `||p_ee - p_target||`
- `rank(A5)`、`cond(A5*A5')`
- 张力是否可行、最小张力裕度

## E3. 轨迹测试（最少一个）


给一段直线或圆弧轨迹，要求：


- warm start（用上一时刻解做初值）
- 输出索力/锚点高度随时间曲线，检查平滑性指标确实下降


# F. 立即需要的具体改动点（你可以直接对 Claude 下指令）


1. **删除所有 “add small yaw to break symmetry” 注释与相关逻辑**
yaw 固定为 0，禁止作为“提升 rank”的手段。
2. **平台从 6DOF 改为 5DOF**

- 新增 `HCDR_kinematics_5d`
- 所有 `A_m(6x8)` 改为 `A5(5x8)`
- rank 判据从 6 改为 5
3. **实现 platform-only 的外层 fmincon + 内层 statics QP**

- 外层变量 `z=[q_p;h]`
- 等式约束：末端点到达目标
- 目标：默认 hybrid（com 稳定 + 索力平滑）
- 内层：`statics_5d` 给出 $T^\*$ 并返回不可行 flag（不可行则外层加 penalty）
4. **arm-only 用 Robotics System Toolbox**
  `inverseKinematics` 输出 `q_a_sol`，无需 DLS。
5. **cooperative 分层 + DLS**
  先做最小可跑 DLS（11 维），再加索力门控与次级目标。



#  用户问题

警告: Configuration not statically feasible!
所有配置都显示静力学不可行,这说明微扰力fz_eps=2N太小了,无法产生足够的力矩让LP找到可行解。
可问题时我们的大前提不就是微重力甚至零重力吗？为什么LP可行解要依赖fz_eps?

# 回答

你现在这个现象（`LP 始终 infeasible`，并且 Claude 误判为 “fz_eps 太小导致无法产生足够力矩”）本质上**不是**“扰动太小→力矩不够”，而是你当前 **LP 的建模把 fz_eps 当成了“必须精确平衡的外载”**，再叠加 **tau_min=5N 的硬下界**，会导致在微重力/零重力场景下非常容易“数学上必然不可行”。


下面我把逻辑讲透，并给 Claude 一份明确的改造指引（不需要靠加 yaw，也不需要把 fz_eps 调大来“制造可行解”）。



## 1) 你现在的 LP 在求什么？为什么会对 fz_eps “敏感到致命”？


### 1.1 当前 LP 的可行域其实是一个“扳手多面体/锥”的切片


在 `HCDR_statics_5d.check_feasibility()` 里，你解的是：


$$
\max \ \rho \quad
\text{s.t.}\ 
A_5 T + W_5 = 0,\ 
T \ge \tau_{\min} + \rho,\ 
T \le \tau_{\max}
$$


这里 $A_5\in\mathbb{R}^{5\times 8}$，列向量是每根索单位张力对平台产生的**5D 扳手**（Fx,Fy,Fz,Mx,My），而 $T$ 必须全为正且至少 5N。


这意味着：你允许的外载集合是


$$
\mathcal{W}=\{-A_5 T \mid T\in[\tau_{\min},\tau_{\max}]^8\}
$$


这是一个**有厚度的凸多面体**（更准确：线性映射下的高维超盒）。


**关键点：当 $\tau_{\min} > 0$ 时，$\mathbf{0}$ 往往不在 $\mathcal{W}$ 里。**
也就是说：**“没有外载/外载很小”并不意味着更容易可行，反而可能更难可行**。



### 1.2 为什么在“微重力/零重力”下更容易 infeasible？


你在 `HCDR_config_v2.m` 里设置了：


- `config.cable.tau_min = 5N`（硬下界）
- `config.microg.W5_nominal = [0;0;-fz_eps;0;0]`

于是平衡方程变成：


$$
A_5 T = -W_5 = [0,\ 0,\ +fz\_eps,\ 0,\ 0]^T
$$


**如果某个构型下 8 根索的 $u_{z,i}$（力的 z 分量）全部同号（比如全为正，上拉）**，那么


$$
(F_z)_{\min}=\sum_{i=1}^8 u_{z,i}\,\tau_{\min}
$$


这个最小可达 $F_z$ 可能远大于 2N。
那你要求的 $F_z = fz\_eps = 2N$ 就会落在可行域之外 → **必 infeasible**。


> 这解释了“为什么 Claude 看到 fz_eps 变大就更容易可行”：因为你把目标 $F_z$ 往 $\mathcal{W}$ 的内部推了（靠近/超过由 $\tau_{\min}$ 决定的最小可达扳手幅值）。这不是物理需要，而是建模造成的“外载定位问题”。



### 1.3 这也解释了 “所有构型都显示静力学不可行”


只要你的测试里（比如 `test_modes_v2.m`）默认姿态/高度让大多数构型满足：


- 平台 z 偏低、滑块 h 偏高 → 上/下滑轮都在平台连接点之上
- 则每根索从平台指向锚点的方向 $u_{z,i} > 0$

那么在 `tau_min=5` 时，**系统根本不可能输出 2N 这么小的合力**，更不可能同时满足 Mx/My=0。于是所有点都 infeasible。



## 2) 微重力/零重力场景下，“静力可行性”应该怎么定义？


你现在的思路把微扰 $fz\_eps$ 当作“恒定外载”，这是不对的。微重力场景更合理的定义是：


1. **名义外载为 0（或极小）**：

$$
W_5^{nom} = 0
$$


1. **扰动是一个集合**（比如只考虑 z 向：$|F_z|\le fz\_eps$），它用于**鲁棒性检验**，而不是用于“制造平衡方程”。
2. 平台悬停需要的是 **自应力（self-stress）**：

$$
A_5 T_0 = 0,\quad T_0 \ge \tau_{\min}
$$


这是“在无外载下仍保持张紧/刚度”的正确数学对象。


> 这就是你现在 LP 对 fz_eps 依赖的根本原因：你没有先求自应力，而是强行把微扰写进平衡等式里。



## 3) 给 Claude 的明确改造任务书（静力层必须重写的部分）


下面这套改法能同时满足你的新需求：无重力、yaw=0、roll/pitch 允许、平台-only 做“调滑块高度实现姿态”，并且 **LP 不再依赖 fz_eps 才可行**。



### 任务 A：把“可行性 LP”拆成两类，别混在一起


#### A1）零外载自应力可行性（核心）


新增函数（或改造 `check_feasibility`）：


**Self-stress LP：**


$$
\max \rho \quad
\text{s.t.}\ 
A_5 T = 0,\ 
T \ge \tau_{\min} + \rho,\ 
T \le \tau_{\max}
$$


- 返回：`rho0_max, T0`
- 解释：如果 `rho0_max>0`，说明该构型下**存在可观的自应力裕度**，系统能在零外载下保持张紧。

> 这一步才是“微重力悬停”的门控条件。没有它，你的 planner 会被大量误杀。


#### A2）给定外载的扳手可行性（用于扰动检验，不用于主平衡）


保留原来的：


$$
\max \rho \quad
\text{s.t.}\ 
A_5 T + W_5 = 0,\ 
T \ge \tau_{\min} + \rho,\ 
T \le \tau_{\max}
$$


但只在你做鲁棒性检查时调用，比如测试：


- `W5 = [0;0; +fz_eps; 0;0]` 和 `W5 = [0;0; -fz_eps; 0;0]`
- 看是否都可行，或者计算“最大可抗扰动幅值”。

**注意：**此时 $fz\_eps$ 是“扰动边界”，不是“必须存在的外载”。



### 任务 B：把 config.microg.W5_nominal 改为 0，并把 fz_eps 改成“扰动参数”


在 `HCDR_config_v2.m`：


- `config.microg.W5_nominal = zeros(5,1);`
- `config.microg.fz_eps` 保留，但只用于 robust check / 仿真扰动注入

这一步会立刻消除“LP 必须靠 fz_eps 才可行”的错误依赖。



### 任务 C：平台-only 的优化目标要以 “自应力裕度 + 平滑” 为主，而不是靠外载驱动


你已经说了：平台-only 允许 roll/pitch（yaw=0），并通过改变 4 个滑块高度 $h$ 实现。那平台-only 的核心其实是：


- 决策变量：$x = [q_p;\ h]$（其中 $q_p=[x,y,z,roll,pitch]$）
- 约束：几何边界 + 结构矩阵 rank/σmin 门控
- 目标：让姿态/COM 稳定 + 让张力分配“健康”

**建议的主目标：最大化自应力裕度 rho0_max**


$$
\min\ J(x) =
- w_\rho \rho_0(x)
+ w_h \|h-h_{ref}\|^2
+ w_{rp}(roll^2 + pitch^2)
+ w_{xy}(x^2+y^2)
$$


其中 $\rho_0(x)$ 来自 **Self-stress LP**（A1）。


**直观解释：**


- 你要的不是“用外载逼出一个解”，而是“在零外载下也能找到内部张力平衡，并尽量留裕度去抗扰动”。


### 任务 D：把 “tau_min=5N” 从“可行性门槛”里软化出来（非常关键）


在微重力下，`tau_min=5N` 作为硬下界会导致你需要某种“非零外载”才能平衡到那么高的预张力水平——这正是你现在看到的病。


你有两种选择（推荐第 1 种）：


#### 方案 D1（推荐）：把 tau_min 分成 “物理最小张紧” 和 “求解最小张紧”


- `tau_min_phys = 5N`（最终希望达到的预紧水平）
- `tau_min_solve = 0~1N`（可行性/自应力搜索用）

流程：


1. 用 `tau_min_solve` 找到 `T0`（自应力存在即可）
2. 再用 QP 把 `T` 拉向 `tau_min_phys`，但如果拉不上去就不要判 infeasible，而是输出“可达到的最大平均预紧”。

这会让系统在微重力下更符合物理：**先保证能悬停/能有内部平衡，再追求更高刚度。**


#### 方案 D2：继续用 5N，但必须让 planner 主动把几何调到存在自应力


也就是说：平台-only 优化必须足够强，能把 $u_{z,i}$ 的符号结构调成可相互抵消，否则确实会长期 infeasible。



## 4) 你问的核心：为什么“微重力”反而让 LP 更难可行？


一句话总结：


> 因为你把 `fz_eps` 当成了必须平衡的外载，而你的张力下界 `tau_min=5N` 让系统产生一个“不可消除的最小合扳手”；当外载太小（接近 0）时，反而落在系统可输出扳手集合之外，LP 就必然 infeasible。


正确做法是：


- 名义外载取 0（微重力）
- 先求自应力（$A_5 T=0$）确保能在零外载下张紧
- 扰动 `fz_eps` 只用于“鲁棒性/抗扰能力”检验，而不是用于“让 LP 有解”


## 5) 给 Claude 的“立即可用”调试清单（不需要改太多就能定位问题）


让 Claude 在 `test_modes_v2.m` 每次 LP infeasible 时打印这几个量（能立刻看出是不是我说的这个原因）：


1. 该构型下每根索的 $u_{z,i}$ 符号：

- 如果全是正（或全负），在 `tau_min>0` 时要求小外载几乎必死。

1. 用最简单的“单分量必要条件”估算可达 $F_z$ 范围：

- 对 $F_z = \sum u_{z,i} T_i$，在盒约束下：


$F_{z,\min}=\sum_{u_{z,i}\ge 0} u_{z,i}\tau_{\min} + \sum_{u_{z,i}<0} u_{z,i}\tau_{\max}$


$F_{z,\max}=\sum_{u_{z,i}\ge 0} u_{z,i}\tau_{\max} + \sum_{u_{z,i}<0} u_{z,i}\tau_{\min}$

如果 `fz_eps`（对应目标 $F_z$）落在这个区间之外，那不用跑 linprog 也知道必 infeasible。


1. 直接跑一次 `Self-stress LP (A5*T=0)` 看 `rho0_max`：

- 如果 `rho0_max <= 0`，说明这个几何构型根本没有“零外载自应力”，平台-only 必须通过调整 `h`/`z` 去找。


## 6) 结论（你该怎么跟 Claude 说）


- 现在 LP infeasible 不是 “扰动太小力矩不够”，而是 **你把微扰写进平衡方程 + 用 tau_min=5N 强行抬高了最小可达扳手**。
- 微重力正确流程是：**先求自应力（A5*T=0）作为可行性门控**，`fz_eps` 只做扰动鲁棒性测试。
- 平台-only 的优化目标建议以 **最大化自应力裕度 rho0_max** 为主，再叠加滑块平滑/姿态小角度/COM 约束。
- 注释里任何 “add small yaw to break symmetry” 都应删掉；yaw 固定为 0 是结构定义，不靠加 yaw 提 rank。