# TASKBOOK.md — 平面HCDR（微重力）整体运动学/动力学/工作空间（Route B）

## 0. Scope & Hard Constraints（不可违反）
- 平面 CDPR 平台 + 串联机械臂，微/无重力环境（默认 g=0）。
- Part A 只允许参考仓库当前版本（zip: HCDR_ML-copilot-refactor-hcdr-statics-checks）。
- [新增注释] §9 提及外部仓库仅用于“符号/方法对照”，不作为 Part A 代码直接参考来源。
- 索弹性暂不进入主模型，但必须预留接口（不破坏主流程）。
- 工作空间 Level-1（索长给够）不做；仅做几何退化 + 张力/力矩可行性（Level-2 类）。
- 几何退化筛选：不再使用 `min_i |u_{z,i}| >= eps_z`；由于平面问题 z 锁死，u_z 固定且不参与筛选。
- 三模式 IK：必须实现“解析消元(A)”与“显式变量(B)”两套，默认 B。
- Route B 动力学：必须按论文 (18)(19)(20) 的结构实现：
  - Pinocchio: crba -> M；rnea(qdd=0) -> h
  - 计算 actuation-space bias (h_a)：仅用伪逆/阻尼伪逆；绝对禁止在此阶段加张力盒约束 (Box Constraints)。
  - HQP 仅解 u_{a,wo}；最终 u_a = u_{a,wo} + h_a

## 1. Model Definition（平面广义坐标与作动）
### 1.1 广义坐标
q = [ x, y, psi, q_m^T ]^T \in \mathbb{R}^{3 + n_m}
- x,y: 平台在基座坐标系的平面位置
- psi: 平台绕 z 的偏航角
- q_m: 机械臂关节（n_m=6 或 7，以 config 指定）

z, roll, pitch 全部锁死：z=z0，roll=pitch=0。

### 1.2 作动向量与映射
- 绳索张力 T \in \mathbb{R}^{n_c}（默认 n_c=8）
- 机械臂力矩 \tau_m \in \mathbb{R}^{n_m}
u_a = [ T^T, \tau_m^T ]^T \in \mathbb{R}^{n_c + n_m}

平面广义力（平台）w_{2D} = [F_x, F_y, M_z]^T
w_{2D} = A_{2D}(q) * T   (A_{2D} 维度为 3 x n_c)

定义转换矩阵（平面退化对应论文 S）：
S^T = \begin{bmatrix} A_{2D} & 0 \\ 0 & I_{n_m} \end{bmatrix} \in \mathbb{R}^{(3+n_m) \times (n_c+n_m)}

## 2. Deliverables（必须交付的代码文件）
### 2.1 核心函数（新增 planar 版本，保留旧 5d 不动）
- HCDR_config_planar.m
- HCDR_kinematics_planar.m
- HCDR_statics_planar.m
- HCDR_visualize_planar.m
- ik_solve_three_modes_planar.m
- plan_platform_only_planar.m / plan_arm_only_planar.m / plan_cooperative_planar.m / plan_to_target_planar.m
- workspace_scan_planar.m（工作空间扫描/分类输出）

### 2.2 Route B 动力学（MATLAB + Python Pinocchio）
Python 侧：
- pin_model_build.py：构建“单一运动学链”的 pinocchio model（planar root + arm）
- pin_terms.py：提供 get_M_h(q, qd) -> (M, h)

MATLAB 侧：
- pin_get_M_h.m：MATLAB wrapper（py 调用）返回 double
- hcdr_bias_map_planar.m：实现 D4：从 h -> h_a（伪逆/阻尼伪逆）
- hqp_routeB_solve.m：HQP 求解 u_{a,wo}（含张力/力矩边界）
- simulate_routeB_step.m：一步动力学/控制闭环（可选 demo）

### 2.3 单元测试 / 回归测试（matlab.unittest）
- tests/test_planar_kinematics.m
- tests/test_planar_A2D_rank.m
- tests/test_planar_self_stress.m
- tests/test_planar_IK_three_modes.m
- tests/test_routeB_bias_mapping.m
- tests/test_routeB_HQP_feasible.m

## 3. Part A — 运动学（参考现有 HCDR_kinematics_5d 的接口习惯）
### 3.1 需要输出的量（planar）
给定 (q, config)：
- p_platform = [x,y,z0]
- R_platform = Rz(psi)
- arm FK：T_0e (world->ee)，p_ee
- cable 几何：anchor points a_i（world），platform attach b_i（platform frame）
- unit vectors u_i = (a_i - (p_platform + R_platform*b_i)) / ||a_i - (p_platform + R_platform*b_i)||
- A_{2D}：w_{2D} = \sum_i [ u_{i,xy} ; (r_i \times u_i)_z ] * T_i
  其中 r_i = (R_platform * b_i)_{xyz}（相对平台参考点的力臂）

几何退化筛选：
- rank(A_{2D}) == 3
- sigma_min(A_{2D}) >= eps_sigma（数值门槛）
不再使用 u_z 条件。

### 3.2 索弹性预留接口（不启用）
- 在 cable 几何输出里预留：l_i, l0_i, delta_l_i
- 动力学主流程不使用；后续可插入：T = T_cmd + K*delta_l

## 4. Part B — 静力/张力可行性（Level-2）
### 4.1 自应力（micro-g, w_ext=0）
目标：存在 T 使 A_{2D} T = 0 且 T_min <= T <= T_max
实现：
- 用 linprog/quadprog 求一个可行点（优先：最小范数 + slack）
- 输出：is_feasible, T_feas, nullspace_dim, diagnostics

### 4.2 给定外载（可选）
若给定 w_{2D\_ext}：
求解 A_{2D} T + w_{2D\_ext} = 0 且 T_min <= T <= T_max

## 5. Part C — 三模式 IK（参考 ik_solve_three_modes.m 的“统一入口/分模式”结构）
### 5.1 模式定义
- Mode 1: platform-only
  - q_m 固定在 home（或给定），决策变量：x,y,psi（默认显式变量）
- Mode 2: arm-only
  - x,y,psi 固定（或给定），决策变量：q_m
- Mode 3: cooperative
  - 决策变量：x,y,psi,q_m

### 5.2 两种实现（必须都做）
A) 解析消元（用于快速扫描）
- 决策变量：psi, q_m（或只 q_m）
- x,y 由末端等式反解得到
- 仍需检查：平台边界、A_{2D} 非退化、张力可行性

B) 显式变量（默认，用于 HQP/多目标）
- 决策变量：x,y,psi,q_m
- 约束：p_ee(q) = p_d（或带 slack）
- 目标：最小移动/最小关节变化/最大张力裕度（可加权）

### 5.3 求解器建议
- fmincon (SQP) 或 lsqnonlin（若纯等式）
- 输出统一结构：q_sol, mode_id, success, cost, diag(A_{2D}, sigma_min, feasible_T)

## 6. Part D — Route B 整体动力学 + HQP
### 6.1 Pinocchio 计算（必须）
- 用 crba(model,data,q) 得 M(q)
- 用 rnea(model,data,q,qd,qdd=0) 得 bias h(q,qd)
- micro-g：model.gravity=0（或在调用前置零）

动力学方程：
M qdd + h = S^T u_a    (论文式17)
Route B 等效形式：
M qdd = S^T u_{a,wo}   (论文式18)

### 6.2 D4：h -> h_a（必须按此实现，绝对禁止施加盒约束）
切分：
h = [ h_p^T , h_m^T ]^T  (h_p \in \mathbb{R}^3, h_m \in \mathbb{R}^{n_m})
- 机械臂部分：h_{a,m} = h_m
- 绳索部分：h_{a,T} = A_{2D}^+ h_p
  推荐使用阻尼伪逆防止计算奇异：
  h_{a,T} = A_{2D}^T * (A_{2D}*A_{2D}^T + \lambda*I)^{-1} * h_p
拼接：
h_a = [ h_{a,T}^T , h_{a,m}^T ]^T

**注意：此阶段计算出的 h_{a,T} 仅为理论补偿值，允许为负数或极大值，坚决不能进行裁剪或限制！** 真实的物理边界留给 HQP 去处理。

### 6.3 HQP
决策变量：qdd, u_{a,wo}, slack
约束：
- M qdd = S^T u_{a,wo}
- 张力真实物理边界：T_min <= u_{T,wo} + h_{a,T} <= T_max
- 力矩真实物理边界：\tau_min <= u_{m,wo} + h_m <= \tau_max
任务（示例）：
- 末端加速度跟踪：J(q) qdd + Jdot qd = xdd_des + s
目标：
min ||s||^2 + \alpha||u_{T,wo}||^2 + \beta||u_{m,wo}||^2

输出：
最终控制指令 u_a = u_{a,wo} + h_a

## 7. Definition of Done（验收标准）
- `matlab -batch "results=runtests('tests'); assertSuccess(results)"` 全绿
- 典型 demo（至少 2 个）：
  1) 随机 q：A_{2D} rank=3 的统计通过；自应力可行率输出
  2) 三模式到达同一目标点：能正确分类 mode 可达性，并输出一条可视化轨迹
- Route B demo：
  - 随机状态下：能从 pinocchio 得到 (M,h)，准确映射出 h_a，并由 HQP 得到 u_{a,wo}，最终 u_a 合成成功且不违背物理边界。

## 8. [ADDED-2026-03-03] URDF/可视化与口径补充（新增注释标记）
<!-- [ADDED-2026-03-03-BEGIN] -->
> [新增注释] 以下条目为后续迭代补充，不替代上文硬约束；若冲突以上文硬约束为准。

1) 6R 主链口径
- [新增注释] `n_m=6` 分支优先使用 URDF FK 链，`n_m=2` 测试分支保持原简化链不变。
- [新增注释] `base_offset (= offset_in_platform)` 由机械臂模型内部处理，外层调用禁止重复加/减。

2) 末端点定义
- [新增注释] demo/IK/FK 统一使用“可视化夹爪末端实体点”为 target 对齐点（tip point）。
- [新增注释] 若 URDF 不含夹爪几何，需显式定义 `tool_offset_in_ee` 作为替代末端点。

3) 可视化一致性
- [新增注释] 三模式单窗口 3D 三子图要求显示运动过程而非仅终态。
- [新增注释] 轨迹线必须来源于同一末端点定义，保证“模型末端点”与“迹线”重合。

4) 动力学 demo 与 Pinocchio
- [新增注释] Route-B dynamics demo 默认“优先 Pinocchio；失败自动回退 provider”，并在终端明确当前 provider 来源。
- [新增注释] 无论 provider 来源如何，D4 与 HQP 约束顺序保持不变：先 `h -> h_a`（无盒约束），后在 HQP 施加物理边界。
<!-- [ADDED-2026-03-03-END] -->


## 9. [ADDED-2026-03-05] v3.0 迭代路线图：从单层 QP 升级→真正 HQP + 闭环 WBC
<!-- [ADDED-2026-03-05-BEGIN] -->
> [新增注释] 以下条目基于论文 "Whole-body control of redundant HCDR with manipulator HQP approach" 开源实现 (cold-deuu/HCDR_HQP) 与 MuJoCo CDPR 仿真器 (Yang51st/Mujoco-CDPR-Simulator) 的分析，是 v6-codex 接下来的核心升级方向。

### 9.1 已识别差距总览

| # | 差距项 | 当前状态 | 论文要求 | 优先级 |
|---|--------|---------|---------|--------|
| G1 | HQP 求解器是单层 QP | `hqp_routeB_solve.m` 只有一层 `quadprog` | 多层严格优先级求解 | P0 |
| G2 | 缺少任务空间闭环跟踪 | 无 $\dot{J}$、无 $\ddot{x}_{ref}$、无 PD 控制 | $\ddot{x}_{ref} = \ddot{x}_d + K_p e + K_d \dot{e}$ | P0 |
| G3 | 缺少 `mass_tilda` / `jacob_tilda` 符号映射 | 直接使用 $M$, $J$ | $\tilde{J} = J M^{-1}$，$\Lambda = (\tilde{J} \tilde{J}^T)^{-1}$ | P1 |
| G4 | 无奇异性回避机制 | — | SVD 分解 $J$，分离 singular/non-singular 子空间 | P1 |
| G5 | 无关节限位不等式约束 | — | $q_{min} \leq q + \dot{q} \Delta t + \frac{1}{2} \ddot{q} \Delta t^2 \leq q_{max}$ | P1 |
| G6 | 无冗余关节归中任务 | — | $\ddot{q}_{posture} = K_p (q_{mid} - q) - K_d \dot{q}$ | P2 |
| G7 | 无时域轨迹仿真回路 | `simulate_routeB_step.m` 仅单步 demo | 需完整 for-loop：$q_{k+1} = q_k + \dot{q}_k dt + \frac{1}{2} \ddot{q}_k dt^2$ | P1 |
| G8 | 无 MuJoCo/物理引擎验证 | — | 可选但建议用于拉力闭环验证 | P3 |
| G9 | Z 向末端偏差未收敛 | mode2: ~0.10m, mode3: ~0.02m | 先修 IK 再上闭环 | P0 |

### 9.2 Phase 1（P0）：修复 Z 偏差 + 引入闭环 + 升级 HQP

#### Task 9.2.1 — 收敛 IK Z 偏差
- **目标**：mode2 Z 偏差 < 0.005 m，mode3 Z 偏差 < 0.002 m
- **方法**：检查 URDF FK 链的 `tool_offset_in_ee` 是否与可视化末端一致；检查 `base_offset` 是否被重复施加（参见 §8 硬约束）
- **验收**：`test_planar_IK_three_modes.m` 在“可达目标点集合”上增加 Z 精度断言（不可达样本单独标注，不计入该阈值统计）

#### Task 9.2.2 — 实现全身雅可比 $J_{wb}(q)$ 与 $\dot{J}_{wb}(q,\dot{q})$
- **符号定义**（对齐论文）：
  - 广义坐标 $q \in \mathbb{R}^{n}$，$n = 3 + n_m$（平面平台 + 机械臂）
  - 末端位姿 $x_e \in \mathbb{R}^{m}$（本阶段固定 $m=3$，采用位置任务 $[x,y,z]$；姿态任务后续单独扩展）
  - 全身雅可比：$\dot{x}_e = J_{wb}(q) \dot{q}$
  - 全身雅可比微分：$\ddot{x}_e = J_{wb} \ddot{q} + \dot{J}_{wb} \dot{q}$
- **新文件**：`src/jacobian_whole_body.m`
  - 输入：`(q, qd, config)` → 输出：`(J_wb, Jdot_qd)`
  - 平台部分：解析推导 $J_p$（对 $[x, y, \psi]$ 的偏导）
  - 机械臂部分：`pinocchio.computeJointJacobians` + `pinocchio.getFrameJacobian`
  - $\dot{J}\dot{q}$：使用 `pinocchio.getFrameJacobianTimeVariation` 或数值差分
- **参考**：`cold-deuu/HCDR_HQP` 中 `robot.hpp` 的 `jacobianWorld()` 与 `jacobianWorld_d()`
- **测试**：`tests/test_jacobian_whole_body.m` — 数值差分 vs 解析对比

#### Task 9.2.3 — 实现闭环 PD 任务空间控制器
- **新文件**：`src/task_space_pd_controller.m`
- **核心公式**（论文符号）：
  ```
  e = x_d - x_e                          % 位置误差
  ė = ẋ_d - J_wb * q̇                    % 速度误差
  ẍ_ref = ẍ_d + K_p * e + K_d * ė       % 参考加速度（虚拟控制输入）
  ```
- **输出**：`xdd_ref`（即 $\ddot{x}_{ref}$），供 HQP Level-2 使用
- **参考**：`cold-deuu/HCDR_HQP` 中 `math::pd_control()` 函数
- **增益建议**：$K_p = \text{diag}(100,100,100)$，$K_d = \text{diag}(20,20,20)$（初始值，需调参）

#### Task 9.2.4 — 升级 `hqp_routeB_solve.m` 为真正的多层 HQP
- **决策变量**（对齐论文）：
  ```
  z = [ q̈ᵀ, u_{a,wo}ᵀ, sᵀ ]ᵀ ∈ ℝ^{n + (n_c+n_m) + m_slack}
  ```
  其中 $n = 3 + n_m$，$n_c = 8$，$m_{slack}$ = 末端跟踪维度

- **Level 1（最高优先级）— 动力学可行性 + 物理边界**：
  - 等式约束：$M \ddot{q} = S^T u_{a,wo}$
  - 不等式约束（论文 Box Constraints）：
    - $T_{min} - h_{a,T} \leq u_{T,wo} \leq T_{max} - h_{a,T}$
    - $\tau_{min} - h_m \leq u_{m,wo} \leq \tau_{max} - h_m$
  - 目标函数：$\min \| u_{a,wo} \|^2$（最小作动能量）

- **Level 2（次高优先级）— 末端轨迹跟踪**：
  - 继承并冻结 Level-1 已满足的等式约束与上一层最优残差（按层级最优值延续，不强制写死为 0）
  - 新增等式/软约束：$J_{wb} \ddot{q} + \dot{J}_{wb} \dot{q} = \ddot{x}_{ref} + s$
  - 目标函数：$\min \| s \|^2 + \alpha \| u_{a,wo} \|^2$

- **Level 3（低优先级，可选）— 冗余优化**：
  - 张力方差最小化：$\min \text{Var}(T)$
  - 关节归中：$\min \| K_p (q_{mid} - q_m) - K_d \dot{q}_m - \ddot{q}_m \|^2$
  - 奇异性回避：参考 `Singularity_Avoidance_Task.hpp` 的 SVD 分解

- **求解策略**（对齐 `cold-deuu/HCDR_HQP` 的 `HQP_solver.cpp`）：
  ```
  for level = 1 to N:
      A_eq_accumulated = [A_eq_prev (slack_prev 列置零); A_eq_level]
      b_eq_accumulated = [b_eq_prev - slack_prev_optimal; b_eq_level]
      solve quadprog(Q_level, c_level, A_eq_accumulated, b_eq_accumulated, A_ineq, b_ineq)
      freeze slack_level
  end
  ```
- **参考**：`HQP_solver.cpp` 的 `solve()` 方法（逐层积累等式约束，使用 `eiquadprog`）
- **MATLAB 对应**：使用 `quadprog` 替代 `eiquadprog`

### 9.3 Phase 2（P1）：增强鲁棒性 + 时域仿真

#### Task 9.3.1 — 实现 `mass_tilda` / `jacob_tilda` 有效动力学映射
- **定义**（对齐论文符号）：
  ```
  M̃ = M⁻¹                        % 广义质量逆
  J̃ = J_wb * M⁻¹                  % 有效雅可比
  Λ = (J̃ * J̃ᵀ)⁻¹                 % 操作空间惯性矩阵
  J̃_bar = M⁻¹ * J_wb' * Λ        % 动力学一致伪逆
  N = I - J̃_bar * J_wb            % 零空间投影矩阵
  ```
- **好处**：Level-2/3 的任务可以在操作空间表达，与关节空间解耦
- **新文件**：`src/operational_space_dynamics.m`
- **参考**：`cold-deuu/HCDR_HQP` 中 `State` 结构体的 `mass`, `floating_M`, `franka_M`, `J` 字段

#### Task 9.3.2 — 实现关节限位不等式约束
- **公式**（对齐 `JointLimitAvoidanceTask.hpp`）：
  ```
  q_min ≤ q + q̇*Δt + 0.5*q̈*Δt² ≤ q_max
  ```
  转换为对 $\ddot{q}$ 的不等式：
  ```
  A_ineq * z ≤ b_ineq
  其中 A_ineq 提取 q̈ 对应列，b_ineq = 2/Δt² * (q_max - q - q̇*Δt) [上界]
                                  b_ineq = 2/Δt² * (q - q_min + q̇*Δt) [下界取负]
  ```
- **新文件**：`src/task_joint_limit_avoidance.m`

#### Task 9.3.3 — 实现奇异性回避（SVD 分解）
- **方法**（对齐 `Singularity_Avoidance_Task.hpp`）：
  1. 对 $J_{wb}$ 做 SVD：$J_{wb} = U \Sigma V^T$
  2. 找到 $\sigma_i < \epsilon_{singular}$ 的奇异方向
  3. 将任务分解为 non-singular 子空间和 singular 子空间
  4. Non-singular 部分放入 Level-2（正常跟踪）
  5. Singular 部分放入 Level-3（降权跟踪或阻尼最小范数）
- **新文件**：`src/task_singularity_avoidance.m`

#### Task 9.3.4 — 时域闭环仿真回路
- **升级文件**：`src/simulate_routeB_loop.m`（从 `simulate_routeB_step.m` 扩展）
- **仿真流程**：
  ```
  for k = 1 : N_steps
      1. 读取当前状态 (q_k, qd_k)
      2. 计算 Pinocchio 动力学项 (M, h)
      3. 计算 h -> h_a（D4 映射，无盒约束）
      4. 计算全身雅可比 J_wb, Jdot_qd
      5. 生成期望轨迹 (x_d, xd_d, xdd_d) = trajectory_generator(t_k)
      6. 计算闭环参考加速度 xdd_ref = PD_controller(x_d, x_e, ...)
      7. 多层 HQP 求解 → qdd_k, u_{a,wo,k}
      8. 合成 u_a = u_{a,wo} + h_a
      9. 状态积分：qd_{k+1} = qd_k + qdd_k*dt, q_{k+1} = q_k + qd_k*dt + 0.5*qdd_k*dt²
     10. 记录：轨迹误差、张力历史、关节力矩历史
  end
  ```
- **轨迹发生器**：`src/trajectory_se3_cubic.m`（参考 `cold-deuu/HCDR_HQP` 的 `SE3_cubic.hpp`，三次多项式插值 SE3 轨迹）
- **验收**：
  - 位置跟踪误差 RMSE < 5 mm（稳态）
  - 所有张力 $T_i > 0$ 且 $T_i < T_{max}$
  - 所有关节力矩 $|\tau_j| < \tau_{max}$

### 9.4 Phase 3（P2-P3）：冗余优化 + 仿真对接

#### Task 9.4.1 — 关节归中冗余任务（Level-3）
- $\ddot{q}_{posture,des} = K_p (q_{mid} - q_m) - K_d \dot{q}_m$
- 投影到零空间：$\ddot{q}_{null} = N \cdot \ddot{q}_{posture,des}$
- 参考：`Joint_Posture_Task.hpp`

#### Task 9.4.2 — 张力分配优化（Level-3）
- 在满足 Level-1/2 后，最小化张力方差：
  $\min \sum_i (T_i - \bar{T})^2$（均匀张力分配）
- 或最小化总张力能量：$\min \| T \|^2$

#### Task 9.4.3 — [可选] MuJoCo 仿真对接
- 参考 `Yang51st/Mujoco-CDPR-Simulator` 的 `CDPR_Base` 类构建 HCDR 的 MuJoCo XML
- 扩展：在末端平台上挂载 GEN3-lite URDF（通过 `include` 或 `attach`）
- 将 HQP 输出的 $u_a$ 作为 MuJoCo actuator 的控制输入
- 验证：物理引擎下张力-运动的一致性

### 9.5 新增交付文件清单

| 文件路径 | 描述 | Phase |
|----------|------|-------|
| `src/jacobian_whole_body.m` | 全身雅可比 $J_{wb}$ 与 $\dot{J}_{wb}\dot{q}$ | P0 |
| `src/task_space_pd_controller.m` | 闭环 PD 控制器，输出 $\ddot{x}_{ref}$ | P0 |
| `src/hqp_multi_level_solve.m` | 多层 HQP 求解器（替代单层版本） | P0 |
| `src/operational_space_dynamics.m` | $\tilde{M}$, $\tilde{J}$, $\Lambda$, $\bar{J}$, $N$ 计算 | P1 |
| `src/task_joint_limit_avoidance.m` | 关节限位不等式约束生成 | P1 |
| `src/task_singularity_avoidance.m` | SVD 奇异性回避任务 | P1 |
| `src/simulate_routeB_loop.m` | 完整时域仿真回路 | P1 |
| `src/trajectory_se3_cubic.m` | SE3 三次多项式轨迹发生器 | P1 |
| `src/task_joint_posture.m` | 冗余关节归中任务 | P2 |
| `src/task_tension_distribution.m` | 张力均匀分配优化 | P2 |
| `tests/test_jacobian_whole_body.m` | J_wb 数值 vs 解析对比测试 | P0 |
| `tests/test_hqp_multi_level.m` | 多层 HQP 层级优先级验证 | P0 |
| `tests/test_pd_tracking.m` | 闭环 PD 跟踪单步收敛性 | P0 |
| `tests/test_time_domain_sim.m` | 时域仿真轨迹误差/力矩/张力边界 | P1 |

### 9.6 符号对照表（你的代码 ↔ 论文 ↔ cold-deuu 实现）

| 你的代码 | 论文符号 | cold-deuu 代码 | 说明 |
|----------|---------|---------------|------|
| `q` | $q \in \mathbb{R}^n$ | `q` | 广义坐标 |
| `qd` | $\dot{q}$ | `v` | 广义速度 |
| `qdd` | $\ddot{q}$ | `q_ddot_des` | 广义加速度（决策变量） |
| `M` (from pin) | $M(q)$ | `state.mass` | 广义质量矩阵 |
| `h` (from pin) | $h(q,\dot{q})$ | `state.gravity` + NLE | 非线性效应（科里奥利+重力） |
| `A_2D` | $W$ (3D) / $A$ (平面) | `state.W` | 绳索力映射矩阵 |
| `S_T` | $S^T$ | `St_matrix` | 选择/作动映射矩阵 |
| `h_a` | — | — | 作动空间偏置（你自定义的 D4 映射） |
| `u_a_wo` | $u_{a,wo}$ | QP 求解结果 | 去偏后作动指令 |
| `u_a` | $u_a = u_{a,wo} + h_a$ | `pub_state.tension` / `.arm_torque` | 最终作动指令 |
| — (待新增) | $\tilde{J} = J M^{-1}$ | `m_jacob_tilda` | 有效雅可比 |
| — (待新增) | $\tilde{M} = M^{-1}$ | `m_mass_tilda` | 广义质量逆 |
| — (待新增) | $\Lambda = (\tilde{J}\tilde{J}^T)^{-1}$ | 隐含在 task 内 | 操作空间惯性 |
| — (待新增) | $\ddot{x}_{ref}$ | PD control output | 闭环参考加速度 |
| — (待新增) | $\dot{J}\dot{q}$ | `jacobianWorld_d` | 雅可比微分偏置 |

### 9.7 验收标准（对 §7 的增补）

在原有 §7 基础上增加：
1. **P0 验收**：
   - Z 偏差：mode2 < 5mm，mode3 < 2mm
   - `test_jacobian_whole_body.m`：数值差分误差 < $10^{-6}$
   - `test_hqp_multi_level.m`：Level-1 约束残差 < $10^{-8}$，Level-2 slack 范数有界
   - `test_pd_tracking.m`：静态目标点 PD 输出 $\ddot{x}_{ref}$ 符号/量级正确

2. **P1 验收**：
   - `test_time_domain_sim.m`：
     - 100 步仿真（dt=0.001s），轨迹跟踪 RMSE < 5mm
     - 全程 $T_i \in [T_{min}, T_{max}]$
     - 全程 $|\tau_j| \leq \tau_{max}$
     - 关节角 $q_j \in [q_{min,j}, q_{max,j}]$

3. **P2 验收**：
   - 加入冗余任务后，关节离中心距离减小
   - 张力方差较无优化版本降低 > 30%

<!-- [ADDED-2026-03-05-END] -->



## 10. [ADDED-2026-03-08] v3.1 速度级IK + workspace分层 + Route-B闭环 + MuJoCo后端
<!-- [ADDED-2026-03-08-BEGIN] -->

你现在要直接修改当前 MATLAB 仓库 HCDR_v6-codex。不要只给建议，要实际改代码、补函数、改 demo、补测试，并在最后输出“修改文件清单 + 核心设计说明 + 运行方法 + 已知限制”。

【本节目标】
当前项目接下来的主线很明确：
1. 在完成 whole-body dynamics 控制之外，还要分析三种 mode 的工作空间；
2. 工作空间的第一层必要条件是“几何可达 = IK 有解”；
3. 因此 IK 不能停留在当前位置级/静态式求解，必须升级为基于 Jacobian 的速度级 IK；
4. 对 Route-B / whole-body dynamics，先确保每个控制周期都能稳定算出平滑的 u_a 和 qdd；
5. 然后将求出的力/力矩送入 MuJoCo，由 MuJoCo 做物理推进得到 q 和 q_dot。

--------------------------------------------------
一、总原则
--------------------------------------------------
A. 保留当前 IK 相关代码和 demo 的外壳，不要粗暴删除；但“速度级 IK + Pinocchio + Route-B + MuJoCo”要成为主路径。
B. workspace 判定必须分层，不允许继续把“几何可达”“张力可行”“动力学可行”混成一个标签。
C. 命名上严格区分：
   - 末端位姿矩阵：X_cur / X_des / T_cur / T_des
   - 质量矩阵：M_mass 或 Mq
   不要再把位姿矩阵和质量矩阵都命名成 M。
D. 所有新增参数统一放到 cfg / opts，不要散落硬编码：
   - Kp, Kd
   - lambda_ik
   - dt
   - iter_max
   - err_tol
   - smoothness weights
   - joint/platform limits
E. 主链一旦启用真实机械臂（n_m = 6 且 use_urdf_kinematics = true），凡是 FK / Jacobian / 末端位置姿态 / workspace 几何判定 / 动力学几何量，一律统一走 URDF + Pinocchio，不允许再用手写简单公式作为主路径。
F. cfg.arm.DH 不必物理删除，但只能保留为 fallback / 旧测试兼容 / 占位，不允许继续作为真实机械臂主链的几何来源。

--------------------------------------------------
二、IK 部分：升级为速度级 IK 主路径
--------------------------------------------------
目标：把三种 mode 的 IK 升级为基于 Jacobian 的速度级 IK，并以速度级 IK 收敛作为几何可达 workspace 的必要条件。

统一主公式采用：

xdot_cmd = xdot_d + Kp * (x_d - x_current)

qdot = J^+ * xdot_cmd + (I - J^+ * J) * qdot_0

其中：
- J^+ 默认使用阻尼伪逆；
- 若当前代码变量名仍使用 xdot_d，也可以保留原变量名，但语义必须明确：真正送入伪逆求解的是“前馈 + 误差反馈”后的命令速度，而不是死的外部常数速度；
- q_next = q + dt * qdot；
- 需要统一的收敛 / 发散 / 奇异 / 越界判定。

【Jacobian 来源要求】
IK 部分的 Jacobian 必须明确来自 Pinocchio 解析结果，不允许继续用手写 DH Jacobian 作为主路径。
调用链必须清楚写出来：
1. pin.forwardKinematics(model, data, q)
2. pin.updateFramePlacements(model, data)
3. pin.computeJointJacobians(model, data, q)
4. pin.getFrameJacobian(model, data, ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

若做完整 6D 任务：
- 直接使用 J6D

若做 planar 任务，只做 [x, y, psi]：
- 从 J6D 中抽取
  - 平移 x 行
  - 平移 y 行
  - 绕 z 的角速度行
- 组成 J_task

参考坐标系必须统一：
- 若误差使用世界系位置误差 + 世界系姿态误差，则 Jacobian 也必须使用世界对齐表达；
- 默认使用 LOCAL_WORLD_ALIGNED；
- 不允许误差和 Jacobian 混用不同参考系。

【三种 mode 的要求】
1. mode1: platform-only
   - 机械臂冻结，qdot_m = 0
   - 仅平台自由度参与
   - 保留平台姿态正则化等二级项

2. mode2: arm-only
   - 平台冻结，qdot_f = 0
   - 仅机械臂关节参与
   - 二级项可包括关节归中、joint-limit avoidance

3. mode3: cooperative
   - 一级任务：末端移动任务，必须优先满足
   - 二级任务：posture regularization + joint-limit avoidance
   - 不要把“最小张力”直接作为 IK 层主正则项
   - 默认先实现 null-space projector 路径
   - 可选保留一个小型 two-level HQP velocity IK 开关，但先保证默认路径稳定

【数值机制】
必须加入：
- 阻尼伪逆
- 最大迭代次数
- 误差阈值
- 步长 dt
- joint/platform limits
- 失败原因标记：
  success / max_iter / jacobian_degenerate / limit_hit / nan_detected / solver_error

【需要改动】
建议修改或新增：
- scripts/demo_ik_three_modes_planar.m
- src/ik_solve_three_modes_planar.m
- src/solve_velocity_ik_three_modes_planar.m
- src/integrate_velocity_ik_step.m
- src/build_task_jacobian_planar_from_pin.m

demo_ik_three_modes_planar.m 需要同步升级：
- 保留三模式展示和可视化；
- 求解主路径改为速度级 IK；
- 日志新增：
  * convergence flag
  * iter count
  * final error
  * min singular value
  * 是否触发 joint-limit avoidance
  * qdot 平滑性指标

--------------------------------------------------
三、workspace 部分：把速度级 IK 可解纳入几何可达判定
--------------------------------------------------
当前 workspace 不能再只做静态一次判定，必须拆成两层：

第 1 层：geometry-reachable workspace
判据：
- 给定候选目标点（必要时含姿态），从指定初值出发，速度级 IK 在最大迭代数内收敛；
- 末端误差低于阈值；
- 全过程没有 Jacobian 数值崩溃、变量越界、NaN。

第 2 层：tension/torque-feasible workspace
判据：
- 在第 1 层几何可达的基础上，再叠加已有张力可行性 / 力矩约束 / 结构矩阵约束。

输出结果中必须明确区分：
- geom_reachable_mode1
- geom_reachable_mode2
- geom_reachable_mode3
- tension_feasible_mode1
- tension_feasible_mode2
- tension_feasible_mode3

不要再把“几何可达”和“张力可行”混成一个单一标签。

【需要改动】
建议修改或拆分：
- src/workspace_scan_planar.m
- src/workspace_scan_geom_reachable_planar.m
- src/workspace_scan_tension_feasible_planar.m
- src/workspace_scan_three_modes_planar.m

要求：
1. 先把 planar 版本做好，不要同时硬扩 full 3D；
2. 输出 summary 统计和可视化接口；
3. 几何层失败原因要能记录；
4. tests 要补针对性用例。

--------------------------------------------------
四、whole-body dynamics / Route-B：这是重点
--------------------------------------------------
scripts/demo_dynamics_routeB_planar.m 是当前关键脚本。需要把当前 Route-B 主路径升级为真正闭环形式：

每个控制周期必须执行：
1. 从当前 q, qd 出发，调用 Pinocchio；
2. 由 Pinocchio 得到：
   - forwardKinematics(model, data, q)
   - 当前末端位姿 X_cur
   - 当前 Jacobian J
   - Jdot_qdot
   - 质量矩阵 M_mass
   - 非线性项 h = C(q,qd)*qd + g(q)
3. 构造末端误差：
   - 若当前主线仍是 planar：
     e = [ex; ey; epsi]
   - 若已有完整 SE(3) 版本：
     X_err = X_cur^{-1} * X_des
     e = [ep; er]
4. 构造任务空间期望加速度：
   a_des = xdd_d + Kd * (xd_d - xdot) + Kp * e
   若当前 demo 只做 regulation，可令 xd_d = 0, xdd_d = 0。

【任务约束形式】
本阶段固定采用你确认的形式：

J * qdd + Jdot_qdot + s = a_des

也就是：
- 使用等式型任务
- 带 slack
- 暂时不优先做区间不等式版本

【动力学约束】
保持：
M_mass * qdd = S' * u_a_wo

以及：
u_a = u_a_wo + h_a

其中：
- u_a_wo 是 HQP/QP 的主控制输入变量
- h_a 是映射到执行器空间的补偿项

【平滑性要求】
当前你不是只要“算出来”，而是要“平滑地算出来”。
因此 hqp_routeB_solve.m / hqp_multi_level_solve.m 中必须加入或完善：

一级任务 / 硬约束：
- 动力学一致性
- 边界约束（张力 / 力矩上下限）

二级任务：
- 末端任务跟踪
- slack 最小化

三级或代价项：
- 最小化 ||u_a_wo||^2
- 最小化 ||qdd||^2
- 最小化与上一时刻解的差异：
  ||u_a_wo - u_a_wo_prev||^2
  ||qdd - qdd_prev||^2

必须加入：
- previous solution memory
- diagnostics:
  * task_residual
  * dyn_residual
  * slack_norm
  * tension_margin
  * torque_margin
  * du_norm
  * dqdd_norm
  * solver_status
  * fail_reason

【需要改动】
建议修改或新增：
- scripts/demo_dynamics_routeB_planar.m
- src/simulate_routeB_step.m
- src/hqp_routeB_solve.m
- src/hqp_multi_level_solve.m
- src/task_space_pd_controller.m
- src/pin_get_pose_jacobian_terms.m
- src/pin_get_dynamics_terms.m

--------------------------------------------------
五、Pinocchio 接入要求
--------------------------------------------------
Pinocchio 必须成为主路径，而不是装饰性的 fallback。

【必须新增 URDF-Pinocchio 最小测试】
第一次必须先做下面这个最小测试，并把它加入 task：
1. buildModelFromUrdf
2. neutral(q)
3. forwardKinematics
4. computeJointJacobians
5. getFrameJacobian
6. crba + rnea

建议新增：
- tests/test_pinocchio_urdf_minimal.m

测试内容至少包括：
- 能成功构建 model/data
- neutral q 维度正确
- FK 能读到当前末端 frame 位姿
- Frame Jacobian 维度正确且数值有限
- crba 返回的 M_mass 维度正确
- rnea 返回的 h 维度正确且数值有限

若该测试失败：
- 必须明确报错位置和失败阶段
- 不允许静默回退到 DH 主链

【对 URDF 主链的处理】
当 n_m = 6 且启用真实机械臂后：
- 不允许自己再用简单数学公式算当前几何尺寸、末端位置、末端姿态；
- 不允许自己再用手写 DH Jacobian 做主路径；
- 必须统一调用刚写好的 Pinocchio 封装函数。

--------------------------------------------------
六、MuJoCo 物理仿真后端
--------------------------------------------------
当 qdd 和 u_a 的控制链平滑后，新增 MuJoCo 仿真后端。

目标不是替换控制器，而是：
- 控制器负责算 qdd / u_a
- MuJoCo 负责前向物理推进，返回 q_next / qd_next

建议新增：
- src/simulate_mujoco_step_planar.m
- src/pack_mujoco_payload_from_routeB.m
- 若需要 Python bridge：
  - python/mujoco_bridge_step.py

【关键要求】
在 simulate_routeB_step.m 的最末尾，必须将 MuJoCo Python 引擎需要调用的量单独封装为一个子函数，不要把 MuJoCo 打包逻辑散落在主控制流程中。

这个子函数至少打包：
- q
- qd
- u_a
- cable tensions
- arm torques
- dt
- gravity / microgravity flag
- actuator ordering
- 可选 qdd

demo_dynamics_routeB_planar.m 中新增 backend 选项：
- "integrator"
- "mujoco"

要求：
1. integrator 路径先可跑；
2. mujoco 路径实现最小闭环；
3. 若 URDF -> MuJoCo 转换或依赖问题未完全跑通，必须如实标注未完成项，不要伪造结果。

--------------------------------------------------
七、测试与交付
--------------------------------------------------
请实际修改代码，并至少补下面这些测试：

1. IK 三模式基础测试
- tests/test_velocity_ik_mode1.m
- tests/test_velocity_ik_mode2.m
- tests/test_velocity_ik_mode3.m

2. workspace 分层测试
- tests/test_workspace_geom_reachable_planar.m
- tests/test_workspace_tension_feasible_planar.m

3. Route-B / dynamics 单步测试
- tests/test_routeB_single_step_planar.m
- tests/test_routeB_smoothness_terms.m

4. Pinocchio 最小测试
- tests/test_pinocchio_urdf_minimal.m

5. MuJoCo smoke test（若后端可运行）
- tests/test_mujoco_backend_smoke.m

--------------------------------------------------
八、最终输出要求
--------------------------------------------------
最后必须输出一份简洁报告，内容包括：
1. 修改了哪些文件
2. 为什么这样设计
3. mode3 的二级任务怎么定义
4. workspace 现在如何分层
5. Route-B 主链现在如何走
6. Pinocchio 如何成为 IK / FK / Jacobian / dynamics 主路径
7. MuJoCo payload 子函数如何封装
8. 还存在哪些限制 / 下一步建议

--------------------------------------------------
九、验收标准
--------------------------------------------------
1. mode1:
   - 机械臂冻结
   - 平台可通过速度级 IK 平滑将末端推向目标

2. mode2:
   - 平台冻结
   - 机械臂可通过速度级 IK 平滑将末端推向目标

3. mode3:
   - cooperative 正常工作
   - 一级任务优先满足末端任务
   - 二级任务确实起作用，但不能压制一级任务

4. workspace:
   - 能分别输出三种 mode 的几何可达工作空间
   - 能再叠加张力/力矩可行性
   - 不再混成单一标签

5. dynamics:
   - 每个控制周期都真实使用当前 q、qd 进入 Pinocchio 主路径
   - 能算出有限且较平滑的 qdd 与 u_a
   - 有清晰 diagnostics
   - demo_dynamics_routeB_planar.m 可运行

6. MuJoCo:
   - 至少实现最小可运行闭环接口
   - 若有依赖问题，必须诚实标注未完成项

现在开始直接改代码。
若发现当前某些函数职责混乱，请在尽量少破坏现有接口的前提下做局部重构。
优先保证主链路正确，再做风格清理。

<!-- [ADDED-2026-03-08-END] -->

[MAINT-NOTE-2026-03-08]
If any statements in Section 10 lines 414~807 conflict with
REWRITE-10.2/10.3/10.4 (lines 807~950), the rewrite content has priority.



[REWRITE-10.2-IK任务定义]

将原先所有把 planar / IK 主任务写成 [x, y, psi]^T 的表述，统一改为：

IK 主任务定义为末端三维位置任务：
x_task = [x; y; z]

也就是说，速度级 IK 的主误差只包含末端位置误差：
e_pos = p_des - p_cur = [x_des - x_cur; y_des - y_cur; z_des - z_cur]

命令任务速度采用：
xdot_cmd = xdot_d + Kp * (p_des - p_cur)

速度级 IK 主公式保持：
qdot = J^+ * xdot_cmd + (I - J^+ J) * qdot_0

其中：
- 主任务只负责跟踪末端位置 [x, y, z]
- 不再把 psi 作为主任务分量
- 不允许再在主误差中写入 psiDesired
- 若需要限制末端偏航 psi，只能作为软约束、零空间正则项或限幅项处理，不能作为“必须达到指定值”的一级任务


[REWRITE-10.2-Jacobian来源]

将原先“若做 planar 任务，从 J6D 中抽取 x, y, wz 组成 J_task”的表述，改为：

IK 部分的 Jacobian 必须明确来自 Pinocchio 解析结果，不允许继续用手写 DH Jacobian 作为主路径。
调用链必须清楚写出来：
1. pin.forwardKinematics(model, data, q)
2. pin.updateFramePlacements(model, data)
3. pin.computeJointJacobians(model, data, q)
4. pin.getFrameJacobian(model, data, ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

当前版本的 IK 主任务是末端三维位置任务，因此：
- 先获取末端完整 6D Jacobian J6D
- 再从 J6D 中抽取平移的 x、y、z 三行
- 组成位置任务 Jacobian J_task_pos

即：
J_task_pos = J6D([vx, vy, vz], :)

不要再使用 [x, y, psi] 对应的 Jacobian 抽取逻辑。
不要再把绕 z 的角速度行作为主任务 Jacobian 的一部分。


[REWRITE-10.2-mode说明]

对三种 mode 的说明补充如下：

mode1: platform-only
- 机械臂冻结，qdot_m = 0
- 仅平台变量参与
- 一级任务仍然是末端三维位置 [x, y, z] 跟踪
- 若需要限制平台/末端偏航 psi，只能作为二级正则项，而不是主任务目标

mode2: arm-only
- 平台冻结，qdot_f = 0
- 仅机械臂关节参与
- 注意：mode2 依然是在“解机械臂关节角/关节速度”
- 但其一级任务是通过这些关节变量去满足末端三维位置 [x, y, z] 跟踪
- 不要把 mode2 错写成“只跟踪 x,y,psi”
- 若 z 不进入误差函数，则当前 mode2 的任务定义就是错误的

mode3: cooperative
- 一级任务：末端三维位置 [x, y, z] 跟踪，必须优先满足
- 二级任务：posture regularization + joint-limit avoidance
- psi 不作为必须达到指定值的主任务量
- 若需要对 psi 做一定限制，应通过零空间项或软约束实现，例如：
  * 尽量保持接近当前/初始 yaw
  * 避免过大 yaw 偏转
  * 不允许 yaw 超出给定安全范围
- 但不能把 psiDesired 当成一级任务目标强行跟踪


[ADD-10.2-psi补充说明]

补充说明：
当前项目的 target 是 3D 点目标 [x_des, y_des, z_des]。
因此，IK 主任务中不应凭空引入 psiDesired 作为第三个误差分量。

如果代码中出现 psiDesired，则只能有两种合法用途：
1. 作为二级正则目标（例如尽量保持当前/初始 yaw）
2. 作为约束边界参考（例如 yaw 不超过某个阈值）

psiDesired 不得进入一级位置任务误差：
e_primary != [x_err; y_err; psi_err]

一级主误差必须为：
e_primary = [x_err; y_err; z_err]


[REWRITE-10.3-workspace判据]

将 workspace 第一层几何可达判据中的 IK 收敛条件补充明确为：

geometry-reachable workspace 的判据基于“末端三维位置任务 IK 是否收敛”：
- 给定目标 p_des = [x_des, y_des, z_des]
- 从指定初值出发，速度级 IK 在最大迭代数内收敛
- 位置误差 ||p_des - p_cur|| 低于阈值
- 全程没有 Jacobian 数值崩溃、变量越界、NaN

注意：
- 第一层几何可达判据不要求末端达到指定 psi
- psi 只能作为附加限制或二级评估指标，不能取代 z 成为第三个主任务量


[REWRITE-10.4-RouteB误差定义]

将原先“若当前主线仍是 planar：e = [ex; ey; epsi]”改为：

当前版本 Route-B 若与 IK / workspace 主链保持一致，则末端主任务误差应定义为三维位置误差：
e = [ex; ey; ez]

对应任务空间参考加速度：
a_des = xdd_d + Kd * (xd_d - xdot) + Kp * e

其中：
- xdot 为末端三维线速度
- e 为末端三维位置误差
- 当前阶段不要把 e 的第三项写成 epsi

若后续确实需要对 psi 做限制，应作为附加软任务、零空间正则项或单独低优先级任务，而不是替代 z 进入主任务

[ADDED-2026-03-09-RouteB-PRIORITY-CLARIFY]

补充消歧说明（避免误读）：
- 924~935 行的改写目的，是统一 **末端误差定义口径** 为 `e=[ex;ey;ez]`，禁止回退到 `epsi` 口径。
- 该段并不单独规定 Route-B 多层求解中的“任务优先级顺序”。
- 当 Route-B 层级设计需要明确优先级时，按当前工程主线执行：
  1) 先满足动力学一致性与物理边界；
  2) 再优先维持平台姿态（平台通道高优先）；
  3) 末端跟踪作为后续层/软任务在可行域内最小化误差。
- 同时保持误差量口径不变：末端位置误差始终为 3D `xyz`。


[ADD-10.4-偏航限制口径]

补充说明：
本项目当前对 psi 的要求是“存在一定限制”，而不是“必须达到某个指定值”。

因此在 Route-B / IK / workspace 的当前主线中，psi 的处理原则统一为：
- 不作为一级必须跟踪的目标
- 不定义强制性的 psiDesired 进入主误差
- 可作为：
  * 姿态正则项
  * yaw 限幅约束
  * 零空间优化项
  * 低优先级附加任务

Codex 在实现时，不要再把 psi 写成与 x/y/z 同优先级的主任务分量。



## 12.1 [ADDED-2026-03-08] 张力下界安全裕量修复：从“最小作动能量”升级为“安全预紧 + 平滑分配”

<!-- [ADDED-2026-03-08-TENSION-MARGIN-FIX-BEGIN] -->

> [新增注释] 经过对当前 `demo_dynamics_routeB_planar.m` 及其所调用的 `hqp_routeB_solve.m / hqp_multi_level_solve.m / simulate_routeB_step.m` 的检查，当前 Route-B 求解器虽然已经加入了 `slack`、`prev_u_a_wo / prev_qdd` 平滑项和多层 QP 外壳，但其张力分配主目标本质上仍是“**最小作动能量/最小无偏作动范数**”。在微/零重力、无显著外载的场景下，这会自然导致某些索长期贴近张力下界。该行为对“数学可行”并不矛盾，但对后续 MuJoCo 高保真闭环与正式验证不安全，因此本节对 Route-B 的张力分配目标做明确修正。

### 12.1.1 当前问题的根因归类

1. 当前 HQP / QP 的主目标并非“维持安全预紧”，而是倾向于：

- 最小化 $|u_{T,wo}|^2$；
- 最小化 $|u_{m,wo}|^2$；
- 或最小化整体 $|u_{a,wo}|^2$；
- 同时用少量平滑项抑制与上一时刻的差异。

1. 在微/零重力条件下：

- 平台 + 机械臂不需要依靠绳索去持续悬吊重力；
- 若末端任务加速度和外载都较小，则满足动力学与任务约束所需的广义扳手较小；
- 优化器会自然倾向于把若干根索压到最小张力附近；
- 因此“长期贴下界”不是偶然 bug，而是当前目标函数的直接结果。

1. 结论：

- 当前 Route-B 可作为“最小作动能量基线版本”；
- 但不能继续作为 MuJoCo 正式验证前的最终张力策略；
- 必须将“远离张力下界”写成明确的求解目标，而不是只停留在分析结论里。

### 12.1.2 正式确立的主方案（必须实现）

本项目后续 Route-B 张力调整方案，正式确定为：

**方案 A：显式安全裕量约束 + 参考张力正则 + 时间平滑**

即，除原有物理约束

$$
\underline f \le f \le \bar f
$$

外，进一步强制引入“工作安全下界”

$$
f \ge \underline f + \Delta f_{safe}
$$

其中：

- $f = u_{T,wo} + h_{a,T}$ 为实际绳索张力；
- $\underline f$ 为物理最小张力；
- $\Delta f_{safe} > 0$ 为额外安全裕量；
- $\underline f + \Delta f_{safe}$ 称为“工作安全下界”，而非仅仅“物理不断绳下界”。

同时，不再把张力目标写成单纯

$$
\min \|u_{T,wo}\|^2
$$

而改为围绕一个参考预紧力分布：

$$
f_{ref} = \underline f + \Delta f_{safe} + \Delta f_{center}
$$

其中：

- $\Delta f_{center} \ge 0$ 为附加居中预紧量；
- 若暂时不做复杂分配，可先取各索相同标量值；
- 后续如有需要，再扩展为与构型/模式相关的向量参考值。

对应新的二级/三级代价项，采用：

$$
\min \; w_s \|s\|^2 + w_f \|f - f_{ref}\|^2
+ w_u \|u_{a,wo} - u_{a,wo}^{prev}\|^2
+ w_q \|\ddot q - \ddot q^{prev}\|^2
+ w_a \|\ddot q\|^2
+ w_\tau \|u_{m,wo}\|^2
$$

说明：

1. $f$ 必须用真实张力

$$
f = u_{T,wo} + h_{a,T}
$$

来写，不要再仅对 $u_{T,wo}$ 做“最小范数”；

2. $|f-f_{ref}|^2$ 是当前主方案的核心，作用是维持内部预紧/自应力，而不是让索长期贴边；
3. $|u_{a,wo}-u_{a,wo}^{prev}|^2$ 与 $|\ddot q-\ddot q^{prev}|^2$ 保留，用于时间平滑；
4. 若当前版本不方便一次性改成完整多层 HQP，可先在现有 `quadprog` 结构中实现该代价项，再逐步分层。

### 12.1.3 为什么正式选方案 A，而不是只加 barrier

本项目当前优先采用“显式安全裕量约束 + 参考张力正则”，而不是仅靠 barrier，原因如下：

1. **可解释性强**

- 能清楚地区分：
  - 物理下界 $\underline f$
  - 工作安全下界 $\underline f + \Delta f_{safe}$
  - 目标预紧力 $f_{ref}$
- 便于在 MATLAB 日志、CSV、MuJoCo 验证中统一解释。

1. **数值上更稳**

- 仅靠 barrier 容易在接近边界时产生刚性较强的数值行为；
- 而显式下界 + 二次正则更容易直接并入当前 `quadprog` 体系。

1. **更符合微重力场景**

- 微重力下，关键问题不是“抗重力悬吊”，而是“主动维持足够的内部预紧与结构稳定性”；
- 因此引入 $f_{ref}$ 比继续追求最小张力更合理。

### 12.1.4 对现有 TaskBook 条目的明确修订

从本节开始，以下旧口径不再作为微重力 Route-B 的最终张力策略：

1. “Level 1 目标函数：$\min |u_{a,wo}|^2$”
2. “目标：$\min |s|^2 + \alpha|u_{T,wo}|^2 + \beta|u_{m,wo}|^2$”
3. 一切默认鼓励“张力越小越好”的表述

这些表述可保留为：

- 基线版本；
- 初始可行性版本；
- 论文公式对照版本；

但在当前项目的微重力正式主链中，必须改写为：

- 先满足动力学一致性和物理边界；
- 再满足工作安全下界；
- 再围绕参考预紧力分布做平滑优化。

### 12.1.5 Codex 必须做的代码级修改

#### Task 12.1.5.1 — 在 `hqp_routeB_solve.m / hqp_multi_level_solve.m` 中加入工作安全下界

将当前张力约束

$$
\underline f \le f \le \bar f
$$

升级为

$$
\underline f + \Delta f_{safe} \le f \le \bar f
$$

并通过

$$
f = u_{T,wo} + h_{a,T}
$$

映射到优化变量上。

要求：

- `cfg` 中新增：
  - `T_safe_margin`
  - `T_center_offset`
- 若 `T_safe_margin = 0`，则退化回旧版本，便于回归测试；
- demo 默认不允许再取 0。

#### Task 12.1.5.2 — 在目标函数中加入参考张力正则

在 Route-B / HQP 代价中新增：

$$
\|f-f_{ref}\|^2
$$

其中

$$
f_{ref} = \underline f + \Delta f_{safe} + \Delta f_{center}
$$

要求：

- 支持标量和向量两种 `f_ref` 配置；
- 默认先实现常值参考张力；
- 优先以“真实张力 $f$”建模，不要只对 $u_{T,wo}$ 做正则。

#### Task 12.1.5.3 — 保留并强化时间平滑

保留已有：

$$
\|u_{a,wo}-u_{a,wo}^{prev}\|^2,\qquad \|\ddot q-\ddot q^{prev}\|^2
$$

并新增以下要求：

- 若某一步最小张力裕量骤降，诊断中必须能直接看出来；
- 平滑项权重必须通过 `cfg / opts` 可调；
- 不允许把平滑项写死在脚本内部。

#### Task 12.1.5.4 — diagnostics 必须升级

除已有 residual / slack / solver_status 外，强制新增并导出：

- `tension_margin_low_min`
- `tension_margin_low_mean`
- `num_steps_near_lower_bound`
- `near_lower_bound_ratio`
- `worst_cable_index`
- `tension_spread`
- `tension_balance_metric`
- `f_ref`
- `T_safe_margin`

并在 `demo_dynamics_routeB_planar.m` 结束时直接给出：

- `allow_mujoco_smoke_test`
- `allow_formal_mujoco_validation`

### 12.1.6 MuJoCo 准入门槛的明确更新

从本节开始，MuJoCo 的准入规则明确分成两层：

1. **允许进入 smoke test**

- Route-B 主链可运行；
- `qdd`、`u_a` 有限；
- 残差小；
- 接口贯通；
- 即使张力裕量仍偏小，也允许进入最小闭环联调。

1. **不允许进入 formal validation**
    除 smoke test 条件外，还必须同时满足：

- 已启用工作安全下界：

  $$
  f \ge \underline f + \Delta f_{safe}
  $$

- 最小张力裕量不再长期贴边；

- 多步闭环下张力、$\ddot q$、$u_a$ 平滑；

- 对初值、步长、目标点小范围变化具有基本鲁棒性。

### 12.1.7 Definition of Done（本节新增验收标准）

只有同时满足以下条件，才认为“张力下界安全裕量修复完成”：

1. Route-B 求解器中已经不再把“最小张力范数”作为唯一张力目标；
2. 已实现：
   - 显式工作安全下界
   - 参考张力正则
   - 时间平滑项
3. 多步仿真中：
   - `tension_margin_low_min` 明显大于 0；
   - 不再出现长期多步贴近下界；
4. `demo_dynamics_routeB_planar.m` 导出的 CSV / MAT 中包含新增张力安全 diagnostics；
5. 只有在满足以上条件后，MuJoCo 才允许从 smoke test 进入 formal validation。

<!-- [ADDED-2026-03-08-TENSION-MARGIN-FIX-END] -->
