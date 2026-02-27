# TASKBOOK.md — 平面HCDR（微重力）整体运动学/动力学/工作空间（Route B）

## 0. Scope & Hard Constraints（不可违反）
- 平面 CDPR 平台 + 串联机械臂，微/无重力环境（默认 g=0）。
- Part A 只允许参考仓库当前版本（zip: HCDR_ML-copilot-refactor-hcdr-statics-checks）。
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