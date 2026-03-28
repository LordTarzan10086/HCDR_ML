# C++ 静力工作空间项目交接摘要

## 1. 项目目标与硬约束

目标是为一个**平面化分析口径下的混合缆索驱动平台 + 机械臂系统**，建立一个**完全独立于 MATLAB** 的 C++ 工作空间分析工程，用于输出更真实、更密集的 WFW 点云结果。原始 `HCDR_v6-codex` 目录只允许作为**只读参考源**，不能调用其中 MATLAB 脚本/函数，也不能修改其文件；新实现必须放在独立目录中，例如 `cpp_workspace_static/`，并且应当能**单独编译、单独运行**。统一分析口径为：

- $g=0$
- $\dot q=0$
- $\ddot q=0$
- $w_{\text{ext}}=0$
- 不考虑运动过程
- 不考虑动态项
- 不把质心变化写成额外重力矩
- 机械臂关节力矩**不**作为 static workspace 否决条件
- 张力边界默认可配置，用户先后给过两版口径：早期要求默认 $T_{\min}=5,\text{N}$、$T_{\max}=100,\text{N}$，但后续配置模板里明确要求至少支持 `T_min = 5`、`T_max = 500`。

实现语言与依赖要求是：

- C++17
- 优先使用 Eigen、Pinocchio C++ API
- 直接读取 `kortex_description/robots/gen3_lite_gen3_lite_2f_local.urdf`
- 配置文件用 JSON 或 YAML
- 导出至少支持 CSV 和 PLY/XYZ 点云。

------

## 2. 三种模式的**最终建模口径**

### Mode 1: platform-only

机械臂冻结在初始构型 $q_{m,\text{init}}$，末端相对平台的局部偏置固定为 $p_{\text{tip}0}^P$。Mode 1 的**主结果不能锁死 $\psi$**。其主判据是：对每个目标末端点 $p_{\text{des}}$，是否存在 $\psi,x,y,T$ 使得

pdes=[xyz0]+Rz(ψ) ptip0Pp_{\text{des}} = \begin{bmatrix} x\\y\\z_0 \end{bmatrix} + R_z(\psi)\,p_{\text{tip}0}^{P}pdes=xyz0+Rz(ψ)ptip0P

且

A2D(x,y,ψ) T=0,Tmin⁡≤Ti≤Tmax⁡.A_{2D}(x,y,\psi)\,T=0,\qquad T_{\min}\le T_i\le T_{\max}.A2D(x,y,ψ)T=0,Tmin≤Ti≤Tmax.

推荐实现方式不是对 EE 逐点逆解，而是：

1. 扫 $\psi$；

2. 对每个 $\psi$ 扫平台中心 $(x,y)$；

3. 检查平台静力可行性；

4. 若可行，则映射得到

   pEEW=[xyz0]+Rz(ψ)ptip0P;p_{EE}^W= \begin{bmatrix} x\\y\\z_0 \end{bmatrix} + R_z(\psi)p_{\text{tip}0}^P;pEEW=xyz0+Rz(ψ)ptip0P;

5. 对全部 $\psi$ 取并集得到 **Mode1 EE union**。

Mode 1 必须区分两类输出：

- **主结果**：`mode1_ee_union_*`，这是末端 EE 工作空间；
- **诊断结果**：`mode1_platform_fixedpsi_*`，例如固定 $\psi=45^\circ$ 时的平台中心 CWS，这张图才是判断“星形/多边形边界是否合理”的依据，不能再拿 EE union 图去看星形对称性。

### Mode 2: arm-only

平台冻结在初始位姿 $q_{f0}$。从 URDF / Pinocchio 读取真实 joint limits，在关节空间高密度采样 $q_m$，计算

ptipP(qm)p_{\text{tip}}^P(q_m)ptipP(qm)

再变换到世界系：

pEEW=[x0y0z0]+Rz(ψ0) ptipP(qm).p_{EE}^W= \begin{bmatrix} x_0\\y_0\\z_0 \end{bmatrix} + R_z(\psi_0)\,p_{\text{tip}}^P(q_m).pEEW=x0y0z0+Rz(ψ0)ptipP(qm).

Mode 2 的判据就是几何可达 + joint limits 满足，不额外引入平台静力 per-point 否决。

### Mode 3: cooperative

Mode 3 不能锁死 $\psi$，也不能简单理解为 Mode1 × Mode2。主判据是：对每个目标末端点 $p_{\text{des}}$，是否存在 $q_m,\psi,x,y,T$ 使得

pdes=[xyz0]+Rz(ψ) ptipP(qm),p_{\text{des}} = \begin{bmatrix} x\\y\\z_0 \end{bmatrix} + R_z(\psi)\,p_{\text{tip}}^P(q_m),pdes=xyz0+Rz(ψ)ptipP(qm),

同时满足

A2D(x,y,ψ) T=0,Tmin⁡≤Ti≤Tmax⁡,A_{2D}(x,y,\psi)\,T=0,\qquad T_{\min}\le T_i\le T_{\max},A2D(x,y,ψ)T=0,Tmin≤Ti≤Tmax,

且 $q_m$ 满足 joint limits。推荐实现是先预计算机械臂局部点云 $S_a={p_{\text{tip}}^P(q_m)}$，再对每个 $\psi$ 计算平台静力可行集 $S_p(\psi)$，将 $R_z(\psi)S_a$ 与每个可行平台中心叠加，最后对全部 $\psi$ 取并集得到 `mode3_union_cloud`。

------

## 3. 平台静力可行性模块的目标定义

用户明确要求新建独立模块，例如 `platform_static_feasibility.h/.cpp`。输入为 $(x,y,\psi)$、平台几何参数、$T_{\min},T_{\max}$；输出为 `feasible`、`gamma`、可选 `T_star`。推荐判定是**gamma 裕量判定**：

- 变量：$T,\gamma$

- 约束：

  A2D(x,y,ψ)T=0A_{2D}(x,y,\psi)T=0A2D(x,y,ψ)T=0

  Tmin⁡+γ≤Ti≤Tmax⁡−γT_{\min}+\gamma\le T_i\le T_{\max}-\gammaTmin+γ≤Ti≤Tmax−γ

  γ≥0\gamma\ge 0γ≥0

- 目标：maximize $\gamma$

若外部 LP/QP 依赖不方便，可先实现 bounded self-stress feasibility，但结果必须稳定可信。后续真实实现里，Codex 最终走的是 OSQP / QP 路线。

------

## 4. 输出要求

用户要求三模式都支持**体素哈希 / voxel downsampling**，默认体素大小可配置，例如 `0.01 ~ 0.02 m`，并且应在生成过程中直接做 voxel hash，避免内存爆炸；每个点允许附加标签如 `psi`、`gamma`、`mode id`。

至少应导出：

- Mode1 主结果：`mode1_ee_union_cloud.csv/.ply`，以及 `XY/XZ/YZ` 投影 CSV；
- Mode1 诊断结果：`mode1_platform_fixedpsi_45deg_cloud.csv/.ply`、`mask/gamma` 等；
- Mode2：`mode2_cloud.csv/.ply` 以及 `XY/XZ/YZ` 投影；
- Mode3：`mode3_union_cloud.csv/.ply` 以及 `XY/XZ/YZ` 投影；
- 可选：`mode3_fixedpsi_45deg_*`。

README 必须说明：

1. 三种模式的判定方式；
2. 为什么 Mode1 主结果是 EE union；
3. 为什么星形图对应 `platform-center fixed-psi CWS`；
4. 各类输出文件分别表示平台中心图、EE 点云、union 结果、fixed-psi 诊断结果；
5. 原始 `v6-codex` 文件夹只读，不调用、不修改。

------

## 5. 已经落地的两个工程

### A. `cpp_workspace_static/`：三模式完整 C++ 工程

Codex 已在 `cpp_workspace_static/` 下新建独立 C++17 工程，直接读取 Pinocchio + URDF，不依赖 MATLAB 扫描，也没有修改原有 MATLAB 代码文件。该工程包含独立的：

- 平台静力判据模块（OSQP）
- 机械臂 FK / 采样模块（Pinocchio）
- Mode1 / Mode2 / Mode3 分析
- 体素去重
- CSV / PLY 导出
- 三视图投影导出。

已明确列出的新增/修改文件包括：

- `cpp_workspace_static/CMakeLists.txt`
- `cpp_workspace_static/README.md`
- `cpp_workspace_static/app/main.cpp`
- `cpp_workspace_static/config/workspace_static.json`
- `cpp_workspace_static/config/workspace_static_smoke.json`
- `cpp_workspace_static/include/...`
- `cpp_workspace_static/src/...`
- `cpp_workspace_static/tests/test_main.cpp`
- `cpp_workspace_static/scripts/build_vs2019.ps1`
- `cpp_workspace_static/scripts/run_smoke.ps1`。

构建链路最后采用的是 VS2019 + MATLAB 自带 CMake/Ninja + `hcdr_pin` conda 环境，期间遇到过 `boost_json` 链接问题，后切换到 `nlohmann/json` 规避。

### B. `cpp_workspace_static_mode1/`：只保留 Mode1 的独立工程

后来用户将工作缩小到**只修 Mode1**，Codex 又新建了 `cpp_workspace_static_mode1/`。该工程只保留 Mode1，重点修复：

- `EE union` 与 `platform fixed-psi` 的输出混淆；
- 扫描网格应基于平台中心，而不是 EE 偏置中心；
- 退化 `A_{2D}` 不应继续送入求解器；
- 自检输出与日志。

这个 Mode1-only 工程的主入口是 `main_mode1_workspace.cpp`，并带有专用测试、构建脚本和运行脚本。

------

## 6. 已验证通过的内容

### 三模式工程 `cpp_workspace_static/`

这版工程已完成以下验证：

- `workspace_static_tests.exe` 通过；
- `run_smoke.ps1` 通过；
- `workspace_static_validation.json` 运行通过；
- `workspace_static.json` 全量运行通过，耗时约 **19 分 43 秒**。

full run 关键结果是：

- `mode1_ee_union`：**31261 voxels**
- `mode1_platform_fixedpsi_45deg`：**0 voxels**
- `mode2`：**180847 voxels**
- `mode3_union`：**11695052 voxels**
- `mode3_fixedpsi_45deg`：**0 voxels**
- `mode1` 和 `mode3` 在当前静力判据下都只有 **3 个 psi 切片**有可行点。

结果质量说明里还明确写了：

- `psi=45^\circ` 和 `psi=90^\circ` 下，`center / edge_x / corner` 采样点都显示 `primal infeasible`；
- 从 `mode1_platform_psi_summary.csv` 和 `mode3_platform_psi_summary.csv` 看，可行切片集中在 $\psi=-180^\circ,0^\circ,180^\circ$；
- 这与仓库中已有 MATLAB 汇总一致，即历史结果里也存在 `mode1_fixedpsi_points = 0`、`mode3_fixedpsi_points = 0`。

### Mode1-only 工程 `cpp_workspace_static_mode1/`

这版工程测试通过，且输出文件、运行日志、导出文件列表都已生成。其 full run 关键数值是：

- fixed-psi 扫描总点数：`40401`
- `rank < 3` 剔除：`0`
- `sigma_min < eps_rank` 剔除：`0`
- `gamma >= 0` 可行：`0`
- `EE union raw / voxel`：`0 / 0`
- `psi = 0, ±180 deg` 整个切片 rank deficient，其余切片满秩但全部 `primal infeasible`。

用户要求 Codex 明确回答的三件事，Codex 已给出明确结论：

- fixed-psi 图画的是**平台中心**：是；
- EE union 图是否单独分开：是；
- 退化 `A_{2D}` 是否已硬剔除：是。

------

## 7. 已确认正确、可直接继承的认识

以下内容在对话里已经被反复确认，不应再推翻：

1. **Mode1 主结果和诊断结果必须分离**。`mode1_ee_union_*` 是末端工作空间；`mode1_platform_fixedpsi_*` 才是看星形/多边形边界的平台中心图。
2. **Mode1 的 EE union 本来就只落在单层 $z=z_0$ 平面上**，这不是 bug。
3. **对称几何下 $\psi=0^\circ / \pm180^\circ$ 出现 rank-2 退化**，这是符合直觉的。
4. `w_ext_2d=[F_x,F_y,M_z]` 接口已经接入静力等式，默认仍为零。
5. `force-only` 诊断链路、nullspace / positive-tension 诊断链路已经建好，用于区分“力平衡可行”与“完整 $F_x/F_y/\tau_z$ 平衡可行”。在力-only 诊断下某些退化姿态可行，而正式 3-DOF 主结果为空，说明主要矛盾在 $\tau_z$ 平衡。

------

## 8. 当前最关键的矛盾与风险

### 风险 1：`cpp_workspace_static` 与 `cpp_workspace_static_mode1` 的判据口径并不完全一致

三模式工程 `cpp_workspace_static` 的 full run 有非零 `mode1_ee_union` 和 `mode3_union`，并且明确写到“当前实现遵循主判据，只检查 `A_2D T = 0` 和张力边界；如果后续决定 `rank<3` 也必须否掉，结果会明显收缩”。而 Mode1-only 工程则把“rank<3 直接 false、sigma_min<eps 直接 false”作为**硬规则**接入主结果，最终得到全空集。也就是说，两个工程之间最大的分歧其实不是导出格式，而是**退化矩阵是否允许贡献自应力可行解**。这件事必须先统一，否则后续谁都算不准。

### 风险 2：`fixed-psi = 45^\circ` 为空，目前既可能是物理结论，也可能暴露几何问题

三模式工程给出的解释是：当前结果与仓库里的 MATLAB 汇总一致，因此 `45^\circ` 固定姿态为空**不一定是实现错误**；Mode1-only 工程则进一步指出：如果严格采用“满秩 + gamma 可行”口径，这套几何下整个 Mode1 都成空集，因此下一步应重点检查**索路由 / 锚点配对几何**，而不是只盯着求解器。

### 风险 3：Mode3 当前朴素算法在 full config 下非常重

对话里有一个非常关键的容量估计：`mode2_arm_samples = 200000` 且 `voxel_size = 0.01` 时，仅局部 arm cloud 就约有 **180,847** 个体素；如果 Mode3 继续采用“每个可行平台点 × 每个机械臂点”的笛卡尔叠加，就会不可接受。因此后续必须改成**按 z 分层的 2D Minkowski / 栅格并集 / 批量体素叠加**之类的算法。

------

## 9. 下一位接手者应优先做什么

### 第一优先级：先统一“正式可行性判据”

需要先决定，最终交付口径到底是：

- **A.** 只要满足张力边界下的自应力可行性就算可行，即使某些姿态 `rank<3`；
- **B.** 必须严格要求 `A_{2D}` 满秩且 `\sigma_{\min}\ge \varepsilon`，退化姿态一律否掉。

用户在 Mode1-only 阶段明确要求的是 **B**，并要求“退化 A2D 已被硬剔除”。如果最终仍沿用这个要求，那么 `cpp_workspace_static_mode1/` 的判据口径比三模式主工程更接近用户最近一次明确指令。

### 第二优先级：检查平台几何，而不是先继续调 OSQP

Mode1-only 工程给出的结论已经很直白：在严格口径下结果为空，下一步应检查**索路由 / 锚点配对几何**，尤其是：

- 机架锚点与平台附着点的配对是否正确；
- $A_{2D}$ 的力矩臂方向与符号是否一致；
- 3D 缆绳方向投影到 2D 后是否正确归一化；
- `psi=45^\circ`、`90^\circ` 下中心点 / 边点 / 角点为什么全部 `primal infeasible`。

### 第三优先级：Mode3 算法重构

在三模式工程里，Mode3 需要从当前的朴素笛卡尔组合改成更强的组合算法，否则 full config 虽然已经能跑，但代价高且不利于进一步提密度。对话里已经明确建议改成“按 z 分层的 2D Minkowski / 栅格并集”方向。

### 第四优先级：以现有诊断文件为起点，不要重新摸黑

已有高价值诊断文件包括：

- `platform_sample_pose_diagnostics.csv`
- `mode1_platform_psi_summary.csv`
- `mode3_platform_psi_summary.csv`
- `mode1_run_log.txt`
- `mode1_output_files.txt`。

这些文件已经能直接告诉接手者哪些 $\psi$ 切片可行、哪些采样点 infeasible、当前是否是几何问题而非导出问题。

------

## 10. 建议 antigravity / Codex 的接手策略

### 方案 1：以 `cpp_workspace_static/` 为主线继续

适用于目标仍是三模式完整交付。建议动作：

1. 先复现 `workspace_static.json` full run；
2. 读取现有 `*_psi_summary.csv` 和 `platform_sample_pose_diagnostics.csv`；
3. 明确是否接受 rank-deficient slice 参与可行性；
4. 若不接受，则把 `cpp_workspace_static` 的静力判据与 `cpp_workspace_static_mode1` 的硬剔除逻辑统一；
5. 重构 Mode3 的组合算法。

### 方案 2：以 `cpp_workspace_static_mode1/` 为主线先把几何查清

适用于目标是先回答“为什么 fixed-psi / Mode1 会空”。建议动作：

1. 复用 Mode1-only 的严格口径；
2. 检查索路由 / 锚点配对 / 力矩臂公式；
3. 逐点核对 $\psi=0^\circ,45^\circ,90^\circ$ 的中心、边、角采样；
4. 在确认几何没错后，再把修正反哺回 `cpp_workspace_static/`。

------

## 11. 当前可直接使用的运行入口

三模式工程已经给出的运行入口是：

- 构建：`powershell -ExecutionPolicy Bypass -File cpp_workspace_static\scripts\build_vs2019.ps1`
- 测试：`cpp_workspace_static\build\Release\workspace_static_tests.exe`
- smoke：`powershell -ExecutionPolicy Bypass -File cpp_workspace_static\scripts\run_smoke.ps1`
- full：`workspace_static_runner.exe --config .\config\workspace_static.json`。

Mode1-only 工程也有自己的构建 / smoke / full 输出目录、测试和日志，且桌面上还留有打包好的代码 zip：`cpp_workspace_static_mode1_code.zip`。

------

## 12. 一句话结论

**这次会话真正留下的不是“一个已经完全正确的最终答案”，而是两套都能跑的 C++ 资产和一个已经被逼出来的核心分歧：在严格 3-DOF + 非退化口径下，Mode1 很可能是空集，因此后续工作的关键不是继续堆导出和脚本，而是先统一静力判据，并重点检查平台索路由 / 锚点配对几何。** 三模式工程已经能 full run 并产出大规模点云；Mode1-only 工程已经把“平台中心 fixed-psi 图”和“EE union 图”彻底拆清，并把“退化 A2D 硬剔除”落实到了实现。







# 1. 项目目标

- 机器人结构：一个平面化分析口径下的混合缆索驱动平台 + 机械臂系统；平台由多根索约束，机械臂使用 URDF/Pinocchio 读取几何，Mode1 中机械臂冻结。
- 当前任务：先做一个完全独立于 MATLAB 的 C++ 工作空间分析工程；后续又把工作缩小到只修 Mode1，重点定位“真实空集”问题。
- 想最终实现什么：用 C++17 + Eigen + Pinocchio 做可信的静力工作空间分析，正确区分 EE union 与 fixed-psi platform-center CWS，并给出高密度点云、投影和诊断结果。

# 2. 当前建模假设

- 坐标系定义：世界系 W；平台系 P；末端/工具点相对平台的冻结偏置记为 p_tip0^P；平台位姿用 [x, y, z0] 和绕 z 的 psi 表示。
- 广义坐标定义：在 Mode1 下只保留平台平面自由度 q_f = [x, y, psi]；机械臂关节 q_m 固定在初始构型。
- 末端任务变量：p_EE^W = [x, y, z0]^T + Rz(psi) * p_tip0^P。
- 索长/关节角/平台位姿之间关系：平台几何模块根据机架锚点、平台附着点、[x,y,psi] 计算每根索的 3D 向量、平面投影、长度、平面单位方向和 A_2D。
- 是否平面化：是。主静力判据是平面 3-DOF：Fx/Fy/tau_z。
- 是否考虑重力/外力/弹性/滑轮效应：默认 g=0、qdot=0、qdd=0、w_ext=0；不考虑弹性，不考虑机械臂重力矩否决，不做复杂滑轮摩擦/包角模型，只保留简化几何与平面投影系数。

# 3. 当前代码结构

- 主入口：
- 三模式原型入口：main.cpp
- Mode1 定向入口：main_mode1_workspace.cpp
- 关键配置项：
- T_min / T_max
- eps_rank
- fixed_psi_deg
- psi_samples
- mode1_xy_resolution
- z0
- frameHalfLength
- voxel_size
- w_ext
- diagnostic_force_only_mode
- force_only_torque_row_norm_eps

# 4. 已经确认正确的内容

- 哪些公式/模块你认为已经没问题：
- Mode1 主结果与诊断结果的文件命名、输出口径已经分开，不再混淆 EE union 和 platform fixed-psi CWS。
- rank < 3 或 sigma_min < eps_rank 的硬剔除已经接入主结果，不再把退化 A_2D 混进正式 WFW。
- w_ext_2d = [Fx, Fy, Mz] 的接口已经接入静力等式，默认仍为零。
- force-only 诊断模式、nullspace/positive-tension 诊断链路已经建好。
- 哪些结果与理论一致：
- psi = 0 / ±180 deg 一类姿态出现 rank-2 退化，和对称几何的直觉一致。
- 在 force-only 诊断下，退化姿态可以出现可行解，而正式 3-DOF 主结果仍为空，这与“主要矛盾在 tau_z 平衡”一致。
- Mode1 EE union 本来就应落在单层 z=z0 平面上，这一点已与实现和导出一致。

# 5. 当前主要问题

- 问题1：
- 现象：Mode1 fixed-psi 与 Mode1 EE union 在严格 3-DOF 主判据下仍然是空集。
- 预期：至少在某些 psi 下应出现非空的平台中心可行域，并映射成非空 EE union。
- 已尝试：修复 A_2D 退化剔除；独立拆分 Mode1；接入 w_ext；加入 force-only 诊断；加入 nullspace/positive-tension 诊断；重新跑 full/smoke/test。
- 问题2：
- 现象：原先怀疑的 3D->2D 张力映射 bug 修掉后，默认几何下结果几乎不变。
- 预期：如果原 bug 是主因，修正后可行域应显著变化。
- 已尝试：把优化变量统一成真实张力 T_i，并让 A_2D 乘上 c_i = L_xy / L_3d；对典型姿态输出 L_xy、L_3d、c_i、旧/新列范数。结论是默认几何下 L_xy = L_3d、c_i = 1，所以这个 bug 在当前参数下是“休眠 bug”。

# 6. 关键实验与结论

- Case A：先完成三模式原型 cpp_workspace_static，验证工程链路、URDF 读取、CSV/PLY/投影导出、Mode2/Mode3 点云生成都能跑通。
- Case B：把工作收缩为专门的 cpp_workspace_static_mode1，严格修 Mode1。在修正映射、加入 rank/sigma 硬剔除、接入 w_ext 和 nullspace/force-only 诊断后，主结果仍为空，但诊断表明更可能是几何本身不支持非退化姿态下的全正张力自平衡，而不是求解器数值错误。
- 哪些参数一改就崩：
- psi 非常敏感，接近 0/±180 deg 时会直接进入 rank-2 退化分支。
- 机架/平台在 z 向的相对几何一旦改变，c_i = L_xy/L_3d 就会马上变化，直接影响 A_2D 的物理含义。
- 哪些参数最敏感：
- psi
- 机架半尺寸 / 平台附着点几何
- pulley_spacing 与平台上下附着点高度关系
- T_min/T_max
- eps_rank

# 7. 下一步做什么

- 帮你定位数值问题：
- 优先检查默认机架锚点与平台附着点的几何定义，尤其是 z 向高度关系和索路由是否真的符合物理原型。
- 对若干典型姿态直接人工核算 A_2D 每列的力矩方向与符号，确认不是索编号/附着点顺序错位。
- 在不放宽主判据的前提下，继续用 positive self-stress 诊断区分“几何无解”和“bounds 无解”。
- 帮你重构工作空间求解流程：
- 保留 Mode1 专用工程，继续把“主结果”和“诊断结果”彻底分层。
- 若后续确认几何模型要改，先只改 platform_geometry 和配置，不碰求解器接口。
- 等 Mode1 非空原因完全搞清楚后，再决定是否回到三模式总工程合并。