# 对话压缩版：逆动力学、IK 与工作空间

## 0. 这次对话最后形成的总认识

这轮讨论最后沉淀出的核心框架是：

1. **原始 HCDR_HQP 代码里的“IK”不是传统几何闭式 IK**，而是
    **Pinocchio 提供 FK/Jacobian + HQP/QP 求解任务约束**，本质属于
    **微分/加速度层 IK + 全身逆动力学**。
2. 对你自己的 MATLAB 仓库，后续主链路被明确分成两层：
   - **几何层 / 工作空间第一层**：用 **速度级 IK** 判定“几何可达 = IK 有解”；
   - **动力学层 / whole-body dynamics**：再用 **Route-B / HQP** 求平滑的
      $u_a$ 和 $\ddot q$。
3. 三种 mode 的工作空间不能混成一个概念，后续统一分成：
   - `geom_reachable_*`
   - `wrench_feasible_static_*`
   - `overall_*`
      也就是**几何可达**与**静力/张力可行**必须分层。
4. 后期最关键的诊断结论是：
   - **mode2 的大误差不是“机械臂僵化”，而是一级任务定义错了**，很可能仍在解 $[x,y,\psi]$，而不是 $[x,y,z]$；
   - **mode1 / mode3 不是几何上没到，而是 demo 的成功判定把几何成功与张力可行、A2D、旧状态机混在了一起**。
      换句话说，**几何层和静力层被代码混用了**。
5. 在后续专门做 mode1 静力 WFW 排查时，最后又得到一个更硬的结论：
    **当前默认几何构型下，Mode1 的 3-DOF 静力正张力自平衡在非退化姿态下大概率本来就不存在；主要问题不再是求解器，而是几何本身在 $\tau_z$ 平衡上缺乏全正张力自应力。**
    所以后面工作的重点应从“继续调 solver”转向“构型筛选 / 锚点与走索几何设计”。

------

## 1. 对原始 HCDR_HQP-master 的理解

### 1.1 原项目里有没有 IK 和 workspace

结论是：

- 有 IK 相关实现

  ，但分两部分：

  - CDPR / 动平台部分：有经典的“位姿 $\to$ 索长 / 索方向 / 结构矩阵”的几何求解；
  - 机械臂 / 全身部分：没有单独的几何闭式 IK，而是 Pinocchio + Jacobian + HQP 的任务式求解。

- **没有显式的 workspace 采样 / 可达判定 / 可视化模块**。
   源码里和 workspace 有关的内容主要还是控制指标、奇异性规避、可操作度，不是完整的工作空间分析器。

### 1.2 动平台/CDPR 的几何 IK

平台位姿用齐次矩阵 $M$ 表示，拆成：

- 平移 $T$
- 旋转 $R$

第 $i$ 根索的动平台锚点世界坐标：

Pp,iw=T+RPp,iP_{p,i}^w = T + R P_{p,i}Pp,iw=T+RPp,i

固定端锚点记作 $P_{f,i}$，则索向量和索长为：

di=Pf,i−(T+RPp,i),Li=∥di∥d_i = P_{f,i} - (T + R P_{p,i}), \qquad L_i = \|d_i\|di=Pf,i−(T+RPp,i),Li=∥di∥

同时可构造结构矩阵 / 扳手矩阵 $W$：

wplatform=Wτw_{\text{platform}} = W \tauwplatform=Wτ

其中 $\tau$ 是绳张力向量，$W$ 每列由索方向和力矩臂组成。这部分是后续做静力可行工作空间时的基础，因为**任何静力 WFW 判定都离不开 $W$ 或其 2D/简化版本**。

------

## 2. 原项目中“逆运动学”真正求的是什么

这一段对话里最核心的澄清是：

> 代码并不是“给定末端位姿直接求关节角 $q$”，而是“给定末端目标后，构造任务空间期望加速度，再通过 HQP 求出满足任务与动力学约束的 $\ddot q$ 和执行器输入”。

### 2.1 HQP 的主决策变量

根据原 C++ 代码的维度与取段方式，决策变量可理解为：

x=[q¨ΔtΔτ]x = \begin{bmatrix} \ddot q \\ \Delta t \\ \Delta \tau \end{bmatrix}x=q¨ΔtΔτ

其中：

- $\ddot q \in \mathbb{R}^{13}$：平台 6 + 机械臂 7 的全身广义加速度
- $\Delta t \in \mathbb{R}^{8}$：绳张力修正量
- $\Delta \tau \in \mathbb{R}^{7}$：关节力矩修正量

更贴近论文符号时，也可以写成：

x=[q¨ua,wo],ua,wo=[fp,woτm,wo]x = \begin{bmatrix} \ddot q \\ u_{a,wo} \end{bmatrix} ,\qquad u_{a,wo}= \begin{bmatrix} f_{p,wo}\\ \tau_{m,wo} \end{bmatrix}x=[q¨ua,wo],ua,wo=[fp,woτm,wo]

也就是说，**论文式(18)里的 $\ddot q$ 与 $u_{a,wo}$ 本质上就是代码中 HQP 同时在求的那部分量**。

### 2.2 目标位姿如何变成任务约束

末端误差先由当前位姿与目标位姿构造。原讨论里把这个分成：

1. 位置误差 $e_p$
2. 姿态误差 $e_R$

然后构造任务空间期望加速度：

ades=Kp⊙e+Kd⊙(−v)a_{\text{des}} = K_p \odot e + K_d \odot (-v)ades=Kp⊙e+Kd⊙(−v)

其中 $v$ 是末端当前 twist，等价于一个任务空间 PD。

接着把末端跟踪任务写成：

J(q)q¨+s=adesJ(q)\ddot q + s = a_{\text{des}}J(q)q¨+s=ades

严格说完整形式是：

Jq¨+J˙q˙=adesJ \ddot q + \dot J \dot q = a_{\text{des}}Jq¨+J˙q˙=ades

但代码里常做了简化，写成：

Jq¨+s≈adesJ \ddot q + s \approx a_{\text{des}}Jq¨+s≈ades

其中 $s$ 是 slack，用来缓和任务冲突。
 所以这本质上是**二阶微分 IK / 任务加速度层 IK**。

### 2.3 为什么它同时又像逆动力学

因为同一套 HQP 里还塞了动力学等式，例如：

M(q)q¨=STua,woM(q)\ddot q = S^T u_{a,wo}M(q)q¨=STua,wo

或者更一般地包含张力映射和关节力矩项的广义动力学约束。

最终发布给仿真器 / 控制器的输入是：

ua∗=ua,wo∗+hau_a^* = u_{a,wo}^* + h_aua∗=ua,wo∗+ha

也就是：
 **HQP 解的是不含补偿项的那一段，发布前再把重力/非线性补偿加回去。** 

### 2.4 平台动 / 机械臂动 / 协同 是怎么切换的

原项目不是靠几何 IK 分支切换，而是靠 HQP 里额外加“Fix Task”来锁住某一部分自由度：

- `platform-only`：锁机械臂，让平台承担主任务
- `arm-only`：锁平台，让机械臂承担主任务
- `cooperative`：放开两边，但可附加平台姿态或构型正则项

所以它其实是在用**约束分配自由度**，不是几何求不同支解。

------

## 3. 得到 $\ddot q$ 后，代码里如何得到位移和速度

这也是对话中很关键的一次澄清。

### 3.1 当前原项目主链并没有显式把 $\ddot q$ 积分成 $q_d,\dot q_d$

真实数据流是：

1. HQP 求出 $\ddot q$ 和 $u_{a,wo}$
2. 代码主要取的是后面的执行器输入部分
3. 加回补偿得到实际张力/力矩
4. 把它们直接发给仿真器
5. **仿真器自身积分动力学，更新 $q,\dot q$**

所以当前实现更像：

inverse dynamics / torque control\text{inverse dynamics / torque control}inverse dynamics / torque control

而不是：

IK→(qd,q˙d)→joint servo\text{IK} \to (q_d,\dot q_d) \to \text{joint servo}IK→(qd,q˙d)→joint servo

也就是说，在原始代码主链里，**位移不是控制器自己从 $\ddot q$ 算出来的，而是物理引擎积分出来的**。

### 3.2 理论上若要从 $\ddot q$ 生成期望轨迹，应如何做

对话里明确给了离散积分口径：

q˙dk+1=q˙k+q¨kΔt\dot q_d^{k+1} = \dot q^k + \ddot q^k \Delta tq˙dk+1=q˙k+q¨kΔtqdk+1=qk+q˙kΔt+12q¨kΔt2q_d^{k+1} = q^k + \dot q^k \Delta t + \frac{1}{2}\ddot q^k \Delta t^2qdk+1=qk+q˙kΔt+21q¨kΔt2

但若整机含 free-flyer / quaternion，配置更新就不能简单加法，而应使用流形上的积分，例如 Pinocchio 的 `integrate()` 思路。
 对话同时指出：原仓库里虽存在积分辅助函数，但**没真正进入 whole-body 主控制链**。

------

## 4. 针对你自己的 MATLAB 仓库，后来确定的新主链路

后续对话逐渐把你的项目改造目标收束成一个统一口径：

### 4.1 几何层：速度级 IK 作为主路径

核心公式定为：

x˙cmd=x˙d+Kp(xd−xcur)\dot x_{\text{cmd}} = \dot x_d + K_p (x_d - x_{\text{cur}})x˙cmd=x˙d+Kp(xd−xcur)q˙=J+x˙cmd+(I−J+J)q˙0\dot q = J^+ \dot x_{\text{cmd}} + (I - J^+J)\dot q_0q˙=J+x˙cmd+(I−J+J)q˙0

要求：

- $J^+$ 默认用**阻尼伪逆**
- 加上越界、奇异、NaN、未收敛等失败原因标记
- 迭代推进：

qk+1=qk+Δt q˙kq_{k+1} = q_k + \Delta t\, \dot q_kqk+1=qk+Δtq˙k

- 零空间项 $\dot q_0$ 不能简单置零，特别是 mode3。

### 4.2 mode1 / mode2 / mode3 的重新定义

统一定义为：

- **mode1: platform-only**
   机械臂冻结，只允许平台变量参与速度级 IK
- **mode2: arm-only**
   平台冻结，只允许机械臂关节参与速度级 IK
- **mode3: cooperative**
   平台和机械臂同时参与
   一级任务：末端到达
   二级任务：姿态/构型正则化 + joint-limit avoidance

mode3 推荐的零空间目标写成：

q˙0=−KpostW(q−qref)+q˙avoid\dot q_0 = -K_{post} W (q-q_{ref}) + \dot q_{avoid}q˙0=−KpostW(q−qref)+q˙avoid

强调：

- 不能把“最小张力”直接塞进几何层 IK 的一级主目标
- cooperative 可支持：
  - 默认：null-space projector
  - 可选：小型两层 HQP
     但默认先保证 null-space 路径稳定。

### 4.3 几何 / Pinocchio / URDF 的主路径口径

在真实 6R 机械臂主链中：

- FK
- Jacobian
- 工作空间几何可达判定
- Route-B 用到的动力学几何量

都应**统一走 URDF + Pinocchio**，而不是继续走手写 DH 主路径。
 `cfg.arm.DH` 最多保留为 fallback / 旧测试兼容，不再作为主路径。

同时明确了 Pinocchio 最小测试链：

1. `buildModelFromUrdf`
2. `neutral(q)`
3. `forwardKinematics`
4. `computeJointJacobians`
5. `getFrameJacobian`
6. `crba + rnea`

只要这六步通了，就足以支撑 IK 和 Route-B 主链。

### 4.4 动力学层：Route-B / HQP / Pinocchio / MuJoCo

对话最后给你的项目设定了这样一条路：

1. 每个控制周期都从当前 $(q,\dot q)$ 出发
2. 调 Pinocchio 得到：
   - 当前末端位姿
   - $J$
   - $\dot J \dot q$
   - 质量矩阵 $M_{mass}$
   - 非线性项 $h$
3. 构造任务空间误差和期望加速度：

ades=x¨d+Kd(x˙d−x˙)+Kpea_{des} = \ddot x_d + K_d(\dot x_d - \dot x) + K_p eades=x¨d+Kd(x˙d−x˙)+Kpe

1. 在 HQP 中解：

Jq¨+J˙q˙≈adesJ\ddot q + \dot J \dot q \approx a_{des}Jq¨+J˙q˙≈ades

和

Mmassq¨=STua,woM_{mass}\ddot q = S^T u_{a,wo}Mmassq¨=STua,wo

1. 最终：

ua=ua,wo+hau_a = u_{a,wo} + h_aua=ua,wo+ha

1. 再把 $u_a$ 交给积分器或 MuJoCo 做物理推进。

同时强调了“平滑”：

- 最小化 $|u_{a,wo}|^2$
- 最小化 $|\ddot q|^2$
- 最小化与上一时刻解的差异：

∥ua,wo−ua,woprev∥2,∥q¨−q¨prev∥2\|u_{a,wo} - u_{a,wo}^{prev}\|^2,\qquad \|\ddot q - \ddot q^{prev}\|^2∥ua,wo−ua,woprev∥2,∥q¨−q¨prev∥2

并输出 diagnostics。

------

## 5. 工作空间口径后续是怎么定下来的

这部分对话里有过几次修订，最后稳定为一个很明确的分层口径。

### 5.1 先分清两层

#### 第一层：geometry-reachable

工作空间的第一必要条件是：

> 给定目标点，从指定初值出发，速度级 IK 能在最大迭代数内收敛，且无越界/NaN/Jacobian 崩溃。

也就是：
 **“工作空间第一层 = 几何可达 = 速度级 IK 有解”**。

#### 第二层：static wrench-feasible

在几何可达之上，再叠加平台张力静力可行性、关节限位等约束。
 这时输出字段要分开，而不能只给一个 OK。

------

## 6. 微重力静力 WFW 的最终统一口径

这部分对话中，关于 static workspace 最后的版本大致是：

### 6.1 假设条件

统一取：

- $g = 0$
- $\dot q = 0$
- $\ddot q = 0$
- $w_{ext} = 0$

也就是：

- 微 / 无重力
- 无外接触载荷
- 只做**终点静力判定**
- 不做 dynamic workspace
- 不做路径过程验证

并且明确反复强调：

> 在这一口径下，不能把“质心变化”直接写成额外的重力矩或静力外载。
>  mode3 不是前两种乘积，原因是联合解存在性问题，而不是“质心变化额外产生了静力重力矩”。 

### 6.2 张力与力矩边界

默认配置：

- $T_{min} = 5\ \text{N}$
- $T_{max} = 500\ \text{N}$
- $\tau_{min} = -40\ \text{Nm}$
- $\tau_{max} = 40\ \text{Nm}$

但后续又明确修正：
 在这一节 static workspace 里，**机械臂关节力矩边界不作为主否决条件**，可以保留为记录项。真正的主判据还是：

- 几何可达
- joint limits
- 平台正张力静力可行性。

### 6.3 三种 mode 的 static workspace 最终定义

#### mode1

- 机械臂冻结在初始构型
- 平台变量参与
- 目标点属于 mode1 static workspace，当且仅当：
  1. 几何层可达
  2. 对应平台位姿下存在正张力静力平衡解

#### mode2

- 平台冻结在初始位姿
- 机械臂参与
- 主判据是：
  1. 机械臂几何可达
  2. joint limits 满足

关键修正：
 **mode2 不再对每个目标点额外做平台静力张力可行性否决。**
 平台固定在初始位姿，这不是 per-point 主判据。

#### mode3

- 平台与机械臂共同参与
- 对每个目标点必须真正做联合存在性判定
- 不能写成 mode1 × mode2，也不能写成并集或逻辑拼接
- 必须找到一组联合解：

(qf, qm, T)(q_f,\ q_m,\ T)(qf, qm, T)

同时满足末端到达、关节限位和平台正张力静力可行。

### 6.4 静力正张力判定推荐形式

对给定平台位姿 $q_f$，推荐做 margin-max 判定：

变量：$T,\gamma$

约束：

A2D(qf)T=0A_{2D}(q_f) T = 0A2D(qf)T=0Tmin+γ≤Ti≤Tmax−γT_{min} + \gamma \le T_i \le T_{max} - \gammaTmin+γ≤Ti≤Tmax−γγ≥0\gamma \ge 0γ≥0

目标：

max⁡γ\max \gammamaxγ

这样不只是可行/不可行二值，还能输出“张力安全裕量”热图。

------

## 7. 后期对 IK bug 的关键诊断结论

这是对话后半段最重要的一次代码逻辑校核。

### 7.1 mode2 为什么“看上去机械臂僵化”

结论是：

> 不是机械臂僵化，而是一级任务定义很可能还错着。

理由是：

- 你手动把 mode2 目标改远后，
- 实际结果表现为：
  - $x,y$ 误差很小
  - 只有 $z$ 差了约 $0.30\text{ m}$

这个模式非常典型地说明：

- 机械臂其实在动
- 但一级任务很可能仍在解 $[x,y,\psi]$
- 或第三维没有真正进入误差函数 / Jacobian

因此那一轮最关键的整改意见是：

1. IK 一级主任务必须改成

eprimary=[xerr,yerr,zerr]Te_{primary} = [x_{err}, y_{err}, z_{err}]^Teprimary=[xerr,yerr,zerr]T

1. `J_task` 必须从 Pinocchio 6D Jacobian 中抽**线速度三行**
2. 不允许继续把 `w_z` 当主任务第三行
3. $\psi$ 只能放到零空间二级项里，不能再当一级主任务。

### 7.2 mode1 和 mode3 为什么“明明到了还显示失败”

这里的判断是：

> 几何上其实已经到了，但 demo 的成功标记把几何成功和张力可行、A2D、旧 generator 标记、`max_iter` 状态机混在了一起。

因此要求把成功标记拆成三层：

- `ik_success_geom`
- `tension_feasible`
- `overall_ok`

并修正状态机：
 误差达到阈值时立刻退出，不能最后还被 `max_iter` 覆盖成失败。

------

## 8. 后期对 Mode1 / Mode3 WFW 形状的认识

这部分对话从“为什么 Mode1 图不对称”“为什么像平面而不是星形”逐渐演化成两个层面的认识：

### 8.1 早期可视化口径修正

较早时的判断是：

- **Mode1 在 3D 中本来就应该是一个固定 $z$ 的平面层**，因为 mode1 没有独立的 $z$ 自由度；
- 真正该看的不是 3D 体，而是**固定 $z$ 的 XY 截面图**；
- 如果对所有 $\psi$ 做并集，那么原本某个 fixed-$\psi$ 下的“星形边界”会被扫成更圆滑、更饱满的 union 区域；
- 所以必须严格区分：
  - **Constant Orientation Workspace (fixed psi / CWS)**
  - **orientation-union workspace**

同时强调：

- fixed-$\psi$ 图才适合看“星形”
- union-over-$\psi$ 图适合看姿态自由后的覆盖范围。

### 8.2 后来更深入的 Mode1 诊断：问题不只是图怎么画，而是物理上可能真的为空

在后面专门做 `cpp_workspace_static_mode1/` 排查后，结论升级为：

- 3D$\to$2D 张力映射 bug 确实存在，也修了；
- 但在当前默认样本位姿下，很多情况下 $L_{xy}=L_{3d}$，投影系数 $c_i=1$，说明这条 bug 在当前构型里属于“休眠 bug”；
- 真正的主矛盾是：
  - 非退化姿态下，$A_{2D}$ 为 rank 3
  - 但“存在全正自应力解”的诊断仍是 infeasible
  - $\psi = 0^\circ$ 这类 rank 2 姿态在 force-only 下可行
  - 这说明问题主要集中在 **$\tau_z$ 平衡**，不是 XY 平衡，也不像是 OSQP 设置错了

最终判断变成：

> 修正映射后，Mode1 fixed-psi 仍为空；更可能是当前几何本身不支持非退化偏航姿态下的全正张力自平衡，而不是数值设置问题。
>  因此下一步不该继续调 solver，而应转向索路由 / 锚点布局 / 平台锚点相对中心位置的重设计。 

------

## 9. 这整轮对话最终留给后续工作的“硬结论”

### 9.1 已经基本确定的事情

1. **原 HCDR_HQP 的 IK 不是几何求角，而是 HQP 约束求 $\ddot q$ / $u_{a,wo}$。**
2. **你的 MATLAB 仓库应采用“几何层速度级 IK + 动力学层 Route-B/HQP”的两层架构。**
3. **几何层第一主任务必须是 $[x,y,z]$ 位置任务，而不是旧的 $[x,y,\psi]$。**
    $\psi$ 只能降级为 secondary / null-space regularization。
4. **工作空间必须分层输出，不能再把 geometry、tension feasibility、overall success 混成一个标签。**
5. **mode2 的主问题是一级任务定义错，不是“串联机械臂 IK 不成熟”。** 
6. **mode1 / mode3 之前的大量“失败”有相当一部分是判定逻辑错，不等于几何没到。** 
7. **在微重力 static workspace 这条线上，mode3 不是 mode1 和 mode2 的乘积；必须做联合解搜索。** 
8. **当前默认几何下，Mode1 的非退化 3-DOF static positive-tension self-equilibrium 大概率本来就不存在。**
    这不是继续调求解器能解决的，后续应转向构型设计。 

### 9.2 还没真正完成、但方向已经定死的事情

1. 把 IK 主链彻底改成真正的三维位置主任务：
   - `pin_get_pose_jacobian_terms`
   - `build_task_jacobian_planar_from_pin`
   - `solve_velocity_ik_three_modes_planar`
      这些接口要彻底排除旧的 $[x,y,\psi]$ 口径。
2. 把 demo / workspace 输出彻底分层：
   - `geom_reachable_*`
   - `joint_limit_ok_*`
   - `wrench_feasible_static_*`
   - `overall_*`
3. 在 Mode1 / Mode3 的 WFW 上，区分：
   - fixed-$\psi$ CWS
   - union-over-$\psi$ workspace
4. 若继续研究 Mode1 静力 WFW，应转到“构型筛选器”上：
    对若干代表姿态直接检查

A2D(x,y,ψ)T=0,T>0A_{2D}(x,y,\psi)T = 0,\qquad T>0A2D(x,y,ψ)T=0,T>0

是否存在解，而不是继续执着于把星形图“画出来”。
 后者已经不是绘图问题，而是物理存在性问题。

------

## 10. 一句话总括这次对话

这次对话最终把问题厘清成了三件事：

1. **论文/原代码里的 IK，本质上是基于 Pinocchio + HQP 的微分 IK / 逆动力学，不是闭式几何 IK；**
2. **你自己的代码应该先用速度级 IK 解决几何可达，再用 Route-B / HQP 解决平滑的全身动力学控制；**
3. **在 static workspace 上，Mode1 当前最大的障碍不再是算法，而是构型本身对偏航力矩的正张力自平衡能力不足。**