# 论文 Chapter 5 实验覆盖度核查（v6-codex 当前结果）

- 核查时间：2026-04-28
- 论文：`docs/Whole-body control of redundant hybrid cable-driven robot with manipulator hierarchical quadratic programming approach.pdf`
- 主要已有结果目录：
  - `results/tracking/routeb_trajectory_suite_20260416_105416`
  - `results/tracking/routeb_trajectory_suite_20260416_103205/paper_5_2_2_tracking_report_framework.md`
  - `results/avoidance_experiments/routeb_avoidance_20260428_134508`

## 5.2.1 Whole-body dynamics modeling / nonlinear compensation

论文实验：对比是否使用 whole-body dynamics modeling / nonlinear compensation，末端沿 x 方向运动 0.5 m，比较平台姿态和末端跟踪。

当前覆盖度：部分覆盖。

已具备：
- 当前 Route-B online controller 已接入 Pinocchio/MuJoCo 口径下的在线控制链；
- `results/tracking/routeb_trajectory_suite_20260416_105416` 已有 `line` 轨迹，`line_dx=0.5 m`，并输出三 mode 误差、平台运动、索力和关节力矩图。

缺口：
- 尚未做“关闭/开启 whole-body dynamics compensation”的严格 A/B 对照；
- 当前平台为 planar `[x,y,psi]`，无法完全复现论文中 6D 平台姿态变化；
- 尚未输出论文式 platform orientation 对比图。

补充计划：新增一个 `model_compensation_off/on` 对比开关或离线基线，复用 line 0.5 m 轨迹，输出末端误差、平台 yaw/姿态代理和索力统计。

难度：中等。控制链已有，主要缺 A/B 口径与报告脚本。

## 5.2.2 Tracking performance

论文实验：whole-body / manipulator / CDPR 三种控制 scheme 跟踪三阶段 3D triangle，包含 x/z 平移与 pitch 姿态变化，输出 tracking error、运动分配和张力统计表。

当前覆盖度：较高但口径不同。

已具备：
- `results/tracking/routeb_trajectory_suite_20260416_105416` 已完成三模式统一轨迹 suite：line、triangle、square、circle、helix；
- 14/14 通过当前阈值；最差 RMSE 为 `0.006492 m`，最差最大误差为 `0.015881 m`；
- 已输出三维轨迹、三视图投影、误差时间序列、8 根索张力、机械臂力矩与 summary 图。

缺口：
- 当前主任务是 tip XYZ，不含 pitch 姿态跟踪；
- 当前 triangle 是 planar fair comparison，不是论文 5.2.2 的 x/z/pitch 三阶段轨迹；
- 当前 dt 默认 0.02 s，不对应论文 1 kHz / 100 Hz 实时频率口径。

补充计划：保留现有 planar triangle 作为三 mode 公平横比；新增 paper-inspired 3D triangle，仅对 arm_only/cooperative 运行，platform_only 明确 skip 或只做 planar projection。

难度：中等偏低。轨迹框架已有，主要是新增轨迹定义和报告页。

## 5.2.3 Singularity avoidance

论文实验：末端沿 x 方向运动 0.5 m，使机械臂接近不可达/奇异区域；对比是否使用 singularity avoidance，展示 manipulability / singular value 曲线和构型快照。

当前覆盖度：已建立原型，效果中等。

已具备：
- `python/run_routeb_avoidance_experiments.py` 已支持 singularity avoidance off/on 对比；
- 最新结果：`results/avoidance_experiments/routeb_avoidance_20260428_134508`；
- 更明显目标：`target_delta=[0.575, 0, 0]`；
- off/on 最小奇异值分别约为 `0.00116` / `0.00627`；
- 已输出 `singularity_sigma_min_comparison.png` 和 `singularity_snapshot_comparison.png`。

缺口：
- 开启 singularity avoidance 后仍有 1 次 solver fail / fallback；
- 当前实现是低优先级平台偏置式规避，不是论文完全等价的 SVD-HQP formulation；
- viewer 中效果仍不如数据曲线明显。

补充计划：继续把奇异规避从“低优先级偏置”升级为明确的 mode-aware task，并限制其只在 tracking 仍可满足时介入；另存 viewer 角度模板用于论文截图。

难度：中等偏高。核心难点是避免规避任务伤害主跟踪。

## 5.2.4 Joint limit avoidance

论文实验：人为收紧第 4 关节限位，末端沿 z 方向移动；对比是否启用 joint limit avoidance，展示关节角不越界和平台补偿运动。

当前覆盖度：已建立原型，数据效果明显。

已具备：
- `python/run_routeb_avoidance_experiments.py` 已支持人工限位自动构造与 off/on 对比；
- 最新更明显目标：`target_delta=[0.45, 0, 0.25]`；
- 自动选中第 2 关节作为当前最易触发限位的关节；
- off/on 最小关节裕量由约 `-0.496 rad` 改善到约 `-0.0217 rad`；
- 已输出 `joint_limit_angle_comparison.png`、`joint_margin_comparison.png`、`joint_limit_snapshot_comparison.png`。

缺口：
- 仍有轻微越界，尚未做到完全非负裕量；
- 人工限位当前由 probe 自动生成，论文中是人工指定第 4 关节，需要进一步固定为可解释配置；
- 尚未系统展示“更多平台运动换取关节不越界”的运动分配图。

补充计划：固定一组不离谱的人工限位配置，增加平台运动/机械臂运动分配图；调大 buffer 或 predictive guard 让最小裕量转为非负。

难度：中等。已有数据效果明显，剩余是边界严格性和展示口径。

## 5.3 Weighting cost function

论文实验：调节 manipulator / CDPR 权重，展示运动分配变化。

当前覆盖度：间接覆盖。

已具备：
- 当前 trajectory suite 已输出 `summary_platform_delta.png`；
- cooperative 下已有平台/机械臂运动分配数据。

缺口：
- 尚未做系统权重 sweep，例如 alpha/beta 三档对比；
- 当前控制器权重和论文符号不完全一一对应。

补充计划：新增 cooperative 权重 sweep，固定 line/circle 两条轨迹，输出平台位移、arm joint delta、tracking error 与索力统计。

难度：中等偏低。

## 结论

当前已有结果足以支撑“在线 MuJoCo 控制链、三模式轨迹跟踪、奇异/限位规避原型”的阶段性展示。与论文 Chapter 5 相比，5.2.2 的轨迹跟踪覆盖最充分；5.2.3/5.2.4 已有可用原型和对比图，但还不是论文严格等价实现；5.2.1 和 5.3 还需要专门 A/B 或参数 sweep 实验补齐。
