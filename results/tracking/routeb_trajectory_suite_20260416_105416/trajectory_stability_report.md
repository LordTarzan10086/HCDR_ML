# Route-B 三模式统一轨迹稳定性测试报告

- 生成时间：2026-04-16 10:55:59
- 配置文件：`C:\Users\A\Documents\MATLAB\HCDR_v6-codex\results\online_config\routeb_online_config_manual_smoke.json`
- 输出目录：`C:\Users\A\Documents\MATLAB\HCDR_v6-codex\results\tracking\routeb_trajectory_suite_20260416_105416`
- 后端：Python online controller + native MuJoCo planar HCDR backend，headless。
- 评价阈值：`rmse_norm <= 0.025 m`、`mean_norm <= 0.025 m`、`max_error <= 0.025 m`、`hold_drift <= 0.010 m`，且 solver/fallback/platform-limit 计数为 0。
- 说明：`helix` 为 3D 轨迹，`platform_only` 按当前定义无法跟踪非零 z，因此自动跳过。

## 总览

- 通过数：14/14
- 最差 RMSE：line/cooperative = 0.006492 m
- 最差最大误差：line/cooperative = 0.015881 m
- 最差 hold 漂移：line/cooperative = 0.007052 m

## 轨迹参数

| trajectory | 参数 |
|---|---|
| line | `{"name": "line", "move_duration": 4.0, "settle_duration": 0.8, "line_dx": 0.5, "line_dy": 0.0, "line_dz": 0.0, "line_style": "centered", "side": 0.0, "triangle_side": 0.0, "square_side": 0.0, "radius": 0.0, "helix_dz": 0.0, "circle_dz": 0.0, "turns": 1.0, "path_yaw_deg": 0.0, "circle_start_angle_deg": 0.0}` |
| triangle | `{"name": "triangle", "move_duration": 4.0, "settle_duration": 0.8, "line_dx": 0.0, "line_dy": 0.0, "line_dz": 0.0, "line_style": "centered", "side": 0.14, "triangle_side": 0.14, "square_side": 0.0, "radius": 0.0, "helix_dz": 0.0, "circle_dz": 0.0, "turns": 1.0, "path_yaw_deg": 0.0, "circle_start_angle_deg": 0.0}` |
| square | `{"name": "square", "move_duration": 4.0, "settle_duration": 0.8, "line_dx": 0.0, "line_dy": 0.0, "line_dz": 0.0, "line_style": "centered", "side": 0.16, "triangle_side": 0.0, "square_side": 0.16, "radius": 0.0, "helix_dz": 0.0, "circle_dz": 0.0, "turns": 1.0, "path_yaw_deg": 0.0, "circle_start_angle_deg": 0.0}` |
| circle | `{"name": "circle", "move_duration": 4.0, "settle_duration": 0.8, "line_dx": 0.0, "line_dy": 0.0, "line_dz": 0.0, "line_style": "centered", "side": 0.0, "triangle_side": 0.0, "square_side": 0.0, "radius": 0.12, "helix_dz": 0.0, "circle_dz": 0.0, "turns": 1.0, "path_yaw_deg": 0.0, "circle_start_angle_deg": 0.0}` |
| helix | `{"name": "helix", "move_duration": 6.0, "settle_duration": 0.8, "line_dx": 0.0, "line_dy": 0.0, "line_dz": 0.0, "line_style": "centered", "side": 0.0, "triangle_side": 0.0, "square_side": 0.0, "radius": 0.06, "helix_dz": 0.16, "circle_dz": 0.0, "turns": 3.0, "path_yaw_deg": 0.0, "circle_start_angle_deg": 0.0}` |

## 指标表

| trajectory | mode | pass | rmse_norm m | mean_norm m | max_err m | prehold m | hold_drift m | fail | fallback | limit | platform_delta [x,y,psi] | arm_max_delta rad | max cable N |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|
| line | platform_only | YES | 0.002273 | 0.001762 | 0.003896 | 0.000001 | 0.000206 | 0 | 0 | 0 | [0.4999, 0.0001, -0.0025] | 0.000000 | 9.682 |
| line | arm_only | YES | 0.002735 | 0.002096 | 0.004703 | 0.000001 | 0.000051 | 0 | 0 | 0 | [0.0031, -0.0002, 0.0000] | 0.973421 | 67.815 |
| line | cooperative | YES | 0.006492 | 0.005433 | 0.015881 | 0.002108 | 0.007052 | 0 | 0 | 0 | [0.2533, 0.0004, 0.0007] | 2.367145 | 15.199 |
| triangle | platform_only | YES | 0.001781 | 0.001456 | 0.002998 | 0.000002 | 0.000948 | 0 | 0 | 0 | [0.0000, 0.0000, -0.0000] | 0.000000 | 9.208 |
| triangle | arm_only | YES | 0.002381 | 0.001825 | 0.004267 | 0.000013 | 0.000381 | 0 | 0 | 0 | [-0.0024, 0.0003, -0.0002] | 0.559356 | 121.045 |
| triangle | cooperative | YES | 0.002251 | 0.001830 | 0.003992 | 0.000875 | 0.001329 | 0 | 0 | 0 | [-0.0258, -0.0012, 0.0003] | 0.550901 | 7.000 |
| square | platform_only | YES | 0.002617 | 0.002229 | 0.004448 | 0.000013 | 0.002183 | 0 | 0 | 0 | [-0.0000, 0.0000, 0.0000] | 0.000000 | 12.165 |
| square | arm_only | YES | 0.003755 | 0.002895 | 0.006868 | 0.000054 | 0.000969 | 0 | 0 | 0 | [-0.0056, -0.0004, -0.0003] | 0.854339 | 148.070 |
| square | cooperative | YES | 0.003432 | 0.002771 | 0.006519 | 0.001416 | 0.002077 | 0 | 0 | 0 | [-0.0423, -0.0071, 0.0008] | 0.855861 | 7.000 |
| circle | platform_only | YES | 0.003135 | 0.002448 | 0.005077 | 0.000006 | 0.000303 | 0 | 0 | 0 | [0.0001, -0.0001, 0.0013] | 0.000000 | 10.842 |
| circle | arm_only | YES | 0.004332 | 0.003306 | 0.007724 | 0.000003 | 0.000153 | 0 | 0 | 0 | [0.0015, 0.0006, -0.0001] | 0.630078 | 165.215 |
| circle | cooperative | YES | 0.004328 | 0.003734 | 0.008867 | 0.003086 | 0.001449 | 0 | 0 | 0 | [-0.0231, 0.0016, -0.0001] | 0.641554 | 7.000 |
| helix | arm_only | YES | 0.004970 | 0.003646 | 0.014800 | 0.000002 | 0.000065 | 0 | 0 | 0 | [0.0003, 0.0006, 0.0002] | 0.964020 | 186.070 |
| helix | cooperative | YES | 0.005432 | 0.004229 | 0.015256 | 0.000793 | 0.000275 | 0 | 0 | 0 | [-0.0644, 0.0048, 0.0003] | 0.913588 | 7.000 |

## 张力统计摘要

下表给出每个 run 的全索最大张力范围摘要。完整逐索 min/max/mean/std 保存在 `trajectory_suite_summary.json`。

| trajectory | mode | tension available | min(all cables) N | max(all cables) N | mean(all cables) N |
|---|---:|---:|---:|---:|---:|
| line | platform_only | YES | 7.000 | 9.682 | 7.733 |
| line | arm_only | YES | 7.000 | 67.815 | 7.971 |
| line | cooperative | YES | 7.000 | 15.199 | 7.087 |
| triangle | platform_only | YES | 7.000 | 9.208 | 7.385 |
| triangle | arm_only | YES | 7.000 | 121.045 | 9.449 |
| triangle | cooperative | YES | 7.000 | 7.000 | 7.000 |
| square | platform_only | YES | 7.000 | 12.165 | 7.796 |
| square | arm_only | YES | 7.000 | 148.070 | 9.449 |
| square | cooperative | YES | 7.000 | 7.000 | 7.000 |
| circle | platform_only | YES | 7.000 | 10.842 | 7.648 |
| circle | arm_only | YES | 7.000 | 165.215 | 9.211 |
| circle | cooperative | YES | 7.000 | 7.000 | 7.000 |
| helix | arm_only | YES | 7.000 | 186.070 | 9.691 |
| helix | cooperative | YES | 7.000 | 7.000 | 7.000 |

## 结论

- 若通过数等于总数，本组参数可作为当前三模式统一轨迹展示的稳定基线。
- 若存在未通过项，优先查看该项的 per-mode CSV 与 `trajectory_suite_summary.json` 中的初始化误差、平台边界计数、fallback 计数和张力统计。
- 本报告不声称达到论文 5.2.2 的量级，只用于当前 v6-codex 在线控制链的阶段性稳定性评估。

## 输出文件

- `trajectory_suite_summary.json`：全量 summary 和逐索张力统计。
- `trajectory_suite_summary.csv`：紧凑指标表。
- `<trajectory>/<trajectory>_<mode>.csv`：逐步 tip/desired/error/platform 状态。
