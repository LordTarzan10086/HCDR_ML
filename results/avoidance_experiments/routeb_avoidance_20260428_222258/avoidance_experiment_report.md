# 奇异与关节限位规避对比实验

## 摘要
本实验在当前 Route-B online controller 上通过可选开关对比 avoidance off/on。默认控制器参数不被覆盖，只有实验 run 内部临时打开对应开关。

## 人工关节限位设置
- 选中关节：第 2 关节
- probe 峰值位移：-0.7112 rad

## 结果表
| run | RMSE [m] | Max error [m] | min sigma | min joint margin [rad] | singular active | joint-limit active | fail | fallback |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| singularity_off | 0.004248 | 0.006486 | 0.002218 | 1.575 | 0 | 0 | 0 | 0 |
| singularity_strict_svd | 0.003989 | 0.006659 | 0.08003 | 1.878 | 27 | 0 | 0 | 0 |
| singularity_on | 0.00389 | 0.005572 | 0.08032 | 1.878 | 27 | 0 | 0 | 0 |
| joint_limit_probe | 0.002675 | 0.004567 | 0.1429 | 1.754 | 0 | 0 | 0 | 0 |
| joint_limit_off | 0.002675 | 0.004567 | 0.1429 | -0.4392 | 0 | 0 | 0 | 0 |
| joint_limit_on | 0.003002 | 0.00509 | 0.1429 | -0.005775 | 0 | 130 | 0 | 0 |

## 图表清单
- `figures\singularity_sigma_min_comparison.png`
- `figures\tracking_error_comparison.png`
- `figures\joint_limit_angle_comparison.png`
- `figures\joint_margin_comparison.png`
- `figures\singularity_mujoco_snapshot_comparison.png`
- `figures\joint_limit_mujoco_snapshot_comparison.png`
- `figures\singularity_paper_fig7_like.png`
- `figures\joint_limit_paper_fig8_like.png`

## 数据文件
- `avoidance_summary.csv` / `avoidance_summary.json`
- `singularity_*.csv` 与 `joint_limit_*.csv`：逐步日志
