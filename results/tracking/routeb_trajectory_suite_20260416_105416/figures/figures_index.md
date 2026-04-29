# Route-B 轨迹 Suite 图表索引

- Suite 目录：`C:\Users\A\Documents\MATLAB\HCDR_v6-codex\results\tracking\routeb_trajectory_suite_20260416_105416`
- 图表由导出的 CSV/JSON 数据生成。
- 详细索力和关节力矩图依赖 `_detailed.csv` 文件。

## 三模式统一轨迹跟踪误差汇总

图注：比较每条轨迹、每种控制模式下的 RMSE 范数、最大误差和保持段漂移。

![三模式统一轨迹跟踪误差汇总](summary_tracking_metrics.png)

## 最终平台运动分配汇总

图注：展示运行结束时平台的平移与偏航角分配，用于判断 cooperative 是否产生合理平台运动。

![最终平台运动分配汇总](summary_platform_delta.png)

## 索力统计汇总

图注：统计所有索在每次运行中的最小、均值和最大张力。

![索力统计汇总](summary_cable_force.png)

## 平面圆：期望轨迹与实际末端轨迹三维图

图注：红色为期望末端轨迹，其余曲线为不同 mode 的实际末端轨迹。

![平面圆：期望轨迹与实际末端轨迹三维图](circle_trajectory_3d.png)

## 平面圆：轨迹 XY/XZ/YZ 三视图投影

图注：从 XY、XZ、YZ 三个投影面检查轨迹跟踪形状和空间偏差。

![平面圆：轨迹 XY/XZ/YZ 三视图投影](circle_trajectory_projections.png)

## 平面圆：跟踪误差时间序列

图注：上图为误差范数，下图为 XYZ 分量误差。

![平面圆：跟踪误差时间序列](circle_tracking_errors.png)

## 平面圆：8 根索张力时间序列

图注：展示 8 根索在运行过程中的张力变化。

![平面圆：8 根索张力时间序列](circle_cable_forces.png)

## 平面圆：机械臂关节力矩命令

图注：展示 6 个机械臂关节的力矩命令变化。

![平面圆：机械臂关节力矩命令](circle_arm_torques.png)

## 螺旋线：期望轨迹与实际末端轨迹三维图

图注：红色为期望末端轨迹，其余曲线为不同 mode 的实际末端轨迹。

![螺旋线：期望轨迹与实际末端轨迹三维图](helix_trajectory_3d.png)

## 螺旋线：轨迹 XY/XZ/YZ 三视图投影

图注：从 XY、XZ、YZ 三个投影面检查轨迹跟踪形状和空间偏差。

![螺旋线：轨迹 XY/XZ/YZ 三视图投影](helix_trajectory_projections.png)

## 螺旋线：跟踪误差时间序列

图注：上图为误差范数，下图为 XYZ 分量误差。

![螺旋线：跟踪误差时间序列](helix_tracking_errors.png)

## 螺旋线：8 根索张力时间序列

图注：展示 8 根索在运行过程中的张力变化。

![螺旋线：8 根索张力时间序列](helix_cable_forces.png)

## 螺旋线：机械臂关节力矩命令

图注：展示 6 个机械臂关节的力矩命令变化。

![螺旋线：机械臂关节力矩命令](helix_arm_torques.png)

## 平面直线：期望轨迹与实际末端轨迹三维图

图注：红色为期望末端轨迹，其余曲线为不同 mode 的实际末端轨迹。

![平面直线：期望轨迹与实际末端轨迹三维图](line_trajectory_3d.png)

## 平面直线：轨迹 XY/XZ/YZ 三视图投影

图注：从 XY、XZ、YZ 三个投影面检查轨迹跟踪形状和空间偏差。

![平面直线：轨迹 XY/XZ/YZ 三视图投影](line_trajectory_projections.png)

## 平面直线：跟踪误差时间序列

图注：上图为误差范数，下图为 XYZ 分量误差。

![平面直线：跟踪误差时间序列](line_tracking_errors.png)

## 平面直线：协同模式末端运动分配比例

图注：展示 cooperative 运行过程中平台与机械臂对末端运动的相对贡献比例。

![平面直线：协同模式末端运动分配比例](line_motion_distribution_ratio.png)

## 平面直线：8 根索张力时间序列

图注：展示 8 根索在运行过程中的张力变化。

![平面直线：8 根索张力时间序列](line_cable_forces.png)

## 平面直线：机械臂关节力矩命令

图注：展示 6 个机械臂关节的力矩命令变化。

![平面直线：机械臂关节力矩命令](line_arm_torques.png)

## 平面正方形：期望轨迹与实际末端轨迹三维图

图注：红色为期望末端轨迹，其余曲线为不同 mode 的实际末端轨迹。

![平面正方形：期望轨迹与实际末端轨迹三维图](square_trajectory_3d.png)

## 平面正方形：轨迹 XY/XZ/YZ 三视图投影

图注：从 XY、XZ、YZ 三个投影面检查轨迹跟踪形状和空间偏差。

![平面正方形：轨迹 XY/XZ/YZ 三视图投影](square_trajectory_projections.png)

## 平面正方形：跟踪误差时间序列

图注：上图为误差范数，下图为 XYZ 分量误差。

![平面正方形：跟踪误差时间序列](square_tracking_errors.png)

## 平面正方形：8 根索张力时间序列

图注：展示 8 根索在运行过程中的张力变化。

![平面正方形：8 根索张力时间序列](square_cable_forces.png)

## 平面正方形：机械臂关节力矩命令

图注：展示 6 个机械臂关节的力矩命令变化。

![平面正方形：机械臂关节力矩命令](square_arm_torques.png)

## 平面三角形：期望轨迹与实际末端轨迹三维图

图注：红色为期望末端轨迹，其余曲线为不同 mode 的实际末端轨迹。

![平面三角形：期望轨迹与实际末端轨迹三维图](triangle_trajectory_3d.png)

## 平面三角形：轨迹 XY/XZ/YZ 三视图投影

图注：从 XY、XZ、YZ 三个投影面检查轨迹跟踪形状和空间偏差。

![平面三角形：轨迹 XY/XZ/YZ 三视图投影](triangle_trajectory_projections.png)

## 平面三角形：跟踪误差时间序列

图注：上图为误差范数，下图为 XYZ 分量误差。

![平面三角形：跟踪误差时间序列](triangle_tracking_errors.png)

## 平面三角形：8 根索张力时间序列

图注：展示 8 根索在运行过程中的张力变化。

![平面三角形：8 根索张力时间序列](triangle_cable_forces.png)

## 平面三角形：机械臂关节力矩命令

图注：展示 6 个机械臂关节的力矩命令变化。

![平面三角形：机械臂关节力矩命令](triangle_arm_torques.png)
