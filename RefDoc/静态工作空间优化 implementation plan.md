# 静态工作空间优化 implementation plan

## 1. 核心问题定位分析

目前的 Mode1 静态工作空间为空集的核心原因在于：**几何模型的索路由设计存在原生缺陷**。
通过检查当前 C++ 代码（[platform_geometry.cpp](file:///c:/Users/A/Documents/MATLAB/HCDR_v6-codex/cpp_workspace_static/src/platform_geometry.cpp)）及跨仓库对比原始 MATLAB 原型确认：当前平台 4 个角的上下各 2 根缆索，均径向直连至机架对应的**同象限锚点**。
在这种**纯径向索路由**（Radial Routing）下，当平台发生偏航（$\psi \neq 0$）时，所有 8 根缆绳拉伸产生的恢复力矩全部同号（均为反抗该偏航的恢复力矩）。根据力矩平衡方程 $\Sigma (\vec{r}_i \times \vec{T}_i) = 0$，如果所有力矩臂均同号，显然无法用全正非零的张力（$T_i \ge T_{\min} > 0$）来得到平衡解。这就导致了除 $\psi = 0$ 和 $\pm 180^\circ$ 的退化态（此时可产生正应力但力矩秩亏）拉不出任何非空的静态工作区。

## 2. 改造方案：交叉索路由 (Crossed Cable Routing)

为了破除上述几何奇异，需要引入能够产生顺时针和逆时针双向偏航力矩的索路由设计。我们将重写 [platform_geometry.cpp](file:///c:/Users/A/Documents/MATLAB/HCDR_v6-codex/cpp_workspace_static/src/platform_geometry.cpp) 的 [PlatformGeometry](file:///c:/Users/A/Desktop/code/C++/HCDR_v6-codex--branch_for_WFW/cpp_workspace_static/src/platform_geometry.cpp#9-43) 构造函数。

### 设计详情：
对于平台的第 $i$ 个角点（$i = 0,1,2,3$ 对应第一至第四象限）：
- **当前设计（纯径向）**：
  upper_cable $\to$ 框架角点 $i$
  lower_cable $\to$ 框架角点 $i$
- **修改为对称交叉设计（Symmetric Crossed）**：
  保持上锚点连接顺时针或逆时针偏移的相邻机架角点，以同时提供沿切向的正负偏航张力力矩：
  upper_cable $\to$ 框架角点 $(i + 1) \bmod 4$ （逆时针相邻框架位置）
  lower_cable $\to$ 框架角点 $(i + 3) \bmod 4$ （顺时针相邻框架位置）

*改动文件：* [cpp_workspace_static/src/platform_geometry.cpp](file:///c:/Users/A/Desktop/code/C++/HCDR_v6-codex--branch_for_WFW/cpp_workspace_static/src/platform_geometry.cpp) 及相应的 Mode1 专用仓库中的文件。为保证兼容性，可以首先向配置读取文件 [workspace_config.cpp](file:///c:/Users/A/Documents/MATLAB/HCDR_v6-codex/cpp_workspace_static/src/workspace_config.cpp) 暴露一个 `cable_routing_mode` 配置项（默认设为 `"crossed"`，也可以回退为 `"radial"`）。

## 3. 统一静力判据

在交接文档中提及，Mode1 专有工程剔除了退化的 $A_{2D}$ 以达到严格 3-DOF 平衡。然而在此前的一些 MATLAB Demo 测试里似乎曾包容了秩亏计算，这造成了两边的判据不一。

### 设计详情：
既然新的交叉路由极大地丰富了姿态矩阵的列排布，新几何下 $A_{2D}$ 发生秩亏的几率极大地减小。我们将在 [platform_geometry.cpp](file:///c:/Users/A/Documents/MATLAB/HCDR_v6-codex/cpp_workspace_static/src/platform_geometry.cpp) 和主处理链路中正式确立以下“硬条件”：
1. **严格满秩要求**：仅接受 `rank(A2D) == 3` 且 $\sigma_{\min} \ge \varepsilon_{rank}$ 的姿态；退化姿态一律否决，因为退化下无法抵抗任意方向的扰动。
2. **张力完全平衡要求 (QP 目标判定)**：利用现有的 OSQP Margin $\gamma$ 模型求解，仅当 $\gamma \ge 0$ 时判定力与力矩不仅平衡，而且保留有全正的预紧包络（符合下限阈值 $T_{\min}$）。此时 `feasible = true`。

## 4. 后续 Mode3 算法重构扩展准备

在当前问题（Mode1 空集）得到根本几何修正后，在新的交叉路由前提下，原先由于“任何外围组合都无法力矩平衡”导致的 Mode 3 为空集或计算白费算力的问题也将一并被拔除。后续可以真正设计基于 $\psi$ 分层的批量映射来取代暴力的笛卡尔点对点扫描。这里作为下一步优化暂做登记。

## 5. Verification Plan
### Automated / Compilation Tests
1. 在 `cpp_workspace_static_mode1` 或 `cpp_workspace_static` 根目录下重新编译工程：
   ```powershell
   powershell -ExecutionPolicy Bypass -File .\cpp_workspace_static_mode1\scripts\build_vs2019.ps1
   ```
2. 运行内置的测试例程和工作空间求解流：
   ```powershell
   $env:PATH="C:\Users\A\miniconda3\envs\hcdr_pin\Library\bin;$env:PATH"
   .\cpp_workspace_static_mode1\build\Release\mode1_workspace_runner.exe --config .\cpp_workspace_static_mode1\config\workspace_static_mode1.json
   ```
3. 验证生成文件夹内的 [mode1_run_log.txt](file:///c:/Users/A/Desktop/code/C++/HCDR_v6-codex--branch_for_WFW/cpp_workspace_static_mode1/output/workspace_static_mode1_20260327_105911/mode1_run_log.txt)：
   检查日志内 `ee union voxel points` 或 `fixed psi gamma>=0 feasible` 是否从此前的 0 变为正常数值（预期存在显著的可行域面积）。

### User Review Required
> [!IMPORTANT]
> - 修改锚点和索路由会从根本上改变硬件组装模型。如果在实际物理平台上也是径向直拉组装方案，那该硬件本身就无法实现 $\tau_z$ 姿态全正预紧稳定。若需改回原装，请说明。
> - 本实现计划先在 C++ 分析器中打入这一交叉路由方案。您确认同意将配置模式由此前的全直连方案调整为混合的交叉路由（Crossed Routing）吗？
