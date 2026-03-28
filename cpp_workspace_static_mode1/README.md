# cpp_workspace_static_mode1

Independent C++17 workspace for **Mode 1 static workspace** only.

This project reads parameters and URDF data from the original repository, but it does **not** execute any MATLAB scripts and it does **not** modify any file under `HCDR_v6-codex/`.

## Scope

This project implements only the frozen-arm static workspace:

- `g = 0`
- `qdot = 0`
- `qdd = 0`
- `w_ext = 0`
- arm frozen at `q_m_init`
- arm joint torques are **not** used as a rejection criterion
- tension bounds are configurable, default `T_min = 5 N`, `T_max = 500 N`

Mode 2 and Mode 3 are intentionally excluded.

## Static Mapping Convention

The optimization variables are the **real cable tensions** `T_i`, not planar projected forces.

The corrected planar wrench map is:

- `A_2D_true(0,i) = c_i * u_x`
- `A_2D_true(1,i) = c_i * u_y`
- `A_2D_true(2,i) = c_i * (r_x * u_y - r_y * u_x)`

where:

- `u_xy = l_xy / ||l_xy||`
- `c_i = L_xy / L_3d`

This keeps the physical tension bounds unchanged:

- `T_min <= T_i <= T_max`

The planar external wrench interface is also connected:

- `A_2D * T + w_ext_2d = 0`
- `w_ext_2d = [Fx, Fy, Mz]^T`

The default configuration still uses `w_ext = 0`.

## Mode 1 Outputs

Two outputs are generated and must be interpreted separately.

### 1. Main result: EE union over psi

This is the end-effector workspace:

`p_EE^W = [x, y, z0]^T + Rz(psi) * p_tip0^P`

Workflow:

1. sample `psi`
2. scan the **platform-center** grid `(x, y)`
3. build `A_2D(x, y, psi)`
4. hard reject degenerate poses
5. solve gamma-margin static feasibility
6. map feasible platform centers to EE points
7. voxel-deduplicate the union cloud

Files:

- `mode1_ee_union_cloud.csv`
- `mode1_ee_union_cloud.ply`
- `mode1_ee_union_xy_projection.csv`
- `mode1_ee_union_xy.png`
- `mode1_ee_union_bestpsi.csv`

This result is allowed to lie on a single `z = z0 + const` layer. It is **not** the fixed-psi star plot.

### 2. Diagnostic result: platform-center fixed-psi CWS

This is the plot used to inspect the star / polygon boundary.

Workflow:

1. fix `psi = fixed_psi_deg`
2. scan the **platform-center** grid `(x, y)`
3. build `A_2D(x, y, psi_fixed)`
4. hard reject degenerate poses
5. solve gamma-margin static feasibility
6. export platform-center feasible mask and gamma map

Files:

- `mode1_platform_fixedpsi_45deg_cloud.csv`
- `mode1_platform_fixedpsi_45deg_mask.csv`
- `mode1_platform_fixedpsi_45deg_gamma.csv`
- `mode1_platform_fixedpsi_45deg_xy.png`
- `mode1_platform_fixedpsi_45deg_gamma.png`

This diagnostic plot is in **platform-center XY coordinates**, not EE coordinates and not union-over-psi.

If `mode1_platform_fixedpsi_45deg_cloud.csv` is empty after hard degeneracy rejection, that means the current platform geometry and tension bounds do not admit a positive static self-stress at that fixed psi. That is a physical infeasibility result, not the earlier "whole plane" plotting bug.

## Diagnostic Outputs

These files are for diagnosing why the main WFW is empty. They are not part of the formal Mode 1 workspace result:

- `mode1_nullspace_diagnostic.txt`
- `mode1_force_only_diagnostic.txt`
- `mode1_pose_debug_samples.csv`

`mode1_force_only_diagnostic.txt` is a debugging-only relaxation. It may ignore the yaw-moment row only when:

- `rank(A_2D) == 2`
- `||A_tau||` is below `force_only_torque_row_norm_eps`

This force-only diagnostic is never merged into the main WFW outputs.

## Degeneracy Rejection

Degenerate `A_2D` is rejected **before** OSQP:

- if `rank(A_2D) < 3`: reject
- if `sigma_min(A_2D) < eps_rank`: reject

Only nondegenerate poses are sent to the gamma-margin solver.

## Current Interpretation

For the current default geometry, the diagnostic samples show:

- `L_3d == L_xy`
- `c_i == 1`

because the present anchor heights and platform upper/lower attachment heights coincide. So the corrected 3D-to-2D mapping is implemented, but under the current default geometry it does not numerically change `A_2D`.

If the corrected main result is still empty, inspect `mode1_nullspace_diagnostic.txt` first:

- if `corrected_positive_self_stress = false` at nondegenerate poses, the empty set is most likely caused by geometry not admitting a fully positive self-stress under `Fx/Fy/Mz` equilibrium
- if that diagnostic were feasible while gamma remained infeasible, then the issue would more likely be bounds or solver settings

## Build

```powershell
cd c:\Users\A\Documents\MATLAB\HCDR_v6-codex
powershell -ExecutionPolicy Bypass -File .\cpp_workspace_static_mode1\scripts\build_vs2019.ps1
```

## Run

Full run:

```powershell
$env:PATH="C:\Users\A\miniconda3\envs\hcdr_pin\Library\bin;$env:PATH"
.\cpp_workspace_static_mode1\build\Release\mode1_workspace_runner.exe --config .\cpp_workspace_static_mode1\config\workspace_static_mode1.json
```

Smoke run:

```powershell
powershell -ExecutionPolicy Bypass -File .\cpp_workspace_static_mode1\scripts\run_smoke.ps1
```

## Required Log Fields

The runner prints and logs:

1. fixed-psi scanned grid point count
2. fixed-psi `rank < 3` rejected count
3. fixed-psi `sigma_min < eps_rank` rejected count
4. fixed-psi `gamma >= 0` feasible count
5. EE union raw feasible count and voxel count
6. fixed-psi min / max gamma

The same information is also written to `mode1_run_log.txt`.

## Repository Boundary

- Original `HCDR_v6-codex/` files are treated as read-only references.
- No MATLAB script in the original repository is executed by this project.
- All Mode 1 implementation lives inside `cpp_workspace_static_mode1/`.
