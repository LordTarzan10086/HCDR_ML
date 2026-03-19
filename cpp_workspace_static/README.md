# cpp_workspace_static

`cpp_workspace_static/` is a standalone C++17 workspace for static workspace analysis of the HCDR system.
It reads geometry, parameters, and URDF data from the existing repository, but it does not call MATLAB code and it does not modify the original MATLAB files.

## Scope and Assumptions

This project implements the static workspace definition requested for the three modes under:

- `g = 0`
- `qdot = 0`
- `qdd = 0`
- `w_ext = 0`
- no motion process
- no dynamic terms
- no extra center-of-mass gravity torque term
- arm joint torques are not used as a rejection test
- default tension bounds are configurable and start from `T_min = 5 N`, `T_max = 500 N`

The URDF is read directly from:

- `../kortex_description/robots/gen3_lite_gen3_lite_2f_local.urdf`

## Dependencies

- C++17
- Eigen
- Pinocchio C++ API
- OSQP
- Visual Studio 2019
- MATLAB-bundled CMake on this machine

The default local environment in the scripts is:

- Conda env: `C:\Users\A\miniconda3\envs\hcdr_pin`
- CMake: `C:\Program Files\MATLAB\R2024b\bin\win64\cmake\bin\cmake.exe`

## How Each Mode Is Evaluated

### Mode 1: platform-only

The arm is frozen at `q_m_init`, so the local tip offset in the platform frame is fixed:

- `p_tip0^P`

For each sampled `psi` and each sampled platform center `(x, y)`, the solver checks whether there exists a bounded self-stress:

- `p_EE^W = [x, y, z0]^T + Rz(psi) * p_tip0^P`
- `A_2D(x, y, psi) * T = 0`
- `T_min <= T_i <= T_max`

The main Mode 1 result is therefore the union of end-effector points over all sampled `psi`:

- `mode1_ee_union_*`

The fixed-psi platform-center diagnostic is exported separately:

- `mode1_platform_fixedpsi_45deg_*`

This diagnostic is the correct plot to inspect star-like or polygon-like platform-center CWS boundaries.
The union EE cloud is not the right plot for judging fixed-orientation star symmetry.

### Why the Mode 1 Main Result Is the EE Union

Mode 1 must not lock `psi`.
`psi` changes both:

- the geometric mapping from the platform to the EE point
- the static matrix `A_2D(x, y, psi)`

So the correct main result is the union of all feasible EE points over the sampled `psi` values.

### Mode 2: arm-only

The platform is frozen at `platform_init`.
The arm is sampled directly inside the URDF joint limits read through Pinocchio.

For each sampled arm configuration `q_m`:

- compute `p_tip^P(q_m)` with Pinocchio FK
- map it to the world frame using the fixed platform pose

Mode 2 uses only geometric reachability and joint limits.
There is no per-point platform static rejection in this mode.

### Mode 3: cooperative

Mode 3 must not lock `psi`, and it is not treated as a simple product of Mode 1 and Mode 2.

The solver uses the requested constructive union workflow:

1. Sample the arm locally in the platform frame to build `S_a = { p_tip^P(q_m) }`
2. Sample `psi`
3. For each `psi`, scan feasible platform centers `S_p(psi)` from the static platform solver
4. Rotate the local arm cloud by `Rz(psi)`
5. Add each feasible platform center to the rotated local arm cloud
6. Take the union over all `psi`

The main Mode 3 result is:

- `mode3_union_*`

The optional fixed-psi diagnostic is:

- `mode3_fixedpsi_45deg_*`

## Platform Static Feasibility Module

Static platform feasibility is implemented in:

- `include/platform_static_feasibility.h`
- `src/platform_static_feasibility.cpp`

It solves the requested gamma-margin problem with OSQP:

- `A_2D * T = 0`
- `T_min + gamma <= T_i <= T_max - gamma`
- `gamma >= 0`
- maximize `gamma`

The code also exports a closure diagnostic LP/QP-style check for:

- positive self-stress existence
- closure margin

Important:

- the exported psi diagnostics explicitly report `rank3_grid_points`
- a slice can be bounded-self-stress feasible even when the planar wrench matrix is not full rank
- use the exported diagnostics to decide whether a slice is physically acceptable for your interpretation of static workspace

## New Diagnostics Added for Result Quality

Each run now exports:

- `platform_sample_pose_diagnostics.csv`
- `mode1_platform_psi_summary.csv`
- `mode3_platform_psi_summary.csv`

`platform_sample_pose_diagnostics.csv` contains reproducible checks at:

- `psi = 0 deg`
- `psi = fixed_psi_deg`
- `psi = 90 deg`
- sample points: `center`, `edge_x`, `corner`

The psi summary files report, for each sampled `psi`:

- scanned grid count
- rank-3 grid count
- closure-feasible grid count
- gamma-feasible grid count
- min / max / mean gamma
- max closure margin

These files are the primary explanation path when a fixed-psi cloud is empty or when feasible psi slices are sparse.

## Fixed-Psi Empty Diagnostics

The current diagnostics show that `fixed_psi = 45 deg` is empty under the current geometry and static criterion for both:

- `mode1_platform_fixedpsi_45deg_*`
- `mode3_fixedpsi_45deg_*`

This is not unique to the C++ implementation.
The historical MATLAB summary files already present in the repository report the same behavior, for example:

- `results/workspace_static_cws_20260317_191629/workspace_static_fixed_union_summary.csv`
- `scripts/results/workspace_static_cws_20260317_182306/workspace_static_fixed_union_summary.csv`

Both of those summaries show:

- `mode1_fixedpsi_points = 0`
- `mode3_fixedpsi_points = 0`

So the C++ implementation now explains this with explicit `psi` and sample-pose diagnostics instead of leaving it implicit.

## Output Files

### Mode 1 main result

- `mode1_ee_union_cloud.csv`
- `mode1_ee_union_cloud.ply`
- `mode1_ee_union_xy_projection.csv`
- `mode1_ee_union_xz_projection.csv`
- `mode1_ee_union_yz_projection.csv`

### Mode 1 fixed-psi platform-center diagnostics

- `mode1_platform_fixedpsi_45deg_cloud.csv`
- `mode1_platform_fixedpsi_45deg_cloud.ply`
- `mode1_platform_fixedpsi_45deg_xy_projection.csv`
- `mode1_platform_fixedpsi_45deg_gamma.csv`

### Mode 2

- `mode2_cloud.csv`
- `mode2_cloud.ply`
- `mode2_xy_projection.csv`
- `mode2_xz_projection.csv`
- `mode2_yz_projection.csv`

### Mode 3 main result

- `mode3_union_cloud.csv`
- `mode3_union_cloud.ply`
- `mode3_union_xy_projection.csv`
- `mode3_union_xz_projection.csv`
- `mode3_union_yz_projection.csv`

### Mode 3 fixed-psi diagnostics

- `mode3_fixedpsi_45deg_cloud.csv`
- `mode3_fixedpsi_45deg_cloud.ply`
- `mode3_fixedpsi_45deg_xy_projection.csv`
- `mode3_fixedpsi_45deg_xz_projection.csv`
- `mode3_fixedpsi_45deg_yz_projection.csv`

### Additional diagnostics

- `platform_sample_pose_diagnostics.csv`
- `mode1_platform_psi_summary.csv`
- `mode3_platform_psi_summary.csv`

## Configuration Files

- `config/workspace_static.json`
  The full-density target configuration.
- `config/workspace_static_validation.json`
  A medium-density validation configuration for faster quality checks.
- `config/workspace_static_smoke.json`
  A lightweight smoke configuration for build and pipeline checks.
- `config/workspace_static_mode1_only.json`
  A full-density Mode 1 only configuration that skips Mode 2 and Mode 3.

You can also set these booleans in any config:

- `enable_mode1`
- `enable_mode2`
- `enable_mode3`

## Build and Run

### Build

```powershell
cd cpp_workspace_static
powershell -ExecutionPolicy Bypass -File .\scripts\build_vs2019.ps1
```

### Smoke run

```powershell
cd cpp_workspace_static
powershell -ExecutionPolicy Bypass -File .\scripts\run_smoke.ps1
```

### Validation run

```powershell
cd cpp_workspace_static
.\build\Release\workspace_static_runner.exe --config .\config\workspace_static_validation.json
```

### Full-density run

```powershell
cd cpp_workspace_static
.\build\Release\workspace_static_runner.exe --config .\config\workspace_static.json
```

### Mode 1 only run

```powershell
cd cpp_workspace_static
.\build\Release\workspace_static_runner.exe --config .\config\workspace_static_mode1_only.json
```

## Performance Notes

Mode 3 is the dominant cost.
This code now improves the earlier prototype by:

- caching all platform psi slices before cloud generation
- reusing those slices for Mode 1, Mode 3, and fixed-psi diagnostics
- rotating the local arm cloud once per psi slice
- combining quantized points directly during voxel insertion
- parallelizing the cooperative Mode 3 union across psi slices

`workspace_static_validation.json` is intended for checking result quality before committing to the full run.

## Read-Only Boundary

- The original `HCDR_v6-codex` tree is used only as a read-only reference for parameters, geometry, URDF, and historical logic.
- No MATLAB function is called from this C++ project.
- No original MATLAB file is modified by this C++ project.
- All new implementation files are contained inside `cpp_workspace_static/`.
