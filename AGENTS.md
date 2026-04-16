# AGENTS.md — MATLAB + Pinocchio Route-B HCDR Planar Project

## Environment
- Primary language: MATLAB (R2021b+ recommended)
- Required toolboxes: Optimization Toolbox (fmincon/quadprog), Robotics System Toolbox (optional, legacy)
- Python (for Pinocchio): Python 3.10+ with `pinocchio`, `numpy`
- VS Code:
  - Install MATLAB extension: `MathWorks.language-matlab`
  - Run entrypoints via Command Palette: `MATLAB: Run File`
  - Test command (CI-style):
    `matlab -batch "results=runtests('tests'); assertSuccess(results)"`

## Repo Conventions
- Do not modify existing 5d reference files unless explicitly requested.
- Add new planar files with suffix `_planar` or name `*_planar.m`.
- Prefer pure functions with explicit inputs/outputs.
- Use `arguments` blocks or `validateattributes` for input checks.
- Preallocate arrays; avoid dynamic resizing in loops.
- All numeric outputs must be MATLAB `double`.

## Route B Non-negotiables (D4)
- Compute M via Pinocchio `crba`.
- Compute h via Pinocchio `rnea(qdd=0)`.
- Map `h -> h_a` using pseudo-inverse / damped pseudo-inverse ONLY.
- NEVER apply physical box constraints (Tmin/Tmax) when computing `h_a`.
- Apply physical constraints only inside HQP:
  `Tmin <= T_wo + h_a_T <= Tmax`.

## Task Workflow (Agent Loop)
1) Create/extend tests in `tests/` first (`matlab.unittest`).
2) Implement/modify code in `src/` (or repo root if project stays flat).
3) Run:
   `matlab -batch "results=runtests('tests'); assertSuccess(results)"`
4) If failing: fix code + update tests; repeat until green.
5) Summarize changes and risks at the end.

## Online Python Regression Notes
- For `hcdr_pin` Python commands, use `scripts/run_hcdr_pin_python.cmd` so native runtime environment variables are set before importing NumPy/Pinocchio/MuJoCo.
- Pytest-collected online cases live under `python/test_*_pytest.py`; legacy `python/test_*.py` files may still be script-style and should be run directly when they collect zero pytest tests.
- At the current q=0 hanging-arm initial posture, negative `delta_z` targets are diagnostic stress cases, not default required successes. Do not count them against the default online random-target smoke unless the user explicitly asks for full negative-z robustness.
- The Gen3 Lite 2F tool tip is the midpoint of `LEFT_FINGER_DIST` and `RIGHT_FINGER_DIST` body origins. Do not restore the old opposite local offsets `[-0.04,0,0]` / `[0.04,0,0]`.
- Online 6R demos/tests should build `q0` through `online_config_utils.initial_q_from_payload()`. The default arm posture is `[0, 0.58, 0.92, 0, 0, 0]`, not all-zero q_m; keep `q4=0` so the gripper is not initially skewed.
- Use `python/run_routeb_trajectory_stability_suite.py` for full headless 3-mode trajectory validation. The current phase-end baseline covers line, triangle, square, circle, and helix; helix intentionally skips `platform_only`.
- MuJoCo viewer paper-style lighting/floor/grid are visualization-only changes. Do not tune controller behavior by changing these visual parameters.

## Prompt Template (recommended for Codex/Claude)
Goal: Implement/modify `src/<file>.m` without breaking existing interfaces.
Scope: Only touch `src/` + `tests/` (scripts/ if needed).
Must: Add/Update unit tests.
Done: All tests pass via matlab -batch runtests.

## Deliverables Checklist
- Planar kinematics + A2D + rank checks
- Planar statics/self-stress feasibility
- IK three modes (A elimination + B explicit vars)
- Pinocchio wrapper (M,h) and Route-B bias mapping
- HQP solver skeleton producing u_{a,wo} and final u_a
- Workspace scan tool + visualizer

## Readability / Documentation Rules (IMPORTANT)
- Prioritize learning-friendly, maintainable code over minimal code.
- Add at least one comment for each logical code block (what this block does).
- For key variables, explain meaning at first use (and dimensions/units when relevant).
- For matrix-heavy code, annotate expected dimensions (e.g., N×3, 3×1).
- Explain non-obvious operations (numerical stability tricks, vectorization, indexing logic).
- Each public function must include a clear header comment:
  - purpose
  - inputs / outputs
  - assumptions
  - edge cases
- Prefer descriptive variable names over short names (except simple loop indices).
- If code is generated/refactored, preserve or improve comments instead of removing them.
