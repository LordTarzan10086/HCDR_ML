# Mechanical Arm URDF Visualization Guide

This guide shows how to replace the current line-link arm rendering with a real URDF+mesh model.

## 1) Prepare Files

1. Put your files into the repository, for example:
   - `assets/robot_urdf/robot.urdf`
   - `assets/robot_urdf/meshes/*.stl`
2. Ensure URDF mesh paths are relative and valid.
3. Ensure units are meters (if CAD exported in mm, scale before export).

## 2) Standalone URDF Validation (first check)

Run this in MATLAB first to verify the model itself:

```matlab
robot = importrobot('assets/robot_urdf/robot.urdf');
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];

q = zeros(numel(homeConfiguration(robot)),1);
figure('Color','w');
ax = axes; hold(ax,'on'); axis(ax,'equal'); view(ax,45,25); grid(ax,'on');
show(robot, q, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');
title(ax, 'Standalone URDF Check');
```

If this fails, do not integrate into HCDR yet. Fix URDF/mesh first.

## 3) Coordinate Alignment with This Project

In this project:

1. Platform center in world:
   - `p_platform^O = [x; y; z0]`
2. Platform yaw:
   - `R_platform^O = Rz(psi)`
3. Arm base offset in platform frame:
   - `p_base^P = cfg.arm.offset_in_platform`
4. Arm base in world:
   - `p_base^O = p_platform^O + R_platform^O * p_base^P`

To avoid double offset:

1. Keep `offset_in_platform` inside arm model convention.
2. Do not add/subtract base offset again outside FK/IK formulas.

## 4) Hook URDF Rendering into Existing Visualizer

`HCDR_visualize_planar` already supports optional hook:

```matlab
HCDR_visualize_planar(q, cfg, ...
    'robot_visual_model', struct('render_fn', @my_urdf_render_fn, 'replace_links', true));
```

Where `my_urdf_render_fn(ax, q, cfg)` is your custom renderer.

Minimal template:

```matlab
function my_urdf_render_fn(ax, q, cfg)
    persistent robot
    if isempty(robot)
        robot = importrobot('assets/robot_urdf/robot.urdf');
        robot.DataFormat = 'column';
    end

    q_arm = q(4:end);  % project arm joints

    % Platform pose in world
    x = q(1); y = q(2); psi = q(3);
    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    p_platform_world = [x; y; cfg.z0];
    p_base_world = p_platform_world + Rz * cfg.arm.offset_in_platform(:);

    % Option A (recommended): pre-transform URDF base in URDF itself to match
    % your project frame, then call show(robot, q_arm, ...).
    % Option B: wrap URDF visuals under an hgtransform and apply world transform.
    % (implementation depends on your URDF tree and MATLAB release)
    show(robot, q_arm, 'Parent', ax, 'PreservePlot', false, 'Frames', 'off');

    % Draw base marker to verify alignment
    plot3(ax, p_base_world(1), p_base_world(2), p_base_world(3), 'ko', ...
        'MarkerSize', 6, 'MarkerFaceColor', 'k');
end
```

## 5) Recommended Integration Sequence

1. Make standalone URDF render correct.
2. Match URDF base frame to arm base frame convention.
3. Compare URDF end-effector position with existing FK end-effector marker.
4. Enable `replace_links=true` only after alignment is correct.

## 6) Troubleshooting Checklist

1. Model appears scaled wrong:
   - URDF/CAD units mismatch (mm vs m).
2. Model appears mirrored or rotated:
   - Base axis convention mismatch (right-hand vs exported orientation).
3. Model detached from platform:
   - Base transform or `offset_in_platform` applied twice/missing.
4. End-effector trajectory differs from solver:
   - Joint order mismatch between URDF and `q(4:end)`.
5. Performance drops during animation:
   - Use low-poly meshes for demo and avoid re-importing URDF every frame.
