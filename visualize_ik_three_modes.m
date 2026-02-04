function visualize_ik_three_modes(out, config)
%VISUALIZE_IK_THREE_MODES  Visualize three IK modes in 1x3 subplots (old-style logic)
%
% Input:
%   out: struct containing fields:
%       out.platform.q_p, out.platform.h, (optional) out.platform.q_a
%       out.arm.q_p, out.arm.h, out.arm.q_a
%       out.coop.q_p, out.coop.h, out.coop.q_a
%   config: HCDR_config_v2

    if nargin < 2 || isempty(config)
        config = HCDR_config_v2();
    end

    f = figure('Name','IK Three Modes (5DOF yaw=0)','Color','w');
    tiledlayout(f, 1, 3, 'Padding','compact', 'TileSpacing','compact');

    % -------- platform-only --------
    nexttile; ax1 = gca;
    title(ax1, 'Platform-only');
    q_a1 = [];
    if isfield(out.platform, 'q_a') && ~isempty(out.platform.q_a)
        q_a1 = out.platform.q_a;
    elseif isfield(config, 'arm') && isfield(config.arm, 'q_fixed')
        q_a1 = config.arm.q_fixed;
    end
    HCDR_visualize_5d(out.platform.q_p, out.platform.h, q_a1, config, [], false, ax1);

    % -------- arm-only --------
    nexttile; ax2 = gca;
    title(ax2, 'Arm-only');
    q_p2 = out.arm.q_p;
    if isfield(out.arm, 'h')
    h2 = out.arm.h;
    else
        % arm-only：平台与滑块不动 -> 用初始/参考高度来画
        h2 = config.screw.h_init(:);
    end

    HCDR_visualize_5d(q_p2, h2, out.arm.q_a, config, [], false, ax2);

    % -------- cooperative --------
    nexttile; ax3 = gca;
    title(ax3, 'Cooperative');
    HCDR_visualize_5d(out.coop.q_p, out.coop.h, out.coop.q_a, config, [], false, ax3);
end
