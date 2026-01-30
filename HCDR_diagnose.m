%% HCDR Configuration Diagnostics Tool

function HCDR_diagnose(q_m, h, config)
    % Detailed diagnostics for configuration singularity
    
    fprintf('\n========================================\n');
    fprintf('  Configuration Diagnostics\n');
    fprintf('========================================\n\n');
    
    % Compute cable geometry
    [L, L_hat, r_attach, A_m, a_anchor] = HCDR_kinematics.cable_geometry(q_m, h, config);
    
    %% 1. Cable directions
    fprintf('--- Cable Unit Vectors ---\n');
    fprintf('Cable |    u_x    |    u_y    |    u_z    | Length(m)\n');
    fprintf('------|-----------|-----------|-----------|----------\n');
    for i = 1:8
        fprintf('  %d   | %+8.5f | %+8.5f | %+8.5f | %7.4f\n', ...
            i, L_hat(1,i), L_hat(2,i), L_hat(3,i), L(i));
    end
    
    %% 2. Structure matrix rank and null space
    fprintf('\n--- Structure Matrix Analysis ---\n');
    [U, S, V] = svd(A_m);
    singular_values = diag(S);
    
    fprintf('Rank: %d (threshold 1e-6)\n', rank(A_m, 1e-6));
    fprintf('Condition number: %.2e\n', cond(A_m));
    fprintf('\nSingular values:\n');
    for i = 1:6
        fprintf('  σ_%d = %.6e', i, singular_values(i));
        if singular_values(i) < 1e-4
            fprintf('  ← SMALL!\n');
        else
            fprintf('\n');
        end
    end
    
    % Null space of A_m'
    fprintf('\nNull space dimension: %d\n', size(null(A_m'), 2));
    if size(null(A_m'), 2) > 0
        fprintf('WARNING: A_m is rank-deficient!\n');
        fprintf('Uncontrollable wrench direction:\n');
        null_vec = null(A_m');
        for j = 1:size(null_vec, 2)
            fprintf('  Mode %d: [F: %.3f, %.3f, %.3f; M: %.3f, %.3f, %.3f]\n', ...
                j, null_vec(1:3, j), null_vec(4:6, j));
        end
    end
    
    %% 3. Check symmetry
    fprintf('\n--- Symmetry Check ---\n');
    fprintf('Pulley heights: [%.3f, %.3f, %.3f, %.3f] m\n', h);
    h_std = std(h);
    fprintf('Height std dev: %.4f m', h_std);
    if h_std < 0.01
        fprintf(' ← SYMMETRIC (likely cause of singularity!)\n');
    else
        fprintf('\n');
    end
    
    fprintf('Platform orientation: [%.3f, %.3f, %.3f] rad\n', q_m(4:6));
    angle_norm = norm(q_m(4:6));
    fprintf('Rotation magnitude: %.4f rad', angle_norm);
    if angle_norm < 0.01
        fprintf(' ← ZERO ROTATION\n');
    else
        fprintf('\n');
    end
    
    %% 4. Cable attachment point analysis
    fprintf('\n--- Attachment Point Geometry ---\n');
    fprintf('Platform corners (in {O}):\n');
    for k = 1:4
        fprintf('  Corner %d:\n', k);
        fprintf('    Upper (cable %d): [%.3f, %.3f, %.3f] → [%.3f, %.3f, %.3f]\n', ...
            2*k-1, r_attach(:,2*k-1), a_anchor(:,2*k-1));
        fprintf('    Lower (cable %d): [%.3f, %.3f, %.3f] → [%.3f, %.3f, %.3f]\n', ...
            2*k, r_attach(:,2*k), a_anchor(:,2*k));
    end
    
    %% 5. Recommendations
    fprintf('\n========================================\n');
    fprintf('  Recommendations\n');
    fprintf('========================================\n');
    
    if h_std < 0.01 && angle_norm < 0.01
        fprintf('\nPROBLEM IDENTIFIED: Symmetric configuration!\n\n');
        fprintf('Solutions:\n');
        fprintf('  1. Set different pulley heights, e.g.:\n');
        fprintf('     h_init = [%.2f; %.2f; %.2f; %.2f];\n', ...
            h(1), h(1)+0.05, h(1)-0.05, h(1)+0.02);
        fprintf('\n  2. Or give platform a small rotation:\n');
        fprintf('     platform_pose(4:6) = [0.05; 0.03; 0];\n');
    elseif singular_values(end) < 1e-6
        fprintf('\nConfiguration is near-singular.\n');
        fprintf('Try adjusting pulley heights or platform pose.\n');
    else
        fprintf('\nConfiguration appears feasible.\n');
    end
    
    fprintf('\n');
end