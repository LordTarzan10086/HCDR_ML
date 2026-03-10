function [x_d, xd_d, xdd_d] = trajectory_se3_cubic(t, t0, tf, x0, xf)
%TRAJECTORY_SE3_CUBIC Cubic point trajectory with zero endpoint velocity.
%
%   [X_D, XD_D, XDD_D] = TRAJECTORY_SE3_CUBIC(T, T0, TF, X0, XF)
%   returns desired position/velocity/acceleration for a 3D point:
%       x_d(t) = x0 + s(t) * (xf - x0)
%   with cubic blend:
%       s(tau) = 3*tau^2 - 2*tau^3, tau in [0,1]
%   and boundary conditions:
%       x_d(t0)=x0, x_d(tf)=xf, xd_d(t0)=xd_d(tf)=0.
%
%   Inputs:
%   t:   current time [s]
%   t0:  start time [s]
%   tf:  end time [s], tf > t0
%   x0:  start point, 3x1 [m]
%   xf:  goal point, 3x1 [m]
%
%   Outputs:
%   x_d:   desired position, 3x1 [m]
%   xd_d:  desired velocity, 3x1 [m/s]
%   xdd_d: desired acceleration, 3x1 [m/s^2]

    arguments
        t (1, 1) double
        t0 (1, 1) double
        tf (1, 1) double
        x0 (:, 1) double
        xf (:, 1) double
    end

    if numel(x0) ~= 3 || numel(xf) ~= 3
        error("HCDR:DimMismatch", "x0 and xf must both be 3x1.");
    end
    if ~(tf > t0)
        error("HCDR:ArgInvalid", "tf must be strictly greater than t0.");
    end

    % Clamp normalized time tau to [0,1] for bounded profiles.
    horizonSec = tf - t0;
    tau = min(max((t - t0) / horizonSec, 0.0), 1.0);

    % Cubic blend and derivatives wrt tau.
    s = 3.0 * tau^2 - 2.0 * tau^3;
    ds_dtau = 6.0 * tau - 6.0 * tau^2;
    d2s_dtau2 = 6.0 - 12.0 * tau;

    delta = xf - x0;  % 3x1 [m]
    x_d = x0 + s * delta;
    xd_d = (ds_dtau / horizonSec) * delta;
    xdd_d = (d2s_dtau2 / (horizonSec^2)) * delta;

    x_d = double(x_d);
    xd_d = double(xd_d);
    xdd_d = double(xdd_d);
end
