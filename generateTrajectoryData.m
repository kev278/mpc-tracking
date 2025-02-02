function [refData, timeVec] = generateTrajectoryData( ...
                    Ts, simTime, x0, ...
                    lF, lR, m, Izz, ...
                    muF, ...
                    muR, ...
                    h, g)
% generateTrajectoryData_S_Curve
% -------------------------------------------------------------------------
% Creates a **slow-moving S-curve trajectory** for more natural car-like motion.
%
% OUTPUTS:
%   refData  - (N x 6) array of reference states at each time step
%              [ p_cg_x, p_cg_y, psi, v_x, v_y, psi_dot ]
%   timeVec  - 1 x N vector of time stamps
% -------------------------------------------------------------------------

    % 1) Create a time vector
    timeVec = 0 : Ts : simTime;
    N       = length(timeVec);

    % 2) Define an **S-curve shape**
    x_ref = linspace(0, 100, N);  % Move forward along x-axis
    y_ref = 20 * sin(0.02 * x_ref);  % Generate S-shape in y-direction

    % 3) Compute heading angle (psi) from trajectory slope
    psi_ref = atan2(diff(y_ref, 1, 2), diff(x_ref, 1, 2));
    psi_ref = [psi_ref, psi_ref(end)];  % Repeat last value to maintain size

    % 4) Define a **slow velocity profile**
    vProfile = 0.1 * ones(1, N);  % Move at a constant slow speed (0.5 m/s)

    % 5) Build reference data
    refData = zeros(N, 6);
    refData(:,1) = x_ref;  % p_cg_x
    refData(:,2) = y_ref;  % p_cg_y
    refData(:,3) = psi_ref;  % psi (heading)
    refData(:,4) = vProfile;  % v_x (slow constant speed)
    
end
