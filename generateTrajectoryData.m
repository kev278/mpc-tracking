function [refData, timeVec] = generateTrajectoryData( ...
                    Ts, simTime, x0, ...
                    lF, lR, m, Izz, ...
                    muF, ...
                    muR, ...
                    h, g)
% generateTrajectoryDataDynamic
% -------------------------------------------------------------------------
% Creates a straight-line reference trajectory by forward-simulating
% the same half-car dynamics you use in your MPC.
%
% OUTPUTS:
%   refData  - (N x 6) array of reference states at each time step
%              [ p_cg_x, p_cg_y, psi, v_x, v_y, psi_dot ]
%   timeVec  - 1 x N vector of time stamps
% -------------------------------------------------------------------------

    % 1) Create a time vector
    timeVec = 0 : Ts : simTime;
    N       = length(timeVec);

    % 2) Define the time-varying inputs: [delta; s_fx; s_rx]
    %    For a straight-line trajectory:
    deltaProfile = zeros(1, N);   % No steering (straight line)
    sfxProfile   = 0.0001 * ones(1, N);  % Small constant front slip ratio for forward motion
    srxProfile   = 0.0001 * ones(1, N);  % Small constant rear slip ratio

    % 3) Build a discrete update function handle (just like in your main code)
    f_continuous = @(X,U) halfCarDynamics(X, U, ...
                                lF, lR, m, Izz, ...
                                muF, ...
                                muR, ...
                                h, g);

    f_discrete = @(X,U) rk4_discretization(f_continuous, X, U, Ts);

    % 4) Allocate memory for the reference state
    xRef = zeros(6, N);    % Each column: [p_cg_x; p_cg_y; psi; v_x; v_y; psi_dot]

    % 5) Set the initial state
    xRef(:,1) = x0;

    % 6) Forward-integrate the half-car, step by step
    for k = 1 : (N-1)
        % Construct control input for this step
        u_k = [ deltaProfile(k); ...
                sfxProfile(k); ...
                srxProfile(k) ];
        
        % Advance the state one step
        xRef(:, k+1) = f_discrete( xRef(:, k), u_k );
    end

    % 7) Build the final Nx6 array for the reference trajectory
    refData = xRef.';
    %   => refData(k,:) = [p_cg_x, p_cg_y, psi, v_x, v_y, psi_dot]

end
