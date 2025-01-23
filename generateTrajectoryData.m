function [refData, timeVec] = generateTrajectoryData( ...
                    Ts, simTime, x0, ...
                    lF, lR, m, Izz, ...
                    muF, ...
                    muR, ...
                    h, g)
% generateTrajectoryDataDynamic
% -------------------------------------------------------------------------
% Creates a time-varying reference trajectory by forward-simulating
% the same half-car dynamics you use in your MPC. 
%
% We define time-varying inputs u(t) = [delta(t); s_fx(t); s_rx(t)]
% and numerically integrate from an initial state x0.
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
    %    Example: a small sinusoidal steering of amplitude 0.15 rad,
    %             a mild positive slip up front (0.05),
    %             near-zero slip in the rear (0).
    %    Feel free to modify these as you wish!
    deltaProfile = 0.3 * sin( 0.3 * timeVec );    % [rad]
    sfxProfile   = 0.05 * ones(1, N);              % front slip ratio (positive => drive)
    srxProfile   = 0.00 * ones(1, N);              % rear slip ratio (could also be small if you like)

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
    %    For example, x0 might be [0; 0; 0; 0.1; 0; 0] if you want a small initial speed
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
    %    (Just transpose xRef since each column is a state)
    refData = xRef.';
    %   => refData(k,:) = [p_cg_x, p_cg_y, psi, v_x, v_y, psi_dot]

end
