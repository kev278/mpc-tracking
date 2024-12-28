function main()
    % main
    % --------------------------------------------------------
    % Demonstrates closed-loop Nonlinear MPC on the half-car
    % model. We assume you have:
    %    1) halfCarDynamics.m
    %    2) rk4_discretization.m
    % in the same folder, each using individual parameters,
    % not a struct.
    % --------------------------------------------------------
    clc; clear; close all;

    %% 1) Define all parameters individually (no struct)
    lF  = 1.2;     % m
    lR  = 1.3;     % m
    m   = 1500;    % kg
    Izz = 3000;    % kg*m^2

    Bf  = 11.275;  % /rad (front Pacejka B)
    Cf  = 1.5600;  % front Pacejka C
    Df  = -0.3365; % front Pacejka D (negative from your data)
    muF = 0.337;   % front max friction

    Br  = 18.631;  % /rad (rear Pacejka B)
    Cr  = 1.5600;  % rear Pacejka C
    Dr  = -0.2477; % rear Pacejka D
    muR = 0.248;   % rear max friction

    h   = 0.1;     % CG height
    g   = 9.81;    % gravity

    %% 2) MPC settings
    Ts = 0.1;    % sampling time
    N  = 10;     % prediction horizon

    % Initial state: x = [ p_cg_x; p_cg_y; psi; v_x; v_y; psi_dot ]
    x_current = [0; 0; 0; 0; 0; 0];

    % Reference we want to track (1x6 row for nlmpcmove)
    y_ref = [20 20 0 0 0 0];  % p_cg_x=20, p_cg_y=0, psi=0, v_x=10, v_y=0, psi_dot=0

    simTime = 5;                   % total sim time
    timeVec = 0 : Ts : simTime;    % time steps

    %% 3) Construct a discrete update function handle
    % We'll call your existing halfCarDynamics + rk4_discretization
    f_discrete = @(x, u) rk4_discretization( ...
        @(xx, uu) halfCarDynamics(xx, uu, ...
                                  lF, lR, m, Izz, ...
                                  Bf, Cf, Df, muF, ...
                                  Br, Cr, Dr, muR, ...
                                  h, g), ...
        x, u, Ts);

    %% 4) Nonlinear MPC object
    nx = 6;   % # of states
    ny = 6;   % # of outputs (tracking all 6 states)
    nu = 3;   % # of inputs [delta, s_fx, s_rx]

    nlobj = nlmpc(nx, ny, nu);

    % Discrete-time
    nlobj.Ts = Ts;
    nlobj.PredictionHorizon = N;
    nlobj.ControlHorizon    = N;
    nlobj.Model.IsContinuousTime = false;

    % State and output functions
    nlobj.Model.StateFcn  = @(x, u) f_discrete(x, u);
    nlobj.Model.OutputFcn = @(x, u) x;  % we track all states as outputs

    %% 5) Constraints
    % Inputs: [delta; s_fx; s_rx]
    nlobj.MV(1).Min = -0.3;  nlobj.MV(1).Max = 0.3;   % Steering
    nlobj.MV(2).Min = -0.2;  nlobj.MV(2).Max = 0.2;   % s_fx
    nlobj.MV(3).Min = -0.2;  nlobj.MV(3).Max = 0.2;   % s_rx

    % States: e.g. v_x >= 0, v_y in [-5, 5]
    nlobj.OV(4).Min = 0;     % state(4)=v_x >= 0
    nlobj.OV(5).Min = -5;
    nlobj.OV(5).Max =  5;

    %% 6) Weights
    % If you have more OVs than MVs, MATLAB might warn about zero weights,
    % but that's typically OK. Just ensure at least one output is weighted.
    nlobj.Weights.OutputVariables = [1 1 0.1 0.1 0.1 0.1];
    nlobj.Weights.ManipulatedVariables      = [0 0 0];
    nlobj.Weights.ManipulatedVariablesRate  = [0.1 0.01 0.01];

    %% 7) Closed-loop simulation
    xHistory = x_current;  
    mvHistory = [];
    mv0 = [0; 0; 0];    % initial guess for the solver

    for k = 1 : (length(timeVec) - 1)
        % Solve the MPC problem
        [mv_opt, info] = nlmpcmove(nlobj, x_current, mv0, y_ref, []);

        % Apply the first (optimal) control
        u_applied = mv_opt;

        % Discrete-step update
        x_next = f_discrete(x_current, u_applied);

        % Log
        xHistory  = [xHistory, x_next];
        mvHistory = [mvHistory, u_applied];

        % Prepare for next iteration
        x_current = x_next;
        mv0 = mv_opt;  % warm start
    end

    %% 8) Plots
    figure('Name','Closed-Loop Trajectory');
    plot(xHistory(1,:), xHistory(2,:), 'o-'); grid on;
    xlabel('p_{cg,x}'); ylabel('p_{cg,y}');
    title('Half-Car Trajectory under Nonlinear MPC');

    figure('Name','Control Inputs');
    subplot(3,1,1); plot(timeVec(1:end-1), mvHistory(1,:), '-o'); ylabel('\delta');
    subplot(3,1,2); plot(timeVec(1:end-1), mvHistory(2,:), '-o'); ylabel('s_{fx}');
    subplot(3,1,3); plot(timeVec(1:end-1), mvHistory(3,:), '-o'); ylabel('s_{rx}');
    xlabel('Time [s]');
    sgtitle('Optimal Control Inputs');
end
