function main()
    % main
    % --------------------------------------------------------
    % Demonstrates closed-loop Nonlinear MPC on the half-car
    % model with a time-varying reference trajectory. The
    % reference is generated externally by generateTrajectoryData.m.
    %
    % Prerequisites (all in the same folder):
    %   1) halfCarDynamics.m
    %   2) rk4_discretization.m
    %   3) generateTrajectoryData.m  (the custom trajectory generator)
    %
    % Make sure halfCarDynamics:
    %   - uses the signature: halfCarDynamics(x,u, lF,lR,m,Izz,Bf,Cf,Df,muF,Br,Cr,Dr,muR,h,g)
    %   - has properly parenthesized slip calculations
    % --------------------------------------------------------
    clc; clear; close all;
    load handel
    %% 1) Define vehicle parameters individually (no struct)
    lF  = 1.2;     % [m]
    lR  = 1.3;     % [m]
    m   = 1500;    % [kg]
    Izz = 3000;    % [kg*m^2]

    %Bf  = 11.275;  % /rad (front Pacejka B)
    %Cf  = 1.5600;  % front Pacejka C
    %Df  = -0.3365; % front Pacejka D
    muF = 0.337;   % front max friction

    %Br  = 18.631;  % /rad (rear Pacejka B)
    %Cr  = 1.5600;  % rear Pacejka C
    %Dr  = -0.2477; % rear Pacejka D
    muR = 0.248;   % rear max friction

    h   = 0.1;     % CG height [m]
    g   = 9.81;    % gravity [m/s^2]

    %% 2) Simulation + MPC settings
    Ts       = 0.1;    % sampling time [s]
    N        = 20;     % prediction horizon (# steps)
    simTime  = 20;      % total simulation time [s]

    % Initial state:
    %   x = [ p_cg_x; p_cg_y; psi; v_x; v_y; psi_dot ]
    x_current = [0; 0; 0; 0.1; 0; 0];

    %% 3) Generate the time-varying reference trajectory
    % We'll assume you have a function generateTrajectoryData(Ts, simTime, lF, lR, <etc>).
    % This function returns:
    %   refData -> Nx6 array of reference states at each discrete step
    %   timeVec -> 1xN time vector
    % 
    % The dimension of refData: (#steps) x 6
    %    where each row is: [p_cg_x, p_cg_y, psi, v_x, v_y, psi_dot]
    [refData, timeVec] = generateTrajectoryData( ...
    Ts, simTime, x_current, ...
    lF, lR, m, Izz, ...
    muF, ...
    muR, ...
    h, g);

% Now refData is an Nx6 array of feasible states from the same halfCarDynamics.
% You can pass it to your MPC just like before.
    N_total = length(timeVec);

    % If needed, verify size(refData) == [N_total, 6].

    %% 4) Construct a discrete update function handle (RK4 + halfCarDynamics)
    f_discrete = @(x, u) rk4_discretization( ...
        @(xx, uu) halfCarDynamics(xx, uu, ...
                                  lF, lR, m, Izz, ...
                                  muF, ...
                                  muR, ...
                                  h, g), ...
        x, u, Ts);

    %% 5) Nonlinear MPC object
    nx = 6;  % # states
    ny = 6;  % # outputs (we track all states)
    nu = 3;  % # inputs: [delta, s_fx, s_rx]

    nlobj = nlmpc(nx, ny, nu);
    nlobj.Ts = Ts;
    nlobj.PredictionHorizon = N;
    nlobj.ControlHorizon    = N/5;
    nlobj.Model.IsContinuousTime = false;

    % Use the discrete model
    nlobj.Model.StateFcn  = @(x, u) f_discrete(x, u);
    nlobj.Model.OutputFcn = @(x, u) x;  % track all states as outputs

    %nlobj.Optimization.CustomCostFcn = @(X,U,e,data) ...
    %10 * sum( (X(end, :) - data.References(end, :)).^2 );

    %% 6) Constraints
    nlobj.MV(1).Min = -1.5;  nlobj.MV(1).Max = 1.5;   % steering
    nlobj.MV(2).Min = -1.5;  nlobj.MV(2).Max = 1.5;   % s_fx
    nlobj.MV(3).Min = -1.5;  nlobj.MV(3).Max = 1.5;   % s_rx

    % e.g., v_x >= 0, v_y in [-5, 5]
    nlobj.OV(4).Min = 0;
    nlobj.OV(5).Min = -10;
    nlobj.OV(5).Max =  10;

    %% 7) Weights
    % nlobj.Weights.OutputVariables = [5 5 0.1 0.5 0.1 0.1];
    % nlobj.Weights.ManipulatedVariables      = [0.5 0.5 0.5];
    % nlobj.Weights.ManipulatedVariablesRate  = [0.5 0.5 0.5];
    % Turn off built-in cost weights so only CustomCostFcn is used
    nlobj.Weights.OutputVariables          = zeros(1, ny); % e.g. [0 0 0 0 0 0]
    nlobj.Weights.ManipulatedVariables     = zeros(1, nu); % e.g. [0 0 0]
    nlobj.Weights.ManipulatedVariablesRate = zeros(1, nu); % e.g. [0 0 0]

    
    % Objective Function
    nlobj.Optimization.CustomCostFcn = @(X,U,e,data) ...
    costFunction(X, U, data);


    %% 8) Closed-loop simulation
    xHistory  = x_current;  
    mvHistory = [];
    mv0       = [0.2; 0.2; 0.2];    % initial guess for manipulated variables

    for k = 1 : (N_total - 1)  % step through each sample in timeVec
        % 8a) Build the local reference window for the next N steps
        idxEnd = min(k + N - 1, N_total);  % don't exceed final index
        y_ref_k = refData(k+1 : idxEnd, :);  % subarray of size <= N x 6

        % If it's shorter than N near the end, pad with the last row
        needed = N - size(y_ref_k, 1);
        if needed > 0
            lastRow  = y_ref_k(end, :);
            padArray = repmat(lastRow, needed, 1);
            y_ref_k  = [ y_ref_k; padArray ];
        end

        % 8b) Solve the MPC problem for the current state
        [mv_opt, info] = nlmpcmove(nlobj, x_current, mv0, y_ref_k, []);

        % 8c) Apply the first (optimal) control
        u_applied = mv_opt;

        % 8d) Discrete-step update of the actual system
        x_next = f_discrete(x_current, u_applied);

        % 8e) Log
        xHistory  = [xHistory, x_next];
        mvHistory = [mvHistory, u_applied];

        % 8f) Prepare for next iteration
        x_current = x_next;
        mv0 = mv_opt;  % warm start next time
    end

    %% 9) Plot results
    figure('Name','Closed-Loop Trajectory vs Desired');
    plot(xHistory(1,:), xHistory(2,:), 'ro-'); hold on; grid on;
    plot(refData(:,1), refData(:,2), 'b--','LineWidth',1.5);
    legend('Vehicle Trajectory','Reference Path');
    xlabel('p_{cg,x}'); ylabel('p_{cg,y}');
    title('Closed-Loop MPC with Time-Varying Reference');
    axis equal;

    figure('Name','Control Inputs');
    subplot(3,1,1); plot(timeVec(1:end-1), mvHistory(1,:), '-o'); ylabel('\delta (rad)');
    subplot(3,1,2); plot(timeVec(1:end-1), mvHistory(2,:), '-o'); ylabel('s_{fx}');
    subplot(3,1,3); plot(timeVec(1:end-1), mvHistory(3,:), '-o'); ylabel('s_{rx}');
    xlabel('Time [s]');
    sgtitle('Optimal Control Inputs');

    sound(y,Fs)
end
