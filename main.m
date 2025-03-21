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
%muF = 0.337;   % front max friction
muF = 0.7;

%Br  = 18.631;  % /rad (rear Pacejka B)
%Cr  = 1.5600;  % rear Pacejka C
%Dr  = -0.2477; % rear Pacejka D
%muR = 0.248;   % rear max friction
muR = 0.7;

h   = 0.1;     % CG height [m]
g   = 9.81;    % gravity [m/s^2]

%% 2) Simulation + MPC settings
Ts       = 0.1;    % sampling time [s]
N        = 5;     % prediction horizon (# steps)
simTime  = 20;      % total simulation time [s]

% Initial state:
%   x = [ p_cg_x; p_cg_y; psi; v_x; v_y; psi_dot ]
x_current = [0; 0; 0.1; 0; 0; 0];

%% 3) Generate the time-varying reference trajectory
% We'll assume you have a function generateTrajectoryData(Ts, simTime, x_current, ...)
% This function returns:
%   refData -> Nx6 array of reference states at each discrete step
%   timeVec -> 1xN time vector
[refData, timeVec] = generateTrajectoryData( ...
    Ts, simTime, x_current, ...
    lF, lR, m, Izz, ...
    muF, ...
    muR, ...
    h, g);
% Use only x and y positions for tracking:
refData = refData(:, 1:2);

N_total = length(timeVec);

%% 4) Construct a discrete update function handle (RK4 + halfCarDynamics)
f_discrete = @(x, u) rk4_discretization( ...
    @(xx, uu) halfCarDynamics(xx, uu, ...
                              lF, lR, m, Izz, ...
                              muF, ...
                              muR, ...
                              h, g), ...
    x, u, Ts);

%% 5) Nonlinear MPC object
nx = 6;  % number of states remains 6
ny = 2;  % now we only track x and y
nu = 3;
nlobj = nlmpc(nx, ny, nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = N;
nlobj.ControlHorizon    = round(nlobj.PredictionHorizon/5);
nlobj.Model.IsContinuousTime = false;

% Use the discrete model
nlobj.Model.StateFcn  = @(x, u) f_discrete(x, u);

% Return only x (position) and y (position)
nlobj.Model.OutputFcn = @(x, u) x(1:2);  

%% 6) Constraints
nlobj.MV(1).Min = -0.8;  nlobj.MV(1).Max = 0.8;   % steering
nlobj.MV(2).Min = -0.8;  nlobj.MV(2).Max = 0.8;   % s_fx
nlobj.MV(3).Min = -0.8;  nlobj.MV(3).Max = 0.8;   % s_rx

for i = 1:2
    nlobj.OV(i).Min = -inf;
    nlobj.OV(i).Max =  inf;
end

%% 7) Weights and Custom Cost Function
nlobj.Weights.OutputVariables = [1 1];
nlobj.Weights.ManipulatedVariables     = [5 5 5];
nlobj.Weights.ManipulatedVariablesRate = [5 5 5];

% Objective Function
nlobj.Optimization.CustomCostFcn = @(X,U,e,data) costFunction(X, U, data);
nloptions = nlmpcmoveopt;

%% 8) Closed-loop simulation
xHistory  = x_current;  
mvHistory = [];
mv0       = [0.05; 0.6; 0.6];    % initial guess for manipulated variables
u_previous = mv0;               % initialize previous control input

for k = 1 :(N_total - 1)  % step through each sample in timeVec
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
    [mv_opt, opt, info] = nlmpcmove(nlobj, x_current, mv0, y_ref_k, [], nloptions);
    disp(info.ExitFlag);  % Displays solver status

    % 8c) Check for solver success: if ExitFlag == 1, use new input; otherwise, use previous input
    if info.ExitFlag == 1 || info.ExitFlag == 2
        u_applied = mv_opt;
        u_previous = mv_opt;  % update the previous control input if successful
    else
        disp('Solver did not succeed with ExitFlag 1 or 2. Applying previous control input.');
        u_applied = 1 * u_previous;
        %u_applied = [0; 0; 0];
    end

    % 8d) Discrete-step update of the actual system
    x_next = f_discrete(x_current, u_applied);

    % 8e) Log data
    xHistory  = [xHistory, x_next];
    mvHistory = [mvHistory, u_applied]; 

    % 8f) Prepare for next iteration
    x_current = x_next;
    mv0 = u_previous;  % warm start for next iteration
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