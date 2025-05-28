clc; clear;
%% Define vehicle parameters
lF  = 1.2;     % [m]
lR  = 1.3;     % [m]
m   = 1500;    % [kg]
Izz = 3000;    % [kg*m^2]
muF = 0.9;
muR = 0.9;
h   = 0.1;     % [m]
g   = 9.81;    % [m/s^2]

%% Define NLMP controller for half-car model
nx = 6;   % States: [p_cg_x; p_cg_y; psi; v_x; v_y; psi_dot]
ny = 3;   % Track: [p_cg_x; p_cg_y; psi]
nu = 3;   % Inputs: [delta; s_fx; s_rx]

% Create the NLMP controller object
nlobj = nlmpc(nx, ny, nu);

% Set the state function handle using your dynamics
nlobj.Model.StateFcn = @(x,u) halfCarDynamics(x, u, lF, lR, m, Izz, muF, muR, h, g);

% Specify an output function to return only the first three states
nlobj.Model.OutputFcn = @(x,u) x(1:3);

% Specify sample time, prediction horizon, and control horizon
nlobj.Ts = 0.1;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;

% Define weights for the outputs
nlobj.Weights.OutputVariables = [1 1 1];          
nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];

% Define input constraints (example limits)
nlobj.MV(1).Min = -0.5;   % Steering angle delta [rad]
nlobj.MV(1).Max = 0.5;
nlobj.MV(2).Min = -0.2;   % Front longitudinal slip s_fx
nlobj.MV(2).Max = 0.2;
nlobj.MV(3).Min = -0.2;   % Rear longitudinal slip s_rx
nlobj.MV(3).Max = 0.2;

%% Set up simulation parameters
% Initial state: starting at the origin, moving forward at 10 m/s
x0 = [0; 0; 0; 10; 0; 0];
Tfinal = 10;            % Total simulation time [s]
time = 0:nlobj.Ts:Tfinal;

% Define a simple, straight-line reference trajectory:
% The car moves forward along x, with p_cg_x = 10*t, and p_cg_y = 0, psi = 0.
ref = zeros(length(time), ny);
for k = 1:length(time)
    ref(k, :) = [ 10*time(k), 0, 0 ];
end

% Preallocate arrays for storing simulation results
xHistory = zeros(nx, length(time));
uHistory = zeros(nu, length(time)-1);
xHistory(:,1) = x0;

% Initialize lastMV as a row vector (1×3)
lastMV = [0 0 0];

%% Closed-loop simulation using nlmpcmove
for k = 1:length(time)-1
    % Compute the optimal control input using nlmpcmove
    [u, ~] = nlmpcmove(nlobj, xHistory(:,k), ref(k+1,:)', lastMV);
    
    % Save the computed control input
    uHistory(:,k) = u;
    
    % Simulate one time step ahead using your dynamics function
    xHistory(:,k+1) = halfCarDynamics(xHistory(:,k), u, lF, lR, m, Izz, muF, muR, h, g);
    
    % Update lastMV as a row vector for the next iteration
    lastMV = u(:)';
end

%% Plot the simulation results
figure;
subplot(3,1,1)
plot(time, xHistory(1,:), 'LineWidth',1.5);
hold on;
plot(time, ref(:,1), '--','LineWidth',1.5);
xlabel('Time (s)');
ylabel('p_{cg,x} (m)');
legend('Simulated','Reference');
title('CG X Position');

subplot(3,1,2)
plot(time, xHistory(2,:), 'LineWidth',1.5);
hold on;
plot(time, ref(:,2), '--','LineWidth',1.5);
xlabel('Time (s)');
ylabel('p_{cg,y} (m)');
legend('Simulated','Reference');
title('CG Y Position');

subplot(3,1,3)
plot(time, xHistory(3,:), 'LineWidth',1.5);
hold on;
plot(time, ref(:,3), '--','LineWidth',1.5);
xlabel('Time (s)');
ylabel('\psi (rad)');
legend('Simulated','Reference');
title('Yaw Angle');
grid on;
