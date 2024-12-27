% Multiple Vehicle Trajectories using Kinematic Model
clc; clear; close all;

% Vehicle Parameters
L = 2.5;         % Length of the vehicle [m]
dt = 0.1;        % Time step [s]
t_final = 20;    % Total simulation time [s]
t = 0:dt:t_final; % Time vector

% Initialize Variables for All Trajectories
n = length(t);
x = zeros(5, n);    % X positions for 5 trajectories
y = zeros(5, n);    % Y positions for 5 trajectories
theta = zeros(5, n); % Orientations for 5 trajectories

% Initial Conditions
x(:, 1) = 0; % All start from the same X position
y(:, 1) = 0; % All start from the same Y position
theta(:, 1) = 0; % All start facing along 0 degrees

%% 1. Trajectory 1: Straight Line Motion
v1 = 5; % Constant velocity [m/s]
delta1 = zeros(size(t)); % No steering

for k = 1:n-1
    x(1, k+1) = x(1, k) + v1 * cos(theta(1, k)) * dt;
    y(1, k+1) = y(1, k) + v1 * sin(theta(1, k)) * dt;
end

%% 2. Trajectory 2: Circular Motion with Constant Steering
v2 = 5; % Constant velocity [m/s]
delta2 = 0.2; % Constant steering angle [rad]

for k = 1:n-1
    x(2, k+1) = x(2, k) + v2 * cos(theta(2, k)) * dt;
    y(2, k+1) = y(2, k) + v2 * sin(theta(2, k)) * dt;
    theta(2, k+1) = theta(2, k) + (v2 / L) * tan(delta2) * dt;
end

%% 3. Trajectory 3: S-Turn with Sinusoidal Steering
v3 = 5; % Constant velocity [m/s]
delta3 = 0.3 * sin(0.5 * t); % Sine wave steering input

for k = 1:n-1
    x(3, k+1) = x(3, k) + v3 * cos(theta(3, k)) * dt;
    y(3, k+1) = y(3, k) + v3 * sin(theta(3, k)) * dt;
    theta(3, k+1) = theta(3, k) + (v3 / L) * tan(delta3(k)) * dt;
end

%% 4. Trajectory 4: Sinusoidal Steering with Reduced Speed
v4 = 2 + sin(0.2 * t); % Varying speed [m/s]
delta4 = 0.15 * sin(0.5 * t); % Steering input

for k = 1:n-1
    x(4, k+1) = x(4, k) + v4(k) * cos(theta(4, k)) * dt;
    y(4, k+1) = y(4, k) + v4(k) * sin(theta(4, k)) * dt;
    theta(4, k+1) = theta(4, k) + (v4(k) / L) * tan(delta4(k)) * dt;
end

%% 5. Trajectory 5: U-Turn Motion
v5 = 5; % Constant velocity [m/s]
delta5 = 0.3 * sin(0.1 * pi * t); % Large amplitude for sharp turn

for k = 1:n-1
    x(5, k+1) = x(5, k) + v5 * cos(theta(5, k)) * dt;
    y(5, k+1) = y(5, k) + v5 * sin(theta(5, k)) * dt;
    theta(5, k+1) = theta(5, k) + (v5 / L) * tan(delta5(k)) * dt;
end

%% Plotting All Trajectories
figure;
hold on;
plot(x(1, :), y(1, :), 'b', 'LineWidth', 1.5); % Straight Line
plot(x(2, :), y(2, :), 'm', 'LineWidth', 1.5); % Circular
plot(x(3, :), y(3, :), 'r', 'LineWidth', 1.5); % S-Turn
plot(x(4, :), y(4, :), 'g', 'LineWidth', 1.5); % Sinusoidal Steering
plot(x(5, :), y(5, :), 'k', 'LineWidth', 1.5); % U-Turn

xlabel('X Position [m]');
ylabel('Y Position [m]');
title('Multiple Vehicle Trajectories using Kinematic Model');
legend({'Straight', 'Circular', 'S-Turn', 'Sinusoidal', 'U-Turn'});
grid on;
axis equal;
hold off;
