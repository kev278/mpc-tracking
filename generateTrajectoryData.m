function [refData, v_des, timeVec] = generateTrajectoryData(Ts, simTime, x0, lF, lR, m, Izz, muF, muR, h, g)
    % Generate a 2D S-curve trajectory with a desired velocity profile.
    %
    % Outputs:
    %   refData - [N x 2] array with x-y positions
    %   v_des   - [N x 1] desired forward velocity (m/s)
    %   timeVec - 1 x N time stamps

    timeVec = 0:Ts:simTime;
    N = length(timeVec);
    
    % S-curve in x-y:
    x_ref = linspace(0, 200, N);          % x from 0 to 200 m
    y_ref = 10 * sin(0.02 * x_ref);         % S-shaped y trajectory
    
    % Desired forward velocity (e.g., constant 0.5 m/s)
    v_des = 0.5 * ones(N, 1);
    
    % Only x-y positions are used for tracking
    refData = [x_ref', y_ref'];
end