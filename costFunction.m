function J = costFunction(X, U, data)
    % Custom cost function for 2D trajectory tracking with desired velocity.
    %
    % X: Predicted state trajectory (N x 6)
    % U: Predicted control inputs (N x 3)
    % data: Structure containing:
    %       - data.References: [N x 2] numeric reference for x-y positions
    %       - data.CustomData.v_des: [N x 1] desired forward velocity
    
    % Define weights
    Q_pos         = diag([200, 200]);  % Position tracking (x-y)
    Q_vel         = 50;                % Velocity tracking weight
    R_abs         = diag([5, 5, 5]);     % Control effort penalty
    R_rate        = diag([10, 10, 10]);  % Control increment penalty
    Q_terminal    = diag([250, 250]);    % Terminal position penalty
    Q_terminalVel = 50;                % Terminal velocity penalty
    
    J = 0;
    Npred = size(X, 1);
    
    % Position reference from data (should be a numeric array of size [Npred x 2])
    refPos = data.References;
    
    % Desired velocity from CustomData (size [Npred x 1])
    global v_des_horizon_global;
    v_des = v_des_horizon_global;

    
    % Tracking cost: for each prediction step (starting at k = 2)
    for k = 2:Npred
        pos_error = X(k, 1:2) - refPos(k, :);
        vel_error = X(k, 4) - v_des(k);
        J = J + pos_error * Q_pos * pos_error' + (vel_error)^2 * Q_vel;
    end
    
    % Control effort cost
    for k = 1:size(U, 1)
        J = J + U(k, :) * R_abs * U(k, :)';
    end
    
    % Control rate cost (starting from second control input)
    U_prev = U(1, :);
    for k = 2:size(U, 1)
        dU = U(k, :) - U_prev;
        J = J + dU * R_rate * dU';
        U_prev = U(k, :);
    end
    
    % Terminal cost
    pos_terminal_error = X(end, 1:2) - refPos(end, :);
    vel_terminal_error = X(end, 4) - v_des(end);
    J = J + pos_terminal_error * Q_terminal * pos_terminal_error' + (vel_terminal_error)^2 * Q_terminalVel;
end