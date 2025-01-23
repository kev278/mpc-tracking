function J = costFunction(X, U, data)
    % Custom cost function for nonlinear MPC
    %
    % Inputs:
    %   - X: (N_p+1)-by-nx predicted states
    %   - U: N_c-by-nu predicted inputs
    %   - data.References: (N_p+1)-by-ny reference outputs
    %
    % Output:
    %   - J: Total cost for the prediction horizon

    %% Define Q and R matrices
    % Q: State tracking penalty (position errors get higher weights)
    Q = diag([50, 50, 1, 1, 1, 1]);  % Higher weights on [p_cg_x, p_cg_y]

    % R: Input increment penalty (smaller values to allow faster control adjustments)
    R = diag([0.5, 0.5, 0.5]);

    % Terminal penalty weight (for the final predicted state)
    Q_terminal = diag([500, 500, 10, 10, 10, 10]);  % Very high penalty on final position

    %% Initialize total cost
    J = 0;

    %% Get the number of prediction steps and references
    Npred = size(X, 1);           % N_p + 1 predicted states
    Nref  = size(data.References, 1);
    Ncommon = min(Npred, Nref);   % Ensure we don't exceed array bounds

    %% 1) Sum of state tracking errors over the prediction horizon
    for k = 2 : Ncommon
        dx = X(k, :) - data.References(k, :);
        J = J + dx * Q * dx';  % Quadratic state error term
    end

    %% 2) Sum of input increment penalties
    for k = 2 : size(U, 1)
        dU = U(k, :) - U(k-1, :);
        J = J + dU * R * dU';  % Quadratic input change term
    end

    %% 3) Terminal state penalty (for the final predicted state)
    % Apply a high penalty to the final predicted state deviation
    dx_terminal = X(end, :) - data.References(end, :);
    J = J + dx_terminal * Q_terminal * dx_terminal';  % Quadratic terminal cost
end
