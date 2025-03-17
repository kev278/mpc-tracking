function J = costFunction(X, U, data)
    % Custom cost function for tracking only x and y

    % Q: Tracking penalty for x and y only
    Q = diag([100, 100]); 

    % R: Input change penalty remains unchanged
    R = diag([20, 10, 10]);

    % Terminal cost penalty for x and y only
    Q_terminal = diag([50, 50]);

    J = 0;
    Npred = size(X, 1);
    Nref  = size(data.References, 1);
    Ncommon = min(Npred, Nref);

    % 1) State tracking cost (only use x and y)
    for k = 2:Ncommon
        dx = X(k, 1:2) - data.References(k, :);
        J = J + dx * Q * dx';
    end

    % 2) Input increment cost (unchanged)
    for k = 2:size(U, 1)
        dU = U(k, :) - U(k-1, :);
        J = J + dU * R * dU';
    end

    % 3) Terminal cost for x and y
    dx_terminal = X(end, 1:2) - data.References(end, :);
    J = J + dx_terminal * Q_terminal * dx_terminal';
end
