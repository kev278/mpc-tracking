% --------------------------------------------------------
% This script demonstrates how to compute the state
% derivative x_dot for a half-car model given:
%   - A state vector x = [p_cg_x; p_cg_y; psi; v_x; v_y; psi_dot]
%   - Control inputs u = [delta; s_fx; s_rx]
%   - Vehicle parameters (lF, lR, m, Izz, Bf, Cf, Df, muF, etc.)
% --------------------------------------------------------

%% --------------------------------------------------------------
% Helper function: halfCarDynamics
% --------------------------------------------------------------
function xdot = halfCarDynamics(x, u, ...
                                lF, lR, m, Izz, ...
                                muF, ...
                                muR, ...
                                h, g)
    % Unpack state
    p_cg_x  = x(1);
    p_cg_y  = x(2);
    psi     = x(3);
    v_x     = x(4);
    v_y     = x(5);
    psi_dot = x(6); % numeric

    % Unpack inputs
    delta = u(1);  % steering angle (front wheels)
    s_fx  = u(2);  % front longitudinal slip
    s_rx  = u(3);  % rear  longitudinal slip

    % ---------------------------------------------------------
    % 1) Lateral slip "ratios" for front and rear
    %    (Sometimes expressed as slip angles; here, a ratio.)
    %    s_fy = (v_y + lF * psi_dot) / v_x
    %    s_ry = (v_y - lR * psi_dot) / v_x

    %  If v_x is very close to zero, then:
    %  1. Dividing by a near-zero number would blow up the expression (leading
    %  to extremely large or infinite slip values).
    %  2. Slip ratio (or slip angle) doesnt make sense when the vehicle is 
    %  essentially at rest or not moving forward.
    % ---------------------------------------------------------
    if abs(v_x) < 1e-3
        s_fy = 0;
        s_ry = 0;
    else
        %s_fy = ((v_y + lF * psi_dot) * cos(delta) - v_x * sin(delta)) / (v_x * cos(delta) + (v_y + lF * psi_dot) * sin(delta));
        s_fy = ((v_y + lF * psi_dot)*cos(delta) - v_x*sin(delta)) / ...
       (v_x*cos(delta) + (v_y + lF * psi_dot)*sin(delta));

        s_ry = (v_y - lR * psi_dot) / v_x;
    end

    % ---------------------------------------------------------
    % 2) Total slips at front/rear (magnitude in tire coordinates)
    %    s_f = sqrt( s_fx^2 + s_fy^2 )
    %    s_r = sqrt( s_rx^2 + s_ry^2 )
    % ---------------------------------------------------------
    s_f = sqrt(s_fx^2 + s_fy^2);
    s_r = sqrt(s_rx^2 + s_ry^2);

    % ---------------------------------------------------------
    % 3) (Simplified) Pacejka-like formula for friction "mu"
    %    The sign of Df, Dr in your data is negative, so be consistent.
    %
    %    mu_front_raw = Df * sin( Cf * atan( Bf * s_f ) )
    %    mu_rear_raw  = Dr * sin( Cr * atan( Br * s_r ) )
    %
    %    Then, clamp to +/- muF or muR if desired. (Optional)
    % ---------------------------------------------------------
    % mu_front_raw = Df * sin( Cf * atan( Bf * s_f ) );
    % mu_rear_raw  = Dr * sin( Cr * atan( Br * s_r ) );

    % Optionally clamp to the absolute friction limit:
    % (If your model or thesis says so; otherwise skip.)
    % mu_front = max(-muF, min(muF, mu_front_raw));
    % mu_rear  = max(-muR, min(muR, mu_rear_raw));

    % ---------------------------------------------------------
    % 4) Direction of friction in tire coords
    %    F_{fx} = mu_fx * F_fz  with mu_fx = -(s_fx/s_f)*mu_front
    %    F_{fy} = mu_fy * F_fz  with mu_fy = -(s_fy/s_f)*mu_front
    %    (Minus sign depends on sign conventions; adjust if needed.)
    % ---------------------------------------------------------
    if s_f < 1e-6
        mu_fx = 0;
        mu_fy = 0;
    else
        mu_fx = (s_fx / s_f) * muF;
        mu_fy = (s_fy / s_f) * muF;
    end

    if s_r < 1e-6
        mu_rx = 0;
        mu_ry = 0;
    else
        mu_rx = (s_rx / s_r) * muR;
        mu_ry = (s_ry / s_r) * muR;
    end

    % ---------------------------------------------------------
    % 5) Compute normal loads (very simple version)
    %    For a more accurate load-transfer formula, see your eqn. (6)-(7).
    %    Here, we ignore dynamic load transfer:
    %       F_fz = m*g * (lR / (lF + lR))
    %       F_rz = m*g * (lF / (lF + lR))
    % ---------------------------------------------------------
    F_fz = m * g * (lR - mu_rx * h) / (lF + lR + h * (mu_fx * cos(delta) - ...
        mu_fy * sin(delta) - mu_rx));
    F_rz = m * g - F_fz;

    %F_fz = m*g * (lR / (lF + lR));  
    %F_rz = m*g * (lF / (lF + lR));


    % ---------------------------------------------------------
    % 6) Compute friction forces in tire coordinates
    % ---------------------------------------------------------
    F_fx = mu_fx * F_fz;  % front friction, x-direction (tire coords)
    F_fy = mu_fy * F_fz;  % front friction, y-direction (tire coords)
    F_rx = mu_rx * F_rz;  % rear friction, x-direction
    F_ry = mu_ry * F_rz;  % rear friction, y-direction

    % ---------------------------------------------------------
    % 7) Plug into the vehicle-level equations of motion:
    %    (1) m * v_x_dot = [F_fx cos(delta) - F_fy sin(delta) + F_rx] + m v_y psi_dot
    %    (2) m * v_y_dot = [F_fx sin(delta) + F_fy cos(delta) + F_ry] - m v_x psi_dot
    %    (3) Izz * psi_ddot = lF * [F_fx sin(delta) + F_fy cos(delta)] - lR * F_ry
    % ---------------------------------------------------------
    v_x_dot  = (1/m) * ( F_fx*cos(delta) - F_fy*sin(delta) + F_rx ) + psi_dot*v_y;
    v_y_dot  = (1/m) * ( F_fx*sin(delta) + F_fy*cos(delta) + F_ry ) - psi_dot*v_x;
    psi_ddot = (1/Izz)*( lF*(F_fx*sin(delta) + F_fy*cos(delta)) - lR*F_ry );

    % ---------------------------------------------------------
    % 8) Kinematic relations for p_cg_x_dot, p_cg_y_dot

    %    From: Kinematic and Dynamic Vehicle Models for Autonomous Driving 
    %    Control Design, J. Kong
    % ---------------------------------------------------------
    p_cg_x_dot = v_x*cos(psi) - v_y*sin(psi);
    p_cg_y_dot = v_x*sin(psi) + v_y*cos(psi);

    % ---------------------------------------------------------
    % 9) Assemble xdot
    %    x = [p_cg_x; p_cg_y; psi; v_x; v_y; psi_dot]
    %    xdot = [p_cg_x_dot; p_cg_y_dot; psi_dot; v_x_dot; v_y_dot; psi_ddot]
    % ---------------------------------------------------------
    xdot = zeros(6,1);
    xdot(1) = p_cg_x_dot;
    xdot(2) = p_cg_y_dot;
    xdot(3) = psi_dot;
    xdot(4) = v_x_dot;
    xdot(5) = v_y_dot;
    xdot(6) = psi_ddot;

end