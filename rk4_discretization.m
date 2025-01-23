function x_next = rk4_discretization(f, x, u, T_s)
    % Runge-Kutta 4th order discretization
    %
    % Inputs:
    %   f    - Continuous-time dynamics function handle: f(x, u)
    %   x    - Current state
    %   u    - Current control input
    %   T_s  - Sampling time
    %
    % Output:
    %   x_next - Next state after one time step
    
    k1 = f(x, u);
    k2 = f(x + 0.5 * T_s * k1, u);
    k3 = f(x + 0.5 * T_s * k2, u);
    k4 = f(x + k3  * T_s, u);
    
    x_next = x + (T_s / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
end
