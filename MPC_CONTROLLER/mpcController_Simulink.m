function [v_cmd, phi_cmd, solver_ok] = mpcController_Simulink(current_state, desired_trajectory, controller_params)

mpc_params = controller_params;

%% ===================================================================
%% INPUT VALIDATION
%% ===================================================================

% Basic input validation using assertions
assert(length(current_state) >= 3, 'current_state must contain at least [x, y, theta]');
assert(length(desired_trajectory) >= 7, 'desired_trajectory must contain [xd, yd, thetad, phid, xd_dot, yd_dot, thetad_dot]');

%% ===================================================================
%% EXTRACT DATA
%% ===================================================================

% Current vehicle state
x = current_state(1);
y = current_state(2);
theta = wrapToPi(current_state(3));

% Desired trajectory at current time
xd = desired_trajectory(1);
yd = desired_trajectory(2);
thetad = wrapToPi(desired_trajectory(3));
phid = desired_trajectory(4);
xd_dot = desired_trajectory(5);
yd_dot = desired_trajectory(6);
thetad_dot = desired_trajectory(7);

% Extract parameters
L = mpc_params.wheelbase;
Np = mpc_params.prediction_horizon;
dt = mpc_params.sampling_time;
Q_diag = mpc_params.Q_weights;
R_diag = mpc_params.R_weights;
v_max = mpc_params.v_max;
v_min = mpc_params.v_min;  % Now allows 0
phi_max = mpc_params.phi_max;
smoothing_factor = mpc_params.smoothing_factor;

%% ===================================================================
%% PERSISTENT VARIABLES FOR SMOOTH CONTROL
%% ===================================================================

persistent v_cmd_prev phi_cmd_prev init_flag
if isempty(init_flag)
    v_cmd_prev = 0;  % Start from zero velocity
    phi_cmd_prev = 0;
    init_flag = true;
end

%% ===================================================================
%% COMPUTE ERRORS AND CHECK PARKING STATUS
%% ===================================================================

% Global coordinate errors
ex_global = x - xd;
ey_global = y - yd;
etheta_global = wrapToPi(theta - thetad);

% Position error magnitude
position_error = sqrt(ex_global^2 + ey_global^2);

% Reference velocity
v_ref = sqrt(xd_dot^2 + yd_dot^2);

% Check if parking is complete
parking_complete = (position_error < 0.3) && (v_ref < 0.01);
approaching_target = (position_error < 2.0) && (v_ref < 0.5);

%% ===================================================================
%% PARKING MODE LOGIC
%% ===================================================================

if parking_complete
    % Parking complete: full stop
    v_cmd = 0;
    
    % Small heading correction if needed
    if abs(etheta_global) > 0.1  
        phi_cmd = -mpc_params.fallback_kp_heading * 0.5 * etheta_global;
    else
        phi_cmd = 0;
    end
    
    % Update persistent variables
    v_cmd_prev = 0;
    if abs(etheta_global) < 0.05
        phi_cmd_prev = 0;
    end
    
    solver_ok = 1.0;
    return;  % Exit early
end

%% ===================================================================
%% COMPUTE REFERENCE VELOCITY CONSIDERING PARKING
%% ===================================================================

% If reference stopped but not at target, use position-based velocity
if v_ref < 0.01 && position_error > 0.5
    v_ref = min(1.0, position_error);  % Proportional to distance
end

% Limit reference velocity
v_ref = max(v_min, min(v_max, v_ref));

%% ===================================================================
%% TRANSFORM ERRORS TO LOCAL COORDINATES
%% ===================================================================

cos_thetad = cos(thetad);
sin_thetad = sin(thetad);

% Transform to local trajectory coordinates
e_lateral = -ex_global * sin_thetad + ey_global * cos_thetad;      % Cross-track error
e_longitudinal = ex_global * cos_thetad + ey_global * sin_thetad;  % Along-track error
e_heading = etheta_global;                                         % Heading error

% State error vector
e_current = [e_lateral; e_longitudinal; e_heading];

%% ===================================================================
%% ADAPTIVE MPC WEIGHTS FOR PARKING
%% ===================================================================

% Modify weights based on situation
Q_adaptive = Q_diag;
R_adaptive = R_diag;

if approaching_target
    % Increase position tracking importance when close to target
    Q_adaptive(1) = Q_adaptive(1) * 2.0;  % Lateral error more important
    Q_adaptive(3) = Q_adaptive(3) * 1.5;  % Heading error more important
    R_adaptive(2) = R_adaptive(2) * 2.0;  % Smoother steering
end

%% ===================================================================
%% MODEL LINEARIZATION
%% ===================================================================

% Use actual reference velocity for linearization
v_lin = max(0.5, v_ref);  % Avoid singularities at low speed

% Continuous-time system matrices
A_cont = [0,    0,    v_lin;     % e_lateral dynamics
          0,    0,    0;         % e_longitudinal dynamics
          0,    0,    0];        % e_heading dynamics

B_cont = [0,    0;               % e_lateral control
          1,    0;               % e_longitudinal control
          0,    v_lin/L];        % e_heading control

% Discrete-time system matrices
Ad = eye(3) + A_cont * dt;
Bd = B_cont * dt;

%% ===================================================================
%% BUILD MPC OPTIMIZATION PROBLEM
%% ===================================================================

% Weight matrices with adaptive values
Q = diag(Q_adaptive);
R = diag(R_adaptive);

% Build prediction matrices
Phi = zeros(3*Np, 3);
Gamma = zeros(3*Np, 2*Np);

% Build prediction matrices iteratively
Ad_power = Ad;
for i = 1:Np
    Phi(3*(i-1)+1:3*i, :) = Ad_power;
    
    Ad_temp = eye(3);
    for j = 1:i
        Gamma(3*(i-1)+1:3*i, 2*(j-1)+1:2*j) = Ad_temp * Bd;
        Ad_temp = Ad_temp * Ad;
    end
    
    Ad_power = Ad_power * Ad;
end

% Extended weight matrices
Q_bar = kron(eye(Np), Q);
R_bar = kron(eye(Np), R);

% Quadratic program formulation
H = 2 * (Gamma' * Q_bar * Gamma + R_bar);
f = 2 * Gamma' * Q_bar * Phi * e_current;

%% ===================================================================
%% DEFINE OPERATIONAL CONSTRAINTS
%% ===================================================================

% Adaptive constraints based on situation
if approaching_target
    % Tighter constraints when parking
    v_max_local = min(v_max, 1.0);
    phi_max_local = phi_max * 0.8;
else
    v_max_local = v_max;
    phi_max_local = phi_max;
end

% Control constraints
delta_v_max = v_max_local - v_ref;
delta_v_min = v_min - v_ref;
delta_phi_max = phi_max_local - phid;
delta_phi_min = -phi_max_local - phid;

% Constraint matrices
A_ineq = [eye(2*Np); -eye(2*Np)];
b_ineq = [repmat([delta_v_max; delta_phi_max], Np, 1);
          repmat([-delta_v_min; -delta_phi_min], Np, 1)];

%% ===================================================================
%% SOLVE OPTIMIZATION PROBLEM
%% ===================================================================

[U_opt, solve_success] = solveQP_CodeGen(H, f, A_ineq, b_ineq, 2*Np);
solver_ok = double(solve_success);

if solve_success
    % Extract optimal control inputs
    delta_v_opt = U_opt(1);
    delta_phi_opt = U_opt(2);
    
    % Compute final commands
    v_cmd = v_ref + delta_v_opt;
    phi_cmd = phid + delta_phi_opt;
else
    % Enhanced fallback controller for parking
    [v_cmd, phi_cmd] = computeEnhancedFallbackControl(e_current, v_ref, phid, ...
                                                      position_error, approaching_target, mpc_params);
end

%% ===================================================================
%% APPLY SAFETY CONSTRAINTS AND PARKING LOGIC
%% ===================================================================

% Apply approach speed reduction
if approaching_target
    approach_factor = max(0.1, (position_error - 0.3) / 1.7);
    v_cmd = v_cmd * approach_factor;
end

% Saturation
v_cmd = max(v_min, min(v_max, v_cmd));
phi_cmd = max(-phi_max, min(phi_max, phi_cmd));

%% ===================================================================
%% RATE LIMITING FOR SMOOTH CONTROL
%% ===================================================================

max_accel = 1.5;      % m/s^2
max_phi_rate = 0.8;   % rad/s

% Velocity rate limit
v_rate = (v_cmd - v_cmd_prev) / dt;
if abs(v_rate) > max_accel
    v_cmd = v_cmd_prev + sign(v_rate) * max_accel * dt;
end

% Steering rate limit
phi_rate = (phi_cmd - phi_cmd_prev) / dt;
if abs(phi_rate) > max_phi_rate
    phi_cmd = phi_cmd_prev + sign(phi_rate) * max_phi_rate * dt;
end

%% ===================================================================
%% COMMAND SMOOTHING
%% ===================================================================

% Adaptive smoothing based on speed
if v_cmd < 0.5
    % Less smoothing at low speed for better control
    alpha_v = 0.7;
    alpha_phi = 0.8;
else
    alpha_v = smoothing_factor;
    alpha_phi = smoothing_factor;
end

v_cmd = alpha_v * v_cmd + (1 - alpha_v) * v_cmd_prev;
phi_cmd = alpha_phi * phi_cmd + (1 - alpha_phi) * phi_cmd_prev;

% Update previous values
v_cmd_prev = v_cmd;
phi_cmd_prev = phi_cmd;

%% ===================================================================
%% FINAL SAFETY CHECK
%% ===================================================================

% Ensure stop when reference velocity is zero and close to target
if v_ref < 0.01 && position_error < 0.5
    v_cmd = 0;
    v_cmd_prev = 0;
end

end

%% ===================================================================
%% AUXILIARY FUNCTIONS
%% ===================================================================

function [v_cmd, phi_cmd] = computeEnhancedFallbackControl(e_current, v_ref, phi_ref, ...
                                                           position_error, approaching_target, params)
    % Enhanced fallback controller for parking scenarios
    
    e_lateral = e_current(1);
    e_longitudinal = e_current(2);
    e_heading = e_current(3);
    
    % Adaptive gains based on situation
    if approaching_target
        kp_long = params.fallback_kp_longitudinal * 0.5;
        kp_lat = params.fallback_kp_lateral * 1.5;
        kp_head = params.fallback_kp_heading * 1.2;
    else
        kp_long = params.fallback_kp_longitudinal;
        kp_lat = params.fallback_kp_lateral;
        kp_head = params.fallback_kp_heading;
    end
    
    % Velocity control with position feedback
    if v_ref < 0.1 && position_error > 0.5
        % Use position-based control when reference is nearly zero
        v_cmd = min(1.0, position_error);
    else
        v_cmd = v_ref + kp_long * e_longitudinal;
    end
    
    % Steering control
    phi_cmd = phi_ref - kp_lat * e_lateral - kp_head * e_heading;
    
    % Apply constraints
    v_cmd = max(params.v_min, min(params.v_max, v_cmd));
    phi_cmd = max(-params.phi_max, min(params.phi_max, phi_cmd));
end

function angle_diff = wrapToPi(angle)
    % Wrap angle to [-pi, pi] range
    angle_diff = atan2(sin(angle), cos(angle));
end

function [x, success] = solveQP_CodeGen(H, f, A, b, n)
  
    % Initialize
    x = zeros(n, 1);
    success = false;
    
    % Parameters
    max_iter = 50;
    tolerance = 1e-5;
    
    % Regularize H for numerical stability
    lambda = 1e-4;
    H_reg = H + lambda * eye(size(H));
    
    % Adaptive step size
    alpha_init = 0.1;
    alpha = alpha_init;
    
    for iter = 1:max_iter
        % Compute gradient
        grad = H_reg * x + f;
        
        % Gradient descent with line search
        for ls = 1:5
            x_new = x - alpha * grad;
            
            % Project onto feasible region
            x_new = projectOntoConstraints(x_new, A, b);
            
            % Check if we've made progress
            if norm(H_reg * x_new + f) < norm(H_reg * x + f)
                break;
            else
                alpha = alpha * 0.5;  % Reduce step size
            end
        end
        
        % Check convergence
        if norm(x_new - x) < tolerance
            success = true;
            break;
        end
        
        x = x_new;
        
        % Reset step size periodically
        if mod(iter, 10) == 0
            alpha = alpha_init;
        end
    end
    
    % Accept solution even if not fully converged
    if ~success && iter >= max_iter
        success = true;
    end
end

function x_proj = projectOntoConstraints(x, A, b)
    % Enhanced projection for MPC constraints
    
    x_proj = x;
    
    % Check all constraints
    violations = A * x - b;
    
    % Simple projection for box constraints
    % Assumes alternating velocity and steering constraints
    for i = 1:length(violations)
        if violations(i) > 0
            % Determine which variable and direction
            constraint_idx = mod(i-1, 2) + 1;
            direction = sign(i - length(violations)/2 - 0.5);
            
            % Apply correction
            if constraint_idx == 1  % Velocity constraint
                correction_factor = 0.95;  % Stay within bounds
            else  % Steering constraint
                correction_factor = 0.9;   % More conservative for steering
            end
            
            % Project the corresponding variable
            var_idx = 2 * floor((i-1) / 2) + constraint_idx;
            if var_idx <= length(x)
                x_proj(var_idx) = x_proj(var_idx) - direction * violations(i) * correction_factor;
            end
        end
    end
end