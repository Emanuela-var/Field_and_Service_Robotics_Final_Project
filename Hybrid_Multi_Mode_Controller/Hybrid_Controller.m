function [v_cmd, phi_cmd] = Hybrid_Controller(current_state, desired_trajectory, controller_params)

%% Input validation
if length(current_state) < 3
    error('current_state must contain [x, y, theta]');
end

if length(desired_trajectory) < 7
    error('desired_trajectory must contain [xd, yd, thetad, phid, xd_dot, yd_dot, thetad_dot]');
end

%% Extract states and references
x = current_state(1);
y = current_state(2);
theta = wrapToPi(current_state(3));

xd = desired_trajectory(1);
yd = desired_trajectory(2);
thetad = wrapToPi(desired_trajectory(3));
phid = desired_trajectory(4);
xd_dot = desired_trajectory(5);
yd_dot = desired_trajectory(6);
thetad_dot = desired_trajectory(7);

%% Controller parameters
L = getFieldOrDefault(controller_params, 'wheelbase', 2.8);
k1 = getFieldOrDefault(controller_params, 'k1', 2.0);      
k2 = getFieldOrDefault(controller_params, 'k2', 4.0);     
k3 = getFieldOrDefault(controller_params, 'k3', 1.5);      
v_max = getFieldOrDefault(controller_params, 'v_max', 2.0);
v_min = getFieldOrDefault(controller_params, 'v_min', 0.0);
phi_max = getFieldOrDefault(controller_params, 'phi_max', pi/4);

%% Persistent variables for filtering
persistent v_prev phi_prev init_flag
if isempty(init_flag)
    v_prev = 0;
    phi_prev = 0;
    init_flag = true;
end

%% Error calculation
% Position errors
ex = x - xd;
ey = y - yd;
position_error = sqrt(ex^2 + ey^2);

% Heading error (wrapped to [-pi, pi])
etheta = wrapToPi(theta - thetad);

% Reference velocity
v_ref = sqrt(xd_dot^2 + yd_dot^2);

%% Transform to path-aligned frame
% Longitudinal error (along path direction)
e_long = -ex * cos(thetad) - ey * sin(thetad);

% Lateral error (perpendicular to path)
e_lat = ex * sin(thetad) - ey * cos(thetad);

%% Determine control mode
parking_complete = (position_error < 0.3) && (v_ref < 0.01);
approaching_target = (position_error < 2.0) && (v_ref < 0.5);

%% Velocity control
if parking_complete
    % Parking complete: full stop
    v_cmd = 0;
    
elseif v_ref < 0.01 && position_error > 0.5
    % Reference stopped but not at target: use position control
    v_cmd = k1 * min(position_error, 1.0);
    
else
    % Normal tracking mode
    % Base velocity from feedforward + longitudinal error feedback
    v_base = v_ref + k1 * e_long;
    
    % Slow down for large lateral errors
    lateral_factor = 1.0 / (1.0 + abs(e_lat));
    
    % Slow down for large heading errors
    heading_factor = 1.0 / (1.0 + 2.0 * abs(etheta));
    
    % Combined velocity command
    v_cmd = v_base * lateral_factor * heading_factor;
    
    % Special handling for final approach
    if approaching_target
        approach_factor = max(0.1, position_error / 2.0);
        v_cmd = v_cmd * approach_factor;
    end
end

%% Steering control
if parking_complete || abs(v_cmd) < 0.05
    % When stopped or nearly stopped: only correct heading
    phi_cmd = -k2 * 0.3 * etheta;
    
else
   
    % Feedforward from reference trajectory
    if abs(v_cmd) > 0.1
        phi_feedforward = atan(L * thetad_dot / v_cmd);
    else
        phi_feedforward = 0;
    end
    
    % Lateral error feedback (Stanley controller component)
    k_stanley = k3 * (1 + abs(thetad_dot));  % Adaptive gain
    phi_lateral = atan2(k_stanley * e_lat, max(0.5, abs(v_cmd)));
    
    % Heading error feedback
    phi_heading = -k2 * etheta;
    
    % Combined steering command
    phi_cmd = phi_feedforward + phi_lateral + phi_heading;
    
    % Reduce aggressiveness when approaching target
    if approaching_target
        phi_cmd = phi_cmd * 0.7;
    end
end

%% Saturation
v_cmd = max(v_min, min(v_max, v_cmd));
phi_cmd = max(-phi_max, min(phi_max, phi_cmd));

%% Rate limiting for smooth control
dt = 0.1;
max_accel = 1.5;      % m/s^2
max_phi_rate = 0.8;   % rad/s

% Velocity rate limit
v_rate = (v_cmd - v_prev) / dt;
if abs(v_rate) > max_accel
    v_cmd = v_prev + sign(v_rate) * max_accel * dt;
end

% Steering rate limit
phi_rate = (phi_cmd - phi_prev) / dt;
if abs(phi_rate) > max_phi_rate
    phi_cmd = phi_prev + sign(phi_rate) * max_phi_rate * dt;
end

%% Low-pass filtering for smoothness
alpha_v = 0.7;    % Velocity filter coefficient
alpha_phi = 0.8;  % Steering filter coefficient

v_cmd = alpha_v * v_cmd + (1 - alpha_v) * v_prev;
phi_cmd = alpha_phi * phi_cmd + (1 - alpha_phi) * phi_prev;

%% Update previous values
v_prev = v_cmd;
phi_prev = phi_cmd;

%% Final safety checks
% Ensure complete stop when parking is complete
if parking_complete
    v_cmd = 0;
    v_prev = 0;
    if abs(etheta) < 0.05  % ~3 degrees
        phi_cmd = 0;
        phi_prev = 0;
    end
end

end

function value = getFieldOrDefault(struct, field, default)
    if isfield(struct, field)
        value = struct.(field);
    else
        value = default;
    end
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end