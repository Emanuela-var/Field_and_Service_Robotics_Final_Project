function [v_cmd, phi_cmd] = purePursuitController(current_state, trajectory_points, controller_params)
% Pure Pursuit Controller for Bicycle Model 
% 
% Implements the Pure Pursuit algorithm for trajectory following
% with accurate stopping at the final goal and zero initial/final speed.
%
% Input:
% current_state: [x, y, theta, v] - current vehicle state
% trajectory_points: [N x 4] - trajectory points [x, y, theta, v_desired]
% controller_params: controller parameters (optional)
%
% Output:
% v_cmd: speed command [m/s]
% phi_cmd: steering angle command [rad]


persistent params_initialized pp_params parking_complete
if isempty(params_initialized)
    pp_params = getEmbeddedPurePursuitParameters();
    params_initialized = true;
    parking_complete = false;
end


if nargin > 2 && ~isempty(controller_params)
    pp_params = mergeParameters(pp_params, controller_params);
end

%% ===================================================================
%% INPUT VALIDATION
%% ===================================================================

% Verifica che gli input abbiano le dimensioni corrette
assert(length(current_state) >= 3, 'current_state must contain at least [x, y, theta].');
assert(size(trajectory_points, 2) >= 3, 'trajectory_points dmust contain at least [x, y, theta]');
assert(size(trajectory_points, 1) >= 2, 'trajectory_points dmust contain at least 2 points');

%% ===================================================================
%% DATA EXTRACTION
%% ===================================================================

% Current state of the vehicle
x_current = current_state(1);
y_current = current_state(2);
theta_current = current_state(3);

% Current speed (if available, otherwise use reference speed)
if length(current_state) >= 4
    v_current = current_state(4);
else
    v_current = pp_params.v_default;
end

% Parameter extraction
L = pp_params.wheelbase;
lookahead_base = pp_params.lookahead_distance;
lookahead_time = pp_params.lookahead_time;
v_max = pp_params.v_max;
v_min = pp_params.v_min;
phi_max = pp_params.phi_max;
kp_longitudinal = pp_params.kp_longitudinal;

%% ===================================================================
%% CHECK IF PARKING IS ALREADY COMPLETED
%% ===================================================================

if parking_complete
    v_cmd = 0;
    phi_cmd = 0;
    return;
end

%% ===================================================================
%% STEP 1: FIND THE CLOSEST POINT ON THE TRAJECTORY
%% ===================================================================

% Calculates distances to all points on the trajectory
distances = sqrt((trajectory_points(:,1) - x_current).^2 + ...
                 (trajectory_points(:,2) - y_current).^2);

% Find the index of the nearest point
[min_distance, closest_index] = min(distances);

persistent debug_data
if isempty(debug_data)
    debug_data = struct();
end
debug_data.closest_distance = min_distance;
debug_data.closest_index = closest_index;

%% ===================================================================
%% CALCULATION OF DISTANCE AND STATUS IN RELATION TO THE GOAL
%% ===================================================================

% Final Goal 
final_waypoint = trajectory_points(end, 1:3);
final_x = final_waypoint(1);
final_y = final_waypoint(2);
final_theta = final_waypoint(3);

% Distance and orientation with respect to the goal
dx_goal = final_x - x_current;
dy_goal = final_y - y_current;
dist_to_goal = sqrt(dx_goal^2 + dy_goal^2);
angle_error = wrapToPi(final_theta - theta_current);

remaining_waypoints = size(trajectory_points, 1) - closest_index;
is_final_approach = dist_to_goal < pp_params.final_approach_distance || remaining_waypoints < 20;
is_alignment_phase = dist_to_goal < pp_params.alignment_distance || remaining_waypoints < 10;

%% ===================================================================
%% VERIFICATION OF COMPLETE PARKING CONDITIONS
%% ===================================================================

position_reached = dist_to_goal < pp_params.parking_distance_threshold;
angle_aligned = abs(angle_error) < pp_params.parking_angle_threshold;
velocity_stopped = abs(v_current) < pp_params.parking_velocity_threshold;
at_final_waypoints = closest_index >= size(trajectory_points, 1) - 5;

if position_reached && angle_aligned && velocity_stopped && at_final_waypoints
    parking_complete = true;
    v_cmd = 0;
    phi_cmd = 0;
    
    fprintf('PARKING COMPLETED! Distance: %.3f m, Angle: %.3f rad, Velocity: %.3f m/s\n', ...
            dist_to_goal, angle_error, v_current);
    return;
end

%% ===================================================================
%% STEP 2: CALCULATES ADAPTIVE LOOKAHEAD DISTANCE
%% ===================================================================

% lookahead distance adjusts to current speed
lookahead_distance = lookahead_base + abs(v_current) * lookahead_time;

lookahead_min = pp_params.lookahead_min;
lookahead_max = pp_params.lookahead_max;

% Change lookahead for final stages
if is_alignment_phase
    % Final alignment: much reduced lookahead
    lookahead_distance = max(0.5, min(1.0, dist_to_goal * 0.8));
elseif is_final_approach
    % Final approach: reduced lookahead
    lookahead_distance = max(1.0, min(2.0, dist_to_goal * 0.7));
else
    % Normal tracking
    lookahead_distance = max(lookahead_min, min(lookahead_max, lookahead_distance));
end

debug_data.lookahead_distance = lookahead_distance;

%% ===================================================================
%% STEP 3: FINDS THE TARGET POINT ON THE TRAJECTORY
%% ===================================================================

target_found = false;

% If we are very close to the goal, aim directly at it
if is_alignment_phase || dist_to_goal < 2.0
    target_index = size(trajectory_points, 1);
    target_x = final_x;
    target_y = final_y;
    target_found = true;
else
    % Normal target search
    for i = closest_index:size(trajectory_points, 1)
        dist_to_point = sqrt((trajectory_points(i,1) - x_current)^2 + ...
                             (trajectory_points(i,2) - y_current)^2);
        
        if dist_to_point >= lookahead_distance
            target_index = i;
            target_x = trajectory_points(i, 1);
            target_y = trajectory_points(i, 2);
            target_found = true;
            break;
        end
    end
    
    if ~target_found
        target_index = size(trajectory_points, 1);
        target_x = final_x;
        target_y = final_y;
    end
end

if size(trajectory_points, 2) >= 4
    v_desired_waypoint = trajectory_points(target_index, 4);
else
    v_desired_waypoint = pp_params.v_default;
end

debug_data.target_index = target_index;
debug_data.target_point = [target_x, target_y];

%% ===================================================================
%% STEP 4: CALCULATES THE STEERING ANGLE (CORE OF THE ALGORITHM)
%% ===================================================================

% Vector from vehicle to target point in the global co-ordinate system
dx = target_x - x_current;
dy = target_y - y_current;

% Effective distance to target
actual_distance = sqrt(dx^2 + dy^2);

% Transformation in the vehicle co-ordinate system
alpha = atan2(dy, dx) - theta_current;

% alpha in [-pi, pi]
alpha = wrapToPi(alpha);

% Steering angle calculation using Pure Pursuit geometry
if actual_distance > 0.05  
    % Adaptive gain for different phases
    if is_alignment_phase
        k_gain = 2.0;  % High gain for final convergence
    elseif is_final_approach
        k_gain = 1.5;
    else
        k_gain = 1.0;
    end
    
    phi_cmd = atan(k_gain * 2 * L * sin(alpha) / actual_distance);
else
    phi_cmd = 0;  % if too close to target, zero steering
end

% Apply steering limits
phi_cmd = max(-phi_max, min(phi_max, phi_cmd));

debug_data.alpha = alpha;
debug_data.steering_angle = phi_cmd;

%% ===================================================================
%% STEP 5: ADVANCED LONGITUDINAL CONTROL
%% ===================================================================

% Determines desired speed based on phase
if is_alignment_phase
    % Alignment phase: very low speed proportional to distance
    v_desired = min(pp_params.alignment_v_max, dist_to_goal * 0.3);
    
    if dist_to_goal < 0.5
        v_desired = max(0.05, dist_to_goal * 0.2);
    end
    
elseif is_final_approach
    % Final approach: reduced speed
    v_desired = min(pp_params.final_approach_v_max, dist_to_goal * 0.4);
    v_desired = max(0.1, v_desired);
    
else
    % Normal tracking
    v_desired = v_desired_waypoint;
    
    % Reduce speed in tight bends
    if abs(phi_cmd) > pi/6  
        v_desired = v_desired * 0.7;
    end
end

% Cproportional control with adaptive gain
if is_alignment_phase
    kp_adaptive = kp_longitudinal * 0.3;
elseif is_final_approach
    kp_adaptive = kp_longitudinal * 0.5;
else
    kp_adaptive = kp_longitudinal;
end

v_error = v_desired - v_current;
v_cmd = v_current + kp_adaptive * v_error;

% Apply speed limits
v_cmd = max(v_min, min(v_max, v_cmd));

% Safety override for final stages
if is_alignment_phase && dist_to_goal < 0.3
    v_cmd = min(v_cmd, 0.05);  
end

debug_data.v_desired = v_desired;
debug_data.v_error = v_error;
debug_data.v_cmd = v_cmd;

%% ===================================================================
%% SMOOTHING 
%% ===================================================================

% Low-pass filter to avoid oscillations in controls
if pp_params.enable_smoothing && ~is_alignment_phase
    persistent v_cmd_prev phi_cmd_prev
    if isempty(v_cmd_prev)
        v_cmd_prev = v_cmd;
        phi_cmd_prev = phi_cmd;
    end
    
    % Adaptive Smoothing 
    if is_final_approach
        alpha_smooth = 0.85;
    else
        alpha_smooth = pp_params.smoothing_factor;
    end
    
    v_cmd = alpha_smooth * v_cmd + (1 - alpha_smooth) * v_cmd_prev;
    phi_cmd = alpha_smooth * phi_cmd + (1 - alpha_smooth) * phi_cmd_prev;
    
    v_cmd_prev = v_cmd;
    phi_cmd_prev = phi_cmd;
end

%% ===================================================================
%% STANDING START MANAGEMENT
%% ===================================================================

% If we are at the beginning, ensure a gradual start
if closest_index <= 5 && v_current < 0.1
    v_cmd = min(v_cmd, 0.1 * closest_index);
end

%% ===================================================================
%% DEBUG
%% ===================================================================

if pp_params.enable_debug
    debug_data.timestamp = now;
    debug_data.current_state = current_state;
    debug_data.final_commands = [v_cmd, phi_cmd];
    debug_data.dist_to_goal = dist_to_goal;
    debug_data.is_final_approach = is_final_approach;
    debug_data.is_alignment_phase = is_alignment_phase;
    debug_data.parking_complete = parking_complete;
    
    
    assignin('base', 'pp_debug_data', debug_data);
end

end


function params = getEmbeddedPurePursuitParameters()
    
    params = struct();
    
 
    params.wheelbase = 2.8;              % Wheelbase [m]
    params.v_default = 1.0;              
    
   
    params.lookahead_distance = 4.0;     
    params.lookahead_time = 1.5;         
    params.lookahead_min = 2.0;          
    params.lookahead_max = 15.0;         
    
    
    params.v_max = 3.0;                 
    params.v_min = 0.0;                  
    params.phi_max = pi/4;               
    
   
    params.kp_longitudinal = 0.8;        
    
    
    params.enable_smoothing = true;     
    params.smoothing_factor = 0.7;       
    
    
    params.parking_distance_threshold = 0.15;    
    params.parking_angle_threshold = 0.05;       
    params.parking_velocity_threshold = 0.05;    
    params.final_approach_distance = 3.0;       
    params.final_approach_v_max = 0.3;           
    params.alignment_distance = 1.0;             
    params.alignment_v_max = 0.1;                
    
    
    params.enable_debug = false;         
end

function merged_params = mergeParameters(base_params, override_params)
    
    merged_params = base_params;
    
    
    fields = fieldnames(override_params);
    
    for i = 1:length(fields)
        field = fields{i};
        if isfield(base_params, field)
            merged_params.(field) = override_params.(field);
        end
    end
end

function angle_wrapped = wrapToPi(angle)
    
    angle_wrapped = atan2(sin(angle), cos(angle));
end