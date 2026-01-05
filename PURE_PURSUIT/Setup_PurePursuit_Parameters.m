function [v_cmd, phi_cmd, debug_info] = purePursuitSimulinkFunction(current_state, trajectory_matrix, enable_debug)

v_cmd = 0.0;
phi_cmd = 0.0;
debug_info = zeros(1, 8);


if isempty(current_state) || length(current_state) < 3
    if enable_debug
        debug_info(1) = -1; 
    end
    return;
end

if isempty(trajectory_matrix) || size(trajectory_matrix, 1) < 2 || size(trajectory_matrix, 2) < 3
    if enable_debug
        debug_info(1) = -2; 
    end
    return;
end

%% ===================================================================
%% CONTROLLER PARAMETERS
%% ===================================================================

persistent pp_params parking_state
persistent v_cmd_prev phi_cmd_prev first_run

if isempty(pp_params)
    pp_params = struct();
    
 
    pp_params.wheelbase = 2.8;
    pp_params.v_default = 1.0;
    
   
    pp_params.lookahead_distance = 4.0;
    pp_params.lookahead_time = 1.5;
    pp_params.lookahead_min = 2.0;
    pp_params.lookahead_max = 15.0;
    
    
    pp_params.v_max = 3.0;
    pp_params.v_min = 0.0;
    pp_params.phi_max = pi/4;
    
    
    pp_params.kp_longitudinal = 0.8;
    
    
    pp_params.enable_smoothing = true;
    pp_params.smoothing_factor = 0.7;
    
    
    pp_params.parking_threshold = 0.2;
    pp_params.approach_threshold = 3.0;
    pp_params.align_threshold = 0.5;
end

if isempty(parking_state)
    parking_state = 0;
end

%% ===================================================================

%% ===================================================================

x_current = current_state(1);
y_current = current_state(2);
theta_current = current_state(3);

if length(current_state) >= 4
    v_current = current_state(4);
else
    v_current = pp_params.v_default;
end

%% ===================================================================

%% ===================================================================


n_points = size(trajectory_matrix, 1);
final_x = trajectory_matrix(n_points, 1);
final_y = trajectory_matrix(n_points, 2);


dx_goal = final_x - x_current;
dy_goal = final_y - y_current;
dist_to_goal = sqrt(dx_goal*dx_goal + dy_goal*dy_goal);


distances = zeros(n_points, 1);
for i = 1:n_points
    dx = trajectory_matrix(i, 1) - x_current;
    dy = trajectory_matrix(i, 2) - y_current;
    distances(i) = sqrt(dx*dx + dy*dy);
end
[min_distance, closest_index] = min(distances);


remaining_waypoints = n_points - closest_index;


if parking_state == 0  % TRACKING
    if dist_to_goal < pp_params.approach_threshold || remaining_waypoints < 10
        parking_state = 1;  % APPROACHING
    end
elseif parking_state == 1  % APPROACHING
    if dist_to_goal < pp_params.align_threshold
        parking_state = 2;  % ALIGNING
    end
elseif parking_state == 2  % ALIGNING
    if dist_to_goal < pp_params.parking_threshold
        parking_state = 3;  % PARKED
    end
end


if parking_state == 3  % PARKED
    v_cmd = 0.0;
    phi_cmd = 0.0;
    if enable_debug
        debug_info(1) = dist_to_goal;
        debug_info(2) = 0;
        debug_info(3) = 0;
        debug_info(4) = 0;
        debug_info(5) = 0;
        debug_info(6) = n_points;
        debug_info(7) = 0;
        debug_info(8) = 0;
    end
    return;
end

%% ===================================================================


%% ===================================================================

% Lookahead base
lookahead_distance = pp_params.lookahead_distance + abs(v_current) * pp_params.lookahead_time;


if parking_state == 0  % TRACKING
    lookahead_distance = max(pp_params.lookahead_min, min(pp_params.lookahead_max, lookahead_distance));
elseif parking_state == 1  % APPROACHING
    lookahead_distance = max(0.8, min(2.0, dist_to_goal * 0.7));
elseif parking_state == 2  % ALIGNING
    lookahead_distance = max(0.5, dist_to_goal);
end

%% ===================================================================

%% ===================================================================

% In ALIGNING, POINTS TO goal
if parking_state == 2  % ALIGNING
    target_index = n_points;
    target_x = final_x;
    target_y = final_y;
else
    
    target_index = closest_index;
    target_found = false;
    
    for i = closest_index:n_points
        dx = trajectory_matrix(i, 1) - x_current;
        dy = trajectory_matrix(i, 2) - y_current;
        dist_to_point = sqrt(dx*dx + dy*dy);
        
        if dist_to_point >= lookahead_distance
            target_index = i;
            target_found = true;
            break;
        end
    end
    
   
    if ~target_found || (parking_state == 1 && dist_to_goal < 1.5)
        target_index = n_points;
        target_x = final_x;
        target_y = final_y;
    else
        target_x = trajectory_matrix(target_index, 1);
        target_y = trajectory_matrix(target_index, 2);
    end
end


if size(trajectory_matrix, 2) >= 4
    v_desired = trajectory_matrix(target_index, 4);
else
    v_desired = pp_params.v_default;
end

%% ===================================================================

%% ===================================================================

dx = target_x - x_current;
dy = target_y - y_current;
actual_distance = sqrt(dx*dx + dy*dy);


alpha = atan2(dy, dx) - theta_current;
alpha = wrapToPi(alpha);


if actual_distance > 0.05
    if parking_state == 2  % ALIGNING
        k_pp = 2.5;  
    else
        k_pp = 1.0;
    end
    
    phi_cmd = atan(k_pp * 2 * pp_params.wheelbase * sin(alpha) / actual_distance);
else
    phi_cmd = 0.0;
end


phi_cmd = max(-pp_params.phi_max, min(pp_params.phi_max, phi_cmd));

%% ===================================================================

%% ===================================================================

if parking_state == 0  % TRACKING
    % Controllo normale
    v_error = v_desired - v_current;
    v_cmd = v_current + pp_params.kp_longitudinal * v_error;
    
elseif parking_state == 1  % APPROACHING
    % Velocità decrescente
    v_target = min(0.8, dist_to_goal * 0.4);
    v_target = max(0.2, v_target);
    v_error = v_target - v_current;
    v_cmd = v_current + pp_params.kp_longitudinal * 0.5 * v_error;
    
elseif parking_state == 2  % ALIGNING
    
    if dist_to_goal > 0.3
        v_cmd = 0.15;
    else
        v_cmd = max(0.05, dist_to_goal * 0.5);
    end
end


v_cmd = max(pp_params.v_min, min(pp_params.v_max, v_cmd));


if dist_to_goal < 0.3 && parking_state == 2
    v_cmd = min(v_cmd, 0.1);
end

%% ===================================================================


%% ===================================================================

if pp_params.enable_smoothing && parking_state < 3
    if isempty(first_run)
        v_cmd_prev = v_cmd;
        phi_cmd_prev = phi_cmd;
        first_run = false;
    end
    
    
    if parking_state == 0
        alpha_smooth = pp_params.smoothing_factor;
    elseif parking_state == 1
        alpha_smooth = 0.8;
    else  % ALIGNING
        alpha_smooth = 0.9;
    end
    
    
    if ~(dist_to_goal < 0.3 && v_current < 0.2)
        v_cmd = alpha_smooth * v_cmd + (1 - alpha_smooth) * v_cmd_prev;
        phi_cmd = alpha_smooth * phi_cmd + (1 - alpha_smooth) * phi_cmd_prev;
    end
    
    v_cmd_prev = v_cmd;
    phi_cmd_prev = phi_cmd;
end

%% ===================================================================


%% ===================================================================

if enable_debug
    debug_info(1) = min_distance;
    debug_info(2) = actual_distance;
    debug_info(3) = lookahead_distance;
    debug_info(4) = phi_cmd;
    debug_info(5) = v_error;
    debug_info(6) = target_index;
    debug_info(7) = alpha;
    debug_info(8) = v_desired;
end

end

%% ===================================================================

%% ===================================================================

function angle_wrapped = wrapToPi(angle)
    angle_wrapped = angle - 2*pi * floor((angle + pi) / (2*pi));
    if angle_wrapped > pi
        angle_wrapped = angle_wrapped - 2*pi;
    end
    if angle_wrapped < -pi
        angle_wrapped = angle_wrapped + 2*pi;
    end
end