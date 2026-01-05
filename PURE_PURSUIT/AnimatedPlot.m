%% 1. DATA LOADING AND VERIFICATION
% Check that simulation data are available
required_vars = {'out_vehicle_state', 'out_control_commands'};
for i = 1:length(required_vars)
    if ~exist(required_vars{i}, 'var')
        error('The variable %s is not present in the workspace.\n' + ...
              'Make sure you have run the Simulink simulation before this script.', ...
              required_vars{i});
    end
end

if ~exist('pure_pursuit_trajectory.mat', 'file')
    error('File pure_pursuit_trajectory.mat not found. Run MAIN_PURE_PURSUIT first.m');
end

data = load('pure_pursuit_trajectory.mat');
trajectory_desired = data.trajectory_points;  % [x, y, theta, v_desired]
pp_params = data.pp_params;

costmapData = createParkingLotCostmap();
vehicleDims = costmapData.CollisionChecker.VehicleDimensions;

%% 2. DATA EXTRACTION FROM SIMULINK SIMULATION
% === STATE OF THE VEHICLE ===
[time_actual, state_actual] = extractSimulinkData(out_vehicle_state, 'vehicle_state');
x_actual = state_actual(:, 1);      
y_actual = state_actual(:, 2);       
theta_actual = state_actual(:, 3);  
v_actual = state_actual(:, 4);      

% === CONTROLLER COMMANDS ===
[time_control, control_commands] = extractSimulinkData(out_control_commands, 'control_commands');
v_cmd = control_commands(:, 1);     
phi_cmd = control_commands(:, 2);   

debug_available = exist('out_debug_info', 'var') && ~isempty(out_debug_info);
if debug_available
    
    [time_debug, debug_info] = extractSimulinkData(out_debug_info, 'debug_info');
    
  
    closest_distance = debug_info(:, 1);        
    target_distance = debug_info(:, 2);        
    lookahead_distance = debug_info(:, 3);      
    steering_angle = debug_info(:, 4);          
    velocity_error = debug_info(:, 5);          
    target_index = debug_info(:, 6);            
    alpha_angle = debug_info(:, 7);             
    v_desired_debug = debug_info(:, 8);         
else
    fprintf('Debug information not available.\n');
end


%% 3. TIME SYNCHRONISATION

t_start = time_actual(1);
t_end = time_actual(end);
fprintf('Animation time interval: %.1f - %.1f secondi\n', t_start, t_end);

dt_animation = 0.1;  
time_animation = t_start:dt_animation:t_end;

x_interp = interp1(time_actual, x_actual, time_animation, 'linear', 'extrap');
y_interp = interp1(time_actual, y_actual, time_animation, 'linear', 'extrap');
theta_interp = interp1(time_actual, theta_actual, time_animation, 'linear', 'extrap');
v_interp = interp1(time_actual, v_actual, time_animation, 'linear', 'extrap');

v_cmd_interp = interp1(time_control, v_cmd, time_animation, 'linear', 'extrap');
phi_cmd_interp = interp1(time_control, phi_cmd, time_animation, 'linear', 'extrap');

if debug_available
    lookahead_interp = interp1(time_debug, lookahead_distance, time_animation, 'linear', 'extrap');
    target_index_interp = round(interp1(time_debug, target_index, time_animation, 'nearest', 'extrap'));
    alpha_interp = interp1(time_debug, alpha_angle, time_animation, 'linear', 'extrap');
else
   
    lookahead_interp = pp_params.lookahead_distance * ones(size(time_animation));
    target_index_interp = ones(size(time_animation));
    alpha_interp = zeros(size(time_animation));
end

%% 4. CALCULATION OF PURE PURSUIT TRACKING METRICS
error_x = zeros(size(time_animation));
error_y = zeros(size(time_animation));
error_position = zeros(size(time_animation));
error_theta = zeros(size(time_animation));

for k = 1:length(time_animation)
    
    distances = sqrt((trajectory_desired(:, 1) - x_interp(k)).^2 + ...
                     (trajectory_desired(:, 2) - y_interp(k)).^2);
    [~, closest_idx] = min(distances);
    
    
    error_x(k) = x_interp(k) - trajectory_desired(closest_idx, 1);
    error_y(k) = y_interp(k) - trajectory_desired(closest_idx, 2);
    error_position(k) = sqrt(error_x(k)^2 + error_y(k)^2);
    error_theta(k) = wrapToPi(theta_interp(k) - trajectory_desired(closest_idx, 3));
end

fprintf('Pure Pursuit tracking error statistics:\n');
fprintf('- Mean positional error: %.3f m\n', mean(error_position));
fprintf('- Maximum positional error: %.3f m\n', max(error_position));
fprintf('- Medium orientation error: %.1f degrees\n', mean(abs(error_theta)) * 180/pi);
fprintf('- Maximum orientation error: %.1f degrees\n', max(abs(error_theta)) * 180/pi);

%% 5. SETUP FIGURE FOR PURE PURSUIT ANIMATION

fig = figure('Name', 'Animation Pure Pursuit Controller', 'Position', [100, 100, 1200, 900]);

ax_main = gca;
plot(costmapData, 'Parent', ax_main, 'Inflation', 'off');
hold(ax_main, 'on');

plot(ax_main, trajectory_desired(:, 1), trajectory_desired(:, 2), 'b--', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory');
plot(ax_main, x_actual, y_actual, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual Trajectory');

pose_actual_init = [x_interp(1), y_interp(1), theta_interp(1) * 180/pi];
bicycle_handles = PlotBicycle(pose_actual_init, vehicleDims, phi_cmd_interp(1) * 180/pi, ...
    'Parent', ax_main, 'Color', 'red', 'DisplayName', 'Bicycle');

%trail_line = plot(ax_main, x_interp(1), y_interp(1), 'r:', 'LineWidth', 1, 'DisplayName', 'Scia');

lookahead_circle = plot(ax_main, NaN, NaN, 'g:', 'LineWidth', 2, 'DisplayName', 'Lookahead Circle');
target_point = plot(ax_main, NaN, NaN, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'green', 'DisplayName', 'Target Point');
target_line = plot(ax_main, NaN, NaN, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Target Line');

axis(ax_main, 'equal');
grid(ax_main, 'on');
legend(ax_main, 'Location', 'northeast');
title(ax_main, 'Autonomous Bicycle Parking');
xlabel(ax_main, 'X [m]');
ylabel(ax_main, 'Y [m]');

%% 6. PURE PURSUIT ANIMATION CYCLE

save_video = true;
if save_video
    video_filename = 'BICYCLE_PARKING_ANIMATION_PURE_PURSUIT.mp4';
    fprintf('Video recording preparation: %s\n', video_filename);
    
    video_writer = VideoWriter(video_filename, 'MPEG-4');
    video_writer.FrameRate = 8;
    video_writer.Quality = 95;
    open(video_writer);
end

skip_frames = 10;  
trail_length = 50;

for k = 1:skip_frames:length(time_animation)
    
    pose_current = [x_interp(k), y_interp(k), theta_interp(k) * 180/pi];
    
    
    delete(bicycle_handles);
    bicycle_handles = PlotBicycle(pose_current, vehicleDims, phi_cmd_interp(k) * 180/pi, ...
        'Parent', ax_main, 'Color', 'red');
    
    
    trail_start = max(1, k - trail_length);
    %set(trail_line, 'XData', x_interp(trail_start:k), 'YData', y_interp(trail_start:k));
    
   
    current_lookahead = lookahead_interp(k);
    circle_angles = 0:0.1:2*pi;
    circle_x = x_interp(k) + current_lookahead * cos(circle_angles);
    circle_y = y_interp(k) + current_lookahead * sin(circle_angles);
    set(lookahead_circle, 'XData', circle_x, 'YData', circle_y);
    
    
    if debug_available && target_index_interp(k) <= size(trajectory_desired, 1)
        target_idx = max(1, min(size(trajectory_desired, 1), target_index_interp(k)));
        target_x = trajectory_desired(target_idx, 1);
        target_y = trajectory_desired(target_idx, 2);
        
        set(target_point, 'XData', target_x, 'YData', target_y);
        set(target_line, 'XData', [x_interp(k), target_x], 'YData', [y_interp(k), target_y]);
    end
    
    drawnow;
    
    
    if save_video
        frame = getframe(fig);
        writeVideo(video_writer, frame);
    end
    
    pause(0.01);
end

if save_video
    close(video_writer);
    fprintf('Video saved as: %s\n', video_filename);
end

fprintf('Pure Pursuit animation completed!\n');

%% 7. SAVING PURE PURSUIT METRICS

pure_pursuit_results.time = time_animation;
pure_pursuit_results.error_position = error_position;
pure_pursuit_results.error_theta = error_theta;
pure_pursuit_results.v_cmd = v_cmd_interp;
pure_pursuit_results.phi_cmd = phi_cmd_interp;
pure_pursuit_results.lookahead_distance = lookahead_interp;
pure_pursuit_results.statistics.mean_pos_error = mean(error_position);
pure_pursuit_results.statistics.max_pos_error = max(error_position);
pure_pursuit_results.statistics.mean_theta_error = mean(abs(error_theta));
pure_pursuit_results.statistics.max_theta_error = max(abs(error_theta));

save('pure_pursuit_results.mat', 'pure_pursuit_results', '-v7.3');
fprintf('Pure Pursuit metrics saved in pure_pursuit_results.mat\n');


function [time_vec, data_matrix] = extractSimulinkData(simulink_output, data_name)
  
    
    fprintf('Data extraction %s...\n', data_name);
    
    if isstruct(simulink_output)
        if isfield(simulink_output, 'Time') && isfield(simulink_output, 'Data')
            time_vec = simulink_output.Time;
            raw_data = simulink_output.Data;
        else
            error('Structure %s not recognised', data_name);
        end
    elseif isa(simulink_output, 'timeseries')
        time_vec = simulink_output.Time;
        raw_data = simulink_output.Data;
    else
        error('Unsupported %s format', data_name);
    end
    
   
    if ndims(raw_data) == 3
        data_dims = size(raw_data);
        if data_dims(1) == 1 && data_dims(3) == length(time_vec)
            data_matrix = squeeze(raw_data)';
        elseif data_dims(3) == 1 && data_dims(1) == length(time_vec)
            data_matrix = squeeze(raw_data);
        else
            error('3D format not recognised for %s', data_name);
        end
    else
        data_matrix = raw_data;
        if size(data_matrix, 1) ~= length(time_vec)
            if size(data_matrix, 2) == length(time_vec)
                data_matrix = data_matrix';
            else
                error('Dimensional incompatibility for %s', data_name);
            end
        end
    end
    
   
    if isrow(time_vec)
        time_vec = time_vec';
    end
    
    fprintf('Data %s extracted: %d samples, %d variables\n', ...
            data_name, size(data_matrix, 1), size(data_matrix, 2));
end

function angle_wrapped = wrapToPi(angle)
  
    angle_wrapped = angle - 2*pi * floor((angle + pi) / (2*pi));
end