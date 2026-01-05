%% 1. LOAD AND VERIFY DATA
fprintf('Loading data for animation...\n');

% Verify simulation data is available
if ~exist('out_actual_state', 'var')
    error('Variable out_actual_state not found in workspace.\n' + ...
          'Make sure to run the Simulink simulation before this script.');
end

% Load desired trajectory data
if ~exist('desired_trajectory.mat', 'file')
    error('File desired_trajectory.mat not found. Run Main_destraj.m first');
end

data = load('desired_trajectory.mat');
trajectory_desired = data.trajectory;
time_vector_desired = data.time_vector;

% Load costmap for visualization
costmapData = createParkingLotCostmap();
vehicleDims = costmapData.CollisionChecker.VehicleDimensions;

%% 2. EXTRACT DATA FROM SIMULATION
fprintf('Extracting simulation data...\n');

% === AUTOMATIC DATA DIAGNOSTICS ===
fprintf('=== DATA DIAGNOSTICS ===\n');
fprintf('Type of out_actual_state: %s\n', class(out_actual_state));

% Robust handling of different Simulink output formats
if isstruct(out_actual_state)
    % Case 1: Timeseries object or structure with Time and Data fields
    if isfield(out_actual_state, 'Time') && isfield(out_actual_state, 'Data')
        time_actual = out_actual_state.Time;
        raw_data = out_actual_state.Data;
        fprintf('Format: Timeseries with Time and Data\n');
    else
        error('Unrecognized out_actual_state structure. Available fields: %s', ...
              strjoin(fieldnames(out_actual_state), ', '));
    end
elseif isa(out_actual_state, 'timeseries')
    % Case 2: Pure timeseries object
    time_actual = out_actual_state.Time;
    raw_data = out_actual_state.Data;
    fprintf('Format: Timeseries object\n');
else
    % Case 3: Simple matrix (assume first column = time)
    if size(out_actual_state, 2) >= 4 % x, y, theta + time
        time_actual = out_actual_state(:, 1);
        raw_data = out_actual_state(:, 2:end);
        fprintf('Format: Matrix with time in first column\n');
    else
        error('Unrecognized out_actual_state format. Dimensions: %s', ...
              mat2str(size(out_actual_state)));
    end
end

% Verify dimensions and compatibility
fprintf('Dimensions time_actual: %s\n', mat2str(size(time_actual)));
fprintf('Dimensions raw_data: %s\n', mat2str(size(raw_data)));

% === ROBUST 3D DATA HANDLING ===
if ndims(raw_data) == 3
    fprintf('3D data detected from Simulink\n');
    data_dims = size(raw_data);
    fprintf('Original dimensions: %s\n', mat2str(data_dims));
    
    if data_dims(1) == 1 && data_dims(3) == length(time_actual)
        state_actual = squeeze(raw_data)';
        fprintf('Data reshaped from [1, %d, %d] to [%d, %d]\n', ...
                data_dims(2), data_dims(3), size(state_actual, 1), size(state_actual, 2));
    elseif data_dims(3) == 1 && data_dims(1) == length(time_actual)
        state_actual = squeeze(raw_data);
        fprintf('Data reshaped from [%d, %d, 1] to [%d, %d]\n', ...
                data_dims(1), data_dims(2), size(state_actual, 1), size(state_actual, 2));
    elseif data_dims(3) == 1 && data_dims(2) == length(time_actual)
        temp_data = squeeze(raw_data);
        state_actual = temp_data';
        fprintf('Data reshaped from [%d, %d, 1] to [%d, %d]\n', ...
                data_dims(1), data_dims(2), size(state_actual, 1), size(state_actual, 2));
    else
        error('Unrecognized 3D format. Dimensions: %s, Length(time_actual): %d', ...
              mat2str(data_dims), length(time_actual));
    end
elseif ismatrix(raw_data)
    fprintf('2D data detected\n');
    state_actual = raw_data;
    
    if size(state_actual, 1) ~= length(time_actual)
        if size(state_actual, 2) == length(time_actual)
            state_actual = state_actual';
            fprintf('State data transposed for compatibility\n');
        else
            error('Dimensional incompatibility: time_actual has %d elements, state_actual has dimensions %s', ...
                  length(time_actual), mat2str(size(state_actual)));
        end
    end
else
    error('Unsupported data format. Dimensions: %s', mat2str(size(raw_data)));
end

if isrow(time_actual)
    time_actual = time_actual';
    fprintf('Time converted to column vector\n');
end

fprintf('Final dimensions after processing:\n');
fprintf('- time_actual: %s\n', mat2str(size(time_actual)));
fprintf('- state_actual: %s\n', mat2str(size(state_actual)));

% Verify we have at least 3 state variables [x, y, theta]
if size(state_actual, 2) < 3
    error('state_actual must have at least 3 columns [x, y, theta]. Found: %d', ...
          size(state_actual, 2));
end

% Extract actual state components
x_actual = state_actual(:, 1);      % Actual x position
y_actual = state_actual(:, 2);      % Actual y position  
theta_actual = state_actual(:, 3);  % Actual orientation

% Extract steering angle if available (4th column)
if size(state_actual, 2) >= 4
    steer_actual = state_actual(:, 4);  % Steering angle
    fprintf('Steering angle available\n');
else
    steer_actual = zeros(size(theta_actual));  % Zero steering if not available
    fprintf('Steering angle not available - set to zero\n');
end

fprintf('Data extracted successfully:\n');
fprintf('- %d time samples\n', length(time_actual));
fprintf('- Time interval: %.2f - %.2f seconds\n', time_actual(1), time_actual(end));
fprintf('- X range: %.2f - %.2f m\n', min(x_actual), max(x_actual));
fprintf('- Y range: %.2f - %.2f m\n', min(y_actual), max(y_actual));
fprintf('========================\n');

fprintf('Data loaded:\n');
fprintf('- Desired trajectory: %d points in %.1f seconds\n', ...
        length(trajectory_desired.xd), time_vector_desired(end));
fprintf('- Actual trajectory: %d points in %.1f seconds\n', ...
        length(x_actual), time_actual(end));

%% 3. TIME SYNCHRONIZATION
fprintf('Time synchronization of data...\n');

t_start = max(time_vector_desired(1), time_actual(1));
t_end = min(time_vector_desired(end), time_actual(end));
fprintf('Animation time interval: %.1f - %.1f seconds\n', t_start, t_end);

dt_animation = 0.1;
time_animation = t_start:dt_animation:t_end;

fprintf('Interpolating trajectories...\n');

xd_interp = interp1(time_vector_desired, trajectory_desired.xd, time_animation, 'linear', 'extrap');
yd_interp = interp1(time_vector_desired, trajectory_desired.yd, time_animation, 'linear', 'extrap');
thetad_interp = interp1(time_vector_desired, trajectory_desired.thetad, time_animation, 'linear', 'extrap');

x_interp = interp1(time_actual, x_actual, time_animation, 'linear', 'extrap');
y_interp = interp1(time_actual, y_actual, time_animation, 'linear', 'extrap');
theta_interp = interp1(time_actual, theta_actual, time_animation, 'linear', 'extrap');
steer_interp = interp1(time_actual, steer_actual, time_animation, 'linear', 'extrap');

%% 4. CALCULATE TRACKING METRICS
fprintf('Calculating performance metrics...\n');

error_x = x_interp - xd_interp;
error_y = y_interp - yd_interp;
error_position = sqrt(error_x.^2 + error_y.^2);
error_theta = wrapToPi(theta_interp - thetad_interp);

fprintf('Tracking error statistics:\n');
fprintf('- Mean position error: %.3f m\n', mean(error_position));
fprintf('- Maximum position error: %.3f m\n', max(error_position));
fprintf('- Mean orientation error: %.1f degrees\n', mean(abs(error_theta)) * 180/pi);
fprintf('- Maximum orientation error: %.1f degrees\n', max(abs(error_theta)) * 180/pi);

%% 5. SETUP FIGURE FOR ANIMATION
fprintf('Preparing animation...\n');

fig = figure('Name', 'Autonomous Bicycle Parking', 'Position', [100, 100, 1000, 800]);

% Single main plot
ax_main = axes('Parent', fig);
plot(costmapData, 'Parent', ax_main, 'Inflation', 'off');
hold(ax_main, 'on');

% Plot trajectories
plot(ax_main, trajectory_desired.xd, trajectory_desired.yd, 'b--', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory');
plot(ax_main, x_actual, y_actual, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual Trajectory');

% === BICYCLE INITIALIZATION ===
% Initial pose
pose_actual_init = [x_interp(1), y_interp(1), theta_interp(1) * 180/pi];      % Convert to degrees

% Create the actual bicycle
bicycle_actual_handles = PlotBicycle(pose_actual_init, vehicleDims, steer_interp(1) * 180/pi, ...
    'Parent', ax_main, 'Color', 'red', 'DisplayName', 'Actual Vehicle');

% Trail line
%trail_line = plot(ax_main, x_interp(1), y_interp(1), 'r:', 'LineWidth', 1, 'DisplayName', 'Actual Trail');

axis(ax_main, 'equal');
grid(ax_main, 'on');
legend(ax_main, 'Location', 'northeast');
title(ax_main, 'Autonomous Bicycle Parking');
xlabel(ax_main, 'X [m]');
ylabel(ax_main, 'Y [m]');

%% 6. ANIMATION LOOP
fprintf('Starting animation...\n');

% === VIDEO RECORDING SETUP ===
save_video = true;  % Set to false if you don't want to save the video
if save_video
    video_filename = sprintf('BICYCLE_PARKING_ANIMATION_MPC_CONTROLLER_%s.mp4');
    fprintf('Preparing video recording: %s\n', video_filename);
    
    % Create VideoWriter object
    video_writer = VideoWriter(video_filename, 'MPEG-4');
    video_writer.FrameRate = 15;  % Frames per second in final video
    video_writer.Quality = 95;    % Video quality (0-100)
    open(video_writer);
end

skip_frames = 7;  % Skip frames to speed up animation
trail_length = 50;

for k = 1:skip_frames:length(time_animation)
    % Current pose (convert angles to degrees for PlotBicycle)
    pose_actual = [x_interp(k), y_interp(k), theta_interp(k) * 180/pi];
    
    % Delete previous bicycle
    delete(bicycle_actual_handles);
    
    % Create new bicycle at updated position
    bicycle_actual_handles = PlotBicycle(pose_actual, vehicleDims, steer_interp(k) * 180/pi, ...
        'Parent', ax_main, 'Color', 'red', 'DisplayName', 'Actual Vehicle');
    
    % Update trail
    trail_start = max(1, k - trail_length);
    %set(trail_line, 'XData', x_interp(trail_start:k), 'YData', y_interp(trail_start:k));
    
    % Update title with current metrics
    current_error = error_position(k);
    current_theta_error = abs(error_theta(k)) * 180/pi;
    current_steer = steer_interp(k) * 180/pi;
    title(ax_main, 'Autonomous Bicycle Parking');
    
    drawnow;
    
    % Capture frame for video
    if save_video
        frame = getframe(fig);
        writeVideo(video_writer, frame);
    end
    
    pause(0.01);
end

% === CLOSE VIDEO RECORDING ===
if save_video
    close(video_writer);
    fprintf('Video saved as: %s\n', video_filename);
    fprintf('Video duration: %.1f seconds\n', length(1:skip_frames:length(time_animation)) / video_writer.FrameRate);
end

fprintf('Animation completed!\n');

%% 7. SAVE METRICS (OPTIONAL)
fprintf('Saving tracking metrics...\n');

tracking_results.time = time_animation;
tracking_results.error_position = error_position;
tracking_results.error_theta = error_theta;
tracking_results.statistics.mean_pos_error = mean(error_position);
tracking_results.statistics.max_pos_error = max(error_position);
tracking_results.statistics.mean_theta_error = mean(abs(error_theta));
tracking_results.statistics.max_theta_error = max(abs(error_theta));

save('tracking_results.mat', 'tracking_results', '-v7.3');
fprintf('Metrics saved in tracking_results.mat\n');