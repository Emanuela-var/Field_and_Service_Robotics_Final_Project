clear;
clc;
close all;

rng(3);
fprintf('=== MAIN MPC TRAJECTORY GENERATION ===\n');

% 1. DATA LOADING AND INITIAL SETUP
fprintf('1. Data loading and initial setup...\n');
data = load('GlobalRoutePlan.mat');
routePlan = data.routePlan;
costmap = createParkingLotCostmap();

% Vehicle parameters
L = 2.8; % wheelbase [m]

% Time parameters
dt = 0.1; % time step [s]
T_total = 250; % total simulation time [s] 
time_vector = 0:dt:T_total;

% =======================================================================
% === MPC PARAMETER CONFIGURATION =====================================
% =======================================================================
fprintf('2. MPC PARAMETER CONFIGURATION...\n');

% Optimised MPC parameters for trajectory and parking tracking
mpc_params = struct();

% Basic vehicle parameters
mpc_params.wheelbase = L;                    % wheelbase [m]

% MPC core parameters
mpc_params.prediction_horizon = 6;           % Prediction horizon (steps)
mpc_params.sampling_time = dt;               % Sampling time [s]

% Weight matrices for optimisation
% Q_weights: weights for state errors [e_lateral, e_longitudinal, e_heading]
mpc_params.Q_weights = [2000, 200, 400];     % Status error weights
% R_weights: weights for command variations [delta_v, delta_phi]
mpc_params.R_weights = [0.5, 2];             % Control weights

% Operational Constraints
mpc_params.v_max = 2.0;                      % Maximum speed [m/s]
mpc_params.v_min = 0.0;                      % Minimum speeda [m/s] 
mpc_params.phi_max = pi/6;                   % Maximum steer angle [rad] - 30 gradi

% Robustness parameters
mpc_params.smoothing_factor = 0.85;          % Factor smoothing commands (0-1)
mpc_params.tolerance = 1e-6;                 % Numerical tolerance
mpc_params.max_iterations = 200;             % Maximum solver iterations

% Parameters for controller fallback 
mpc_params.fallback_kp_longitudinal = 0.8;   
mpc_params.fallback_kp_lateral = 0.6;       
mpc_params.fallback_kp_heading = 0.4;        

fprintf('   - Prediction horizon: %d steps (%.1f s)\n', mpc_params.prediction_horizon, mpc_params.prediction_horizon * dt);
fprintf('   - Velocity: %.1f - %.1f m/s\n', mpc_params.v_min, mpc_params.v_max);
fprintf('   - Steer angle max: %.1f gradi\n', mpc_params.phi_max * 180/pi);

% =======================================================================
% === BUS OBJECT CREATION FOR MPC PARAMETERS ===========================
% =======================================================================
fprintf('3. BUS OBJECT CREATION FOR MPC PARAMETERS...\n');

num_elements = 14;
elems_mpc(num_elements) = Simulink.BusElement;

idx = 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'wheelbase';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'prediction_horizon';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'sampling_time';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'Q_weights';
elems_mpc(idx).DataType = 'double';
elems_mpc(idx).Dimensions = [1, 3];

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'R_weights';
elems_mpc(idx).DataType = 'double';
elems_mpc(idx).Dimensions = [1, 2];

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'v_max';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'v_min';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'phi_max';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'smoothing_factor';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'tolerance';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'max_iterations';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'fallback_kp_longitudinal';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'fallback_kp_lateral';
elems_mpc(idx).DataType = 'double';

idx = idx + 1;
elems_mpc(idx) = Simulink.BusElement;
elems_mpc(idx).Name = 'fallback_kp_heading';
elems_mpc(idx).DataType = 'double';

% bus object
mpc_params_bus = Simulink.Bus;
mpc_params_bus.Elements = elems_mpc;

clear elems_mpc idx num_elements;

% Struct for Simulink
mpc_params_for_simulink = mpc_params;

% =======================================================================
% === GLOBAL PATH GENERATION =====================================
% =======================================================================
fprintf('4. GLOBAL PATH GENERATION with RRT*...\n');

% Configurazione planner RRT*
plannerRRTConfig.MinTurningRadius = 4.0;
plannerRRTConfig.ConnectionDistance = 30.0;
plannerRRTConfig.GoalTolerance = [0.5 0.5 10];
plannerRRTConfig.MinIterations = 2000;

motionPlannerRRT = pathPlannerRRT(costmap, ...
    'ConnectionDistance', plannerRRTConfig.ConnectionDistance, ...
    'MinIterations', plannerRRTConfig.MinIterations, ...
    'GoalTolerance', plannerRRTConfig.GoalTolerance, ...
    'MinTurningRadius', plannerRRTConfig.MinTurningRadius);

% Generazione waypoints grezzi
globalWaypoints_raw = [];
initialPose = routePlan{1, 'StartPose'};
finalGoal = routePlan{end, 'EndPose'};

for i = 1:height(routePlan)
    fprintf('   - Segment planning %d/%d...\n', i, height(routePlan));
    startPose = routePlan{i, 'StartPose'};
    endPose = routePlan{i, 'EndPose'};
    
    segmentPathObject = plan(motionPlannerRRT, startPose, endPose);
    segmentPoses = interpolate(segmentPathObject);
    
    if isempty(segmentPoses)
        error('RRT* failed for segment %d.', i);
    end
    
    if i == 1
        globalWaypoints_raw = [globalWaypoints_raw; segmentPoses];
    else
        globalWaypoints_raw = [globalWaypoints_raw; segmentPoses(2:end, :)];
    end
end

fprintf('   - Raw Waypoints generated: %d points\n', size(globalWaypoints_raw, 1));

% =======================================================================
% === PATH SMOOTHING E INTERPOLATION ==================================
% =======================================================================
fprintf('5. Path smoothing with splines...\n');

% Interpolazione per aumentare la densità di punti
if size(globalWaypoints_raw, 1) > 1
    waypoints_interp = interp1(1:size(globalWaypoints_raw, 1), globalWaypoints_raw, ...
                               linspace(1, size(globalWaypoints_raw, 1), 250), 'pchip');
else
    waypoints_interp = globalWaypoints_raw;
end

% Creazione spline per smoothing
path_spline = cscvn(waypoints_interp(:, 1:2)');

% Generazione punti smooth
num_smooth_points = 400;
t_spline = linspace(path_spline.breaks(1), path_spline.breaks(end), num_smooth_points);
smooth_points_xy = fnval(path_spline, t_spline)';

% Calcolo orientamenti dal gradiente della spline
path_derivative = fnder(path_spline);
smooth_derivatives = fnval(path_derivative, t_spline);
smooth_theta = atan2(smooth_derivatives(2, :), smooth_derivatives(1, :))';

% Percorso finale smoothed
globalWaypoints = [smooth_points_xy, smooth_theta];

fprintf('   - Waypoints smooth generated: %d points\n', size(globalWaypoints, 1));

trajectory = generateImprovedTrajectoryForMPC(globalWaypoints, time_vector, L, mpc_params);

fprintf('   - Generated trajectory: %d time points\n', length(trajectory.xd));
fprintf('   - Actual course duration: %.1f s\n', trajectory.actual_duration);
fprintf('   - Initial Velocity: %.3f m/s\n', sqrt(trajectory.xd_dot(1)^2 + trajectory.yd_dot(1)^2));
fprintf('   - Final velocity : %.3f m/s\n', sqrt(trajectory.xd_dot(end)^2 + trajectory.yd_dot(end)^2));

% =======================================================================
% === TIMESERIES PREPARATION FOR SIMULINK =============================
% =======================================================================
fprintf('7. TIMESERIES PREPARATION FOR SIMULINK...\n');

ts_xd = timeseries(trajectory.xd, time_vector);
ts_yd = timeseries(trajectory.yd, time_vector);
ts_thetad = timeseries(trajectory.thetad, time_vector);
ts_phid = timeseries(trajectory.phid, time_vector);
ts_xd_dot = timeseries(trajectory.xd_dot, time_vector);
ts_yd_dot = timeseries(trajectory.yd_dot, time_vector);
ts_thetad_dot = timeseries(trajectory.thetad_dot, time_vector);

ts_xd.Name = 'Desired X Position';
ts_yd.Name = 'Desired Y Position';
ts_thetad.Name = 'Desired Heading';
ts_phid.Name = 'Desired Steering Angle';
ts_xd_dot.Name = 'Desired X Velocity';
ts_yd_dot.Name = 'Desired Y Velocity';
ts_thetad_dot.Name = 'Desired Heading Rate';

% =======================================================================
% === FULL RESCUE =============================================
% =======================================================================

save('desired_trajectory.mat', ...
     'trajectory', 'time_vector', 'mpc_params', 'mpc_params_bus', ...
     'ts_xd', 'ts_yd', 'ts_thetad', 'ts_phid', 'ts_xd_dot', 'ts_yd_dot', 'ts_thetad_dot', ...
     'globalWaypoints', 'globalWaypoints_raw', 'initialPose', 'finalGoal', ...
     'mpc_params_for_simulink', '-v7.3');

save('mpc_parameters.mat', 'mpc_params', '-v7.3');

% =======================================================================
% === VALIDATION AND DIAGNOSTICS ========================================
% =======================================================================
fprintf('9. Trajectory validation...\n');

% Controlli di validità
max_v = max(sqrt(trajectory.xd_dot.^2 + trajectory.yd_dot.^2));
max_phi = max(abs(trajectory.phid));
max_theta_dot = max(abs(trajectory.thetad_dot));

fprintf('   - Maximum velocity: %.2f m/s (limite: %.2f)\n', max_v, mpc_params.v_max);
fprintf('   - Maximum Steer Angle: %.2f deg (limite: %.2f)\n', rad2deg(max_phi), rad2deg(mpc_params.phi_max));
fprintf('   - Maximum angular velocity: %.2f deg/s\n', rad2deg(max_theta_dot));

% Checking constraints
if max_v > mpc_params.v_max * 1.01  % Tolleranza 1%
    warning('Maximum trajectory speed exceeds MPC limit!');
end
if max_phi > mpc_params.phi_max * 1.01
    warning('Maximum steering angle exceeds MPC limit!');
end


% =======================================================================
% === AUXILIARY FUNCTIONS ===============================================
% =======================================================================

function trajectory = generateImprovedTrajectoryForMPC(waypoints, time_vector, wheelbase, mpc_params)
    % Generates trajectory with zero speed at start and end
    % Adapted specifically for MPC with completion time management
    
    % Parametri
    v_max = mpc_params.v_max;
    v_min = mpc_params.v_min;
    a_max = 1.0;  % Maximum acceleration [m/s^2]
    a_min = -1.5; % Maximum deceleration [m/s^2]
    
    % Calculating cumulative distances
    distances = zeros(length(waypoints), 1);
    for i = 2:length(waypoints)
        distances(i) = distances(i-1) + norm(waypoints(i,1:2) - waypoints(i-1,1:2));
    end
    total_distance = distances(end);
    
    % Speed profile generation with ramps
    velocities = zeros(length(waypoints), 1);
    
    % Acceleration and deceleration distances
    accel_distance = min(5.0, total_distance * 0.15);
    decel_distance = min(10.0, total_distance * 0.25);
    
    % Calculating base velocity considering curvature
    for i = 1:length(waypoints)
        
        velocities(i) = v_max;
        
        % Curvature reduction
        if i > 1 && i < length(waypoints)
            angle_change = abs(angdiff(waypoints(i+1,3), waypoints(i,3)));
            if angle_change > pi/12  % Sensitive threshold
                curve_factor = max(0, 1 - (angle_change / (pi/4)));
                velocities(i) = v_min + (v_max - v_min) * curve_factor^1.5;
            end
        end
    end
    
    % Apply acceleration/deceleration ramps
    for i = 1:length(waypoints)
        % Acceleration ramp
        if distances(i) < accel_distance
            accel_factor = distances(i) / accel_distance;
            velocities(i) = velocities(i) * accel_factor;
        end
        
        % Deceleration ramp
        dist_from_end = total_distance - distances(i);
        if dist_from_end < decel_distance
            decel_factor = dist_from_end / decel_distance;
            velocities(i) = velocities(i) * decel_factor;
        end
    end
    
    % Zero velocity force at extremes
    velocities(1) = 0;
    velocities(end) = 0;
    
    velocities = smoothdata(velocities, 'gaussian', max(5, round(length(velocities)/20)));
    
    velocities(1:3) = 0;
    velocities(end-2:end) = 0;
    
    actual_duration = 0;
    for i = 2:length(waypoints)
        segment_dist = distances(i) - distances(i-1);
        avg_velocity = max(0.01, (velocities(i) + velocities(i-1)) / 2);
        actual_duration = actual_duration + segment_dist / avg_velocity;
    end
    
    trajectory = generateTimedTrajectoryMPC(waypoints, velocities, distances, ...
                                           time_vector, wheelbase, actual_duration);
    trajectory.actual_duration = actual_duration;
end

function trajectory = generateTimedTrajectoryMPC(waypoints, velocities, distances, ...
                                                time_vector, wheelbase, actual_duration)
   
    time_waypoints = zeros(length(distances), 1);
    for i = 2:length(distances)
        segment_dist = distances(i) - distances(i-1);
        avg_velocity = max(0.01, (velocities(i) + velocities(i-1)) / 2);
        time_waypoints(i) = time_waypoints(i-1) + segment_dist / avg_velocity;
    end
    
  
    [time_unique, unique_idx] = unique(time_waypoints);
    waypoints_unique = waypoints(unique_idx, :);
    
    
    n_points = length(time_vector);
    xd = zeros(n_points, 1);
    yd = zeros(n_points, 1);
    thetad = zeros(n_points, 1);
    xd_dot = zeros(n_points, 1);
    yd_dot = zeros(n_points, 1);
    thetad_dot = zeros(n_points, 1);
    phid = zeros(n_points, 1);
    
    
    final_x = waypoints(end, 1);
    final_y = waypoints(end, 2);
    final_theta = waypoints(end, 3);
    
    
    dt = time_vector(2) - time_vector(1);
    
    for i = 1:n_points
        t = time_vector(i);
        
        if t <= actual_duration
            
            xd(i) = interp1(time_unique, waypoints_unique(:,1), t, 'pchip');
            yd(i) = interp1(time_unique, waypoints_unique(:,2), t, 'pchip');
            thetad(i) = interp1(time_unique, waypoints_unique(:,3), t, 'pchip');
            
            
            if i > 1
                xd_dot(i) = (xd(i) - xd(i-1)) / dt;
                yd_dot(i) = (yd(i) - yd(i-1)) / dt;
            end
        else
           
            xd(i) = final_x;
            yd(i) = final_y;
            thetad(i) = final_theta;
            xd_dot(i) = 0;
            yd_dot(i) = 0;
        end
    end
    
    
    xd_dot = smoothdata(xd_dot, 'gaussian', 5);
    yd_dot = smoothdata(yd_dot, 'gaussian', 5);
    
    completion_idx = find(time_vector > actual_duration, 1);
    if ~isempty(completion_idx)
        xd_dot(completion_idx:end) = 0;
        yd_dot(completion_idx:end) = 0;
    end
    
   
    xd_dot(1:3) = 0;
    yd_dot(1:3) = 0;
    
    
    thetad_unwrapped = unwrap(thetad);
    thetad_dot = gradient(thetad_unwrapped, dt);
    thetad_dot = smoothdata(thetad_dot, 'gaussian', 5);
    
    
    stopped_idx = (xd_dot.^2 + yd_dot.^2) < 1e-6;
    thetad_dot(stopped_idx) = 0;
    
    
    velocity_magnitude = sqrt(xd_dot.^2 + yd_dot.^2);
    for i = 1:n_points
        if velocity_magnitude(i) > 0.05
            phid(i) = atan(wheelbase * thetad_dot(i) / velocity_magnitude(i));
        else
            phid(i) = 0;
        end
    end
    
    
    max_phi = pi/4;
    phid = max(-max_phi, min(max_phi, phid));
    phid = smoothdata(phid, 'gaussian', 5);
    
    
    thetad = wrapToPi(thetad);
    
    
    trajectory.xd = xd;
    trajectory.yd = yd;
    trajectory.thetad = thetad;
    trajectory.phid = phid;
    trajectory.xd_dot = xd_dot;
    trajectory.yd_dot = yd_dot;
    trajectory.thetad_dot = thetad_dot;
    trajectory.velocity = velocity_magnitude;
end

function angle_diff = angdiff(angle1, angle2)
    angle_diff = angle1 - angle2;
    angle_diff = atan2(sin(angle_diff), cos(angle_diff));
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end