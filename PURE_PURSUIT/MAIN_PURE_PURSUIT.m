clear;
clc;
close all;


rng(110);

% 1. DATA LOADING AND INITIAL SETUP
data = load('GlobalRoutePlan.mat');
routePlan = data.routePlan;
costmap = createParkingLotCostmap();

% Vehicle parameters
L = 2.8; % wheelbase [m]

% Time parameters
dt = 0.1; % time step [s]
T_total = 400; % total simulation time [s]
time_vector = 0:dt:T_total;

% =======================================================================
% === PURE PURSUIT PARAMETER CONFIGURATION ===========================
% =======================================================================
fprintf('2. Pure Pursuit parameter configuration...\n');
debug_enabled = true;
% Pure Pursuit parameters optimised for trajectory tracking
pp_params = struct();

pp_params.wheelbase = L;                     % wheelbase [m]
pp_params.v_default = 1.5;                  % Velocity [m/s]

% Lookahead parameters 
% The lookahead distance determines how “forward” the vehicle looks
pp_params.lookahead_distance = 5.0;         % lookahead distance [m]
pp_params.lookahead_time = 2.0;             % lookahead time [s]
pp_params.lookahead_min = 3.0;              % min lookahead distance [m]
pp_params.lookahead_max = 20.0;             % max lookahead distance [m]

% Operational Constraints
pp_params.v_max = 3.0;                      % Vel max [m/s]
pp_params.v_min = 0;                        % Vel min [m/s]
pp_params.phi_max = pi/4;                   % Max steer angle [rad] 

pp_params.kp_longitudinal = 1.2;            

pp_params.enable_smoothing = true;          
pp_params.smoothing_factor = 0.75;          

pp_params.enable_debug = false;             
pp_params.save_debug_data = false;          

pp_params.parking_distance_threshold = 0.3;  % Distance threshold to consider parking complete [m].
pp_params.parking_angle_threshold = 0.1;     % Angle threshold for alignment [rad]
pp_params.final_approach_distance = 5.0;     % Distance to start final approach [m]
pp_params.final_approach_v_max = 0.5;        % Maximum speed in final approach [m/s]

fprintf('   - Lookahead base: %.1f m\n', pp_params.lookahead_distance);
fprintf('   - Lookahead adaptive: %.1f s * velcity\n', pp_params.lookahead_time);
fprintf('   - Range lookahead: [%.1f, %.1f] m\n', pp_params.lookahead_min, pp_params.lookahead_max);

% =======================================================================
% === BUS OBJECT CREATION FOR PURE PURSUIT PARAMETERS ================
% =======================================================================

num_elements = 16;  
elems_pp(num_elements) = Simulink.BusElement;  

idx = 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'wheelbase';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'v_default';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'lookahead_distance';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'lookahead_time';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'lookahead_min';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'lookahead_max';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'v_max';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'v_min';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'phi_max';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'kp_longitudinal';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'enable_smoothing';
elems_pp(idx).DataType = 'boolean';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'smoothing_factor';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'parking_distance_threshold';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'parking_angle_threshold';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'final_approach_distance';
elems_pp(idx).DataType = 'double';

idx = idx + 1;
elems_pp(idx) = Simulink.BusElement;
elems_pp(idx).Name = 'final_approach_v_max';
elems_pp(idx).DataType = 'double';

pp_params_bus = Simulink.Bus;
pp_params_bus.Elements = elems_pp;

clear elems_pp idx num_elements;

% =======================================================================
% === GLOBAL PATH GENERATION =====================================
% =======================================================================

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

fprintf('   - Raw waypoints generated: %d points\n', size(globalWaypoints_raw, 1));

% =======================================================================
% === PATH SMOOTHING AND INTERPOLATION FOR PURE PURSUIT ================
% =======================================================================

if size(globalWaypoints_raw, 1) > 1
  
    waypoints_interp = interp1(1:size(globalWaypoints_raw, 1), globalWaypoints_raw, ...
                               linspace(1, size(globalWaypoints_raw, 1), 350), 'pchip');
else
    waypoints_interp = globalWaypoints_raw;
end

path_spline = cscvn(waypoints_interp(:, 1:2)');

num_smooth_points = 600;  
t_spline = linspace(path_spline.breaks(1), path_spline.breaks(end), num_smooth_points);
smooth_points_xy = fnval(path_spline, t_spline)';

path_derivative = fnder(path_spline);
smooth_derivatives = fnval(path_derivative, t_spline);
smooth_theta = atan2(smooth_derivatives(2, :), smooth_derivatives(1, :))';

globalWaypoints = [smooth_points_xy, smooth_theta];

fprintf('   - Waypoints smooth generated: %d puints\n', size(globalWaypoints, 1));

% =======================================================================
% === TRAJECTORY GENERATION WITH DESIRED SPEED ==================
% =======================================================================
v_cruise = 2.0;        % Cruise vel [m/s]
v_curve = 1.0;         % Vel in curve [m/s]
v_start_stop = 0.0;    % Init/fin vel [m/s]

curvature = calculatePathCurvature(globalWaypoints);

velocities = generateImprovedVelocityProfile(globalWaypoints, v_cruise, v_curve, curvature);

fprintf('   - Initial Velocity: %.3f m/s\n', velocities(1));
fprintf('   - Final Velocity: %.3f m/s\n', velocities(end));
fprintf('   - Max Velocity: %.2f m/s\n', max(velocities));
fprintf('   - Velocity mean: %.2f m/s\n', mean(velocities(velocities > 0.1)));

trajectory_points = [globalWaypoints, velocities];

% =======================================================================
% === DATA PREPARATION FOR SIMULINK ===================================
% =======================================================================

trajectory_matrix = trajectory_points;

trajectory_timeseries = timeseries(trajectory_matrix, 0:size(trajectory_matrix,1)-1);
trajectory_timeseries.Name = 'Pure Pursuit Trajectory';

initial_pose = [globalWaypoints(1, 1:2), globalWaypoints(1, 3), 0]; 
final_pose = [globalWaypoints(end, 1:2), globalWaypoints(end, 3), 0]; 

% =======================================================================
% === FULL RESCUE =============================================
% =======================================================================

save('pure_pursuit_trajectory.mat', ...
     'trajectory_points', 'trajectory_matrix', 'trajectory_timeseries', ...
     'pp_params', 'pp_params_bus', 'globalWaypoints', 'globalWaypoints_raw', ...
     'initial_pose', 'final_pose', 'velocities', 'curvature', ...
     'initialPose', 'finalGoal', '-v7.3');

save('pure_pursuit_parameters.mat', 'pp_params', '-v7.3');


% =======================================================================
% === VALIDATION AND DIAGNOSTICS ========================================
% =======================================================================

max_v = max(velocities);
min_spacing = min(diff(sqrt(diff(globalWaypoints(:,1)).^2 + diff(globalWaypoints(:,2)).^2)));
max_curvature = max(abs(curvature));

avg_spacing = mean(diff(sqrt(diff(globalWaypoints(:,1)).^2 + diff(globalWaypoints(:,2)).^2)));
min_lookahead = pp_params.lookahead_min;

if min_lookahead/avg_spacing < 3
    warning('Density waypoints may be insufficient for minimum lookahead!');
end

if abs(velocities(1)) > 1e-6 || abs(velocities(end)) > 1e-6
    warning('Initial or final speed is not zero!');
end


function curvature = calculatePathCurvature(waypoints)
 % Calculates the approximate curvature of the path
 % Uses the discrete curvature formula
    
    n = size(waypoints, 1);
    curvature = zeros(n, 1);
    
    for i = 2:n-1
        
        p1 = waypoints(i-1, 1:2);
        p2 = waypoints(i, 1:2);
        p3 = waypoints(i+1, 1:2);
        
        
        v1 = p2 - p1;
        v2 = p3 - p2;
        
        
        cross_prod = v1(1)*v2(2) - v1(2)*v2(1);
        norm_v1 = norm(v1);
        norm_v2 = norm(v2);
        
        if norm_v1 > 1e-6 && norm_v2 > 1e-6
            curvature(i) = 2 * cross_prod / (norm_v1 * norm_v2 * (norm_v1 + norm_v2));
        else
            curvature(i) = 0;
        end
    end
    
    
    curvature(1) = curvature(2);
    curvature(end) = curvature(end-1);
end


function velocities = generateImprovedVelocityProfile(waypoints, v_cruise, v_curve, curvature)
    n = size(waypoints, 1);
    
    
    distances = zeros(n, 1);
    for i = 2:n
        distances(i) = distances(i-1) + norm(waypoints(i,1:2) - waypoints(i-1,1:2));
    end
    total_distance = distances(end);
    
   
    velocities = ones(n, 1) * v_cruise;
    
   
    high_curvature_mask = abs(curvature) > 0.1;
    velocities(high_curvature_mask) = v_curve;
    
   
    accel_distance = min(10.0, total_distance * 0.2); 
    decel_distance = min(10.0, total_distance * 0.2); 
    
  
    for i = 1:n
        if distances(i) < accel_distance
            
            s = distances(i) / accel_distance;
            
            factor = (tanh(6*(s-0.5)) + 1) / 2;
            velocities(i) = velocities(i) * factor;
        else
            break;
        end
    end
    
    
    for i = n:-1:1
        dist_from_end = total_distance - distances(i);
        if dist_from_end < decel_distance
            
            s = dist_from_end / decel_distance;
            
            factor = (tanh(6*(s-0.5)) + 1) / 2;
            velocities(i) = velocities(i) * factor;
        else
            break;
        end
    end
    
    
    zero_points_start = min(5, round(n * 0.01)); 
    zero_points_end = min(5, round(n * 0.01));   
    
    velocities(1:zero_points_start) = 0;
    velocities(end-zero_points_end+1:end) = 0;
    
    
    transition_points = min(10, round(n * 0.02)); 
    
    if zero_points_start + transition_points <= n
        for i = 1:transition_points
            idx = zero_points_start + i;
            factor = i / transition_points;
            velocities(idx) = velocities(idx) * factor * factor; 
        end
    end
    
    if n - zero_points_end - transition_points >= 1
        for i = 1:transition_points
            idx = n - zero_points_end - transition_points + i;
            factor = (transition_points - i + 1) / transition_points;
            velocities(idx) = velocities(idx) * factor * factor; 
        end
    end
    
   
    velocities = smoothdata(velocities, 'gaussian', max(5, round(n/100)));
    
    
    velocities(1:zero_points_start) = 0;
    velocities(end-zero_points_end+1:end) = 0;
    
    
    velocities = max(0, velocities); 
end