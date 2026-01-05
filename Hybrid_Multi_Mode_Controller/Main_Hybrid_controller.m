clear;
clc;
close all;

% 1. DATA UPLOAD AND SETUP
rng(110);
data = load('GlobalRoutePlan.mat');
routePlan = data.routePlan;
costmap = createParkingLotCostmap();

L = 2.8; % wheelbase [m]

% Controller Parameters 
controller_params.wheelbase = 2.8;      % wheelbase [m]
controller_params.k1 = 2.0;             % position gain 
controller_params.k2 = 4.0;             % orientation gain 
controller_params.k3 = 1.5;             % lateral gain
controller_params.v_max = 2.0;          % max velocity [m/s] 
controller_params.v_min = 0.0;          % min velocity [m/s]
controller_params.phi_max = pi/4;       % max steer angle [rad] 

% Bus Object Definition for Parameters
fprintf('Creazione del Bus Object per i parametri del controllore...\n');
elems(1) = Simulink.BusElement; elems(1).Name = 'wheelbase'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'k1';        elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'k2';        elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'k3';        elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'v_max';     elems(5).DataType = 'double';
elems(6) = Simulink.BusElement; elems(6).Name = 'v_min';     elems(6).DataType = 'double';
elems(7) = Simulink.BusElement; elems(7).Name = 'phi_max';   elems(7).DataType = 'double';
controller_params_bus = Simulink.Bus;
controller_params_bus.Elements = elems;
clear elems;

% Time parameters
dt = 0.1; % time step [s]
T_total = 200; % Total simulation time [s] 
time_vector = 0:dt:T_total;

% 2. GENERATE GLOBAL PATH
fprintf('Global path generation with RRT*...\n');

plannerRRTConfig.MinTurningRadius = 4.0;
plannerRRTConfig.ConnectionDistance = 30.0;
plannerRRTConfig.GoalTolerance = [0.5 0.5 10];
plannerRRTConfig.MinIterations = 2000;

motionPlannerRRT = pathPlannerRRT(costmap, ...
    'ConnectionDistance', plannerRRTConfig.ConnectionDistance, ...
    'MinIterations', plannerRRTConfig.MinIterations, ...
    'GoalTolerance', plannerRRTConfig.GoalTolerance, ...
    'MinTurningRadius', plannerRRTConfig.MinTurningRadius);

globalWaypoints_raw = [];
initialPose = routePlan{1, 'StartPose'};
finalGoal = routePlan{end, 'EndPose'};

for i = 1:height(routePlan)
    fprintf(' - Segment Planning %d...\n', i);
    startPose = routePlan{i, 'StartPose'};
    endPose = routePlan{i, 'EndPose'};
    
    segmentPathObject = plan(motionPlannerRRT, startPose, endPose);
    segmentPoses = interpolate(segmentPathObject);
    
    if isempty(segmentPoses)
        error('RRT* failed for the segment %d.', i);
    end
    
    if i == 1
        globalWaypoints_raw = [globalWaypoints_raw; segmentPoses];
    else
        globalWaypoints_raw = [globalWaypoints_raw; segmentPoses(2:end, :)];
    end
end

% PATH SMOOTHING
fprintf('Global path smoothing with splines...\n');
if size(globalWaypoints_raw,1) > 1
    waypoints_interp = interp1(1:size(globalWaypoints_raw,1), globalWaypoints_raw, ...
                              linspace(1,size(globalWaypoints_raw,1),200));
else
    waypoints_interp = globalWaypoints_raw;
end

path_spline = cscvn(waypoints_interp(:, 1:2)');

num_smooth_points = 300;
t_spline = linspace(path_spline.breaks(1), path_spline.breaks(end), num_smooth_points);
smooth_points_xy = fnval(path_spline, t_spline)';

path_derivative = fnder(path_spline);
smooth_derivatives = fnval(path_derivative, t_spline);
smooth_theta = atan2(smooth_derivatives(2,:), smooth_derivatives(1,:))';

globalWaypoints = [smooth_points_xy, smooth_theta];

% 3. TIMED DESIRED TRAJECTORY GENERATION
fprintf('Timed desired trajectory generation...\n');
trajectory = generateImprovedTrajectory(globalWaypoints_raw, time_vector, L, controller_params);

% 4. TRAJECTORY SAVING FOR SIMULINK
% Create timeseries for each signal
ts_xd = timeseries(trajectory.xd, time_vector);
ts_yd = timeseries(trajectory.yd, time_vector);
ts_thetad = timeseries(trajectory.thetad, time_vector);
ts_phid = timeseries(trajectory.phid, time_vector);
ts_xd_dot = timeseries(trajectory.xd_dot, time_vector);
ts_yd_dot = timeseries(trajectory.yd_dot, time_vector);
ts_thetad_dot = timeseries(trajectory.thetad_dot, time_vector);

% Save initial pose for Simulink
initialPose = [trajectory.xd(1), trajectory.yd(1), trajectory.thetad(1)];

% Save both trajectory structure and timeseries
save('desired_trajectory.mat', ...
     'trajectory', 'time_vector', 'controller_params', 'controller_params_bus', ...
     'ts_xd', 'ts_yd', 'ts_thetad', 'ts_phid', 'ts_xd_dot', 'ts_yd_dot', 'ts_thetad_dot', ...
     'initialPose', '-v7.3');

fprintf('\nFile "desired_trajectory.mat" creato con successo.\n');
fprintf('Effective course duration: %.2f secondi\n', trajectory.actual_duration);
fprintf('Initial Pose: [%.2f, %.2f, %.2f°]\n', initialPose(1), initialPose(2), initialPose(3)*180/pi);
fprintf('End Pose: [%.2f, %.2f, %.2f°]\n', trajectory.xd(end), trajectory.yd(end), trajectory.thetad(end)*180/pi);
fprintf('Initial Velocty: v=%.3f m/s\n', sqrt(trajectory.xd_dot(1)^2 + trajectory.yd_dot(1)^2));
fprintf('Final Velocity: v=%.3f m/s\n', sqrt(trajectory.xd_dot(end)^2 + trajectory.yd_dot(end)^2));

% % Plot di verifica della traiettoria generata
% figure('Name', 'Verifica Traiettoria Generata');
% 
% subplot(2,2,1);
% plot(trajectory.xd, trajectory.yd, 'b-', 'LineWidth', 2);
% hold on;
% plot(trajectory.xd(1), trajectory.yd(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% plot(trajectory.xd(end), trajectory.yd(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% axis equal;
% grid on;
% xlabel('X [m]');
% ylabel('Y [m]');
% title('Traiettoria Desiderata');
% legend('Traiettoria', 'Start', 'Goal');
% 
% subplot(2,2,2);
% v_profile = sqrt(trajectory.xd_dot.^2 + trajectory.yd_dot.^2);
% plot(time_vector, v_profile, 'b-', 'LineWidth', 2);
% hold on;
% % Evidenzia la durata effettiva del percorso
% if isfield(trajectory, 'actual_duration')
%     xline(trajectory.actual_duration, 'r--', 'LineWidth', 2);
%     text(trajectory.actual_duration, max(v_profile)*0.8, 'Fine percorso', ...
%          'HorizontalAlignment', 'right');
% end
% grid on;
% xlabel('Tempo [s]');
% ylabel('Velocità [m/s]');
% title('Profilo di Velocità');
% ylim([0, max(v_profile)*1.1]);
% 
% subplot(2,2,3);
% plot(time_vector, trajectory.thetad * 180/pi, 'b-', 'LineWidth', 2);
% grid on;
% xlabel('Tempo [s]');
% ylabel('Orientamento [°]');
% title('Profilo di Orientamento');
% 
% subplot(2,2,4);
% plot(time_vector, trajectory.phid * 180/pi, 'b-', 'LineWidth', 2);
% grid on;
% xlabel('Tempo [s]');
% ylabel('Angolo di Sterzo [°]');
% title('Profilo Angolo di Sterzo');
% ylim([-35, 35]);

function angle_diff = angdiff(angle1, angle2)
    angle_diff = angle1 - angle2;
    angle_diff = atan2(sin(angle_diff), cos(angle_diff));
end