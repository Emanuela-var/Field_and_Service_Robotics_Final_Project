function trajectory = generateImprovedTrajectory(globalWaypoints_raw, time_vector, L, controller_params)
% Improved trajectory generation with zero initial/final velocity
% and better time management for parking

    % Parameters for trajectory generation
    v_max = 1.5;  
    v_min = 0.0;
    max_curvature = 0.25;  
    
    % 1. PRE-PROCESSING: Remove waypoints that are too close together
    min_distance = 0.3;  
    filtered_waypoints = filterCloseWaypoints(globalWaypoints_raw, min_distance);
    
    % 2. ADVANCED SMOOTHING WITH CURVATURE CONSTRAINTS
    smoothed_waypoints = smoothPathWithCurvatureConstraint(filtered_waypoints, max_curvature);
    
    % 3. CALCULATE PATH LENGTH AND CURVATURES
    distances = computeDistances(smoothed_waypoints);
    curvatures = calculateCurvature(smoothed_waypoints);
    
    % 4. GENERATE VELOCITY PROFILE WITH ZERO ENDS
    velocities = generateSmoothVelocityProfile(distances, curvatures, v_max);
    
    % 5. COMPUTE ACTUAL TIME FOR EACH WAYPOINT
    [time_waypoints, actual_duration] = computeTimeProfile(distances, velocities);
    
    % 6. GENERATE TIMED TRAJECTORY
    trajectory = generateTimedTrajectory(smoothed_waypoints, time_waypoints, ...
                                       actual_duration, time_vector, L);
    
end

function filtered_waypoints = filterCloseWaypoints(waypoints, min_distance)
    filtered_waypoints = waypoints(1, :); 
    
    for i = 2:size(waypoints, 1)
        last_point = filtered_waypoints(end, 1:2);
        current_point = waypoints(i, 1:2);
        distance = norm(current_point - last_point);
        
        if distance >= min_distance
            filtered_waypoints = [filtered_waypoints; waypoints(i, :)];
        end
    end
end

function smoothed_waypoints = smoothPathWithCurvatureConstraint(waypoints, max_curvature)
    n_points = size(waypoints, 1);
    
    % Compute distances along path
    distances = computeDistances(waypoints);
    
    % Dense interpolation for smoothing
    n_interp = max(1000, n_points * 50);
    s_interp = linspace(0, distances(end), n_interp);
    
    % Smooth interpolation with pchip
    x_interp = interp1(distances, waypoints(:,1), s_interp, 'pchip');
    y_interp = interp1(distances, waypoints(:,2), s_interp, 'pchip');
    
    % Apply moving average filter for extra smoothness
    window_size = max(5, round(n_interp/100));
    x_smooth = smoothdata(x_interp, 'movmean', window_size);
    y_smooth = smoothdata(y_interp, 'movmean', window_size);
    
    % Compute orientation from smoothed path
    dx = gradient(x_smooth);
    dy = gradient(y_smooth);
    theta_smooth = atan2(dy, dx);
    
    % Remove high frequency components
    theta_smooth = smoothdata(unwrap(theta_smooth), 'movmean', window_size*2);
    
    % Check curvature
    curvature = calculateCurvatureFromDerivatives(x_smooth, y_smooth, dx, dy);
    
    % Resample to reasonable number of points
    n_final = min(500, n_interp);
    indices = round(linspace(1, n_interp, n_final));
    
    smoothed_waypoints = [x_smooth(indices)', y_smooth(indices)', theta_smooth(indices)'];
end

function distances = computeDistances(waypoints)
    % Compute cumulative distance along path
    distances = zeros(size(waypoints, 1), 1);
    for i = 2:length(distances)
        distances(i) = distances(i-1) + norm(waypoints(i,1:2) - waypoints(i-1,1:2));
    end
end

function curvature = calculateCurvatureFromDerivatives(x, y, dx, dy)
    ddx = gradient(dx);
    ddy = gradient(dy);
    
    numerator = abs(dx.*ddy - dy.*ddx);
    denominator = (dx.^2 + dy.^2).^(3/2);
    
    curvature = numerator ./ denominator;
    curvature(denominator < 1e-6) = 0;
    curvature(isnan(curvature)) = 0;
    curvature(isinf(curvature)) = 0;
end

function curvatures = calculateCurvature(waypoints)
    n = size(waypoints, 1);
    curvatures = zeros(n, 1);
    
    % Use three-point curvature formula
    for i = 2:n-1
        p1 = waypoints(i-1, 1:2);
        p2 = waypoints(i, 1:2);
        p3 = waypoints(i+1, 1:2);
        
        % Menger curvature
        a = norm(p2 - p1);
        b = norm(p3 - p2);
        c = norm(p3 - p1);
        
        if a > 1e-6 && b > 1e-6 && c > 1e-6
            s = (a + b + c) / 2;  % Semi-perimeter
            area = sqrt(max(0, s*(s-a)*(s-b)*(s-c)));  % Heron's formula
            curvatures(i) = 4 * area / (a * b * c);
        end
    end
    
    % Extrapolate endpoints
    curvatures(1) = curvatures(2);
    curvatures(end) = curvatures(end-1);
    
    % Smooth curvature
    curvatures = smoothdata(curvatures, 'movmean', min(10, round(n/10)));
end

function velocities = generateSmoothVelocityProfile(distances, curvatures, v_max)
    n = length(distances);
    velocities = zeros(n, 1);
    
    % Maximum lateral acceleration
    a_lat_max = 1.5;  % m/s^2
    
    % Velocity from curvature constraint
    for i = 1:n
        if curvatures(i) > 1e-6
            v_curve = sqrt(a_lat_max / curvatures(i));
            velocities(i) = min(v_max, v_curve);
        else
            velocities(i) = v_max;
        end
    end
    
    % Smooth the velocity profile
    velocities = smoothdata(velocities, 'movmean', max(5, round(n/20)));
    
    % Apply acceleration constraints
    a_max = 1.0;  % m/s^2
    d_max = 1.2;  % m/s^2 (deceleration)
    
    % Forward pass (acceleration limit)
    velocities(1) = 0;  % Start from zero
    for i = 2:n
        ds = distances(i) - distances(i-1);
        if ds > 1e-6
            v_max_accel = sqrt(velocities(i-1)^2 + 2*a_max*ds);
            velocities(i) = min(velocities(i), v_max_accel);
        end
    end
    
    % Backward pass (deceleration limit)
    velocities(end) = 0;  % End at zero
    for i = n-1:-1:1
        ds = distances(i+1) - distances(i);
        if ds > 1e-6
            v_max_decel = sqrt(velocities(i+1)^2 + 2*d_max*ds);
            velocities(i) = min(velocities(i), v_max_decel);
        end
    end
    
    % Final smoothing
    velocities = smoothdata(velocities, 'gaussian', max(10, round(n/30)));
    
    % Ensure zero at endpoints
    ramp_length = max(5, round(n/50));
    for i = 1:ramp_length
        factor = (i-1) / ramp_length;
        velocities(i) = velocities(i) * factor;
        velocities(n-i+1) = velocities(n-i+1) * factor;
    end
    
    velocities(1) = 0;
    velocities(end) = 0;
end

function [time_waypoints, actual_duration] = computeTimeProfile(distances, velocities)
    n = length(distances);
    time_waypoints = zeros(n, 1);
    
    % Integrate time along path
    for i = 2:n
        ds = distances(i) - distances(i-1);
        v_avg = 0.5 * (velocities(i) + velocities(i-1));
        if v_avg > 1e-6
            dt = ds / v_avg;
        else
            dt = 0;
        end
        time_waypoints(i) = time_waypoints(i-1) + dt;
    end
    
    actual_duration = time_waypoints(end);
end

function trajectory = generateTimedTrajectory(waypoints, time_waypoints, actual_duration, time_vector, L)
    % Remove duplicate time points
    [time_unique, unique_idx] = unique(time_waypoints);
    waypoints_unique = waypoints(unique_idx, :);
    
    % Initialize arrays
    n_time = length(time_vector);
    xd = zeros(n_time, 1);
    yd = zeros(n_time, 1);
    thetad = zeros(n_time, 1);
    xd_dot = zeros(n_time, 1);
    yd_dot = zeros(n_time, 1);
    thetad_dot = zeros(n_time, 1);
    phid = zeros(n_time, 1);
    velocity = zeros(n_time, 1);
    
    % Final state
    final_state = waypoints_unique(end, :);
    
    % Generate trajectory for each time point
    dt = time_vector(2) - time_vector(1);
    
    for i = 1:n_time
        t = time_vector(i);
        
        if t <= actual_duration
            % Interpolate position during motion
            xd(i) = interp1(time_unique, waypoints_unique(:,1), t, 'pchip');
            yd(i) = interp1(time_unique, waypoints_unique(:,2), t, 'pchip');
            thetad(i) = interp1(time_unique, waypoints_unique(:,3), t, 'pchip');
        else
            % Stay at final position after completion
            xd(i) = final_state(1);
            yd(i) = final_state(2);
            thetad(i) = final_state(3);
        end
    end
    
    % Compute velocities using gradient
    xd_dot = gradient(xd, dt);
    yd_dot = gradient(yd, dt);
    
    % Set velocities to zero after trajectory completion
    completion_idx = find(time_vector > actual_duration, 1);
    if ~isempty(completion_idx)
        xd_dot(completion_idx:end) = 0;
        yd_dot(completion_idx:end) = 0;
    end
    
    % Smooth velocities
    xd_dot = smoothdata(xd_dot, 'gaussian', 5);
    yd_dot = smoothdata(yd_dot, 'gaussian', 5);
    
    % Enforce zero velocity at start and end
    xd_dot(1:3) = 0;
    yd_dot(1:3) = 0;
    xd_dot(end-2:end) = 0;
    yd_dot(end-2:end) = 0;
    
    % Compute angular velocity
    thetad_unwrapped = unwrap(thetad);
    thetad_dot = gradient(thetad_unwrapped, dt);
    thetad_dot = smoothdata(thetad_dot, 'gaussian', 5);
    
    % Zero angular velocity when stopped
    thetad_dot(xd_dot.^2 + yd_dot.^2 < 1e-6) = 0;
    if ~isempty(completion_idx)
        thetad_dot(completion_idx:end) = 0;
    end
    
    % Compute linear velocity and steering angle
    velocity = sqrt(xd_dot.^2 + yd_dot.^2);
    
    for i = 1:n_time
        if velocity(i) > 0.05
            phid(i) = atan(L * thetad_dot(i) / velocity(i));
        else
            phid(i) = 0;
        end
    end
    
    % Limit and smooth steering angle
    max_phi = pi/6;  % 30 degrees
    phid = max(-max_phi, min(max_phi, phid));
    phid = smoothdata(phid, 'gaussian', 5);
    
    % Store results
    trajectory.xd = xd;
    trajectory.yd = yd;
    trajectory.thetad = wrapToPi(thetad);
    trajectory.phid = phid;
    trajectory.xd_dot = xd_dot;
    trajectory.yd_dot = yd_dot;
    trajectory.thetad_dot = thetad_dot;
    trajectory.velocity = velocity;
    trajectory.actual_duration = actual_duration;
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end