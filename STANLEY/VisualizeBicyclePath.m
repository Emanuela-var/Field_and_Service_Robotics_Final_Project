function VisualizeBicyclePath(pose, steer, refPoses, costmapStruct, vehicleDimsStruct, varargin)
%helperSLVisualizeBicyclePath visualizes the reference path and bicycle position.

% Handling optional inputs for parking maneuver
if nargin > 5
    isParkingManeuver = varargin{1};
    parkCostmapSize   = varargin{2};
else
    isParkingManeuver = false;
    parkCostmapSize   = 0;
end

% Persistent variables to avoid reinitializing at each call from Simulink
persistent pathPoints vehicleDims costmap vehicleBodyHandle axesHandle isParkingMode

% Initialization at first simulation step
if isempty(costmap)
    % Reconstructs the vehicleDimensions object from struct passed by Simulink
    vehicleDims = vehicleDimensions( ...
        vehicleDimsStruct.Length, ...
        vehicleDimsStruct.Width, ...
        vehicleDimsStruct.Height, ...
        'Wheelbase',        vehicleDimsStruct.Wheelbase, ...
        'RearOverhang',     vehicleDimsStruct.RearOverhang, ...
        'WorldUnits',       char(vehicleDimsStruct.WorldUnits));

    % Reconstructs the vehicleCostmap object from struct
    ccConfig = inflationCollisionChecker(vehicleDims);
    ccConfig.InflationRadius = costmapStruct.InflationRadius; % Uses radius from struct

    costmap = vehicleCostmap(costmapStruct.Costs, ...
        'FreeThreshold',     costmapStruct.FreeThreshold, ...
        'OccupiedThreshold', costmapStruct.OccupiedThreshold, ...
        'MapLocation',       costmapStruct.MapExtent([1, 3]), ...
        'CellSize',          costmapStruct.CellSize, ...
        'CollisionChecker',  ccConfig);
end

% Initialization of other persistent variables
if isempty(pathPoints)
    pathPoints = zeros(size(refPoses, 1), 2);
end
if isempty(isParkingMode)
    isParkingMode = false;
end
if isempty(axesHandle)
    axesHandle = 0;
end

% ALWAYS check if handle is valid, before any drawing operation.
if isempty(axesHandle) || ~isgraphics(axesHandle, 'axes')

    % If handle is not valid, create a new figure and a new drawing area.
    fh1=figure;
    fh1.Name        = 'Automated Parking Valet - Hybrid Planner';
    fh1.NumberTitle = 'off';
    axesHandle      = axes(fh1); % Save the new valid handle
    plot(costmap, 'Parent', axesHandle, 'Inflation', 'on');
    legend off
    axis tight
    title(axesHandle, 'Global Map and Path');
    hold(axesHandle, 'on');

    % Force path redraw on new figure
    pathPoints = zeros(size(refPoses, 1), 2); 
end

% Draw path only if it has changed
if ~isequal(pathPoints, refPoses(:,1:2))
    plot(axesHandle, refPoses(:,1), refPoses(:,2),'b', 'LineWidth', 2, 'DisplayName', 'Local Path');
    pathPoints = refPoses(:,1:2);
end

% First check if bicycle graphic handles exist and are valid
if ~isempty(vehicleBodyHandle) && all(isgraphics(vehicleBodyHandle))
    delete(vehicleBodyHandle); % Delete old bicycle
end

% Draw new bicycle in its current position
vehicleBodyHandle = PlotBicycle(pose, vehicleDims, steer, 'Parent', axesHandle);
% -------------------------------------------------------------------

% Logic to visualize parking maneuver i
if isParkingManeuver && ~isParkingMode 
    isParkingMode   = isParkingManeuver;
    fh2=figure;
    fh2.Name        = 'Parking Maneuver';
    fh2.NumberTitle = 'off';
    localMapAxes    = axes(fh2);
    plot(costmap, 'Parent', localMapAxes, 'Inflation', 'on');
    legend off
    axis tight
    title(localMapAxes, 'Local Parking Map');
    hold(localMapAxes, 'on');

    plot(localMapAxes, refPoses(:,1), refPoses(:,2),'b', 'LineWidth', 2);

    PlotBicycle(pose, vehicleDims, steer, 'Parent', localMapAxes);

    localMapAxes.XLim = [pose(1)-parkCostmapSize/2, pose(1)+parkCostmapSize/2];
    localMapAxes.YLim = [pose(2)-parkCostmapSize/2, pose(2)+parkCostmapSize/2];
end

set(axesHandle, 'DataAspectRatio', [1 1 1]);
% Force graphic window update
drawnow('limitrate');
end