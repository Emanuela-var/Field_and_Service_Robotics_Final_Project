function varargout = PlotBicycle(varargin)
%PlotBicycle draws a bicycle using vehicle dimensions.
%
%   PlotBicycle(bicyclePose, vehicleDims) draws a bicycle on the current
%   axis. The pose is specified as [x,y,theta] and the dimensions
%   are provided as a vehicleDimensions object. The function extracts the
%   Wheelbase and estimates the wheel radius for visualization.
%
%   PlotBicycle(bicyclePose, vehicleDims, steer) also orients the
%   front wheel using the specified steering angle (in degrees).
%
%   PlotBicycle(...,Name,Value) specifies additional arguments
%   as name-value pairs.
%
%   'Parent'        Handle of the axis on which to display the bicycle.
%
%   'Color'         Color of the bicycle frame.
%
%   'DisplayName'   Name for the legend entry.
%

[bicyclePose, vehicleDims, steer, parent, displayName, color] = parseInputs(varargin{:});

% Convert the vehicleDimensions object to a struct suitable for the bicycle for visualization only.
bicycleGeom.Wheelbase = vehicleDims.Wheelbase;
% Estimate a reasonable wheel radius based on wheelbase for visualization
bicycleGeom.WheelRadius = vehicleDims.Wheelbase / 3; 

% Generate the geometric shapes for the bicycle
bicycleShapes = helperBicyclePolyshape(bicyclePose, bicycleGeom, steer);
frameShape = bicycleShapes(1);
wheels     = bicycleShapes(2:end);

% Perform the plotting
hAx = newplot(parent);
if isempty(color)
    % Get the next color from ColorOrder
    color = hAx.ColorOrder( hAx.ColorOrderIndex,: );
    if hAx.ColorOrderIndex < size(hAx.ColorOrder, 1)
        hAx.ColorOrderIndex = hAx.ColorOrderIndex+1;
    else
        hAx.ColorOrderIndex = 1;
    end
end

% Check hold state
if ishold(hAx)
    oldState = 'on';
else
    oldState = 'off';
end

% Activate hold
hold(hAx, 'on')
restoreHoldState = onCleanup(@()hold(hAx, oldState));

% Draw the shapes
% NOTE: Now the frame is a rectangle, so 'LineWidth' is no longer needed
hShape = plot(hAx, frameShape, 'DisplayName', displayName, ...
    'FaceColor', color, 'EdgeColor', color);

% Wheels are always black
hWheels = plot(hAx, wheels , 'FaceColor', 'k', 'EdgeColor', 'k');

% Add tags and userdata
hShape.Tag = 'bicycleFrame';
hShape.UserData = [bicyclePose steer];
[hWheels.Tag] = deal('bicycleWheels');

setupLegend(hShape, hWheels, displayName);

if nargout>0
    varargout{1} = [hShape ; hWheels];
end
end

%--------------------------------------------------------------------------
function bicycleShapes = helperBicyclePolyshape(pose, geom, steer)
%helperBicyclePolyshape creates polyshape objects for a bicycle.

% Decomposition of inputs
x = pose(1);
y = pose(2);
theta = pose(3);

wheelbase   = geom.Wheelbase;
wheelRadius = geom.WheelRadius;

% Convert angles to radians
thetaRad = deg2rad(theta);
steerRad = deg2rad(steer);

% Frame: Represented as a thin rectangle to create a valid polyshape
frameThickness = geom.WheelRadius / 10; % Visual thickness of the frame
halfThick = frameThickness / 2;
frame_x = [0,         wheelbase, wheelbase, 0        ];
frame_y = [-halfThick, -halfThick, halfThick, halfThick];
frame = polyshape(frame_x, frame_y);

% Wheels: approximated as 20-sided polygons
t = linspace(0, 2*pi, 21); % Create 21 points to include the endpoint
t = t(1:end-1);           % Discard the last point, which is a duplicate of the first
wheel_x = wheelRadius * cos(t);
wheel_y = wheelRadius * sin(t);
wheelPoly = polyshape(wheel_x, wheel_y);

% Rear wheel (at the origin of the bicycle reference system)
rearWheel = wheelPoly;

% Front wheel (at wheelbase distance and steered)
frontWheel = rotate(wheelPoly, steer);
frontWheel = translate(frontWheel, [wheelbase, 0]);

% Combine parts before global transformation
bicycleParts = [frame; rearWheel; frontWheel];

% Apply global rotation and translation to all parts
bicycleParts = rotate(bicycleParts, theta, [0, 0]);
bicycleShapes = translate(bicycleParts, [x, y]);
end

%--------------------------------------------------------------------------
function setupLegend(hObjLeg, hObjNonLeg, displayName)
if isempty(displayName)
    hObjNonLeg = [hObjLeg(:); hObjNonLeg(:)];
else
    [hObjLeg.DisplayName] = deal(displayName);
end
turnOffLegend(hObjNonLeg);
end

%--------------------------------------------------------------------------
function turnOffLegend(hObj)
for n = 1 : numel(hObj)
    hObj(n).Annotation.LegendInformation.IconDisplayStyle = 'off';
end
end

%--------------------------------------------------------------------------
% Input parsing
%--------------------------------------------------------------------------
function [pose, dims, steer, parent, name, color] = parseInputs(varargin)
p = inputParser;
p.FunctionName = mfilename;
p.addRequired('bicyclePose',        @validatePose);
p.addRequired('vehicleDims',        @validateVehicleDimensions);
p.addOptional('steer',          0,  @validateSteer);
p.addParameter('DisplayName',   '', @validateDisplayName);
p.addParameter('Parent',        [], @validateParent);
p.addParameter('Color',         '', @validateColor);
p.parse(varargin{:});
res = p.Results;
pose    = res.bicyclePose;
dims    = res.vehicleDims;
steer   = res.steer;
parent  = res.Parent;
name    = res.DisplayName;
color   = res.Color;
end

%--------------------------------------------------------------------------
function validatePose(pose)
validateattributes(pose, {'single', 'double'}, ...
    {'real', 'row', 'numel', 3, 'finite'}, mfilename, 'bicyclePose');
end

%--------------------------------------------------------------------------
function validateVehicleDimensions(dims)
% Validate that the input is a vehicleDimensions object
validateattributes(dims, {'vehicleDimensions'}, {'scalar'}, mfilename, ...
    'vehicleDims');
end

%--------------------------------------------------------------------------
function validateSteer(steer)
validateattributes(steer, {'single','double'}, {'real','scalar','finite'}, ...
    mfilename, 'steer');
end

%--------------------------------------------------------------------------
function validateDisplayName(name)
validateattributes(name, {'char','string'}, {'scalartext'}, ...
    mfilename, 'DisplayName');
end

%--------------------------------------------------------------------------
function tf = validateParent(parent)
tf = true;
if isempty(parent)
    return;
end
if ~ishghandle(parent) || ~strcmp(get(parent,'Type'), 'axes')
    error(message('driving:pathPlannerRRT:validParent'))
end
end

%--------------------------------------------------------------------------
function validateColor(color)
if isempty(color)
    return;
end
if ischar(color) || isstring(color)
    validateattributes(color, {'char','string'}, ...
        {'nonempty','scalartext'}, 'plot', 'Color');
    
    specOptions = {'red','green','blue','yellow','magenta',...
        'cyan','white','black','r','g','b','y','m','c','w','k'};
    
    validatestring(color, specOptions, 'plot', 'Color');
else
    validateattributes(color, {'double'}, ...
        {'nonempty','>=', 0, '<=', 1, 'size', [1 3]}, ...
        'plot', 'Color');
end
end