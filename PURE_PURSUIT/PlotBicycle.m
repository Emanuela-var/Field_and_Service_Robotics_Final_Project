function varargout = PlotBicycle(varargin)
%PlotBicycle draws a bicycle using the dimensions of a vehicle.
%
% PlotBicycle(bicyclePose, vehicleDims) draws a bicycle on the current
% axis. The pose is specified as [x,y,theta] and the dimensions
% are given as a vehicleDimensions object. The function extracts the
% Wheelbase and estimates the wheel radius for display.
%
% PlotBicycle(bicyclePose, vehicleDims, steer) also orients the
% front wheel using the specified steering angle (in degrees).
%
% PlotBicycle(...,Name,Value) specifies additional arguments
% as name-value pairs.
%
% 'Parent' Handle of the axis on which to display the bicycle.
%
% 'Color' Colour of the bicycle frame.
%
% 'DisplayName' Name for the item in the legend.
%

[bicyclePose, vehicleDims, steer, parent, displayName, color] = parseInputs(varargin{:});

bicycleGeom.Wheelbase = vehicleDims.Wheelbase;

bicycleGeom.WheelRadius = vehicleDims.Wheelbase / 3; 


bicycleShapes = helperBicyclePolyshape(bicyclePose, bicycleGeom, steer);
frameShape = bicycleShapes(1);
wheels     = bicycleShapes(2:end);


hAx = newplot(parent);
if isempty(color)
    
    color = hAx.ColorOrder( hAx.ColorOrderIndex,: );
    if hAx.ColorOrderIndex < size(hAx.ColorOrder, 1)
        hAx.ColorOrderIndex = hAx.ColorOrderIndex+1;
    else
        hAx.ColorOrderIndex = 1;
    end
end


if ishold(hAx)
    oldState = 'on';
else
    oldState = 'off';
end


hold(hAx, 'on')
restoreHoldState = onCleanup(@()hold(hAx, oldState));


hShape = plot(hAx, frameShape, 'DisplayName', displayName, ...
    'FaceColor', color, 'EdgeColor', color);


hWheels = plot(hAx, wheels , 'FaceColor', 'k', 'EdgeColor', 'k');


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
%


x = pose(1);
y = pose(2);
theta = pose(3);

wheelbase   = geom.Wheelbase;
wheelRadius = geom.WheelRadius;


thetaRad = deg2rad(theta);
steerRad = deg2rad(steer);

frameThickness = geom.WheelRadius / 10; 
halfThick = frameThickness / 2;
frame_x = [0,         wheelbase, wheelbase, 0        ];
frame_y = [-halfThick, -halfThick, halfThick, halfThick];
frame = polyshape(frame_x, frame_y);

t = linspace(0, 2*pi, 21); 
t = t(1:end-1);           
wheel_x = wheelRadius * cos(t);
wheel_y = wheelRadius * sin(t);
wheelPoly = polyshape(wheel_x, wheel_y);

rearWheel = wheelPoly;

frontWheel = rotate(wheelPoly, steer);
frontWheel = translate(frontWheel, [wheelbase, 0]);


bicycleParts = [frame; rearWheel; frontWheel];


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
%
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