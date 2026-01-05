function UtilityBus()
% UtilityBus: Defines all the Simulink Bus objects necessary for the project, with a robust and unconditional structure.
fprintf('Creating and loading System Bus definitions...\n');
%% 1. Bus for Speed Configuration (speedConfigBus)
clear elems;
elems(1) = Simulink.BusElement; elems(1).Name = 'StartSpeed';
elems(2) = Simulink.BusElement; elems(2).Name = 'EndSpeed';
elems(3) = Simulink.BusElement; elems(3).Name = 'MaxSpeed';
% Set default properties for all elements
for i=1:numel(elems)
 elems(i).Dimensions = 1;
 elems(i).DataType = 'double';
end
speedConfigBus = Simulink.Bus;
speedConfigBus.Elements = elems;
%% 2. Bus for Planner Configuration (plannerConfigBus)
clear elems;
elems(1) = Simulink.BusElement; elems(1).Name = 'ConnectionDistance';
elems(2) = Simulink.BusElement; elems(2).Name = 'MinIterations';
elems(3) = Simulink.BusElement; elems(3).Name = 'GoalTolerance'; elems(3).Dimensions = [1 3];
elems(4) = Simulink.BusElement; elems(4).Name = 'MinTurningRadius';
elems(5) = Simulink.BusElement; elems(5).Name = 'IsParkManeuver'; elems(5).DataType = 'boolean'; % Made unconditional
% Set default properties for all elements
for i=1:numel(elems)
if isempty(elems(i).Dimensions)
 elems(i).Dimensions = 1;
end
if isempty(elems(i).DataType)
 elems(i).DataType = 'double';
end
end
plannerConfigBus = Simulink.Bus;
plannerConfigBus.Elements = elems;
%% 3. Bus for Vehicle State Information (vehicleInfoBus)
clear elems;
elems(1) = Simulink.BusElement; elems(1).Name = 'CurrPose'; elems(1).Dimensions = [1 3];
elems(2) = Simulink.BusElement; elems(2).Name = 'CurrVelocity';
elems(3) = Simulink.BusElement; elems(3).Name = 'CurrYawRate';
elems(4) = Simulink.BusElement; elems(4).Name = 'CurrSteer';
elems(5) = Simulink.BusElement; elems(5).Name = 'Direction';
% Set default properties for all elements
for i=1:numel(elems)
if isempty(elems(i).Dimensions)
 elems(i).Dimensions = 1;
end
 elems(i).DataType = 'double';
end
vehicleInfoBus = Simulink.Bus;
vehicleInfoBus.Elements = elems;
%% 4. Assignment of ALL Buses to Base Workspace
assignin('base','speedConfigBus', speedConfigBus);
assignin('base','plannerConfigBus', plannerConfigBus);
assignin('base','vehicleInfoBus', vehicleInfoBus);
fprintf('All System Bus definitions have been created and loaded.\n');
end