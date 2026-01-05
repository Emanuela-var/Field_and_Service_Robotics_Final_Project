function outputArgs = createSimulinkStructs(inputObject)
% createSimulinkStructs: Converts specific MATLAB objects (like
% vehicleDimensions or vehicleCostmap) into structs suitable for Simulink.
% This function relies on 'copyObjectFieldsToStruct.m'.

if isa(inputObject, 'vehicleDimensions')

    % Handle a vehicleDimensions object
    vehicleDimensionsStruct = struct;
    vehicleDimensionsStruct = copyObjectFieldsToStruct(vehicleDimensionsStruct, inputObject);
    % Convert WorldUnits to a compatible integer type for Simulink
    vehicleDimensionsStruct.WorldUnits = uint8(vehicleDimensionsStruct.WorldUnits);

    outputArgs{1} = vehicleDimensionsStruct;

elseif isa(inputObject, 'vehicleCostmap')

    % Handle a vehicleCostmap object, which produces two structs

    % 1. Create the costmap struct
    costmapDataStruct = struct;
    costmapDataStruct = copyObjectFieldsToStruct(costmapDataStruct, inputObject);
    % The CollisionChecker object itself cannot be passed, so remove it.
    costmapDataStruct = rmfield(costmapDataStruct, 'CollisionChecker');
    % Manually add necessary properties from the CollisionChecker
    costmapDataStruct.InflationRadius = inputObject.CollisionChecker.InflationRadius;
    % Extract the cost matrix
    costmapDataStruct.Costs = getCosts(inputObject);

    % 2. Create the vehicle dimensions struct from the costmap's checker
    vehicleDimensionsStruct = struct;
    vehicleDimensionsStruct = copyObjectFieldsToStruct(vehicleDimensionsStruct, inputObject.CollisionChecker.VehicleDimensions);
    vehicleDimensionsStruct.WorldUnits = uint8(vehicleDimensionsStruct.WorldUnits);

    % Assign to the output cell array
    outputArgs{1} = vehicleDimensionsStruct;
    outputArgs{2} = costmapDataStruct;

else
    % Throw an error if the object type is not supported
    error('createSimulinkStructs: Invalid input object type. Only vehicleDimensions and vehicleCostmap are supported.');
end

end