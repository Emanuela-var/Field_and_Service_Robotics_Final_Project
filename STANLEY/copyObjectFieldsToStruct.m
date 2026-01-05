function targetStruct = copyObjectFieldsToStruct(targetStruct, sourceObject)
% copyObjectFieldsToStruct: A utility to copy public properties from an
% object to a struct, which is more compatible with Simulink interfaces.

% Get all public property names from the source object.
propNames = properties(sourceObject);

% Loop through each property and assign its value to the target struct.
for i = 1:numel(propNames)
    fieldName = propNames{i};
    targetStruct.(fieldName) = sourceObject.(fieldName);
end

end