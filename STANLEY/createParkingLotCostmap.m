function costmap = createParkingLotCostmap()
% createParkingLotCostmap creates a costmap for a parking lot

% 1. Load the image from the file
colorImage = imread('my_map.bmp');

% 2. Convert the image from RGB to grayscale (from 3D to 2D)
if size(colorImage, 3) == 3
    grayImage = rgb2gray(colorImage);
else
    % If the image is already grayscale, use it directly.
    grayImage = colorImage;
end

% 3. Convert the map to single-precision format 
combinedMap = im2single(grayImage);

%% 4. Set the parameters and create the costmap.
cellSize = 0.1; 
vehicleDims = vehicleDimensions( ...
    'Length', 5, ...       
    'Width', 2.5, ...      
    'Wheelbase', 2.8, ... 
    'RearOverhang', 1.0);  

collisionChecker = inflationCollisionChecker(vehicleDims);
collisionChecker.InflationRadius = 0.5;

costmap = vehicleCostmap(combinedMap, ...
    'CellSize', cellSize, ...
    'CollisionChecker', collisionChecker);

end