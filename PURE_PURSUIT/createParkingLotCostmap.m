function costmap = createParkingLotCostmap()

colorImage = imread('my_map.bmp');

if size(colorImage, 3) == 3
    grayImage = rgb2gray(colorImage);
else
    
    grayImage = colorImage;
end

combinedMap = im2single(grayImage);

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