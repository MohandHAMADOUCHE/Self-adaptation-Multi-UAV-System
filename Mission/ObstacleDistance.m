function obstacle_distance = ObstacleDistance(SensorDetectionState,SensorDetectedPoint)
    obstacle_distance = 1;    

    if SensorDetectionState==1
        obstacle_distance = norm(SensorDetectedPoint);
    end
end