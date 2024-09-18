function  [returnCode,positionUAV]= Obstacle_avoidance (sim, clientID, UAV_Target_handle,SensorHandle, positionUAV)
    returnCode = -2;
    max_threshold = 5; % distance max to object 
    min_threshold = 0.5; % distance max to object 
    min_threshold_l = 0.5; % distance max to object 

    obstacle = true; %initialisation obstacle  
    
    [~,SensorDetectionState_f,SensorDetectedPoint_f,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(1),sim.simx_opmode_buffer);
    if SensorDetectionState_f == 0 % no obstacle
        OD_f = max_threshold;
    else    %obstacle
        OD_f = ObstacleDistance(SensorDetectionState_f,SensorDetectedPoint_f);
    end
    [~,SensorDetectionState_l,SensorDetectedPoint_l,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(2),sim.simx_opmode_buffer);
    if SensorDetectionState_l == 0 % no obstacle
        OD_l = max_threshold;
    else    %obstacle
        OD_l = ObstacleDistance(SensorDetectionState_l,SensorDetectedPoint_l);
    end
    old_OD_f = OD_f-0.01;
    old_OD_l = OD_l-0.01;

    while (obstacle == true)
        %tuning of the motion to the front 
        if OD_f<=min_threshold 
            if OD_f < old_OD_f 
                cy=  positionUAV(2)-0.01;
            elseif OD_f > old_OD_f 
                cy=  positionUAV(2)+0.01;
            else
                cy=  positionUAV(2);
            end
        else
            cy=  positionUAV(2)-0.01;
        end
        
        if OD_l<=min_threshold_l 
            if  OD_l <= old_OD_l
                cx=  positionUAV(1)+0.01;
            elseif  OD_l > old_OD_l 
                cx=  positionUAV(1)-0.01;
            else
                cx=  positionUAV(1);
            end
        else
            cx=  positionUAV(1)+0.01;
        end
        %tuning of the motion to the right 
        
        cz= positionUAV(3);
       
        [~]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.1);
        [~,position]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
        positionUAV= double(position);

        %front sensor
        [~,SensorDetectionState_f,SensorDetectedPoint_f,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(1),sim.simx_opmode_buffer);
        if SensorDetectionState_f == 0 % no obstacle
            OD_f = max_threshold;
        else    %obstacle
            OD_f = ObstacleDistance(SensorDetectionState_f,SensorDetectedPoint_f);
        end
        % left sensor
                [~,SensorDetectionState_l,SensorDetectedPoint_l,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(2),sim.simx_opmode_buffer);
        if SensorDetectionState_l == 0 % no obstacle
            OD_l = max_threshold;
        else    %obstacle
            OD_l = ObstacleDistance(SensorDetectionState_l,SensorDetectedPoint_l);
        end
        if OD_f < min_threshold %%|| OD_l < min_threshold_l
            old_OD_f = OD_f;
            old_OD_l = OD_l;
            obstacle= true; 
            returnCode=-2;
        else
            obstacle= false; 
            returnCode=0;
        end
    end
    
end 