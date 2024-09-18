function    [res,positionUAV] = Inspection_Version1 (sim, clientID, UAV_Target_handle, positionUAV, buildingHandle)
    FrontUAVangle = 1.5707963;
    returnCode = -1;
    precision = 1;

    abs_eulerAnglesUAV_building= zeros(1,3);
    position_WPj= zeros(1,3);
    abs_eulerAnglesUAV_= zeros(1,3);
    %init position and oriotation
    opmode = sim.simx_opmode_streaming;
    [~,abs_eulerAnglesUAV_(1,:)]=sim.simxGetObjectOrientation(clientID,UAV_Target_handle,buildingHandle, opmode);
    [~,abs_eulerAnglesUAV_building(1,:)]=sim.simxGetObjectOrientation(clientID,UAV_Target_handle, buildingHandle, opmode);
    [~,position_WPj(1,:)]= sim.simxGetObjectPosition(clientID,buildingHandle,-1,opmode);
    pause(0.1);

    %get position and oriotation
    opmode= sim.simx_opmode_buffer; 
    [~,abs_eulerAnglesUAV_(1,:)]=sim.simxGetObjectOrientation(clientID,UAV_Target_handle,buildingHandle, opmode);
    [~,abs_eulerAnglesUAV_building(1,:)]=sim.simxGetObjectOrientation(clientID,UAV_Target_handle, buildingHandle, opmode);
    [~,position_WPj(1,:)]= sim.simxGetObjectPosition(clientID,buildingHandle,-1,opmode); 

    % rotation as the building 
    while((round(double(abs_eulerAnglesUAV_(3)),1)) ~= round(FrontUAVangle,1))
            if abs_eulerAnglesUAV_(3) > FrontUAVangle || abs_eulerAnglesUAV_(3) < -FrontUAVangle
                angle = abs_eulerAnglesUAV_(3)- 0.1;
            elseif abs_eulerAnglesUAV_(3) < FrontUAVangle
                angle = abs_eulerAnglesUAV_(3) + 0.1;
            else
                break
            end
            [returnCode]=sim.simxSetObjectOrientation(clientID, UAV_Target_handle, buildingHandle ,[0     0     angle], sim.simx_opmode_oneshot);
            pause(0.2);
            [~,abs_eulerAnglesUAV_]=sim.simxGetObjectOrientation(clientID,UAV_Target_handle, buildingHandle, sim.simx_opmode_buffer);  
    end
    
        %compute of parametric equations between position and the next waypoint 

        l = position_WPj(1) - positionUAV(1);
        m = position_WPj(2) - positionUAV(2);
        n = position_WPj(3) - positionUAV(3);
        while ( (round(positionUAV(1),precision) ~=  round(position_WPj(1),precision)) || ...
                (round(positionUAV(2),precision) ~=  round(position_WPj(2),precision))  || ...
                (round(positionUAV(3),precision) ~=  round(position_WPj(3),precision))) 
            % to do if the returncode of simxGetObjectPosition ~= 0 The function is not executed fine! 
                cx=  positionUAV(1)+l*0.01;
                cy=  positionUAV(2)+m*0.01;
                cz= positionUAV(3)+n*0.01;
            [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
            pause(0.1);
            [returnCode,position]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
            positionUAV= double(position);
        end
    
    % inspection with S motion 
    iter_max=10
    %->
    for i = 1:iter_max*5
        cx=  positionUAV(1);
        cy=  positionUAV(2)+0.02;
        cz= positionUAV(3);
        [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.1);
        [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
    end 
    % <-
    for i = 1:10*iter_max
        cx=  positionUAV(1);
        cy=  positionUAV(2)-0.02;
        cz= positionUAV(3);
        [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.1);
        [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
    end  
    %down
    for i = 1:iter_max
        cx=  positionUAV(1);
        cy=  positionUAV(2);
        cz= positionUAV(3)-i*0.01;
        [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.1);
        [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
    end    
    %right
    for i = 1:10*iter_max
        cx=  positionUAV(1);
        cy=  positionUAV(2)+0.02;
        cz= positionUAV(3);
        [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.1);
        [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
    end   
    %down
    for i = 1:iter_max
        cx=  positionUAV(1);
        cy=  positionUAV(2);
        cz= positionUAV(3)-i*0.01;
        [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.1);
        [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
    end 
    % left
    for i = 1:10*iter_max
        cx=  positionUAV(1);
        cy=  positionUAV(2) - 0.02;
        cz= positionUAV(3);
        [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.1);
        [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
    end  
end


