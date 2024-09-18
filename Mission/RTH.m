function  [returnCode,positionUAV]= RTH (sim, clientID, UAV_Target_handle,WP_handle, positionUAV,position_WP,eulerAnglesUAV)
    %%%%% rotation of the UAVangle
    FrontUAVangle = 1.5707963;
    returnCode = 0;
    position_WPj = position_WP;
    precision = 1;

    while ((round(positionUAV(1),precision) ~=  round(position_WPj(1),precision)) || ...
                (round(positionUAV(2),precision) ~=  round(position_WPj(2),precision))  || ...
                (round(positionUAV(3),precision) ~=  round(position_WPj(3),precision)))
        l = position_WPj(1) - positionUAV(1);
        m = position_WPj(2) - positionUAV(2);
        angle = l/m;

        while((round(double(eulerAnglesUAV(3)),1)) ~= round(FrontUAVangle,1))
            if eulerAnglesUAV(3) > FrontUAVangle || eulerAnglesUAV(3) < -FrontUAVangle
                angle = eulerAnglesUAV(3)- 0.1;
            elseif eulerAnglesUAV(3) < FrontUAVangle
                angle = eulerAnglesUAV(3) + 0.1;
            else
                break
            end
            [returnCode]=sim.simxSetObjectOrientation(clientID, UAV_Target_handle, WP_handle ,[0     0     angle], sim.simx_opmode_oneshot);
            pause(0.2);
            [~,eulerAnglesUAV]=sim.simxGetObjectOrientation(clientID,UAV_Target_handle, WP_handle, sim.simx_opmode_buffer);  
        end
       pause(0.3);

        returnCode = 0; 

        %compute of parametric equations between position and the next waypoint 

        l = position_WPj(1) - positionUAV(1);
        m = position_WPj(2) - positionUAV(2);
        n = position_WPj(3) - positionUAV(3);
        while ( (round(positionUAV(1),precision) ~=  round(position_WPj(1),precision)) || ...
                (round(positionUAV(2),precision) ~=  round(position_WPj(2),precision))  || ...
                (round(positionUAV(3),precision) ~=  round(position_WPj(3),precision))) 
            % to do if the returncode of simxGetObjectPosition ~= 0 The function is not executed fine! 
                cx=  positionUAV(1)+l*0.008;
                cy=  positionUAV(2)+m*0.008;
                cz= positionUAV(3)+n*0.008;
            [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
            pause(0.1);
            [returnCode,position]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
            positionUAV= double(position);
        end
        if  ( (round( positionUAV(1),precision) ~=  round(position_WP(1),precision))  || (round(positionUAV(2),precision) ~=  round(position_WP(2),precision))  || (round(positionUAV(3),precision) ~=  round(position_WP(3),precision)))
                returnCode = -1; 
        end
    end
    
end 