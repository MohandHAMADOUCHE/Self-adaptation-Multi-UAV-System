function    [returnCode,positionUAV] = Landing (sim, clientID, UAV_Target_handle, positionUAV, Home)
    returnCode = -1;
    if Home == 1
        h= 1.41 ;  % on the roof of the building 
    else
        h= 0.2;
    end
    while positionUAV(3) > h
        cx=  positionUAV(1);
        cy=  positionUAV(2);
        cz= positionUAV(3)-0.01;
        [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.1);
        [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
    end
end