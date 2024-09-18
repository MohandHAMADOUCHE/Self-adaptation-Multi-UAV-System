function    [returnCode,positionUAV] = Takeoff (sim, clientID, UAV_Target_handle, positionTakeoff)

    [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);

    while positionUAV(3) <= positionTakeoff(3)
        cx=  positionUAV(1);
        cy=  positionUAV(2);
        cz= positionUAV(3)+0.01;
        [res]=sim.simxSetObjectPosition(clientID,UAV_Target_handle,-1,[cx cy cz],sim.simx_opmode_oneshot);
        pause(0.2);
        [returnCode,positionUAV]=sim.simxGetObjectPosition(clientID, UAV_Target_handle,-1, sim.simx_opmode_buffer);
    end
end