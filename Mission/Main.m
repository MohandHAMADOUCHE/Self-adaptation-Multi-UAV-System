function Main (conflict, table_left, table_right, Policy_UAV1, Policy_UAV2, InterAction_UAV1, interAction_UAV2, InterStatusU1, InterStatusU2) 
%clc;clear;close all;
disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

% add path to the MDP toolbox
addpath("E:\Traveaux_THESE\Simulation Mission\MDPtoolbox");



if (clientID>-1)

    disp('Connected to remote API server in CoppeliaSim');
    
    simTime=400;

     % Now try to retrieve data in a blocking fashion (i.e. a service call):
    [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
    if (res==sim.simx_return_ok)
        fprintf('Number of objects in the scene: %d\n',length(objs));
    else
        fprintf('Remote API function call returned with error code: %d\n',res);
    end
    % Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim! msg from MATLAB',sim.simx_opmode_oneshot);
    
    Handle=ObjectHandle(clientID,sim);
    %LunchInterface;
    Simulation(clientID,sim,Handle,simTime, conflict, table_left, table_right, Policy_UAV1, Policy_UAV2, InterAction_UAV1, interAction_UAV2, InterStatusU1, InterStatusU2);

    sim.simxFinish(-1);
    disp("End of simulations");
end 
sim.delete(); 

end