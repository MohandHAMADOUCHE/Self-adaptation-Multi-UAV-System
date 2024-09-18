function Handle = ObjectHandle(clientID,sim)

    RobotHandle = zeros(2,1);
    TargetHandle = zeros(2,7);
    SensorHandle = zeros(2,3);
    buildingHandle = zeros(2,1);
    
   %%Getting Object Handles
   %Robot Handles
    [~,RobotHandle(1,:)]=sim.simxGetObjectHandle(clientID,'Quadcopter_target',sim.simx_opmode_blocking);
    [~,RobotHandle(2,:)]=sim.simxGetObjectHandle(clientID,'Quadcopter_target#0',sim.simx_opmode_blocking);
    
    %SensorHandles
    [~, SensorHandle(1,1)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_F1', sim.simx_opmode_blocking);
    [~, SensorHandle(1,2)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_L1', sim.simx_opmode_blocking);
    [~, SensorHandle(1,3)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_R1', sim.simx_opmode_blocking);
    [~, SensorHandle(2,1)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_F1#0', sim.simx_opmode_blocking);
    [~, SensorHandle(2,2)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_L1#0', sim.simx_opmode_blocking);
    [~, SensorHandle(2,3)] = sim.simxGetObjectHandle(clientID,'Proximity_sensor_R1#0', sim.simx_opmode_blocking);
   %TargetHandles

    [~,TargetHandle(1,1)]=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking);
    [~,TargetHandle(1,2)]=sim.simxGetObjectHandle(clientID,'Dummy0',sim.simx_opmode_blocking);
    [~,TargetHandle(1,3)]=sim.simxGetObjectHandle(clientID,'Dummy1',sim.simx_opmode_blocking);
    [~,TargetHandle(1,4)]=sim.simxGetObjectHandle(clientID,'Dummy2',sim.simx_opmode_blocking);
    [~,TargetHandle(1,5)]=sim.simxGetObjectHandle(clientID,'Dummy3',sim.simx_opmode_blocking);    
    [~,TargetHandle(1,6)]=sim.simxGetObjectHandle(clientID,'Dummy4',sim.simx_opmode_blocking);
    [~,TargetHandle(1,7)]=sim.simxGetObjectHandle(clientID,'Dummy5',sim.simx_opmode_blocking);
    %[~,TargetHandle(1,8)]=sim.simxGetObjectHandle(clientID,'Dummy6',sim.simx_opmode_blocking);

    
    
    [~,TargetHandle(2,1)]=sim.simxGetObjectHandle(clientID,'Dummy7',sim.simx_opmode_blocking);
    [~,TargetHandle(2,2)]=sim.simxGetObjectHandle(clientID,'Dummy8',sim.simx_opmode_blocking);
    [~,TargetHandle(2,3)]=sim.simxGetObjectHandle(clientID,'Dummy9',sim.simx_opmode_blocking);
    [~,TargetHandle(2,4)]=sim.simxGetObjectHandle(clientID,'Dummy12',sim.simx_opmode_blocking);
    [~,TargetHandle(2,5)]=sim.simxGetObjectHandle(clientID,'Dummy13',sim.simx_opmode_blocking);
    [~,TargetHandle(2,6)]=sim.simxGetObjectHandle(clientID,'Dummy10',sim.simx_opmode_blocking);
    [~,TargetHandle(2,7)]=sim.simxGetObjectHandle(clientID,'Dummy11',sim.simx_opmode_blocking);

    [~,buildingHandle(1,1)]=sim.simxGetObjectHandle(clientID,'Dummy6',sim.simx_opmode_blocking);
    [~,buildingHandle(2,1)]=sim.simxGetObjectHandle(clientID,'Dummy14',sim.simx_opmode_blocking);


    Handle=[RobotHandle, SensorHandle, TargetHandle, buildingHandle];
end

