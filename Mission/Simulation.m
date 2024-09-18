
function Simulation(clientID,sim,Handle,simTime, conflict, table_left, table_right, Policy_UAV1, Policy_UAV2, InterAction_UAV1, InterAction_UAV2, InterStatusU1,InterStatusU2)
    nb_WP_UAV1 = 7;
    nb_WP_UAV2 = 7;
    nb_UAV = 2;
    
    %Object Handles
    RobotHandle = Handle(:,1);
    SensorHandle = Handle(:,2:4);
    TargetHandle = Handle(:,5:nb_WP_UAV1+4);
    buildingHandle = Handle(:,nb_WP_UAV1+5);

    
    %Simulation preparation      
    RobotPosition=zeros(nb_UAV,3);
    TargetPositionFrame = zeros(1,3);
    TargetPositionWPFrame=zeros(nb_UAV,nb_WP_UAV1,3);
    abs_eulerAnglesUAV_ = zeros(nb_UAV,3);
    abs_eulerAnglesWP_ = zeros(nb_UAV,nb_WP_UAV1,3);    
    abs_eulerAnglesUAV_building =  zeros(nb_UAV,3);    
    %abs_eulerAnglesWP_U = zeros(nb_WP_UAV1,3);       
    %abs_eulerAnglesWP_U1 = zeros(nb_WP_UAV2,3);       
    %abs_eulerAnglesWP_U2 = zeros(nb_WP_UAV2,3);       
    updatetarget = 0;
    BasePosition_UAV = zeros (nb_UAV,3);

    itr = 0;
    elapsedTime = 0;
    tic;
    stop = 0;
    U1_state = 0; 
    U2_state = 0; 
    obs_avoid_U1 = false;
    %obs_avoid_U2 = false;
    End_mis_UAV1 = 0; 
    End_mis_UAV2 = 0; 
    End_inspec_B1 = false; 
    flagAdaptU1= false;
    flagAdaptU2= false; 
    Home1= false;
    Home2= false;

    %motion= input('To move the Robot, Press W,A,S,D for Forward, Left, Reverse, Right, and Press e to end','s');
            
            %[returnCode,position_ctrlPt1]=sim.simxGetObjectPosition(clientID, ctrlPt1,-1, sim.simx_opmode_buffer)
      
    while (stop~=1 && elapsedTime<=simTime ) 
        if itr==0
            opmode = sim.simx_opmode_streaming;
            U1_state = U1_state+1; 
            U2_state = U2_state+1; 

            itr=itr+1;
            %% UAV 1
                uav=1; 
                % interface
                set(InterStatusU1,'String','Status: Start ');

                %Get Position data
                [~,RobotPosition(uav,:)] = sim.simxGetObjectPosition(clientID,RobotHandle(uav,:),-1,opmode);
                
                %Get Sensor data
                [~,~,~,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(uav,1),opmode);
                [~,~,~,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(uav,2),opmode);
                [~,~,~,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(uav,3),opmode);
                
                 for wp_u1=1: nb_WP_UAV1  
                    [~,abs_eulerAnglesUAV_(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:), TargetHandle(uav,wp_u1), opmode);
                    [~,TargetPositionWPFrame(uav,wp_u1,:)]= sim.simxGetObjectPosition(clientID,TargetHandle(uav,wp_u1),-1,opmode);
                    [~,abs_eulerAnglesWP_(uav,wp_u1,:)]=sim.simxGetObjectOrientation(clientID,TargetHandle(uav,wp_u1),-1, opmode);
                 end
                 [U1_proba_vect, U1_reward_vect] = UAV1_Init_mission();
                 U1_proba_obstacle = U1_proba_vect(5);
            %% UAV 2
                uav=2; 
                % interface
                set(InterStatusU2,'String','Status: Start ');
                
                %Get Position data
                [~,RobotPosition(uav,:)] = sim.simxGetObjectPosition(clientID,RobotHandle(uav,:),-1,opmode);
                
                %Get Sensor data
                [~,~,~,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(uav,1),opmode);
                [~,~,~,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(uav,2),opmode);
                [~,~,~,~,~] = sim.simxReadProximitySensor(clientID,SensorHandle(uav,3),opmode);

                 for wp_u2=1: nb_WP_UAV2  
                    [~,abs_eulerAnglesUAV_(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:), TargetHandle(uav,wp_u2), opmode);
                    [~,TargetPositionWPFrame(uav,wp_u2,:)]= sim.simxGetObjectPosition(clientID,TargetHandle(uav,wp_u2),-1,opmode);
                    [~,abs_eulerAnglesWP_(uav,wp_u2,:)]=sim.simxGetObjectOrientation(clientID,TargetHandle(uav,wp_u2),-1, opmode);
                 end
                 [U2_proba_vect, U2_reward_vect] = UAV2_Init_mission();
                 U2_proba_obstacle = U2_proba_vect(5);
                 for UAV_i = 1:2 
                     for b=1: 2
                             [~,abs_eulerAnglesUAV_building(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:), buildingHandle(b,1), opmode);
                             [~,abs_eulerAnglesUAV_(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:), buildingHandle(b,1), opmode);
                     end
                 end
        else
            opmode = sim.simx_opmode_buffer;
            
            if (End_mis_UAV1 ~=1)
                %% UAV 1
                uav=1; 
                
                U1_proba_vect(5) = U1_proba_obstacle; 
                [U1_Policy, nil, U1_Q, U1_P, U1_R] = UAV1_Construction_Rresolution_Mission(U1_proba_vect, U1_reward_vect); 
                
                %interface update
                    updateTablesinInterface(table_left, table_right, Policy_UAV1, Policy_UAV2, U1_Policy, uav); 
                if (flagAdaptU1)
                    
                    set(InterStatusU1,'String','Status: conflict !!! ');
                    pause(3);

                    scenario = 2; 
                    [policy_after_up, Q_updated, Reward_updated] = Self_Adapt(U1_Q, U1_P, U1_R, scenario);
                    U1_Policy = policy_after_up; 
                %interface update
                    updateTablesinInterface(table_left, table_right, Policy_UAV1, Policy_UAV2, U1_Policy, uav); 
                    set(InterStatusU1,'String','Status: new Policy ready');
                    pause(3);

                end
                

                transpose(U1_Policy)
                U1_action = U1_Policy(U1_state);
                
                %interface update
                    set(InterStatusU1,'String',['Status: State ', num2str(U1_state)]);
                    set(InterAction_UAV1,'String',['Current Action: ', U1_Name_action(U1_action)]);

                fprintf(' {UAV 1 --> state: %d, action %d: %s}\n',U1_state,U1_action,U1_Name_action(U1_action));

                switch U1_action
                    case 1 % takeoff
                       [~,BasePosition_UAV(uav,:)] = sim.simxGetObjectPosition(clientID,TargetHandle(uav,1),-1,opmode);
                       [res,RobotPosition(uav,:)] = Takeoff(sim, clientID, RobotHandle(uav,:), BasePosition_UAV(uav,:));
                       if (res== 0)
                           U1_state = U1_state+1;
                       end
                    case 2 % go to next WP
                        if (obs_avoid_U1)
                            wp_u1 = last_state;
                        else
                            wp_u1=U1_state;
                        end
                        [~,TargetPositionWPFrame(uav,wp_u1,:)]= sim.simxGetObjectPosition(clientID,TargetHandle(uav,wp_u1),-1,opmode);
                        TargetPositionFrame (1,1) = TargetPositionWPFrame(uav,wp_u1,1);
                        TargetPositionFrame (1,2) = TargetPositionWPFrame(uav,wp_u1,2);
                        TargetPositionFrame (1,3) = TargetPositionWPFrame(uav,wp_u1,3);
                        [~,abs_eulerAnglesUAV_(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:),TargetHandle(uav,wp_u1), opmode);
                         abs_eulerAnglesUAV_U1 (1,1) = abs_eulerAnglesUAV_(uav,1); abs_eulerAnglesUAV_U1 (1,2) = abs_eulerAnglesUAV_(uav,2); abs_eulerAnglesUAV_U1 (1,3) = abs_eulerAnglesUAV_(uav,3);

                        [res,RobotPosition(uav,:)]= Go_to_WP (sim, clientID,RobotHandle(uav,:), SensorHandle(uav,1) , TargetHandle(uav,wp_u1), RobotPosition(uav,:),TargetPositionFrame,abs_eulerAnglesUAV_U1);
                        pause(0.1);
                        if res == 0
                            obs_avoid_U1 = false;
                            if U1_state == 13
                                U1_state = last_state + 1  ;
                            else
                                U1_state = U1_state + 1;
                            end
                        elseif res == -2
                                disp("obstacle detected");
                                proba_obstacle_U1 = 0.99;
                                U1_state = 12;
                                last_state =wp_u1;
                                
                                set(InterStatusU1,'String','Status: obstacle detected !');

                        end

                        if (conflict && wp_u1==5)
                            U1_proba_vect(1) = 0.9; 
                            U1_proba_vect(2) = 0.3;
                            U1_proba_vect(3) = 0.95;
                            U1_reward_vect(2) = 1; 
                            flagAdaptU1= true; 
                        end
                    case 3 % RTH
                        wp_u1=1;  %[TargetHandle(1,1)] is the base target
                        [~,abs_eulerAnglesUAV_(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:),TargetHandle(uav,wp_u1), opmode);
                        abs_eulerAnglesUAV_U1 (1,1) = abs_eulerAnglesUAV_(uav,1); abs_eulerAnglesUAV_U1 (1,2) = abs_eulerAnglesUAV_(uav,2); abs_eulerAnglesUAV_U1 (1,3) = abs_eulerAnglesUAV_(uav,3);

                          [res,RobotPosition(uav,:)] = RTH (sim, clientID, RobotHandle(uav,:),TargetHandle(uav,wp_u1), RobotPosition(uav,:), BasePosition_UAV(uav,:),abs_eulerAnglesUAV_U1);
                        if res == 0
                            U1_state = 10;
                        end
                    case 4 % inspection V1
                         
                        [res,RobotPosition(uav,:)] = Inspection_Version1 (sim, clientID, RobotHandle(uav,:), RobotPosition(uav,:), buildingHandle(1,1));
                        if res == 0
                            U1_state = 9;
                            End_inspec_B1= true; 
                        end
                    case 5  %landing
                        if wp_u1 == 1 
                            Home1 = true; 
                        end 
                        [res,RobotPosition(uav,:)] = Landing (sim, clientID, RobotHandle(uav,:), RobotPosition(uav,:), Home1);
                        if res == 0 && wp_u1 == 1
                            End_mis_UAV1= 1; 
                            disp("End of mission UAV 1");
                            
                            set(InterStatusU1,'String','Status: End mission  ');

                        else
                            End_mis_UAV1= 1; 
                            flagAdaptU2= true; 
                            scenario = 1; 
                            disp("Error on the mission UAV 1");
                            set(InterStatusU1,'String','Status: Error ! ');

                        end
                    case 6  
                        disp("nop action");
                    case 7 % obstacle avoidance
                        [res,RobotPosition(uav,:)]= Obstacle_avoidance (sim, clientID,RobotHandle(uav,:), SensorHandle(uav,:), RobotPosition(uav,:));
                        if (res == 0) 
                            U1_state = 13;
                            obs_avoid_U1 = true;
                        end
                    otherwise
                            disp("inexistence action");
                end
                set(InterAction_UAV1,'String','Current Action: ');

            end
     
            
             if (End_mis_UAV2 ~=1) 
                
                %% UAV 2
                uav=2;
                [U2_Policy, nil, U2_Q, U2_P, U2_R] = UAV2_Construction_Rresolution_Mission(U2_proba_vect, U2_reward_vect); 
                    %interface update
                    updateTablesinInterface(table_left, table_right, Policy_UAV1, Policy_UAV2, U2_Policy, uav); 
                if flagAdaptU2 == true
                    updatetarget = updatetarget + 1; % only the first time
                    % interface
                    
                    set(InterStatusU2,'String','Status: conflict !!! ');
                    pause(3);

                    [policy_after_up, Q_updated, Reward_updated] = Self_Adapt(U2_Q, U2_P, U2_R, scenario);
                    U2_Policy = policy_after_up; 
                    %interface update
                    updateTablesinInterface(table_left, table_right, Policy_UAV1, Policy_UAV2, U2_Policy, uav); 
                    set(InterStatusU2,'String','Status: new Policy ready');
                    pause(3);
                    if (updatetarget == 1)
                        set(InterStatusU2,'String','Status: target postion received ');
                        pause(3);
                        TargetHandle(uav,7) = TargetHandle(3-uav,7);
                        wp_u2 = 6; 
                        [~,abs_eulerAnglesUAV_(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:),TargetHandle(uav,wp_u2+1), sim.simx_opmode_streaming);
                        [~,TargetPositionWPFrame(uav,wp_u2+1,:)]= sim.simxGetObjectPosition(clientID,TargetHandle(uav,wp_u2+1),-1,sim.simx_opmode_streaming);
                    end
                end
                transpose(U2_Policy)
                U2_action = U2_Policy(U2_state);
                
                %interface update
                    set(InterStatusU2,'String',['Status: State ', num2str(U2_state)]);
                    set(InterAction_UAV2,'String',['Current Action: ', U2_Name_action(U2_action)]);
                fprintf(' {UAV 2 --> state: %d, action %d: %s}\n',U2_state,U2_action,U2_Name_action(U2_action));

                 switch U2_action
                    case 1 % takeoff
                        RobotHandle(uav,:)
                       [~,BasePosition_UAV(uav,:)] = sim.simxGetObjectPosition(clientID,TargetHandle(uav,1),-1,opmode);
                       [res,RobotPosition(uav,:)] = Takeoff(sim, clientID, RobotHandle(uav,:), BasePosition_UAV(uav,:));
                       if (res== 0)
                           U2_state = U2_state+1;
                           wp_u2 = 1; 
                       end
                    case 2 % go to next WP

                            wp_u2 = wp_u2+1 ;

                        [~,TargetPositionWPFrame(uav,wp_u2,:)]= sim.simxGetObjectPosition(clientID,TargetHandle(uav,wp_u2),-1,opmode);
                        TargetPositionFrame (1,1) = TargetPositionWPFrame(uav,wp_u2,1); TargetPositionFrame (1,2) = TargetPositionWPFrame(uav,wp_u2,2); TargetPositionFrame (1,3) = TargetPositionWPFrame(uav,wp_u2,3);
                        [~,abs_eulerAnglesUAV_(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:),TargetHandle(uav,wp_u2), opmode);
                        abs_eulerAnglesUAV_U2 (1,1) = abs_eulerAnglesUAV_(uav,1); abs_eulerAnglesUAV_U2 (1,2) = abs_eulerAnglesUAV_(uav,2); abs_eulerAnglesUAV_U2 (1,3) = abs_eulerAnglesUAV_(uav,3);
                        [res,RobotPosition(uav,:)]= Go_to_WP (sim, clientID,RobotHandle(uav,:), SensorHandle(uav,1) , TargetHandle(uav,wp_u2), RobotPosition(uav,:),TargetPositionFrame,abs_eulerAnglesUAV_U2);
                        pause(0.1);
                        if res == 0
                            obs_avoid_U2 = false;
                            if wp_u2 == 7
                                U2_state = 5;
                            end
                        elseif res == -2
                            disp("obstacle detected");
                            U2_proba_obstacle = 0.99;
                            U2_state = 9;
                            wp_u2 =wp_u2 - 1;
                            last_state_U2 =wp_u2;
                            
                            set(InterStatusU2,'String','Status: obstacle detected !');

                        end

                    case 3 % RTH
                        wp_u2=1;  %[TargetHandle(2,1)] is the base target
                        [~,abs_eulerAnglesUAV_(uav,:)]=sim.simxGetObjectOrientation(clientID,RobotHandle(uav,:),TargetHandle(uav,wp_u2), opmode);
                        abs_eulerAnglesUAV_U2 (1,1) = abs_eulerAnglesUAV_(uav,1); abs_eulerAnglesUAV_U2 (1,2) = abs_eulerAnglesUAV_(uav,2); abs_eulerAnglesUAV_U2 (1,3) = abs_eulerAnglesUAV_(uav,3);
                        [res,RobotPosition(uav,:)] = RTH (sim, clientID, RobotHandle(uav,:),TargetHandle(uav,wp_u2), RobotPosition(uav,:), BasePosition_UAV(uav,:),abs_eulerAnglesUAV_U2);
                        if res == 0
                            U2_state = 7;
                            
                        end
                    case 4 % Inspection_Version2 UP-DOWN motion
                        if (End_inspec_B1) 
                            [res,RobotPosition(uav,:)] = Inspection_Version2 (sim, clientID, RobotHandle(uav,:), RobotPosition(uav,:), buildingHandle(2,1));
                                if res == 0
                                    U2_state = 6;
                                end
                        end
                        
                    case 5 % Inspection_Version1 in S motion
                        [res,RobotPosition(uav,:)] = Inspection_Version1 (sim, clientID, RobotHandle(uav,:), RobotPosition(uav,:), buildingHandle(1,1));
                        if res == 0
                            U2_state = 6;
                        end
                        
                    case 6  %landing
                        if wp_u2 == 1 
                            Home2 = true; 
                        end 
                        [res,RobotPosition(uav,:)] = Landing (sim, clientID, RobotHandle(uav,:), RobotPosition(uav,:), Home2);
                        if res == 0
                            End_mis_UAV2= 1; 
                            disp("End of mission UAV 2");
                            
                            set(InterStatusU2,'String','Status: End mission  ');

                        end
                    case 7  
                        disp("nop action");
                    case 8 % obstacle avoidance
                        [res,RobotPosition(uav,:)]= Obstacle_avoidance (sim, clientID,RobotHandle(uav,:), SensorHandle(uav,:), RobotPosition(uav,:));
                        if (res == 0) 
                            U2_state = 10;
                        end
                    otherwise
                            disp("inexistence action");
                 end
                set(InterAction_UAV2,'String','Current Action: ');
             end
        end

        if     (End_mis_UAV1 +  End_mis_UAV2) ==  2  
              stop=true;
        end
        
        elapsedTime = toc;  
        pause(0.3);
        %motion = getkeywait(2);
        continue 
    end
end 
