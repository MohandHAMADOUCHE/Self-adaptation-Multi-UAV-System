function [proba_vect, reward_vect, discount] = UAV1_Init_mission()

    discount = 0.95;

    % Proba received from BN and from autopilot
    
    P_sys = 0.9;
    P_bat = 0.8;
    P_land = 0.1;
    P_Obs = 0.01;
    P_NOP = 0.01; 
    
    
    %% construction proba_vect & reward_vect
    
	proba_vect = [P_sys, P_bat, P_land, P_NOP, P_Obs];   %[0.9, 0.6, 0.8, 0.96, 0.82, 0.7, 0.1, 0.10, 0.6, 0.7, 0.1];   
				% 1: P_sys, 2:P_NOP,3: P_bat , 4:P_land
    reward_vect = [5.0, 25.0, 25.0, 40.0, 57.0,  1.0, 10] ;
                %1:A1, 2:A2, 3:RTH, 4: landing action, 5: r_flip, 6: NOP

          

             

        
end
 