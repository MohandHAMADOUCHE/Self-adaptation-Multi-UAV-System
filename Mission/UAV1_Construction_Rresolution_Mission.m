%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%    construction and resolution of the MDP 
%-------------------------------------------------------

function [Policy, V, Q, P, R] = UAV1_Construction_Rresolution_Mission(proba_vect, reward_vect)

% % 	proba_vect = [P_sys, P_bat, P_land, P_NOP];   %[0.9, 0.6, 0.8, 0.96, 0.82, 0.7, 0.1, 0.10, 0.6, 0.7, 0.1];   
% % 				% 1: P_sys, 2:P_NOP,3: P_bat , 4:P_land
% % 		reward_vect = [5.0, 35.0, 1.0, 30.0, 40.0,  1.0] ;
                %1:A1, 2:A2, 3:RTH, 4: landing action, 5: r_flip, 6: NOP
  


	nbr_S = 13;
	nbr_A = 7;
%% Probability matrix 
P(:,:,:) = zeros(nbr_S, nbr_S, nbr_A);

%Take-Off A1
P(1,1,1) = 1 - proba_vect(1); P(1,2,1) = proba_vect(1);
        
%GO to WPi A2
P(2,2,2) = 1 - (proba_vect(2) + proba_vect(5)) ; P(2,3,2) = proba_vect(2) ; P(2,12,2) = proba_vect(5);
P(3,3,2) = 1 - (proba_vect(2) + proba_vect(5)); P(3,4,2) = proba_vect(2) ; P(3,12,2) = proba_vect(5);
P(4,4,2) = 1 - (proba_vect(2) + proba_vect(5)); P(4,5,2) = proba_vect(2) ; P(4,12,2) = proba_vect(5);
P(5,5,2) = 1 - (proba_vect(2) + proba_vect(5)); P(5,6,2) = proba_vect(2) ; P(5,12,2) = proba_vect(5);
P(6,6,2) = 1 - (proba_vect(2) + proba_vect(5)); P(6,7,2) = proba_vect(2) ; P(6,12,2) = proba_vect(5);
P(7,7,2) = 1 - (proba_vect(2) + proba_vect(5)); P(7,8,2) = proba_vect(2) ; P(7,12,2) = proba_vect(5);

  
% (RTH A3)
p_RTH= max (proba_vect(1), proba_vect(2)); 
P(3,10,3)  = p_RTH; P(3,3,3)  = 1- p_RTH; 
P(4,10,3)  =  p_RTH; P(4,4,3)  = 1- p_RTH; 
P(5,10,3)  =  p_RTH; P(5,5,3)  = 1- p_RTH; 
P(6,10,3)  =  p_RTH; P(6,6,3)  = 1- p_RTH; 
P(7,10,3)  =  p_RTH; P(7,7,3)  = 1- p_RTH; 
P(8,10,3)  =  p_RTH; P(8,8,3)  = 1- p_RTH; 
P(9,10,3)  =  p_RTH; P(9,9,3)  = 1- p_RTH; 
%flip
P(8,8,4)  = 1-  proba_vect(1); P(8,9,4)  = proba_vect(1);


% LANDING (A5)
P(2,11,5) = proba_vect(3); P(2,2,5) = 1 - proba_vect(3);
P(3,11,5) = proba_vect(3); P(3,3,5) = 1 - proba_vect(3);
P(4,11,5) = proba_vect(3); P(4,4,5) = 1 - proba_vect(3);
P(5,11,5) = proba_vect(3); P(5,5,5) = 1 - proba_vect(3);
P(6,11,5) = proba_vect(3); P(6,6,5) = 1 - proba_vect(3);
P(7,11,5) = proba_vect(3); P(7,7,5) = 1 - proba_vect(3);
P(8,11,5) = proba_vect(3); P(8,8,5) = 1 - proba_vect(3);
P(9,11,5) = proba_vect(3); P(9,9,5) = 1 - proba_vect(3);
P(10,11,5) = proba_vect(3); P(10,10,5) = 1 - proba_vect(3);


% NOP       
P(11,11,6) = 1; 

% obstacle avoidance

P(12,12,7) = proba_vect(5); P(12,13,7) = 1- proba_vect(5);
 
%% Rewards matrix 
R(:,:) = zeros(nbr_S, nbr_A);

R(1,1) = reward_vect(1); 
R(2,2) = reward_vect(2);  R(2,5) = reward_vect(5);
R(3,2) = reward_vect(2); R(3,3) = reward_vect(3); R(3,5) = reward_vect(5);

R(4,2) = reward_vect(2); R(4,3) = reward_vect(3);R(4,5) = reward_vect(5);

R(5,2) = reward_vect(2); R(5,3) = reward_vect(3); R(5,5) = reward_vect(5);

R(6,2) = reward_vect(2); R(6,3) = reward_vect(3); R(6,5) = reward_vect(5);

R(7,2) = reward_vect(2); R(7,3) = reward_vect(3); R(7,5) = reward_vect(5);

R(8,3) = reward_vect(3); R(8,4) = reward_vect(4); R(8,5) = reward_vect(5);

R(9,3) = reward_vect(3); R(9,5) = reward_vect(5);
R(10,5) = reward_vect(5);

R(11,6) = reward_vect(6);

R(12,7) = reward_vect(7);
R(13,2) = reward_vect(2); R(13,3) = reward_vect(3);

%% resolution   
    discount = 0.95;
    [V, Q, Policy] = mdp_policy_iteration(P, R, discount);



end