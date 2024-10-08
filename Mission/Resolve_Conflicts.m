function [policy_after_up, Q_updated, Reward_updated] = Resolve_Conflicts(Q_to_update, Proba, Reward, ...
                                                                        statesinconflict, action_NOP) 
% check of arguments  to found size of the problem
S = size(Reward,1); A = size(Reward,2); 
discount = 0.95; 
%%
%%%%%%%%%%%%%%%%% 
[V, policy] = max( Q_to_update,[],2);
Reward_to_up = Reward;
%%%%%%%%%%%%%%%%%
forced_action = action_NOP;
finish = 0;
for state=1:S
    if (statesinconflict(state) == 1)
        executed_action = policy(state);    
        result = Q_to_update(state,executed_action) - discount*Proba(state,:,forced_action)*V;
        Reward_to_up(state,forced_action) = round(result) + 1;
    end
end
[V_new, Q_new, Pol_new] = mdp_policy_iteration(Proba, Reward_to_up, discount,policy);
while (finish == 0)
    finish = 1;
    for state=1:S
      if ((statesinconflict(state) == 1) && (Pol_new(state) ~= forced_action ))
        executed_action = policy(state);    
        result = Q_new(state,executed_action) - discount*Proba(state,:,forced_action)*V_new;
        Reward_to_up(state,forced_action) = round(result) + 1;
        [V_new, Q_new, Pol_new] = mdp_policy_iteration(Proba, Reward_to_up, discount,Pol_new);
        finish = 0;
      end
    end  
end 
policy_before_up = policy;
policy_after_up = Pol_new;
Q_updated = Q_new;
Reward_updated = Reward_to_up;

end 