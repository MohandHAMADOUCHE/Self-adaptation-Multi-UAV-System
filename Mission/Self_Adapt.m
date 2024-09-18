function [policy_after_up, Q_updated, Reward_updated] = Self_Adapt (Q_to_update, Proba, Reward, scenario)


    switch  scenario
        case 1

            %% wished_action = 5; Function 2 inspection 
            wished_action = 5; 
            % statesinconflict = [6];   % S6 fuction 2 than
            % funtion 1 
            statesinconflict   = zeros(1,11);
            statesinconflict(5) = 1; 
            [policy_after_up, Q_updated, Reward_updated] = Resolve_Conflicts(Q_to_update, Proba, Reward, statesinconflict, wished_action);
        case 2
            %% wished_action = 5; Lunding
            wished_action = 5; 
            % statesinconflict = [6, 7];   % S6 fuction 2 than
            % funtion 1 
            statesinconflict   = zeros(1,13);
            statesinconflict(6) = 1; statesinconflict(7) = 1; 
            [policy_after_up, Q_updated, Reward_updated] = Resolve_Conflicts(Q_to_update, Proba, Reward, statesinconflict, wished_action);
    end
  
    % sending request to the edge for UAV replacement 
    % function to send the request and the target postion
    disp('new policy ready!')
end

