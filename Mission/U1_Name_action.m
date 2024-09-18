function [n_action]=U1_Name_action(action)

    switch action
        case 1
            n_action = 'Takeoff';
        case 2
            n_action = 'Go to next WP';
        case 3
            n_action = 'RTH';
        case 4
            n_action = 'Inspection Version 1';
        case 5
            n_action = 'Land';
        case 6
            n_action = 'NOP';     
        otherwise
            n_action = 'Obstacle avoidance';     
    end
end 