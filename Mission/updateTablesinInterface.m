function updateTablesinInterface(table_left, table_right, Policy_UAV1, Policy_UAV2,Policy, uav)

    if (uav == 1)
        S= 13; 
        for i = 1:S
            % Ajout du résultat à l'historique correspondant
            Policy_UAV1{i,1} = ['State ', num2str(i)];
            Policy_UAV1{i,2} = U1_Name_action(Policy(i));

            % Mise à jour des objets "uitable" avec les nouveaux historiques
            set(table_left,'Data',Policy_UAV1);   
        end
    else
        S=10; 
        for i = 1:S
            % Ajout du résultat à l'historique correspondant
            Policy_UAV2{i,1} = ['State ', num2str(i)];
            Policy_UAV2{i,2} = U2_Name_action(Policy(i));

            % Mise à jour des objets "uitable" avec les nouveaux historiques
            set(table_right,'Data',Policy_UAV2);
        end
    end
    pause(1);
end