clc;clear;close all;
conflict = false; 
addpath('Mission');


% Création de la fenêtre graphique
fig = figure('Position',[100 100 650 600]);


% Create a panel for the radio buttons
p1 = uipanel('Parent',fig,'Title','Select an option: Simulation with Conflict ? ','Units','normalized','Position',[0.1 0.78 0.8 0.11] ,'fontsize',12);

% Create the radio buttons
r1 = uicontrol('Parent',p1,'Style','radiobutton','String','Yes','Units','normalized','Position',[0.2 0.4 0.4 0.3]);
r2 = uicontrol('Parent',p1,'Style','radiobutton','String','No','Units','normalized','Position',[0.7 0.4 0.4 0.3]);


uicontrol(fig,'Style','text','Position',[70 550 500 30],'String','Simulation team of UAVs mission','HorizontalAlignment','center','fontsize',18,'ForegroundColor','blue','FontWeight','bold');

uicontrol(fig,'Style','text','Position',[70 400 500 30],'String','-.- ------------------------------------------------------- -.-','HorizontalAlignment','center','fontsize',18,'ForegroundColor','blue','FontWeight','bold');

% Ajout de la ligne verticale entre les deux tableaux
uicontrol(fig,'style','text','String','|','position',[310 360 20 10],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 330 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 300 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 270 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 240 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 210 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 180 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 150 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 120 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 90 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 60 20 30],'fontsize',18);
uicontrol(fig,'style','text','String','|','position',[310 50 20 10],'fontsize',18);


% Ajout des champs text en dessus et en dessous de chaque tableau
uicontrol(fig,'Style','text','Position',[125 380 90 30],'String','UAV 1','HorizontalAlignment','center','fontsize',17);
uicontrol(fig,'Style','text','Position',[425 380 90 30],'String','UAV 2','HorizontalAlignment','center','fontsize',17);


% Création des deux objets "uitable" pour afficher l'historique des résultats
table_left = uitable(fig,'Position',[50 100 240 265],'ColumnName',{'State','Action'},'ColumnWidth',{80 120},'Data',cell(13,2));

%uicontrol(fig,'style','text','String','|','position',[310 50 30 50],'fontsize',18);
table_right = uitable(fig,'Position',[350 140 240 210],'ColumnName',{'State','Action'},'ColumnWidth',{80 120},'Data',cell(10,2));

% Initialisation des tableaux pour stocker l'historique des résultats
Policy_UAV1 = cell(13,2);
Policy_UAV2 = cell(10,2);

% 
% InterStatusU1 = uicontrol(fig,'Style','text','Position',[10 60 250 20],'String','Mission UAV 1: Idle','HorizontalAlignment','center','fontsize',11,'FontWeight','bold');
% InterStatusU2 = uicontrol(fig,'Style','text','Position',[300 60 250 20],'String','Mission UAV 2: Idle','HorizontalAlignment','center','fontsize',11,'FontWeight','bold');
% 
% InterAction_UAV1 = uicontrol(fig,'Style','text','Position',[50 25 250 20],'HorizontalAlignment','center','fontsize',11);
% InterAction_UAV2 = uicontrol(fig,'Style','text','Position',[350 25 250 20],'HorizontalAlignment','center','fontsize',11);
% Create panels for each UAV's status and action display fields
p2 = uipanel('Parent',fig,'Title','Online status - UAV 1','Units','normalized','Position',[0.06 0.02 0.39 0.14],'fontsize',12);
p3 = uipanel('Parent',fig,'Title','Online status - UAV 2','Units','normalized','Position',[0.53 0.02 0.39 0.14],'fontsize',12);

% Create the status and action display fields inside the panels
InterStatusU1 = uicontrol(p2,'Style','text','Position',[10 30 250 20],'String','Status: Idle','HorizontalAlignment','left','fontsize',11,'FontWeight','bold');
InterAction_UAV1 = uicontrol(p2,'Style','text','Position',[10 5 250 20],'HorizontalAlignment','left','fontsize',11);

InterStatusU2 = uicontrol(p3,'Style','text','Position',[10 30 250 20],'String','Status: Idle','HorizontalAlignment','left','fontsize',11,'FontWeight','bold');
InterAction_UAV2 = uicontrol(p3,'Style','text','Position',[10 5 250 20],'HorizontalAlignment','left','fontsize',11);
for i = 1:13
    % Calcul du résultat à stocker
    
    % Ajout du résultat à l'historique correspondant
    Policy_UAV1{i,1} = ['State ', num2str(i)];
    Policy_UAV1{i,2} = 'NOP';
    
    % Mise à jour des objets "uitable" avec les nouveaux historiques
    set(table_left,'Data',Policy_UAV1);
    
    set(InterAction_UAV1,'String',['Current Action: ', '...']);
    % Pause de 1 seconde pour permettre à l'utilisateur de voir le résultat
    %pause(1);
end

for i = 1:10
    % Calcul du résultat à stocker
    
    % Ajout du résultat à l'historique correspondant
    Policy_UAV2{i,1} = ['State ', num2str(i)];
    Policy_UAV2{i,2} = 'NOP';
    
    % Mise à jour des objets "uitable" avec les nouveaux historiques
    set(table_right,'Data',Policy_UAV2);

    set(InterAction_UAV2,'String',['Current Action: ', '...']);
    % Pause de 1 seconde pour permettre à l'utilisateur de voir le résultat
    %pause(1);
end

% Create a button to trigger table updates
btn = uicontrol('Style', 'pushbutton', 'String', 'Start mission', 'Position', [150 430 150 30],'fontsize',14,'ForegroundColor','green','FontWeight','bold', 'Callback', {@StartMission, table_left, table_right, Policy_UAV1, Policy_UAV2, InterAction_UAV1, InterAction_UAV2, r1, r2, InterStatusU1, InterStatusU2});

btn2 = uicontrol('Style', 'pushbutton', 'String', 'End mission', 'Position', [350 430 150 30],'fontsize',14,'ForegroundColor','red','FontWeight','bold', 'Callback', {@StopMission});

%----------------------------------------

function StartMission(~, ~, table_left, table_right, Policy_UAV1, Policy_UAV2, InterAction_UAV1, InterAction_UAV2, r1, r2, InterStatusU1, InterStatusU2)
    % Check which radio button is selected
    if get(r1,'Value')
msgbox('Case study with conflict ...', 'custom', struct('FontSize', 16, 'Interpreter', 'none', 'WindowStyle', 'modal'));
        conflict = 1; 
        pause(3);
        Main(conflict, table_left, table_right, Policy_UAV1, Policy_UAV2, InterAction_UAV1, InterAction_UAV2, InterStatusU1, InterStatusU2);
        msgbox('End of simulations...', 'custom', struct('FontSize', 16, 'Interpreter', 'none', 'WindowStyle', 'modal'));
        StopMission;
    elseif get(r2,'Value')
        msgbox('Case study without conflict ...', 'custom', struct('FontSize', 16, 'Interpreter', 'none', 'WindowStyle', 'modal'));
        conflict = 0;
        pause(3);
        Main(conflict, table_left, table_right, Policy_UAV1, Policy_UAV2, InterAction_UAV1, InterAction_UAV2, InterStatusU1, InterStatusU2);
        msgbox('End of simulations...', 'custom', struct('FontSize', 16));
        StopMission;
    else
         warndlg('Warning message: Please select an option .... ', 'Warning')  
    end
end


function StopMission(~, ~)
    disp("End of simulations");
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections

    sim.simxStart('127.0.0.1',19999,true,true,5000,5); 
    sim.simxFinish(-1);
    sim.delete(); 
 
    pause(3);
    clc;clear;close all;
end

