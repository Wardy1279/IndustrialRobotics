function [] = Main()
%MAIN Summary of this function goes here
%% Runs the Simulation.
    clc
    clf
    clear all
    
    hold on;

    %% Creates environment, returns interactive objects and robots
    %% (including grippers).
    [SBrobot, URrobot, URGripper1, URGripper2, SBGripper1, SBGripper2, nozzleObj, clothObj] = Environment.CreateEnvironment();

    %% SprayBot Starting Position
    qSprayStart = [-223*pi/180, 0, 0, 0, 0, 0]; % Spray Bot initial Joint Angles.
    SBrobot.model.animate(qSprayStart); % Render Spray bot at initial joint angels.
    
    %% Render Plane that acts as window pane.
    [x z] = meshgrid(-1.8:0.2:1.8, 0.5:1.3/18:1.8); % Generate x and z data
    y = zeros(size(x, 1)); % Initialise array of y values same size and x values.
    y(:) = 0.46723; % Change all y values to intended plane location.
    windowPane = surf(x, y, z); % Render window pane surface.
    set(windowPane, 'facealpha', 0.1); % Change Transparency of windowPane.

    %% Spray cone (THIS COULD BE A SPRAY FUNCTION SO TAHT THE CONE ONLY GENERATES WHEN THAT FUCNTION IS CALLED).
    sprayBotEndEffectorTr = SBrobot.model.fkine(qSprayStart).T; % Transform of End Effector.
    SBrobot.model.delay = 0;
    startSpray = sprayBotEndEffectorTr(1:3,4);  % Starting Point location of Spray Cone.
    [X, Y, Z] = cylinder([0, 0.2], 30); % Define starting end ending radius of Spray Cone.
    Z = Z * 0.5; % Define Length of Spray Cone.
    updatedSprayConePoints = [sprayBotEndEffectorTr * [X(:), Y(:), Z(:), ones(numel(X), 1)]']';
    sprayConePointsSize = size(X);
    sprayCone_h = surf(reshape(updatedSprayConePoints(:,1), sprayConePointsSize) ... % Render Spray Cone.
                      ,reshape(updatedSprayConePoints(:,2), sprayConePointsSize) ...
                      ,reshape(updatedSprayConePoints(:,3), sprayConePointsSize));
    set(sprayCone_h, 'FaceAlpha', 0.4); % Change transparency of Spray Cone.
    view(3);
    

    %% Intersection Point Storage
    for i = 1 : size(updatedSprayConePoints, 3) - 1
        for faceIndex = 1:size()

        end
    end

    %% Wiping of Intersection Points

    % qlims = URrobot.model.qlim(:,:);
    % stepRads = deg2rad(60);
    % pointCloudSize = prod(floor((qlims(1:5,2)-qlims(1:5,1))/stepRads + 1));
    % pointCloud = zeros(pointCloudSize,3);
    % counter = 1;
    % disp("-------------------------------");
    % tic
    % disp("Calculating Potential Poses.");
    % disp("-------------------------------");
    % 
    % for q1 = qlims(1,1):stepRads:qlims(1,2)
    %     for q2 = qlims(2,1):stepRads:qlims(2,2)
    %         for q3 = qlims(3,1):stepRads:qlims(3,2)
    %             for q4 = qlims(4,1):stepRads:qlims(4,2)
    %                 for q5 = qlims(5,1):stepRads:qlims(5,2)       
    %                     q6 = 0; % q6 does not effect xyz, so we can forgo looping through its joint configs
    %                     % to save time
    %                     q = [q1,q2,q3,q4,q5,q6];
    %                     tr = URrobot.model.fkine(q).T;
    %                     % Uncomment if you want to see the robot arm cycle through all the poses.
    %                     % Makes process of calculating about 20x longer.
    %                     % URrobot.model.animate(q);
    %                     % pause(0);
    %                     pointCloud(counter,:) = tr(1:3,4)';
    %                     counter = counter + 1;
    %                     if mod(counter/pointCloudSize * 100,1) == 0
    %                         disp(['After ', num2str(toc),' seconds, completed ',num2str(counter/pointCloudSize * 100),'% of poses']);
    %                     end
    %                 end
    %             end
    %         end
    %     end
    % end
    %  pointCloudPlot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
    % 
    % disp("-------------------------------");
    input("Press Enter:");
end

