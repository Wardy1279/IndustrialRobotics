function [] = Main()
%MAIN Summary of this function goes here
%% Runs the Simulation.
    clc
    clf
    clear all
    
    hold on;
    
    steps = 100;
    %% Create UI
    
    global isStopped
    isStopped = false;

    fig = uifigure;
    g = uigridlayout(fig,[1 3]);
    g.RowHeight = {'0x'};
    g.ColumnWidth = {'1x','1x','1x'};
    estopButton = uibutton(g, ...
        "Text","Emergency Stop", ...
        "ButtonPushedFcn", @(src,event) Estop());
    estopButton.Layout.Row = 2;
    estopButton.Layout.Column = 2;
    
    c = uibutton(g, ...
        "Text","Resume", ...
        "ButtonPushedFcn", @(src,event) Resume());
    c.Layout.Row = 2;
    c.Layout.Column = 3;

    %% Creates environment, returns interactive objects and robots
    %% (including grippers).
    [SBrobot, URrobot, URGripper1, URGripper2, SBGripper1, SBGripper2, nozzleObj, clothObj] = Environment.CreateEnvironment();
    
    %% Create work area
    cubePoints = CreateWorkArea(2);

    %% Insert person
    persons = person(1);
    
    %% SprayBot Starting Position
    qSprayStart = [-145 * pi/180, 0, -14 * pi/180, 93.6 * pi/180, -80 * pi/180 , -pi]; % Spray Bot initial Joint Angles.
    SBrobot.model.animate(qSprayStart); % Render Spray bot at initial joint angels.
    %% Render Plane that acts as window pane.
    [x_surf, z_surf] = meshgrid(-1.8:0.2:1.8, 0.5:1.3/18:1.8); % Generate x_surf and z_surf data
    y_surf = zeros(size(x_surf, 1)); % Initialise array of y_surf values same size and x_surf values.
    y_surf(:) = 0.46723; % Change all y_surf values to intended plane location.
    windowPane = surf(x_surf, y_surf, z_surf); % Render window pane surface.
    set(windowPane, 'facealpha', 0.1); % Change Transparency of windowPane.
    
    %% Spray cone (THIS COULD BE A SPRAY FUNCTION SO TAHT THE CONE ONLY GENERATES WHEN THAT FUCNTION IS CALLED).
    q0 = zeros(1,6);

    % qNozzle = [-101 * pi/180, 43.2 * pi/180, -28.8 * pi/180, -7.2 * pi/180, 0, 0];
    qNozzle = SBrobot.model.ikine(nozzleObj.nozzleModel{1}.base);

    neutralSprayBotTr = SBrobot.model.fkine(q0).T; % Neutral Spray Bot Position
    sprayBotEndEffectorTr = SBrobot.model.fkine(qSprayStart).T % Final Transform of End Effector.
    SBrobot.model.delay = 0;
    
    % SBrobot.model.teach(qSprayStart);
    
    initialSprayBotToNozzle = jtraj(q0, qNozzle, steps);
    for i = 1:length(initialSprayBotToNozzle)
        if isStopped == false
            SBrobot.model.animate(initialSprayBotToNozzle(i,:));
            SBGripper1.model.base = SBrobot.model.fkine(initialSprayBotToNozzle(i,:));
            SBGripper1.model.animate([0,0]);
            SBGripper2.model.base = SBrobot.model.fkine(initialSprayBotToNozzle(i,:));
            SBGripper2.model.animate([0,0]);
            pause(0);
        else
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end
    
    nozzleToSprayPosition = jtraj(qNozzle, qSprayStart, steps);
    for i = 1:length(nozzleToSprayPosition)
        if isStopped == false
            SBrobot.model.animate(nozzleToSprayPosition(i,:));
            SBGripper1.model.base = SBrobot.model.fkine(nozzleToSprayPosition(i,:));
            SBGripper1.model.animate([0,0]);
            SBGripper2.model.base = SBrobot.model.fkine(nozzleToSprayPosition(i,:));
            SBGripper2.model.animate([0,0]);
            nozzleObj.nozzleModel{1}.base = SBrobot.model.fkine(nozzleToSprayPosition(i,:));
            nozzleObj.nozzleModel{1}.animate(0);
            pause(0);
        else
           while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end 
    end
   
    nozzleTransform = nozzleObj.nozzleModel{1}.fkine(0).T;  % Nozzle Transform
    startSpray = nozzleTransform(1:3,4); % Starting point of spray cone.
    [X, Y, Z] = cylinder([0, 0.2], 100); % Define starting end ending radius of Spray Cone.
    Z = Z * 0.75; % Define Length of Spray Cone.
    updatedSprayConePoints = [sprayBotEndEffectorTr * [X(:), Y(:), Z(:), ones(numel(X), 1)]']';
    sprayConePointsSize = size(X);
    sprayCone_h = surf(reshape(updatedSprayConePoints(:,1), sprayConePointsSize) ... % Render Spray Cone.
                      ,reshape(updatedSprayConePoints(:,2), sprayConePointsSize) ...
                      ,reshape(updatedSprayConePoints(:,3), sprayConePointsSize));
    set(sprayCone_h, 'FaceAlpha', 0.4); % Change transparency of Spray Cone.
    
    %% Intersection Point Storage
    % centerOfBaseOfCone = sprayBotEndEffectorTr(1:3, 4) + 0.5 * sprayBotEndEffectorTr(1:3,3);
    % vectorFromBaseToApexOfCone = centerOfBaseOfCone - startSpray;
    % unitVectorOfCone = vectorFromBaseToApexOfCone/norm(vectorFromBaseToApexOfCone);
    % plot3([startSpray(1), centerOfBaseOfCone(1)],[startSpray(2), centerOfBaseOfCone(2)],[startSpray(3), centerOfBaseOfCone(3)], "b--");
    % coneHeight = 0.5;
    % coneRadius = 0.2;
    % theta = deg2rad(23.578);
    % 
    % x_apex = startSpray(1,1);
    % y_apex = startSpray(2,1);
    % z_apex = startSpray(3,1);
    % 
    % for i = 1:length(x_surf)
    %     % 3D values of point.
    %     for j = 1:length(x_surf)
    %         % Vect
    %         vectorFromApexToPoint = [x_surf(i,j) - x_apex, y_surf(i,j) - y_apex, z_surf(i,j) - z_apex];
    %         unitVectorFromApexToPoint = vectorFromApexToPoint/norm(vectorFromApexToPoint);
    %         angleBetweenPointUnitVectorAndConeUnitVector = dot(unitVectorFromApexToPoint, unitVectorOfCone);
    % 
    % 
    %         if abs(angleBetweenPointUnitVectorAndConeUnitVector) <= theta 
    %             display("point is inside the cone");
    %             plot3(x_surf(i,j), y_surf(i,j), z_surf(i,j), "r*");
    %         else
    %             display("point is outside the cone");
    %             plot3(x_surf(i,j), y_surf(i,j), z_surf(i,j), "y*");
    %         end
    %     end
    % end
    
    %% Temporary for demo video
    intersectionPoints = {[-0.4, 0.4672, 0.7167] ...
                         ,[-0.2, 0.4672, 0.7167] ...
                         ,[-0.4, 0.4672, 0.7899] ...
                         ,[-0.2, 0.4672, 0.7899] ...
                         ,[-0.4, 0.4672, 0.8611] ...
                         ,[-0.2, 0.4672, 0.8611] ...
                         ,[-0.4, 0.4672, 0.9333] ...
                         ,[-0.2, 0.4672, 0.9333]};
    
    for i = 1:length(intersectionPoints)
        plot3(intersectionPoints{i}(1), intersectionPoints{i}(2), intersectionPoints{i}(3), "r*")
    end
    
    pause(2);
    delete(sprayCone_h);

    sprayBotToNeutralPath = jtraj(qSprayStart, q0, steps);

    for i = 1:length(sprayBotToNeutralPath)
        if isStopped == false
            SBrobot.model.animate(sprayBotToNeutralPath(i, :));
            SBGripper1.model.base = SBrobot.model.fkine(sprayBotToNeutralPath(i,:));
            SBGripper1.model.animate([0,0]);
            SBGripper2.model.base = SBrobot.model.fkine(sprayBotToNeutralPath(i,:));
            SBGripper2.model.animate([0,0]);
            nozzleObj.nozzleModel{1}.base = SBrobot.model.fkine(sprayBotToNeutralPath(i,:));
            nozzleObj.nozzleModel{1}.animate(0);
            pause(0);
        else
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end

    %% Wiping of Intersection Points
    
    % for i = 1:length(plottedBricks)
    %     armToBrickInitial = jtraj(q0, ur3.model.ikcon(SE3(initialBricks{i}).T * transl(0,0,0.0344) * ...
    %         troty(pi), q1Guess), 200);
    %     %% Move Arm to Initial Brick Position
    %     for j = 1:length(armToBrickInitial)
    %         ur3.model.animate(armToBrickInitial(j,:));
    %         pause(0)
    %     end
    %     q0 = armToBrickInitial(end, :);
    %     %% Move Arm and Brick to Final Brick Position
    %     initialBrickToFinalBrick = jtraj(q0, ur3.model.ikcon(SE3(finalBricks{i}).T * transl(0,0,0.0344) * troty(pi), q2Guess), 200);
    %     for j = 1:length(initialBrickToFinalBrick)
    %         ur3.model.animate(initialBrickToFinalBrick(j,:));
    %         endEffectorTr = ur3.model.fkine(initialBrickToFinalBrick(j,:)).T;
    %         disp(endEffectorTr);
    %         newBrickVertices = [brickVertices{i},ones(size(brickVertices{i},1),1)] * SE3(-initialBricks{i}).T' * endEffectorTr';
    %         set(plottedBricks{i}, "Vertices", newBrickVertices(:,1:3));
    %         drawnow();
    %         pause(0)
    %     end
    %     q0 = initialBrickToFinalBrick(end, :);
    %     disp(floor(i/9 * 100) + "% of the Wall has been Assembled.");
    % end


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

function [] = Estop()
%ESTOP Summary of this function goes here
%   Detailed explanation goes here
display('ESTOP PRESSED');
global isStopped;
isStopped = true;
end

function [] = Resume()
%ESTOP Summary of this function goes here
%   Detailed explanation goes here
display('RESUME PRESSED');
global isStopped;
isStopped = false;
end
