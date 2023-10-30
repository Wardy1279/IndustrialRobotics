function [] = Main()
%MAIN Summary of this function goes here
%% Runs the Simulation.
    clc
    clf
    clear all
    
    hold on;
    
    steps = 100;
    qLastUR = [0,0,0,0,0,0];
    qLastSB = [0,0,0,0,0,0];

    global robotSelection
    robotSelection = 0;
    %% Creates environment, returns interactive objects and robots
    %% (including grippers).
    [SBrobot, URrobot, URGripper1, URGripper2, SBGripper1, SBGripper2, nozzleObj, clothObj] = Environment.CreateEnvironment();

    %% Teach UI
    fig = uifigure;
    g = uigridlayout(fig,[1 3]);
    g.RowHeight = {'1x', '0.3x'};
    g.ColumnWidth = {'1x','1x','1x'};
    b = uibutton(g, ...
        "Text","Teach Omron", ...
        "ButtonPushedFcn", @(src,event) TeachSelection(1));
    b.Layout.Row = 1;
    b.Layout.Column = 1;
    
    c = uibutton(g, ...
        "Text","Teach IRB120", ...
        "ButtonPushedFcn", @(src,event) TeachSelection(2));
    c.Layout.Row = 1;
    c.Layout.Column = 2;

    d = uibutton(g, ...
        "Text","Teach Cartesian", ...
        "ButtonPushedFcn", @(src,event) TeachSelection(3));
    d.Layout.Row = 1;
    d.Layout.Column = 3;

    e = uibutton(g, ...
        "Text","Exit Teach", ...
        "ButtonPushedFcn", @(src,event) TeachSelection(4));
    e.Layout.Row = 2;
    e.Layout.Column = 3;
    
    pause(2);
    while robotSelection ~= 4
        pause(1);
        if robotSelection == 1
            qLastUR = TeachJoy(URrobot, qLastUR);
        elseif robotSelection == 2
            qLastSB = TeachJoy(SBrobot, qLastSB);
        elseif robotSelection == 3
            inputRobotSelection = input('Robot Selection (1 = Omron, 2 = IRB120): ');
            inputX = input('X: ');
            inputY = input('Y: ');
            inputZ = input('Z: ');
            if inputRobotSelection == 1
                try
                    qLastUR = URrobot.model.ikine([1,0,0,inputX;0,1,0,inputY;0,0,1,inputZ;0,0,0,1]);
                    URrobot.model.animate(qLastUR);
                    disp(URrobot.model.fkine(URrobot.model.getpos()).T);
                end
            elseif inputRobotSelection == 2
                try
                    qLastSB = SBrobot.model.ikine([1,0,0,inputX;0,1,0,inputY;0,0,1,inputZ;0,0,0,1]);
                    SBrobot.model.animate(qLastSB);
                    disp(SBrobot.model.fkine(SBrobot.model.getpos()).T);
                end
            end
            drawnow();
            robotSelection = 0;
        end
    end
    
    %% Create Estop UI
    
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

    %% Create work area
    % cubePoints = CreateWorkArea(2);

    %% Insert person
    % persons = person(1);
    
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
    % q0 = zeros(1,6);

    % qNozzle = [-101 * pi/180, 43.2 * pi/180, -28.8 * pi/180, -7.2 * pi/180, 0, 0];
    qNozzle = SBrobot.model.ikine(nozzleObj.nozzleModel{1}.base);

    neutralSprayBotTr = SBrobot.model.fkine(qLastSB).T; % Neutral Spray Bot Position
    sprayBotEndEffectorTr = SBrobot.model.fkine(qSprayStart).T % Final Transform of End Effector.
    SBrobot.model.delay = 0;
    
    % SBrobot.model.teach(qSprayStart);
    
    initialSprayBotToNozzle = jtraj(qLastSB, qNozzle, steps);
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
    %             plot3(x_surf(i,j), y_surf(i,j), z_surf(i,j), "y*");
    %         else
    %             display("point is outside the cone");
    %             plot3(x_surf(i,j), y_surf(i,j), z_surf(i,j), "r*");
    %         end
    %     end
    % end
    
    %% Temporary for demo video
    intersectionPoints = {[-0.4, 0.4672, 0.9333] ...
                         ,[-0.2, 0.4672, 0.9333] ...
                         ,[-0.4, 0.4672, 1.0056] ...
                         ,[-0.2, 0.4672, 1.0056] ...
                         ,[-0.4, 0.4672, 1.0778] ...
                         ,[-0.2, 0.4672, 1.0778] ...
                         ,[-0.4, 0.4672, 1.15] ...
                         ,[-0.2, 0.4672, 1.15]};

    for i = 1:length(intersectionPoints)
        intersectionPoints_h{i} = plot3(intersectionPoints{i}(1), intersectionPoints{i}(2), intersectionPoints{i}(3), "r*");
    end
    
    pause(2);
    delete(sprayCone_h);

    sprayBotToNeutralPath = jtraj(qSprayStart, qLastSB, steps);
    
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
    q0 = zeros(1,6);
    qElbowUp = [0, -pi/3, pi/3, 0, 0, 0];
    armToElbowUp = jtraj(q0, qElbowUp, steps/2); % Moving arm to an elbow up configuration stops arm from colliding with table.
    
    for i = 1:length(armToElbowUp)
        URrobot.model.animate(armToElbowUp(i,:))
        URGripper1.model.base = URrobot.model.fkine(armToElbowUp(i,:));
        URGripper1.model.animate([0,0]);
        URGripper2.model.base = URrobot.model.fkine(armToElbowUp(i,:));
        URGripper2.model.animate([0,0]);
        pause(0);
    end
    q0 = armToElbowUp(end, :); % Reinitialise Starting Position.

    qWipeGuess = [pi/2, -pi/3, 100*pi/180, -2*pi/3, -pi/2, 0];
    armToWipe = jtraj(q0, URrobot.model.ikcon(clothObj.clothModel{1}.base.T * transl(0,0,0.3) * troty(pi), qWipeGuess), steps);
    for i = 1:length(armToWipe)
        URrobot.model.animate(armToWipe(i,:));
        URGripper1.model.base = URrobot.model.fkine(armToWipe(i,:));
        URGripper1.model.animate([0,0]);
        URGripper2.model.base = URrobot.model.fkine(armToWipe(i,:));
        URGripper2.model.animate([0,0]);
        pause(0);
    end
    q0 = armToWipe(end,:); % Reinitialise Starting Position.   

    qIntersectionGuess = [-83*pi/180, -140*pi/180, 2*pi/3, -150*pi/180, -pi/2, 0];
    for i = 1:length(intersectionPoints)
        armToIntersectionPoint = jtraj(q0, URrobot.model.ikcon(SE3(intersectionPoints{i}).T * transl(0,-0.2,0) * trotx(-pi/2), qIntersectionGuess), steps);
        for j = 1:length(armToIntersectionPoint)
            URrobot.model.animate(armToIntersectionPoint(j,:));
            URGripper1.model.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
            URGripper1.model.animate([0,0]);
            URGripper2.model.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
            URGripper2.model.animate([0,0]);
            clothObj.clothModel{1}.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
            clothObj.clothModel{1}.animate(0);
            pause(0);
        end
        delete(intersectionPoints_h{i});
        q0 = armToIntersectionPoint(end,:);
    end

    qNeutral = [0, -pi/3, pi/2, -2*pi/3, 0, 0];
    armToNeutral = jtraj(q0, qNeutral, steps);

    for i = 1:length(armToNeutral)
        URrobot.model.animate(armToNeutral(i,:));
        URGripper1.model.base = URrobot.model.fkine(armToNeutral(i,:));
        URGripper1.model.animate([0,0]);
        URGripper2.model.base = URrobot.model.fkine(armToNeutral(i,:));
        URGripper2.model.animate([0,0]);
        clothObj.clothModel{1}.base = URrobot.model.fkine(armToNeutral(i,:));
        clothObj.clothModel{1}.animate(0);
        pause(0);
    end
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

function [] = TeachSelection(selection)
disp('Pressed button')
global robotSelection
robotSelection = selection;
end
