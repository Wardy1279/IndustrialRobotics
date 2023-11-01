function [] = Main()
%MAIN Summary of this function goes here
%% Runs the Simulation.
    clc
    clf
    clear all
    
    hold on;
    
    steps = 100;

    %% Creates environment, returns interactive objects and robots
    %% (including grippers).
    [SBrobot, URrobot, URGripper1, URGripper2, SBGripper1, SBGripper2, nozzleObj, clothObj] = Environment.CreateEnvironment();

    [qLastUR, qLastSB] = Teach(URrobot, SBrobot);
    
    %% Create Estop UI
    % There were some serious scoping issues regarding asynchronously
    % stopping the robot, The estop ui must be created here to address this

    global isStopped
    isStopped = false;

    fig = uifigure;
    g = uigridlayout(fig,[1 3]);
    g.RowHeight = {'0x'};
    g.ColumnWidth = {'1x','1x','1x'};
    estopButton = uibutton(g, ...
        "Text","Emergency Stop", ...
        "ButtonPushedFcn", @(src,event) EStopCallback(1));
    estopButton.Layout.Row = 2;
    estopButton.Layout.Column = 2;

    c = uibutton(g, ...
        "Text","Resume", ...
        "ButtonPushedFcn", @(src,event) EStopCallback(2));
    c.Layout.Row = 2;
    c.Layout.Column = 3;

    %% Create work area
    % cubePoints = CreateWorkArea(2);

    %% Insert person
    % persons = person(1);
    
    %% SprayBot Starting Position
    % qSprayStart = [-145 * pi/180, 0, -14 * pi/180, 93.6 * pi/180, -80 * pi/180 , -pi]; % Spray Bot initial Joint Angles.
    % SBrobot.model.animate(qSprayStart); % Render Spray bot at initial joint angels.
    %% Render Plane that acts as window pane.
    [x_surf, z_surf] = meshgrid(-1.8:0.2:1.8, 0.5:1.3/18:1.8); % Generate x_surf and z_surf data
    y_surf = zeros(size(x_surf, 1)); % Initialise array of y_surf values same size and x_surf values.
    y_surf(:) = 0.46723; % Change all y_surf values to intended plane location.
    windowPane = surf(x_surf, y_surf, z_surf); % Render window pane surface.
    set(windowPane, 'facealpha', 0.1); % Change Transparency of windowPane.
    
    %% Moving Wiping robot out of the way
    max_angle = 2*pi;
    min_angle = -2*pi;
    % % Convert q to within range -2*pi<q<2*pi
    for i = 1:numel(qLastUR)
        while qLastUR(i) < min_angle
            qLastUR(i) = qLastUR(i) + 2 * pi;
        end
        while qLastUR(i) > max_angle
            qLastUR(i) = qLastUR(i) - 2 * pi;
        end
    end

    for i = 1:numel(qLastSB)
        while qLastSB(i) < min_angle
            qLastSB(i) = qLastSB(i) + 2 * pi;
        end
        while qLastSB(i) > max_angle
            qLastSB(i) = qLastSB(i) - 2 * pi;
        end
    end

    q0 = zeros(1,6);
    armToElbowUp = jtraj(qLastUR, q0, steps); % Moving arm to an elbow up configuration stops arm from colliding with table.
    
    for i = 1:length(armToElbowUp)
        if isStopped == false
            URrobot.model.animate(armToElbowUp(i,:))
            URGripper1.model.base = URrobot.model.fkine(armToElbowUp(i,:));
            URGripper1.model.animate([0,0]);
            URGripper2.model.base = URrobot.model.fkine(armToElbowUp(i,:));
            URGripper2.model.animate([0,0]);
            pause(0);
        else
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end
    %% Spray cone (THIS COULD BE A SPRAY FUNCTION SO TAHT THE CONE ONLY GENERATES WHEN THAT FUCNTION IS CALLED).
    qNozzle = SBrobot.model.ikine(nozzleObj.nozzleModel{1}.base);
    qSprayStart = [-145 * pi/180, 0, -14 * pi/180, 93.6 * pi/180, -80 * pi/180 , -pi]; % Spray Bot initial Joint Angles.

    neutralSprayBotTr = SBrobot.model.fkine(qLastSB).T; % Neutral Spray Bot Position
    sprayBotEndEffectorTr = SBrobot.model.fkine(qSprayStart).T; % Final Transform of End Effector.
    SBrobot.model.delay = 0;
    
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
    centerOfConeBase = sprayBotEndEffectorTr(1:3, 4) + 0.5 * sprayBotEndEffectorTr(1:3,3); 
    vectorFromApexToBaseOfCone = centerOfConeBase - startSpray; % Generate center line of spray cone.
    unitVectorOfCone = vectorFromApexToBaseOfCone/norm(vectorFromApexToBaseOfCone); % Generate Unit vector of center line.
    plot3([startSpray(1), centerOfConeBase(1)],[startSpray(2), centerOfConeBase(2)],[startSpray(3), centerOfConeBase(3)], "b--"); % plot center line on figure.
    thetaOfSprayCone = 23.578;
    
    % Initialising apex point of cone.
    x_apex = startSpray(1,1); 
    y_apex = startSpray(2,1);
    z_apex = startSpray(3,1);
    
    intersectionPoints = {}; % Creating variable to store intersection points.
    for i = 1:length(x_surf)
        % 3D values of point.
        for j = 1:length(x_surf)
            % Vect
            vectorFromApexToPoint = [x_surf(i,j) - x_apex, y_surf(i,j) - y_apex, z_surf(i,j) - z_apex]; % Calculating vector from spray nozzle to point on plane.
            unitVectorFromApexToPoint = vectorFromApexToPoint/norm(vectorFromApexToPoint); % unit vector
            cosTheta = max(min(dot(unitVectorFromApexToPoint,unitVectorOfCone)/(norm(unitVectorFromApexToPoint)*norm(unitVectorOfCone)),1),-1); % Calculating angle between point line and cone center line.
            thetaInDegrees = real(acosd(cosTheta)); % Converting to degrees
            
            if abs(thetaInDegrees) <= thetaOfSprayCone && norm(vectorFromApexToPoint) < 0.75 % Finding if point is within the length and spread of spray cone.
                % display("point is inside the cone");
                intersectionPoints{length(intersectionPoints) + 1} = [x_surf(i,j), y_surf(i,j), z_surf(i,j)]; % Storing point location
                intersectionPoints_h{length(intersectionPoints)} = plot3(x_surf(i,j), y_surf(i,j), z_surf(i,j), "r*"); % Plotting point on figure.
            end
        end
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
    qElbowUp = [0, -pi/3, pi/3, 0, 0, 0];
    armToElbowUp = jtraj(q0, qElbowUp, steps/2); % Moving arm to an elbow up configuration stops arm from colliding with table.
    
    for i = 1:length(armToElbowUp)
        if isStopped == false
            URrobot.model.animate(armToElbowUp(i,:))
            URGripper1.model.base = URrobot.model.fkine(armToElbowUp(i,:));
            URGripper1.model.animate([0,0]);
            URGripper2.model.base = URrobot.model.fkine(armToElbowUp(i,:));
            URGripper2.model.animate([0,0]);
            pause(0);
        else
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end

    qCloth = URrobot.model.ikine(clothObj.clothModel{1}.base);
    armToWipe = jtraj(qElbowUp, qCloth, steps);
    for i = 1:length(armToWipe)
        if isStopped == false
            URrobot.model.animate(armToWipe(i,:));
            URGripper1.model.base = URrobot.model.fkine(armToWipe(i,:));
            URGripper1.model.animate([0,0]);
            URGripper2.model.base = URrobot.model.fkine(armToWipe(i,:));
            URGripper2.model.animate([0,0]);
            pause(0);
        else
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end

    q0 = armToWipe(end,:); % Reinitialise Starting Position.   
    qIntersectionGuess = [-83*pi/180, -140*pi/180, 2*pi/3, -150*pi/180, -pi/2, 0];
    
    for i = 1:length(intersectionPoints)
        armToIntersectionPoint = jtraj(q0, URrobot.model.ikcon(SE3(intersectionPoints{i}).T * transl(0,-0.2,0) * trotx(-pi/2), qIntersectionGuess), steps);
            
            for j = 1:length(armToIntersectionPoint)
                if isStopped == false
                    URrobot.model.animate(armToIntersectionPoint(j,:));
                    URGripper1.model.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
                    URGripper1.model.animate([0,0]);
                    URGripper2.model.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
                    URGripper2.model.animate([0,0]);
                    clothObj.clothModel{1}.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
                    clothObj.clothModel{1}.animate(0);
                    pause(0);
                else
                    while isStopped == true
                        pause(1)
                        % Damn, penalty time
                    end
                end
            end
        delete(intersectionPoints_h{i});
        q0 = armToIntersectionPoint(end,:);
    end
    intersectionPoints = {};

    qNeutral = [0, -pi/3, pi/2, -2*pi/3, 0, 0];
    armToNeutral = jtraj(q0, qNeutral, steps);

    for i = 1:length(armToNeutral)
        if isStopped == false
            URrobot.model.animate(armToNeutral(i,:));
            URGripper1.model.base = URrobot.model.fkine(armToNeutral(i,:));
            URGripper1.model.animate([0,0]);
            URGripper2.model.base = URrobot.model.fkine(armToNeutral(i,:));
            URGripper2.model.animate([0,0]);
            clothObj.clothModel{1}.base = URrobot.model.fkine(armToNeutral(i,:));
            clothObj.clothModel{1}.animate(0);
            pause(0);
        else
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end

    input("End of Simulation");
end

%% Estop Callback function
function [] = EStopCallback(selection)
global isStopped;
if selection == 1
    disp('ESTOP PRESSED');
    isStopped = true;
else
    disp('RESUME PRESSED');
    isStopped = false;
end
end