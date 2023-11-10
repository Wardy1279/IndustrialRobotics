function [] = Main()
%MAIN Summary of this function goes here
%% Runs the Simulation.
    clc
    clf
    clear all
    
    hold on;
    
    steps = 50;
    
    %% init Arduino
    a = arduino();
    writeDigitalPin(a, 'D9', 1);
    %% OPTIONAL SETTINGS
    % Turn on light cutain demonstration
    settingsLightCurtainEnable = false;

    % Teach UI
    % From assignment overview: 2) 'A valid addition is to use a joystick
    % or gamepad to augment and replace mouse GUI button presses'
    settingsTeach = false;

    %% Creates environment, returns interactive objects and robots
    %% (including grippers).
    [SBrobot, URrobot, URGripper1, URGripper2, SBGripper1, SBGripper2, nozzleObj, clothObj] = Environment.CreateEnvironment();
    
    if settingsTeach == true
        [qLastUR, qLastSB] = Teach(URrobot, SBrobot, URGripper1, URGripper2, SBGripper2, SBGripper2);
    else
        qLastUR = zeros(1,6);
        qLastSB = zeros(1,6);
    end
    
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
        "ButtonPushedFcn", @(src,event) EStopCallback(1, a));
    estopButton.Layout.Row = 2;
    estopButton.Layout.Column = 2;

    c = uibutton(g, ...
        "Text","Resume", ...
        "ButtonPushedFcn", @(src,event) EStopCallback(2, a));
    c.Layout.Row = 2;
    c.Layout.Column = 3;

    if settingsLightCurtainEnable == true
        cubePoints = CreateWorkArea(2);
        persons = person(1);
    end

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
    
    if settingsLightCurtainEnable == true
        persons.personModel{1}.base = SE3(eul2rotm([0,0,0]), [-3, -0.5, 0]);
        persons.personModel{1}.animate(0);
    end
    %% Moving arm to an elbow up configuration stops arm from colliding with table.
    qInitial = zeros(1,6);
    armToElbowUp = jtraj(qLastUR, qInitial, steps); % Moving arm to an elbow up configuration stops arm from colliding with table.
    qMatrixURGripper1 = URGripper1.GetOpenqMatrix(URGripper1, steps);
    qMatrixURGripper2 = URGripper2.GetOpenqMatrix(URGripper2, steps);
    
    intersection = collisionDetection(URrobot,armToElbowUp);

    for i = 1:length(armToElbowUp)
        physicalEstop = readDigitalPin(a, 'D8');
        if physicalEstop == true
            EStopCallback(1, a);
        end
        if (isStopped == false) && (intersection == false)
            URrobot.model.animate(armToElbowUp(i,:))
            URGripper1.model.base = URrobot.model.fkine(armToElbowUp(i,:));
            URGripper1.model.animate(qMatrixURGripper1(i,:));
            URGripper2.model.base = URrobot.model.fkine(armToElbowUp(i,:));
            URGripper2.model.animate(qMatrixURGripper2(i,:));
            pause(0);
        else
            if intersection == true
                disp("Collision Detected. Operation Paused")
                pause;
            end
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end
    %% Spray cone (THIS COULD BE A SPRAY FUNCTION SO TAHT THE CONE ONLY GENERATES WHEN THAT FUCNTION IS CALLED).
    qNozzle = SBrobot.model.ikine(nozzleObj.nozzleModel{1}.base);
    qSprayStart = [-145 * pi/180, 0, -14 * pi/180, 93.6 * pi/180, -80 * pi/180 , -pi]; % Spray Bot initial Joint Angles.

    % neutralSprayBotTr = SBrobot.model.fkine(qLastSB).T; % Neutral Spray Bot Position
    sprayBotEndEffectorTr = SBrobot.model.fkine(qSprayStart).T; % Final Transform of End Effector.
    qMatrixSBGripper1 = SBGripper1.GetOpenqMatrix(SBGripper1, steps);
    qMatrixSBGripper2 = SBGripper2.GetOpenqMatrix(SBGripper2, steps);
    SBrobot.model.delay = 0;
    
    initialSprayBotToNozzle = jtraj(qLastSB, qNozzle, steps);

    intersection = collisionDetection(SBrobot,initialSprayBotToNozzle);

    for i = 1:length(initialSprayBotToNozzle)
        physicalEstop = readDigitalPin(a, 'D8');
        if physicalEstop == true
            EStopCallback(1, a);
        end
        if (isStopped == false) && (intersection == false)
            if settingsLightCurtainEnable
                try
                    persons.personModel{1}.base = SE3(eul2rotm([0,0,0]), [-3+i/35, -0.5, 0]);
                    isStopped = UpdateCollisions(persons, cubePoints);
                    if isStopped == true
                        writeDigitalPin(a, 'D9', 0);
                    end
                end
            end
            SBrobot.model.animate(initialSprayBotToNozzle(i,:));
            SBGripper1.model.base = SBrobot.model.fkine(initialSprayBotToNozzle(i,:));
            SBGripper1.model.animate(qMatrixSBGripper1(i,:));
            SBGripper2.model.base = SBrobot.model.fkine(initialSprayBotToNozzle(i,:));
            SBGripper2.model.animate(qMatrixSBGripper2(i,:));
            pause(0);
        else
            if intersection == true
                disp("Collision Detected. Operation Paused")
                pause;
            end
            while isStopped == true
                pause(1)
                try
                    delete(persons);
                end
                % Damn, penalty time
            end
        end
    end
    
    nozzleToSprayPosition = jtraj(qNozzle, qSprayStart, steps);

    intersection = collisionDetection(SBrobot,nozzleToSprayPosition);

    %% Grip Nozzle
    qMatrixSBGripper1 = SBGripper1.GetCloseqMatrix(SBGripper1, steps);
    qMatrixSBGripper2 = SBGripper2.GetCloseqMatrix(SBGripper2, steps);

    for i = 1:steps
        SBGripper1.model.animate(qMatrixSBGripper1(i,:));
        SBGripper2.model.animate(qMatrixSBGripper2(i,:));
        pause(0);
    end
    %% Nozzle to Spray Position

    for i = 1:length(nozzleToSprayPosition)
        physicalEstop = readDigitalPin(a, 'D8');
        if physicalEstop == true
            EStopCallback(1, a);
        end
        if (isStopped == false) && (intersection == false)
            SBrobot.model.animate(nozzleToSprayPosition(i,:));
            SBGripper1.model.base = SBrobot.model.fkine(nozzleToSprayPosition(i,:));
            SBGripper1.model.animate([0,0]);
            SBGripper2.model.base = SBrobot.model.fkine(nozzleToSprayPosition(i,:));
            SBGripper2.model.animate([0,0]);
            nozzleObj.nozzleModel{1}.base = SBrobot.model.fkine(nozzleToSprayPosition(i,:));
            nozzleObj.nozzleModel{1}.animate(0);
            pause(0);
        else
            if intersection == true
                disp("Collision Detected. Operation Paused")
                pause;  
            end
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end 
    end
    
    %% Spray on Surface
    [intersectionPoints, intersectionPoints_h] = sprayBottle(nozzleObj, sprayBotEndEffectorTr, x_surf, y_surf, z_surf);

    sprayBotToNeutralPath = jtraj(qSprayStart, zeros(1,6), steps);
    
    intersection = collisionDetection(SBrobot,sprayBotToNeutralPath);

    for i = 1:length(sprayBotToNeutralPath)
        physicalEstop = readDigitalPin(a, 'D8');
        if physicalEstop == true
            EStopCallback(1, a);
        end
        if (isStopped == false) && (intersection == false)
            SBrobot.model.animate(sprayBotToNeutralPath(i, :));
            SBGripper1.model.base = SBrobot.model.fkine(sprayBotToNeutralPath(i,:));
            SBGripper1.model.animate(qMatrixSBGripper1(end,:));
            SBGripper2.model.base = SBrobot.model.fkine(sprayBotToNeutralPath(i,:));
            SBGripper2.model.animate(qMatrixSBGripper2(end,:));
            nozzleObj.nozzleModel{1}.base = SBrobot.model.fkine(sprayBotToNeutralPath(i,:));
            nozzleObj.nozzleModel{1}.animate(0);
            pause(0);
        else
            if intersection == true
                disp("Collision Detected. Operation Paused")
                pause;
            end
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end
    
   
    %% Wiping of Intersection Points
    % q0 = zeros(1,6);
    qElbowUp = [0, -pi/3, pi/3, 0, 0, 0]
    armToElbowUp = jtraj(qInitial, qElbowUp, steps/2); % Moving arm to an elbow up configuration stops arm from colliding with table.
    
    intersection = collisionDetection(URrobot,armToElbowUp);

    for i = 1:length(armToElbowUp)
        physicalEstop = readDigitalPin(a, 'D8');
        if physicalEstop == true
            EStopCallback(1, a);
        end
        if (isStopped == false) && (intersection == false)
            URrobot.model.animate(armToElbowUp(i,:))
            % disp(armToElbowUp(i,:));
            URGripper1.model.base = URrobot.model.fkine(armToElbowUp(i,:));
            URGripper1.model.animate(qMatrixURGripper1(end,:));
            URGripper2.model.base = URrobot.model.fkine(armToElbowUp(i,:));
            URGripper2.model.animate(qMatrixURGripper2(end,:));
            pause(0);
        else
            if intersection == true
                disp("Collision Detected. Operation Paused")
                pause;
            end
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end

    qCloth = URrobot.model.ikine(clothObj.clothModel{1}.base)
    armToWipe = jtraj(qElbowUp, qCloth, steps);

    intersection = collisionDetection(URrobot,armToWipe);

    for i = 1:length(armToWipe)
        physicalEstop = readDigitalPin(a, 'D8');
        if physicalEstop == true
            EStopCallback(1, a);
        end
        if (isStopped == false) && (intersection == false)
            URrobot.model.animate(armToWipe(i,:));
            % disp(armToWipe(i,:));
            URGripper1.model.base = URrobot.model.fkine(armToWipe(i,:));
            URGripper1.model.animate(qMatrixURGripper1(end,:));
            URGripper2.model.base = URrobot.model.fkine(armToWipe(i,:));
            URGripper2.model.animate(qMatrixURGripper2(end,:));
            pause(0);
        else
            if intersection == true
                disp("Collision Detected. Operation Paused")
                pause;
            end
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end

    q0 = armToWipe(end,:); % Reinitialise Starting Position.   

    %% Grip Cloth
    qMatrixURGripper1 = URGripper1.GetCloseqMatrix(URGripper1, steps);
    qMatrixURGripper2 = URGripper2.GetCloseqMatrix(URGripper2, steps);

    for i = 1:length(qMatrixSBGripper1)
        URGripper1.model.animate(qMatrixURGripper1(i,:));
        URGripper2.model.animate(qMatrixURGripper2(i,:));
        drawnow();
    end
    
    %% Move Wiper to Intersection Points
    qIntersectionGuess = [-pi/2, -140*pi/180, 2*pi/3, -150*pi/180, -pi/2, 0];
    
    for i = 1:length(intersectionPoints)
        armToIntersectionPoint = jtraj(q0, URrobot.model.ikcon(SE3(intersectionPoints{i}).T * transl(0,-0.2,0) * trotx(-pi/2), qIntersectionGuess), steps);
        
        % FOR SOME REASON THIS COLLISION DETECTION DOES NOT DETECT THE WALL-CUBE
        intersection = collisionDetection(URrobot,armToIntersectionPoint);
        
        % print q goal
        disp(['Goal q for point ' num2str(i) ': ' num2str(URrobot.model.ikcon(SE3(intersectionPoints{i}).T * transl(0,-0.2,0) * trotx(-pi/2)))]);
            
            for j = 1:length(armToIntersectionPoint)
                physicalEstop = readDigitalPin(a, 'D8');
                    if physicalEstop == true
                        EStopCallback(1, a);
                    end
                if (isStopped == false) && (intersection == false)
                    URrobot.model.animate(armToIntersectionPoint(j,:));
                    % disp(armToIntersectionPoint(j,:));
                    URGripper1.model.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
                    URGripper1.model.animate(qMatrixURGripper1(end,:));
                    URGripper2.model.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
                    URGripper2.model.animate(qMatrixURGripper2(end,:));
                    clothObj.clothModel{1}.base = URrobot.model.fkine(armToIntersectionPoint(j,:));
                    clothObj.clothModel{1}.animate(0);
                    pause(0);
                else
                    if intersection == true
                        disp("Collision Detected. Operation Paused")
                        pause;
                    end
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

    %% Move Arm to Neutral Position
    qNeutral = [-2*pi, -pi/3, pi/2, -2*pi/3, 0, 0];
    armToNeutral = jtraj(q0, qNeutral, steps);

    intersection = collisionDetection(URrobot,armToNeutral);

    for i = 1:length(armToNeutral)
        physicalEstop = readDigitalPin(a, 'D8');
        if physicalEstop == true
            EStopCallback(1, a);
        end
        if (isStopped == false) && (intersection == false)
            URrobot.model.animate(armToNeutral(i,:));
            % disp(armToNeutral(i,:));
            URGripper1.model.base = URrobot.model.fkine(armToNeutral(i,:));
            URGripper1.model.animate(qMatrixURGripper1(end,:));
            URGripper2.model.base = URrobot.model.fkine(armToNeutral(i,:));
            URGripper2.model.animate(qMatrixURGripper2(end,:));
            clothObj.clothModel{1}.base = URrobot.model.fkine(armToNeutral(i,:));
            clothObj.clothModel{1}.animate(0);
            pause(0);
        else
            if intersection == true
                disp("Collision Detected. Operation Paused")
                pause;
            end
            while isStopped == true
                pause(1)
                % Damn, penalty time
            end
        end
    end

    input("End of Simulation");
end

%% Estop Callback function
function [] = EStopCallback(selection, a)
global isStopped;
if selection == 1
    disp('ESTOP PRESSED');
    isStopped = true;
    writeDigitalPin(a, 'D9', 0);
else
    disp('RESUME PRESSED');
    isStopped = false;
    writeDigitalPin(a, 'D9', 1);
end
end