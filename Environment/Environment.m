classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        function [SBrobot, URrobot, URgripper1, URgripper2, SBgripper1, SBgripper2 nozzleObj, clothObj] = CreateEnvironment()
            %% Create robots
            SBrobot = SprayBot(SE3(eul2rotm([0,0,0]), [0.3,0,0.5]).T);
            % robot1.model;
            SBrobot.model.plot([0,0,0,0,0,0], 'joints', 'workspace', [-2 2 -2 2 -0.0001 2]);
            URrobot = UR10(SE3(eul2rotm([0,0,0]), [-0.3,0,0.5]).T);
            URrobot.model;

            %% Create grippers
            % gripper for UR10 (left and right fingers)
            URgripper1 = UR10Gripper1();
            URgripper1.model;
            URgripper2 = UR10Gripper2();
            URgripper2.model;

            % gripper for SprayBot
            SBgripper1 = UR10Gripper1();
            SBgripper1.model;
            SBgripper2 = UR10Gripper2();
            SBgripper2.model;
            
            % Move gripper to arms end-effector
            URgripper1.model.base = URrobot.model.fkine(URrobot.model.getpos());
            URgripper1.model.animate([0,0]);
            URgripper2.model.base = URrobot.model.fkine(URrobot.model.getpos());
            URgripper2.model.animate([0,0]);

            SBgripper1.model.base = SBrobot.model.fkine(SBrobot.model.getpos());
            SBgripper1.model.animate([0,0]);
            SBgripper2.model.base = SBrobot.model.fkine(SBrobot.model.getpos());
            SBgripper2.model.animate([0,0]);

            %% Insert moveable objects (cloth and nozzle)
            nozzleObj = Nozzle(1);
            nozzleObj.nozzleModel{1}.base = SE3(eul2rotm([0,0,pi/2]), [0.2,-0.4,0.6]).T;
            nozzleObj.nozzleModel{1}.animate(0);

            clothObj = Cloth(1);
            clothObj.clothModel{1}.base = SE3(eul2rotm([0,0,0]), [-0.2,-0.4,0.4]).T;
            clothObj.clothModel{1}.animate(0);

            %% Surf concrete
            surf([-3,-3;3,3] ...
            ,[-3,3;-3,3]...
            ,[0,0;0,0] ...
            ,'CData',imread('concrete.jpg') ...
            ,'FaceColor','texturemap');

            %% Wall
            h_1 = PlaceObject('wall.ply',[-0.5,-0.4,0]);
            verts = [get(h_1,'Vertices'), ones(size(get(h_1,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_1,'Vertices',verts(:,1:3));
            
            %% Barriers
            h_2 = PlaceObject('barrier1.5x0.2x1m.ply',[0.5,-3,0]);
            verts = [get(h_2,'Vertices'), ones(size(get(h_2,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 1.5;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_2,'Vertices',verts(:,1:3));

            h_3 = PlaceObject('barrier1.5x0.2x1m.ply',[0.5,3,0]);
            verts = [get(h_3,'Vertices'), ones(size(get(h_3,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 1.5;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_3,'Vertices',verts(:,1:3));

            h_6 = PlaceObject('barrier1.5x0.2x1m.ply',[0,-2.8,0]);
            verts = [get(h_6,'Vertices'), ones(size(get(h_6,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * 2.5;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_6,'Vertices',verts(:,1:3));

            %% Safety
            h_8 = PlaceObject('fireExtinguisherElevated.ply',[2.9,-3,0.5]);
            verts = [get(h_8,'Vertices'), ones(size(get(h_8,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_8,'Vertices',verts(:,1:3));

            h_9 = PlaceObject('emergencyStopWallMounted.ply',[2.1,2.9,0.8]);
            verts = [get(h_9,'Vertices'), ones(size(get(h_9,'Vertices'),1),1)] * trotz(pi);
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_9,'Vertices',verts(:,1:3));

            h_10 = PlaceObject('emergencyStopButton.ply',[-1.4,-2,1.6]);
            verts = [get(h_10,'Vertices'), ones(size(get(h_10,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * 0.3;
            verts(:,2) = verts(:,2) * 0.3;
            verts(:,3) = verts(:,3) * 0.3;
            set(h_10,'Vertices',verts(:,1:3));

            %% Work Station
            h_11 = PlaceObject('base.ply',[0.5,0,0]);
            verts = [get(h_11,'Vertices'), ones(size(get(h_11,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,1) = verts(:,1) * 0.3;
            verts(:,2) = verts(:,2) * 0.5;
            verts(:,3) = verts(:,3) * 0.3;
            set(h_11,'Vertices',verts(:,1:3));
        end
    end
end

