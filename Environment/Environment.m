classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        function [robot1, robot2] = CreateEnvironment()
            % Create robots (just for testing scale, can be deleted)
            robot1 = SprayBot(SE3(eul2rotm([0,0,0]), [0.3,0,0.5]).T);
            % robot1.model;
            robot1.model.plot([0,0,0,0,0,0], 'joints', 'workspace', [-2 2 -2 2 -0.0001 2]);
            robot2 = UR10(SE3(eul2rotm([0,0,0]), [-0.3,0,0.5]).T);
            robot2.model;

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
            h_2 = PlaceObject('barrier1.5x0.2x1m.ply',[0,-1.5,0]);
            verts = [get(h_2,'Vertices'), ones(size(get(h_2,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_2,'Vertices',verts(:,1:3));

            h_3 = PlaceObject('barrier1.5x0.2x1m.ply',[1.5,-1.5,0]);
            verts = [get(h_3,'Vertices'), ones(size(get(h_3,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_3,'Vertices',verts(:,1:3));

            h_4 = PlaceObject('barrier1.5x0.2x1m.ply',[0,1.5,0]);
            verts = [get(h_4,'Vertices'), ones(size(get(h_4,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_4,'Vertices',verts(:,1:3));

            h_5 = PlaceObject('barrier1.5x0.2x1m.ply',[1.5,1.5,0]);
            verts = [get(h_5,'Vertices'), ones(size(get(h_5,'Vertices'),1),1)] * trotz(pi/2);
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_5,'Vertices',verts(:,1:3));

            h_6 = PlaceObject('barrier1.5x0.2x1m.ply',[0.75,-2.2,0]);
            verts = [get(h_6,'Vertices'), ones(size(get(h_6,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_6,'Vertices',verts(:,1:3));

            h_7 = PlaceObject('barrier1.5x0.2x1m.ply',[-0.75,-2.2,0]);
            verts = [get(h_7,'Vertices'), ones(size(get(h_7,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_7,'Vertices',verts(:,1:3));

            %% Safety
            h_8 = PlaceObject('fireExtinguisherElevated.ply',[1.6,-2.2,0.5]);
            verts = [get(h_8,'Vertices'), ones(size(get(h_8,'Vertices'),1),1)];
            verts(:,1) = verts(:,1) * 0.7;
            verts(:,2) = verts(:,2) * 0.7;
            verts(:,3) = verts(:,3) * 0.7;
            set(h_8,'Vertices',verts(:,1:3));

            h_9 = PlaceObject('emergencyStopWallMounted.ply',[1.4,2.3,0.8]);
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

