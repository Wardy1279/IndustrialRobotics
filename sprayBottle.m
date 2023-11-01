function [intersectionPoints, intersectionPoints_h] = sprayBottle(nozzleObj, sprayBotEndEffectorTr)

    global isStopped
   
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
end