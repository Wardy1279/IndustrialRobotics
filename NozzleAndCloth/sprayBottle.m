function [intersectionPoints, intersectionPoints_h] = sprayBottle(nozzleObj, sprayBotEndEffectorTr, x_surf, y_surf, z_surf)
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
                display("point is inside the cone");
                intersectionPoints{length(intersectionPoints) + 1} = [x_surf(i,j), y_surf(i,j), z_surf(i,j)]; % Storing point location
                intersectionPoints_h{length(intersectionPoints)} = plot3(x_surf(i,j), y_surf(i,j), z_surf(i,j), "r*"); % Plotting point on figure.
            end
        end
    end

    for i = 1:length(intersectionPoints)
        intersectionPoints_h{i} = plot3(intersectionPoints{i}(1), intersectionPoints{i}(2), intersectionPoints{i}(3), "r*");
    end
    
    pause(2);
    delete(sprayCone_h);
end