function [isCollision] = UpdateCollisions(persons, cubePoints)
    persons.personModel{1}.animate(0);
    
    base = persons.personModel{1}.base.T;
    bounds = persons.GetBounds();
    
    centerPoint = [base(1,4),base(2,4),(bounds(6)+bounds(5))/2];
    % radii = [bounds(2)-bounds(1),bounds(4)-bounds(3),bounds(6)-bounds(5)]
    radii = [(bounds(2)-bounds(1))/2 + 0.1,(bounds(4)-bounds(3))/2 + 0.1,(bounds(6)-bounds(5))/2 + 0.1];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    view(3);
    
    % 2.2
    % ellipsoidAtOrigin_h = surf(X,Y,Z);
    % Make the ellipsoid translucent (so we can see the inside and outside points)
    
    % 2.4
    % algebraicDist = GetAlgebraicDist(cubePoints, centerPoint, radii);
    algebraicDist = ((cubePoints(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((cubePoints(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((cubePoints(:,3)-centerPoint(3))/radii(3)).^2;
    pointsInside = find(algebraicDist < 1);
    % disp(['There are ', num2str(size(pointsInside,1)),' points inside']);
    pause(0.05)
    try
        delete(ellipsoidAtOrigin_h);
    end
    if size(pointsInside,1) > 0
        isCollision = 1;
        disp('Light Curtain has detected an object! Halting system');
    else
        isCollision = 0;
    end
end

