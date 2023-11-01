function [cubePoints] = CreateWorkArea(dim)
%CREATEWORKAREA Summary of this function goes here
%   Detailed explanation goes here
    [Y,Z] = meshgrid(-dim:0.2:dim,0:0.4:dim);
    sizeMat = size(Y);
    X = repmat(dim,sizeMat(1),sizeMat(2));
    
    % Combine one surface as a point cloud
    cubePoints = [X(:),Y(:),Z(:)];
    
    % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
    cubePoints = [ cubePoints ...
                 ; cubePoints * rotz(pi/2) ...
                 ; cubePoints * rotz(pi)]; 
    
    % Plot the cube's point cloud
    cubePoints = cubePoints + repmat([0,0,0],size(cubePoints,1),1);
    cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');

end

