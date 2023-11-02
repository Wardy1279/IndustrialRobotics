%% Collision Avoidance for Table and Window
function [intersection] = collisionDetection(robot, qMatrix)
    
     %% Plotting the Cubes
    % 2.2 and 2.3
    centerpnt1 = [-0.3,0.8,1];
    centerpnt2 = [0,-0.25,-0.01];
    side1 = 1.5;
    side2 = 1;
    q = zeros(1,6);  
    plotOptions.plotFaces = true;
    [vertex1,faces1,faceNormals1] = RectangularPrism(centerpnt1-side1/2, centerpnt1+side1/2,plotOptions);
    [vertex2,faces2,faceNormals2] = RectangularPrism(centerpnt2-side2/2, centerpnt2+side2/2,plotOptions);
    axis equal

    %% Colision Detection
    % Get the transform of every joint (i.e. start and end of every link)
    tr = zeros(4,4,robot.model.n+1);
    tr(:,:,1) = robot.model.base;
    L = robot.model.links;
    for i = 1 : robot.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
    
    % Go through each link and also each triangle face of the wall
    for i = 1 : size(tr,3)-1    
        for faceIndex1 = 1:size(faces1,1)
            vertOnPlane1 = vertex1(faces1(faceIndex1,1)',:);
            [intersectP1,check1] = LinePlaneIntersection(faceNormals1(faceIndex1,:),vertOnPlane1,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check1 == 1 && IsIntersectionPointInsideTriangle(intersectP1,vertex1(faces1(faceIndex1,:)',:))
                disp("Wall Intersection");
                intersection = true;
                return
            else
                intersection = false;
            end
        end    
    end

    % Go through each link and also each triangle face of the table
    for i = 1 : size(tr,3)-1    
        for faceIndex2 = 1:size(faces2,1)
            vertOnPlane2 = vertex2(faces2(faceIndex2,1)',:);
            [intersectP2,check2] = LinePlaneIntersection(faceNormals2(faceIndex2,:),vertOnPlane2,tr(1:3,4,i)',tr(1:3,4,i+1)');
            if check2 == 1 && IsIntersectionPointInsideTriangle(intersectP2,vertex2(faces2(faceIndex2,:)',:))
                disp("Table Intersection");
                intersection = true;
                return
            else
                intersection = false;
            end
        end    
    end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)
    
    u = triangleVerts(2,:) - triangleVerts(1,:);
    v = triangleVerts(3,:) - triangleVerts(1,:);
    
    uu = dot(u,u);
    uv = dot(u,v);
    vv = dot(v,v);
    
    w = intersectP - triangleVerts(1,:);
    wu = dot(w,u);
    wv = dot(w,v);
    
    D = uv * uv - uu * vv;
    
    % Get and test parametric coords (s and t)
    s = (uv * wv - vv * wu) / D;

    if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
        result = 0;
        return;
    end
    
    t = (uv * wu - uu * wv) / D;

    if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
        result = 0;
        return;
    end
    
    result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
    
    if nargin < 6
        returnOnceFound = true;
    end
    
    result = false;
    
    for qIndex = 1:size(qMatrix,1)
        % Get the transform of every joint (i.e. start and end of every link)
        tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    display('Intersection');
                    result = true;
                    if returnOnceFound
                        return
                    end
                end
            end    
        end
    end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)
    links = robot.model.links;
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = robot.model.base;
    
    for i = 1:length(links)
        L = links(1,i);
        
        current_transform = transforms(:,:, i);
        
        current_transform = current_transform * trotz(q(1,i) + L.offset) * transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        transforms(:,:,i + 1) = current_transform;
    end
end