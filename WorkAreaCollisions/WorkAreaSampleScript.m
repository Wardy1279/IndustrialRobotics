clear all
clf

peopleNum = 1;
dim = 1.5;

robot = UR10();
robot.model();
hold on;

cubePoints = CreateWorkArea(dim);

%% Create object
persons = person(peopleNum);
j = 0;
for i = 0:0.05:6
    persons.personModel{1}.base = SE3(eul2rotm([0,0,0]), [-3+i, 1.5, 0.12]);
    isCollision = UpdateCollisions(persons, cubePoints);
    disp(['Collision: ', num2str(isCollision)]);
    if isCollision == false
        robot.model.fkine([j,-j,j,j,0,0]);
        robot.model.animate([j,-j,j,j,0,0]);
        j = j+0.05;
    end
end
