clear all
clf
clc
hold on;

steps = 250;

nozzleObj = Nozzle(1);
clothObj = Cloth(1);

robot = UR10();

% Move nozzle to UR10 base
nozzleObj.nozzleModel{1}.base = robot.model.fkine(robot.model.getpos());
nozzleObj.nozzleModel{1}.animate(0);

% Same concept applies to cloth, uses clothModel instead of nozzleModel
% clothObj.clothModel{1}.base = robot.model.fkine(robot.model.getpos());
% clothObj.clothModel{1}.animate(0);

% Model both grippers (left and right fingers)
gripper1 = UR10Gripper1();
gripper1.model;
gripper2 = UR10Gripper2();
gripper2.model;

% Move gripper to arms end-effector
gripper1.model.base = robot.model.fkine(robot.model.getpos());
gripper1.model.animate([0,0]);
gripper2.model.base = robot.model.fkine(robot.model.getpos());
gripper2.model.animate([0,0]);

% Create qMatrix
qMatrix = jtraj([0,0,0,0,0,0],[pi/2,pi/2,pi/2,pi/2,pi/2,pi/2],steps);
qMatrixGripper1 = UR10Gripper1.GetOpenqMatrix(gripper1, steps); % GetOpenqMatrix is a function I've created, opens fingers by pi/3
qMatrixGripper2 = UR10Gripper2.GetOpenqMatrix(gripper2, steps);

% Animate arm
for i = 1:steps
    % animate UR10 arm
    robot.model.animate(qMatrix(i,:));

    % update gripper model to end effector
    gripper1.model.base = robot.model.fkine(qMatrix(i,:));
    gripper2.model.base = robot.model.fkine(qMatrix(i,:));

    % animate gripper opening
    gripper1.model.animate(qMatrixGripper1(i,:));
    gripper2.model.animate(qMatrixGripper2(i,:));

    % updates nozzle base to end effector
    nozzleObj.nozzleModel{1}.base = robot.model.fkine(qMatrix(i,:));
    nozzleObj.nozzleModel{1}.animate(0);
    drawnow()
end

% create qMatrix for closing gripper
qMatrixGripper1 = UR10Gripper1.GetCloseqMatrix(gripper1, steps);
qMatrixGripper2 = UR10Gripper2.GetCloseqMatrix(gripper2, steps);

% animate gripper closing
for i = 1:steps
    qMatrix = jtraj([pi/2,pi/2,pi/2,pi/2,pi/2,pi/2],[pi/2,pi/2,pi/2,pi/2,pi/2,pi/2],steps);
    gripper1.model.base = robot.model.fkine(qMatrix(i,:));
    gripper1.model.animate(qMatrixGripper1(i,:));
    gripper2.model.base = robot.model.fkine(qMatrix(i,:));
    gripper2.model.animate(qMatrixGripper2(i,:));
    drawnow()
end