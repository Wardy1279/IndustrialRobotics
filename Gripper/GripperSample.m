hold on;

steps = 250;

robot = UR10();

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
qMatrixGripper1 = UR10Gripper1.GetOpenqMatrix(gripper1, steps); % GetOpenqMatrix is a function I've created
qMatrixGripper2 = UR10Gripper2.GetOpenqMatrix(gripper2, steps);

% Animate arm
for i = 1:steps
  robot.model.animate(qMatrix(i,:));
  gripper1.model.base = robot.model.fkine(qMatrix(i,:));
  gripper1.model.animate(qMatrixGripper1(i,:));
  gripper2.model.base = robot.model.fkine(qMatrix(i,:));
  gripper2.model.animate(qMatrixGripper2(i,:));
  drawnow()
end

qMatrixGripper1 = UR10Gripper1.GetCloseqMatrix(gripper1, steps);
qMatrixGripper2 = UR10Gripper2.GetCloseqMatrix(gripper2, steps);

for i = 1:steps
    qMatrix = jtraj([pi/2,pi/2,pi/2,pi/2,pi/2,pi/2],[pi/2,pi/2,pi/2,pi/2,pi/2,pi/2],steps);
    gripper1.model.base = robot.model.fkine(qMatrix(i,:));
    gripper1.model.animate(qMatrixGripper1(i,:));
    gripper2.model.base = robot.model.fkine(qMatrix(i,:));
    gripper2.model.animate(qMatrixGripper2(i,:));
    drawnow()
end