%% To initialise a connection to the ROS computer from Matlab, call rosinit to the correct IP address, and then start to subscribe to the joint states
rosinit('192.168.27.1'); % If unsure, please ask a tutor
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

% To get the current joint state from the real robot

jointStateSubscriber.LatestMessage

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
pause(2); % Pause to give time for a message to appear


% Before sending commands, we create a variable with the joint names so that the joint commands are associated with a particular joint.
jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

%% Elbow Up
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)' % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456; % qNow
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [0,0,0,0,0,0] + [0 -1.0472 1.0472 0 0 0]; % qGoal

endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% Move to brick
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456; % qNow
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [pi/2,-pi/2,0,-pi/2,0,0] + [-0.0516, -1.2762, -0.7865, 0.4920, 1.5708, -0.0516]; % qGoal

endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% To cloth
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)' % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456; % qNow
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [1.2698 -1.0360 1.8305 -0.7945 -0.3010 -1.5708]; % qGoal

endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);


%% To intersection point 1
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)' % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456; % qNow
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [-3.5215 -1.1348 -2.3895 0.38241 0.37968 0.00035995]; % qGoal

endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% To intersection point 2
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)' % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456; % qNow
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [-2.7625 0.24332 -2.3891 -0.99552 -0.37886 -0.00026087]; % qGoal

endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% To intersection point 3
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)' % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456; % qNow
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [-3.5209 -1.0641 -2.184 0.10678 0.37933 -0.00027046]; % qGoal

endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% To intersection point 4
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)' % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456; % qNow
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
nextJointState_123456 = [-3.1212 -0.55019 -2.0926 0.68367 -0.0075483 -1.1824]; % qGoal

endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);