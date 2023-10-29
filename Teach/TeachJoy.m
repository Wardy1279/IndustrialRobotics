function [qLast] = TeachJoy(robot, qLast)
%% setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information

%% Start "real-time" simulation
q = qLast;                 % Set initial robot configuration 'q'

robot.model.fkine(q);          % Plot robot in initial configuration
robot.model.delay = 0.001;    % Set smaller delay when animating

duration = 5;  % Set duration of the simulation (seconds)
dt = 0.3;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    [axes, buttons, povs] = read(joy);
       
    % -------------------------------------------------------------
    % YOUR CODE GOES HERE
    % 1 - turn joystick input into an end-effector velocity command
    Kv = 0.1; % linear velocity gain
    Kw = 0.5; % angular velocity gain
    
    vx = Kv*axes(1);
    vy = Kv*axes(2);
    vz = Kv*(buttons(5)-buttons(7));
    
    wx = Kw*axes(4);
    wy = Kw*axes(3);
    wz = Kw*(buttons(6)-buttons(8));
    
    dx = [vx;vy;vz;wx;wy;wz]; % combined velocity vector
    
    % 2 - use J inverse to calculate joint velocity
    J = robot.model.jacob0(q);
    dq = pinv(J)*dx;
    
    % 3 - apply joint velocity to step robot joint angles 
    q = q + dq'*dt;
      
    % -------------------------------------------------------------
    
    % Update plot
    robot.model.animate(q);  
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end
qLast = robot.model.getpos();
end

