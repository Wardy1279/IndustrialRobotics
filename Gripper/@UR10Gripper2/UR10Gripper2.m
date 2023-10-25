classdef UR10Gripper2 < RobotBaseClass
    %% Gripper for UR10

    properties(Access = public)              
        plyFileNameStem = 'UR10Gripper2';
    end
    
    methods
%% Define robot Function 
    function self = UR10Gripper2(baseTr)
		self.CreateModel();
        if nargin < 1			
			baseTr = eye(4);				
        end
        self.model.base = self.model.base.T * baseTr;
        
        self.PlotAndColourRobot();   
    end

%% Create the robot model
        function CreateModel(self)   
            % Create the UR5 model mounted on a linear rail
            link(1) = Link([0     0.08       -0.04       pi/2    0]);
            link(2) = Link([0     0       0       -pi/2    0]);

            link(2).offset = -pi/5;
            
            self.model = SerialLink(link,'name',self.name);
        end
    end
    methods(Static)
        function qMatrix = GetOpenqMatrix(gripper, steps)
            qNow = gripper.model.getpos();
            qGoal = qNow + [0, pi/3];
            qMatrix = jtraj(qNow, qGoal, steps);
        end

        function qMatrix = GetCloseqMatrix(gripper, steps)
            qNow = gripper.model.getpos();
            qGoal = qNow + [0, -pi/3];
            qMatrix = jtraj(qNow, qGoal, steps);
        end
    end
end