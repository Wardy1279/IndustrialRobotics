classdef SprayBot < RobotBaseClass
    %% SprayBot
    
    properties(Access = public)   
        plyFileNameStem = 'SprayBot';
    end

    methods
%% Constructor
        function self = SprayBot(baseTr)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.29,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)],'offset',0);
            link(2) = Link('d',0,'a',0.27,'alpha',0,'qlim',[deg2rad(-360),deg2rad(360)],'offset',-pi/3);
            link(3) = Link('d',0,'a',0.07,'alpha',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)],'offset',0);
            link(4) = Link('d',0.302,'a',0,'alpha',pi/2,'qlim',[deg2rad(-360),deg2rad(360)],'offset',0);
            link(5) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)],'offset',-pi/6);
            link(6) = Link('d',0.072,'a',0,'alpha',0,'qlim',[deg2rad(-360),deg2rad(360)],'offset',0);

            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
