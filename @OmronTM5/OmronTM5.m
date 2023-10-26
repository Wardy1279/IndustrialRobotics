classdef OmronTM5 < RobotBaseClass
    %% Omron TM5 4kg payload 900mm reach robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'OmronTM5';
    end
    
    
    methods
%% Constructor
function self = OmronTM5(baseTr,useTool,toolFilename)
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
            link(1) = Link('d',0.1452,    'a',0,      'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link('d',0,         'a',-0.329,  'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(3) = Link('d',0,         'a',-0.3115,'alpha',0,'offset',0,'qlim',[deg2rad(-155),deg2rad(155)]);
            link(4) = Link('d',0.106,     'a',0,      'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(5) = Link('d',0.106,     'a',0,      'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(6) = Link('d',0.1132,     'a',0,      'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);           

            self.model = SerialLink(link,'name',self.name);            
        end    
    end
end