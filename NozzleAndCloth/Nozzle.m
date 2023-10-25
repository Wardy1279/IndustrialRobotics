classdef Nozzle < handle
    %BRICKS A class that creates bricks
	%   The bricks can be moved around randomly. It is then possible to query
    %   the current location (base) of the cbricks.    
    
    %#ok<*TRYNC>    
    
    properties
        %> Number of bricks
        brickCount = 1;
        
        %> A cell structure of \c brickCount brick models
        nozzleModel;
    end
    
    methods
        %% ...structors
        function self = Nozzle(brickCount)
            if 0 < nargin
                self.brickCount = brickCount;
            end

            % Create the required number of bricks
            for i = 1:self.brickCount
                self.nozzleModel{i} = self.GetNozzleModel(['Nozzle',num2str(i)]);
                % Not random spawn
                basePose = SE3(eul2rotm([0,0,0]), [0,0,0]);
                self.nozzleModel{i}.base = basePose;
                
                 % Plot 3D model
                plot3d(self.nozzleModel{i},0,'workspace',[-1 1 -1 1 -0.5 2],'view',[-30,30],'delay',0,'noarrow','nowrist');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1 
                    hold on;
                end
            end

            axis equal
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end 
        end
        
        function delete(self)
            for index = 1:self.brickCount
                handles = findobj('Tag', self.nozzleModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end
    end

    methods (Static)
        %% GetBrickModel
        function model = GetNozzleModel(name)
            if nargin < 1
                name = 'Brick';
            end
            [faceData,vertexData] = plyread('Nozzle.ply','tri');
            link1 = Link('alpha',0,'a',0,'d',0.15,'offset',0);
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end