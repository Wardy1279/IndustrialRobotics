function [qLastUR, qLastSB] = Teach(URrobot, SBrobot)
    robotSelection = 0;
    qLastUR = [0,0,0,0,0,0];
    qLastSB = [0,0,0,0,0,0];

    fig = uifigure;
    g = uigridlayout(fig,[1 3]);
    g.RowHeight = {'1x', '0.3x'};
    g.ColumnWidth = {'1x','1x','1x'};
    
    b = uibutton(g, ...
        "Text","Teach Omron", ...
        "ButtonPushedFcn", @(src,event) ButtonCallback(1));
    b.Layout.Row = 1;
    b.Layout.Column = 1;
    
    c = uibutton(g, ...
        "Text","Teach IRB120", ...
        "ButtonPushedFcn", @(src,event) ButtonCallback(2));
    c.Layout.Row = 1;
    c.Layout.Column = 2;

    d = uibutton(g, ...
        "Text","Teach Cartesian", ...
        "ButtonPushedFcn", @(src,event) ButtonCallback(3));
    d.Layout.Row = 1;
    d.Layout.Column = 3;

    e = uibutton(g, ...
        "Text","Exit Teach", ...
        "ButtonPushedFcn", @(src,event) ButtonCallback(4));
    e.Layout.Row = 2;
    e.Layout.Column = 3;
    
    pause(2);
    
    while robotSelection ~= 4
        pause(1);
        
        % disp(robotSelection);
        
        if robotSelection == 1
            qLastUR = TeachJoy(URrobot, qLastUR);
        elseif robotSelection == 2
            qLastSB = TeachJoy(SBrobot, qLastSB);
        elseif robotSelection == 3
            inputRobotSelection = input('Robot Selection (1 = Omron, 2 = IRB120): ');
            inputX = input('X: ');
            inputY = input('Y: ');
            inputZ = input('Z: ');
            
            if inputRobotSelection == 1
                try
                    qLastUR = URrobot.model.ikine([1,0,0,inputX;0,1,0,inputY;0,0,1,inputZ;0,0,0,1]);
                    URrobot.model.animate(qLastUR);
                    disp(URrobot.model.fkine(URrobot.model.getpos()).T);
                end
            elseif inputRobotSelection == 2
                try
                    qLastSB = SBrobot.model.ikine([1,0,0,inputX;0,1,0,inputY;0,0,1,inputZ;0,0,0,1]);
                    SBrobot.model.animate(qLastSB);
                    disp(SBrobot.model.fkine(SBrobot.model.getpos()).T);
                end
                
                drawnow();
                robotSelection = 0;
            end
        end
    end
    
    % Nested function to handle button click events
    function ButtonCallback(selection)
        disp('Pressed button');
        robotSelection = selection;
    end
end