classdef UAVPos3D < handle
    %QUADPLOT Visualization class for quad

    properties (SetAccess = public)
        state;          % state
        rot;            % rotation matrix body to world
        wingspan;       % wingspan
        marker;         % UAV marker position in world frame
    end

    properties (SetAccess = private)
        % Plot Handles
        h_3d
        h_m13;  % marker 1 and 3 handle
        h_m24;  % marker 2 and 4 handle
        h_payload;
        h_shadow;
    end

    methods
        function displayObj = UAVPos3D(droneObj, h_3d, drone_marker_body)
        % Constructor
            displayObj.state = droneObj.GetState();
            displayObj.wingspan = droneObj.l;
            displayObj.rot = Quat2RM(displayObj.state(7:10));

            
            displayObj.h_3d = h_3d;
            view(3);
            % view([90 0])   % To check phi (roll attitude) only
            % view([0 0])    % To check theta (pitch attitude) only
            % view([270 90]) % Top view
            CurrentAxes.ZDir = 'Reverse';
            CurrentAxes.YDir = 'Reverse';
            axis equal; grid on;
            xlim([-5 5]); ylim([-5 5]); zlim([-8 0]);
            xlabel('X[m]'); ylabel('Y[m]'); zlabel('Height[m]');

            hold(displayObj.h_3d, 'on')
            
            %wHb is homogenous transformation matrix from body to world
            wHb = [RPY2Rot(displayObj.state(7:9))' displayObj.state(1:3); 0 0 0 1];
            drone_marker_world_homo = wHb * drone_marker_body;
            displayObj.marker = drone_marker_world_homo(1:3, :);
            
            displayObj.h_m13 = plot3(displayObj.h_3d, ...
                displayObj.marker(1,[1 3]), ...
                displayObj.marker(2,[1 3]), ...
                displayObj.marker(3,[1 3]), ...
                '-ko', 'MarkerSize', 5);
            displayObj.h_m24 = plot3(displayObj.h_3d, ...
                displayObj.marker(1,[2 4]), ...
                displayObj.marker(2,[2 4]), ...
                displayObj.marker(3,[2 4]), ...
                '-ko', 'MarkerSize', 5);
            displayObj.h_payload = plot3(displayObj.h_3d, ...
                displayObj.marker(1,[5 6]), ...
                displayObj.marker(2,[5 6]), ...
                displayObj.marker(3,[5 6]), ...
                'LineWidth', 2);
            displayObj.h_shadow = plot3(displayObj.h_3d, ...
                displayObj.state(1), ...
                displayObj.state(2), 0);
            hold(displayObj.h_3d, 'off')
        end

        % Update quad plot
        function UpdateUAVPlot(displayObj, droneObj, drone_marker_body)
            drone_state = droneObj.GetState();

            %wHb is homogenous transformation matrix from body to world
            wHb = [RPY2Rot(displayObj.state(7:9))' displayObj.state(1:3); 0 0 0 1];
            drone_marker_world_homo = wHb * drone_marker_body;
            drone_markers = drone_marker_world_homo(1:3, :);

            set(displayObj.h_m13, ...
                'XData', drone_markers(1,[1 3]), ...
                'YData', drone_markers(2,[1 3]), ...
                'ZData', drone_markers(3,[1 3]));
            set(displayObj.h_m24, ...
                'XData', drone_markers(1,[2 4]), ...
                'YData', drone_markers(2,[2 4]), ...
                'ZData', drone_markers(3,[2 4]));
            set(displayObj.h_payload, ...
                'XData', drone_markers(1,[5 6]), ...
                'YData', drone_markers(2,[5 6]), ...
                'ZData', drone_markers(3,[5 6]));  
            set(displayObj.h_shadow, ...
                'XData', drone_state(1), ...
                'YData', drone_state(2));
            drawnow;
        end
    end

end
