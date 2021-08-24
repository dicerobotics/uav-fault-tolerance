classdef UAVData < handle
    
    properties (SetAccess = public)
        k           %data sample number
        state
        time_hist
        state_hist
    end
    % Plot Handles
    properties (SetAccess = private)
        h_data
        h_phi
        h_theta
        h_psi
        h_X
        h_Y
        h_Zdot
    end
    methods
        %Constructor
        function displayObj = UAVData(h_data)
            displayObj.k = 0;
            displayObj.h_data = h_data;
            
            displayObj.h_phi = subplot(2,3,1); title('phi[deg]'); grid on;
            displayObj.h_theta = subplot(2,3,2); title('theta[deg]'); grid on;
            displayObj.h_psi = subplot(2,3,3); title('psi[deg]'); grid on;
            displayObj.h_X = subplot(2,3,4); title('X[m]'); grid on;
            displayObj.h_Y = subplot(2,3,5); title('Y[m]'); grid on;
            displayObj.h_Zdot = subplot(2,3,6); title('Zdot[m/s]'); grid on;
        end
        
        function UAVDataPlot(displayObj, uavObj, simTime)
            global R2D
                       
            subplot(displayObj.h_phi); hold(displayObj.h_phi, 'on'); 
            plot(displayObj.h_phi, simTime,uavObj.x(7)*R2D,'b.');
            hold(displayObj.h_phi, 'off');
            
            subplot(displayObj.h_theta); hold(displayObj.h_theta, 'on');  
            plot(displayObj.h_theta, simTime,uavObj.x(8)*R2D,'b.');
            hold(displayObj.h_theta, 'off');
            
            subplot(displayObj.h_psi); hold(displayObj.h_psi, 'on');  
            plot(displayObj.h_psi, simTime,uavObj.x(9)*R2D,'b.');
            hold(displayObj.h_psi, 'off');
            
            subplot(displayObj.h_X); hold(displayObj.h_X, 'on');  
            plot(displayObj.h_X, simTime,uavObj.x(1),'b.');
            hold(displayObj.h_X, 'off');
            
            subplot(displayObj.h_Y); hold(displayObj.h_Y, 'on');  
            plot(displayObj.h_Y, simTime,uavObj.x(2),'b.');
            hold(displayObj.h_Y, 'off');
            
            subplot(displayObj.h_Zdot); hold(displayObj.h_Zdot, 'on');  
            plot(displayObj.h_Zdot, simTime,uavObj.x(6),'b.');
            hold(displayObj.h_Zdot, 'off');
            
        end
        
%         function Record_Hist(displayObj, uavObj)
%             displayObj.k = displayObj.k + 1;
%             displayObj.time_hist(displayObj.k) = uavObj.t;
%             displayObj.state_hist(:,displayObj.k) = uavObj.GetState();
%         end
%         
%         function TruncateHist(displayObj)
%             displayObj.time_hist = displayObj.time_hist(1:displayObj.k);
%             displayObj.state_hist = displayObj.state_hist(:, 1:displayObj.k);
%         end
    end
end
