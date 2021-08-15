close all;
clc; clear;

addpath('./lib');
%% DEFINE
R2D = 180/pi;
D2R = pi/180;

%% INIT. PARAMS.
drone1_params = containers.Map({'mass','armLength','Ixx','Iyy','Izz', ...
    'motorThrustMin', 'motorThrustMax', 'kt', 'kq'},...
    {1.5, 0.265, 0.025, 0.025, 0.055, 0, 7.8957, 4.5e-5, 2.267e-08});
drone2_params = drone1_params; drone3_params = drone1_params;

drone1_initStates = [0, 0, -4, ...                                              % x, y, z
    0, 0, -3, ...                                                               % dx, dy, dz
    0, 0, 0, ...                                                                % phi, theta, psi
    0, 0, 0]';                                                                  % p, q, r
drone2_initStates = drone1_initStates;
drone3_initStates = drone1_initStates;


drone1_initInputs = [0, ...                                                     % ThrottleCMD
    0, 0, 0]';                                                                  % R, P, Y CMD
drone2_initInputs = drone1_initInputs; drone3_initInputs = drone1_initInputs;

drone1_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';
drone2_body = drone1_body;
drone3_body = drone1_body;
                
                
drone1_gains = containers.Map(...
	{'P_phi','I_phi','D_phi',...
	'P_theta','I_theta','D_theta',...
	'P_psi','I_psi','D_psi',...
	'P_x','I_x','D_x', ...
    'P_y','I_y','D_y', ...
    'P_z','I_z','D_z', ...
    },...
    {0.2, 0.0, 0.15,...
	0.2, 0.0, 0.15,...
	0.8, 0.0, 0.3,...
	10.0, 0.2, 0.0,...
    10.0, 0.2, 0.0,...
    10.0, 0.2, 0.0,...
    });

drone2_gains = drone1_gains; drone3_gains = drone1_gains;
simulationTime = 2;         % [sec]

%% BIRTH OF A DRONE
drone1 = Drone(drone1_params, drone1_initStates, drone1_initInputs, drone1_gains, simulationTime);
drone2 = Drone(drone2_params, drone2_initStates, drone2_initInputs, drone2_gains, simulationTime);
drone3 = Drone(drone3_params, drone3_initStates, drone3_initInputs, drone3_gains, simulationTime);

%% Init. 3D Fig.
% fig1 = figure('pos',[0 200 800 800]);
% h = gca;
% view(3);
% fig1.CurrentAxes.ZDir = 'Reverse';
% fig1.CurrentAxes.YDir = 'Reverse';
% % view([-60 30]);
% % view([90 0])   % To check phi (roll attitude) only
% % view([0 0])    % To check theta (pitch attitude) only
% % view([270 90]) % Top view
% axis equal;
% grid on;
% xlim([-5 5]);
% ylim([-5 5]);
% zlim([-8 0]);
% xlabel('X[m]');
% ylabel('Y[m]');
% zlabel('Height[m]');
% 
% hold(gca, 'on');
% 
% drone1_state = drone1.GetState();
% drone2_state = drone2.GetState();
% drone3_state = drone3.GetState();
% 
% 
% wHb1 = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
% wHb2 = [RPY2Rot(drone2_state(7:9))' drone2_state(1:3); 0 0 0 1];
% wHb3 = [RPY2Rot(drone3_state(7:9))' drone3_state(1:3); 0 0 0 1];
% 
% 
% % [Rot(also contains shear, reflection, local sacling), displacement; perspective ,global scaling]
% 
% drone1_world = wHb1 * drone1_body; % [4x4][4x6]
% drone2_world = wHb2 * drone2_body; % [4x4][4x6]
% drone3_world = wHb3 * drone3_body; % [4x4][4x6]
% 
% drone1_atti = drone1_world(1:3, :); 
% drone2_atti = drone2_world(1:3, :); 
% drone3_atti = drone3_world(1:3, :); 
%     
% fig1_drone1_ARM13 = plot3(gca, drone1_atti(1,[1 3]), drone1_atti(2,[1 3]), drone1_atti(3,[1 3]), ...
%         '-ro', 'MarkerSize', 5);
% fig1_drone1_ARM24 = plot3(gca, drone1_atti(1,[2 4]), drone1_atti(2,[2 4]), drone1_atti(3,[2 4]), ...
%         '-bo', 'MarkerSize', 5);
% fig1_drone1_payload = plot3(gca, drone1_atti(1,[5 6]), drone1_atti(2,[5 6]), drone1_atti(3,[5 6]), ...
%         '-k', 'Linewidth', 3);
% fig1_drone1_shadow = plot3(gca,0,0,0,'xk','Linewidth',3);
% 
% 
% fig1_drone2_ARM13 = plot3(gca, drone2_atti(1,[1 3]), drone2_atti(2,[1 3]), drone2_atti(3,[1 3]), ...
%         '-ro', 'MarkerSize', 5);
% fig1_drone2_ARM24 = plot3(gca, drone2_atti(1,[2 4]), drone2_atti(2,[2 4]), drone2_atti(3,[2 4]), ...
%         '-bo', 'MarkerSize', 5);
% fig1_drone2_payload = plot3(gca, drone2_atti(1,[5 6]), drone2_atti(2,[5 6]), drone2_atti(3,[5 6]), ...
%         '-k', 'Linewidth', 3);
% fig1_drone2_shadow = plot3(gca,0,0,0,'xk','Linewidth',3);
% 
% 
% fig1_drone3_ARM13 = plot3(gca, drone3_atti(1,[1 3]), drone3_atti(2,[1 3]), drone3_atti(3,[1 3]), ...
%         '-ro', 'MarkerSize', 5);
% fig1_drone3_ARM24 = plot3(gca, drone3_atti(1,[2 4]), drone3_atti(2,[2 4]), drone3_atti(3,[2 4]), ...
%         '-bo', 'MarkerSize', 5);
% fig1_drone3_payload = plot3(gca, drone3_atti(1,[5 6]), drone3_atti(2,[5 6]), drone3_atti(3,[5 6]), ...
%         '-k', 'Linewidth', 3);
% fig1_drone3_shadow = plot3(gca,0,0,0,'xk','Linewidth',3);
% 
% hold(gca, 'off');

%% Init. Data Fig.
% fig2 = figure('pos',[800 400 800 600]);
% subplot(2,3,1)
% title('phi[deg]')
% grid on;
% hold on;
% subplot(2,3,2)
% title('theta[deg]')
% grid on;
% hold on;
% subplot(2,3,3)
% title('psi[deg]')
% grid on;
% hold on;
% subplot(2,3,4)
% title('X[m]')
% grid on;
% hold on;
% subplot(2,3,5)
% title('Y[m]')
% grid on;
% hold on;
% subplot(2,3,6)
% title('Zdot[m/s]')
% grid on;
% hold on;

%% Init. PWM Fig.
fig3 = figure();%'pos',[800 400 800 600]);
subplot(1,4,1)
title('PWM1')
grid on;
hold on;
subplot(1,4,2)
title('PWM2')
grid on;
hold on;
subplot(1,4,3)
title('PWM3')
grid on;
hold on;
subplot(1,4,4)
title('PWM4')
grid on;
hold on;


%% Main Loop
commandSig(1) = 0;              %x
commandSig(2) = 0;              %y
commandSig(3) = -5.0;           %z
commandSig(4) = 60.0*D2R;       %psi

for i = 1:simulationTime/0.01
    %% Take a step
    drone1.PositionCtrl(commandSig);
    drone1.AttitudeCtrl(commandSig);
    drone1.UpdateState();
	drone1_state = drone1.GetState();

    drone2.PositionCtrl(commandSig);
    drone2.AttitudeCtrl(commandSig);
    drone2.UpdateState();
	drone2_state = drone2.GetState();

    drone3.PositionCtrl(commandSig);
    drone3.AttitudeCtrl(commandSig);
    drone3.UpdateState();
	drone3_state = drone3.GetState();

    %% 3D Plot
%     figure(1)
%     %Drone 1
%     wHb1 = [RPY2Rot(drone1_state(7:9))' drone1_state(1:3); 0 0 0 1];
%     drone1_world = wHb1 * drone1_body;
%     drone1_atti = drone1_world(1:3, :);
%   
% 	set(fig1_drone1_ARM13, ...
%         'XData', drone1_atti(1,[1 3]), ...
%         'YData', drone1_atti(2,[1 3]), ...
%         'ZData', drone1_atti(3,[1 3]));
%     set(fig1_drone1_ARM24, ...
%         'XData', drone1_atti(1,[2 4]), ...
%         'YData', drone1_atti(2,[2 4]), ...
%         'ZData', drone1_atti(3,[2 4]));
%     set(fig1_drone1_payload, ...
%         'XData', drone1_atti(1,[5 6]), ...
%         'YData', drone1_atti(2,[5 6]), ...
%         'ZData', drone1_atti(3,[5 6]));  
% 	  set(fig1_drone1_shadow, ...
% 		  'XData', drone1_state(1), ...
% 		  'YData', drone1_state(2));
% 
%     
%     %Drone 2
%     wHb2 = [RPY2Rot(drone2_state(7:9))' drone2_state(1:3); 0 0 0 1];
%     drone2_world = wHb2 * drone2_body;
%     drone2_atti = drone2_world(1:3, :);
% 
%     set(fig1_drone2_ARM13, ...
%         'XData', drone2_atti(1,[1 3]), ...
%         'YData', drone2_atti(2,[1 3]), ...
%         'ZData', drone2_atti(3,[1 3]));
%     set(fig1_drone2_ARM24, ...
%         'XData', drone2_atti(1,[2 4]), ...
%         'YData', drone2_atti(2,[2 4]), ...
%         'ZData', drone2_atti(3,[2 4]));
%     set(fig1_drone2_payload, ...
%         'XData', drone2_atti(1,[5 6]), ...
%         'YData', drone2_atti(2,[5 6]), ...
%         'ZData', drone2_atti(3,[5 6]));  
% 	set(fig1_drone2_shadow, ...
% 		'XData', drone2_state(1), ...
% 		'YData', drone2_state(2));
%     
% 
%     %Drone 2
%     wHb3 = [RPY2Rot(drone3_state(7:9))' drone3_state(1:3); 0 0 0 1];
%     drone3_world = wHb3 * drone3_body;
%     drone3_atti = drone3_world(1:3, :);
% 
%     set(fig1_drone3_ARM13, ...
%         'XData', drone3_atti(1,[1 3]), ...
%         'YData', drone3_atti(2,[1 3]), ...
%         'ZData', drone3_atti(3,[1 3]));
%     set(fig1_drone3_ARM24, ...
%         'XData', drone3_atti(1,[2 4]), ...
%         'YData', drone3_atti(2,[2 4]), ...
%         'ZData', drone3_atti(3,[2 4]));
%     set(fig1_drone3_payload, ...
%         'XData', drone3_atti(1,[5 6]), ...
%         'YData', drone3_atti(2,[5 6]), ...
%         'ZData', drone3_atti(3,[5 6]));  
% 	set(fig1_drone3_shadow, ...
% 		'XData', drone3_state(1), ...
% 		'YData', drone3_state(2));

    %% Data Plot
%     figure(2)
%     subplot(2,3,1)
%         plot(i/100,drone1_state(7)*R2D,'.');
%         plot(i/100,drone2_state(7)*R2D,'.');
%         plot(i/100,drone3_state(7)*R2D,'.');
%     subplot(2,3,2)
%         plot(i/100,drone1_state(8)*R2D,'.');    
%         plot(i/100,drone2_state(8)*R2D,'.');    
%         plot(i/100,drone3_state(8)*R2D,'.');    
%     subplot(2,3,3)
%         plot(i/100,drone1_state(9)*R2D,'.');
%         plot(i/100,drone2_state(9)*R2D,'.');
%         plot(i/100,drone3_state(9)*R2D,'.');
%     subplot(2,3,4)
%         plot(i/100,drone1_state(1),'.');
%         plot(i/100,drone2_state(1),'.');
%         plot(i/100,drone3_state(1),'.');
%     subplot(2,3,5)
%         plot(i/100,drone1_state(2),'.');
%         plot(i/100,drone2_state(2),'.');
%         plot(i/100,drone3_state(2),'.');
%     subplot(2,3,6)
%         plot(i/100,drone1_state(6),'.');
%         plot(i/100,drone2_state(6),'.');
%         plot(i/100,drone3_state(6),'.');
%     drawnow;

%% Update the propagate model data
    if(rem(i, 50)==0)
        x_tol = ones(12,1) * 0.1;
        miss_match_counter = 0;
        if(abs(drone1.x - drone2.x) < x_tol)
            %refresh the data
            drone3.x = drone1.x;
        else
            miss_match_counter = miss_match_counter + 1;
            if miss_match_counter > miss_match_threshold
                disp("Modular Redundancy Failed");
            end
            d1 = sum(abs(drone3.x - drone1.x));
            d2 = sum(abs(drone3.x - drone2.x));
            
            if(d1 < d2)
                drone3.x = drone1.x;
            else
                drone3.x = drone2.x;
            end
        end
    end
    for itr=1:10
        UpdatePWM(drone1, itr)
        UpdatePWM(drone2, itr)
        UpdatePWM(drone3, itr)

        if xor(drone1.pwm(1), drone2.pwm(1))
            voterOut(1) = drone3.pwm(1);
        else
            voterOut(1) = drone1.pwm(1);
        end
        
        if xor(drone1.pwm(2), drone2.pwm(2))
            voterOut(2) = drone3.pwm(2);
        else
            voterOut(2) = drone1.pwm(2);
        end
        if xor(drone1.pwm(3), drone2.pwm(3))
            voterOut(3) = drone3.pwm(3);
        else
            voterOut(3) = drone1.pwm(3);
        end
        if xor(drone1.pwm(4), drone2.pwm(4))
            voterOut(4) = drone3.pwm(4);
        else
            voterOut(4) = drone1.pwm(4);
        end

        figure(3);
        subplot(4,1,1)
            plot((i/100)+(itr/1000),voterOut(1), '.');  hold on;
        subplot(4,1,2)
            plot((i/100)+(itr/1000),voterOut(2), '.');  hold on;
        subplot(4,1,3)
            plot((i/100)+(itr/1000),voterOut(3), '.');  hold on;
        subplot(4,1,4)
            plot((i/100)+(itr/1000),voterOut(4), '.');  hold on;

        
%         voterOut(1) = (drone1.pwm(1)&&drone2.pwm(1)) || ...
%                    (drone2.pwm(1)&&drone3.pwm(1)) || ...
%                    (drone3.pwm(1)&&drone1.pwm(1));
%         voterOut(2) = (drone1.pwm(2)&&drone2.pwm(2)) || ...
%                    (drone2.pwm(2)&&drone3.pwm(2)) || ...
%                    (drone3.pwm(2)&&drone1.pwm(2));
%         voterOut(3) = (drone1.pwm(3)&&drone2.pwm(3)) || ...
%                    (drone2.pwm(3)&&drone3.pwm(3)) || ...
%                    (drone3.pwm(3)&&drone1.pwm(3));
%         voterOut(4) = (drone1.pwm(4)&&drone2.pwm(4)) || ...
%                    (drone2.pwm(4)&&drone3.pwm(4)) || ...
%                    (drone3.pwm(4)&&drone1.pwm(4));

%         figure(3);
%         subplot(4,1,1)
%             plot((i/100)+(itr/1000),drone1.pwm(1), '.');  hold on;
%         subplot(4,1,2)
%             plot((i/100)+(itr/1000),drone1.pwm(2), '.');  hold on;
%         subplot(4,1,3)
%             plot((i/100)+(itr/1000),drone1.pwm(3), '.');  hold on;
%         subplot(4,1,4)
%             plot((i/100)+(itr/1000),drone1.pwm(4), '.');  hold on;

%             plot((i/100)+(itr/1000),drone2.pwm(1));
%             plot((i/100)+(itr/1000),drone2.pwm(2));
%             plot((i/100)+(itr/1000),drone2.pwm(3));
%             plot((i/100)+(itr/1000),drone2.pwm(4));
% 
%             plot((i/100)+(itr/1000),drone3.pwm(1));
%             plot((i/100)+(itr/1000),drone3.pwm(2));
%             plot((i/100)+(itr/1000),drone3.pwm(3));
%             plot((i/100)+(itr/1000),drone3.pwm(4));
        drawnow;
    end
    
    %% BREAK WHEN CRASH
    if (drone1_state(3) >= 0)
        h = msgbox('Crashed!!!', 'Error','error');
        break;
    end
end
