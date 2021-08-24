close all;
clc; clear;

addpath('./utils');
%% DEFINE
global R2D D2R
R2D = 180/pi;
D2R = pi/180;

total_simulation_time = 2;          % [sec]
sim_time_step = 0.01;               % [sec]

%% INIT. PARAMS.
drone1_params = System_Params_General();
drone2_params = System_Params_General();
drone3_params = System_Params_General();

drone1_initStates = [0, 0, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0]'; %UAV_Init_States_General();
drone2_initStates = [0, 0, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
drone3_initStates = [0, 0, -4, 0, 0, 0, 0, 0, 0, 0, 0, 0]';

drone1_initInputs = UAV_Init_Inputs();
drone2_initInputs = UAV_Init_Inputs();
drone3_initInputs = UAV_Init_Inputs();

drone1_body = UAV_Geometery_General();
drone2_body = UAV_Geometery_General();
drone3_body = UAV_Geometery_General();
          
                
drone1_gains = UAV_Controller_Gains_General();
drone2_gains = UAV_Controller_Gains_General();
drone3_gains = UAV_Controller_Gains_General();



%% BIRTH OF A DRONE
drone1 = Drone(drone1_params, drone1_initStates, drone1_initInputs, drone1_gains, total_simulation_time);
drone2 = Drone(drone2_params, drone2_initStates, drone2_initInputs, drone2_gains, total_simulation_time);
drone3 = Drone(drone3_params, drone3_initStates, drone3_initInputs, drone3_gains, total_simulation_time);

%% Initialize display for 3D Position plot
% figure('pos',[1300 600 500 400]); 
figure(); h_3d = gca;
axis equal; grid on; view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
quadcolors = lines(1);

Q1Pos3D = UAVPos3D(drone1, h_3d, drone1_body);
Q2Pos3D = UAVPos3D(drone2, h_3d, drone2_body);
Q3Pos3D = UAVPos3D(drone3, h_3d, drone3_body);

%% Initialize display for Data plots
figure(); h_data_Q1 = gca; Q1Data = UAVData(h_data_Q1);
figure(); h_data_Q2 = gca; Q2Data = UAVData(h_data_Q2);
figure(); h_data_Q3 = gca; Q3Data = UAVData(h_data_Q3);


%% Init. PWM Fig.
% fig3 = figure();%'pos',[800 400 800 600]);
% subplot(1,4,1)
% title('PWM1')
% grid on;
% hold on;
% subplot(1,4,2)
% title('PWM2')
% grid on;
% hold on;
% subplot(1,4,3)
% title('PWM3')
% grid on;
% hold on;
% subplot(1,4,4)
% title('PWM4')
% grid on;
% hold on;

%% Main Loop
commandSig(1) = 0;              %x
commandSig(2) = 0;              %y
commandSig(3) = -5.0;           %z
commandSig(4) = 60.0*D2R;       %psi

for i = 1:total_simulation_time/sim_time_step
    drone1.PositionCtrl(commandSig);
    drone1.AttitudeCtrl(commandSig);
    drone1.UpdateState();
    Q1Pos3D.UpdateUAVPosPlot(drone1, drone1_body);
    Q1Data.UAVDataPlot(drone1, i*sim_time_step);
    
    drone2.PositionCtrl(commandSig);
    drone2.AttitudeCtrl(commandSig);
    drone2.UpdateState();
    Q2Pos3D.UpdateUAVPosPlot(drone2, drone2_body);
    Q2Data.UAVDataPlot(drone2, i*sim_time_step);
    
    drone3.PositionCtrl(commandSig);
    drone3.AttitudeCtrl(commandSig);
    drone3.UpdateState();
    Q3Pos3D.UpdateUAVPosPlot(drone3, drone3_body);
    Q3Data.UAVDataPlot(drone3, i*sim_time_step);
    
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
        miss_match_threshold = 50;
        if(abs(drone1.x - drone2.x) < x_tol)
            %refresh the data
            drone3.x = drone1.x;
        else
            miss_match_counter = miss_match_counter + 1;
            if (miss_match_counter > miss_match_threshold)
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
        UpdatePWM(drone1, itr);
        UpdatePWM(drone2, itr);
        UpdatePWM(drone3, itr);

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

%         figure(3);
%         subplot(4,1,1)
%             plot((i/100)+(itr/1000),voterOut(1), '.');  hold on;
%         subplot(4,1,2)
%             plot((i/100)+(itr/1000),voterOut(2), '.');  hold on;
%         subplot(4,1,3)
%             plot((i/100)+(itr/1000),voterOut(3), '.');  hold on;
%         subplot(4,1,4)
%             plot((i/100)+(itr/1000),voterOut(4), '.');  hold on;

        
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
%         drawnow;
    end
    
    %% BREAK WHEN CRASH
%     if (drone1_state(3) >= 0)
%         h = msgbox('Crashed!!!', 'Error','error');
%         break;
%     end
end
