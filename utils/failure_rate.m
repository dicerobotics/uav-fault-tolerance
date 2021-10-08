clc; clear all; close all;

%% Hardware Details
% Main FMU Processor: STM32F765
%   32 Bit Arm® Cortex®-M7, 216MHz, 2MB memory, 512KB RAM
% IO Processor: STM32F100
%   32 Bit Arm® Cortex®-M3, 24MHz, 8KB SRAM
% On-board sensors:
%   Accel/Gyro: ICM-20689
%   Accel/Gyro: BMI055
%   Magnetometer: IST8310
%   Barometer: MS5611
%   GPS: u-blox Neo-M8N GPS/GLONASS receiver; 
%   Integrated magnetometer IST8310

%1 why 1-3 theta_JC=40?????
STM32F765 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.12, 'C2', 3e-5*(100)^1.82, ...
    'T_C', 35, 'theta_JC', 40, 'P_max', 3.6*0.42, 'A', 9270, 'Vs', 3.6);
failure_rate_STM32F765 = calcFailureRate(STM32F765)
%2
STM32F100 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.12, 'C2', 3e-5*(100)^1.82, ...
    'T_C', 35, 'theta_JC', 40, 'P_max', 4*0.15, 'A', 9270, 'Vs', 3.6); % WHY supply voltage Vs < V_DD??????
failure_rate_STM32F100 = calcFailureRate(STM32F100) %check max power distribution
%3
ICM_20689 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.03, 'C2', 3e-5*(24)^1.82, ...
    'T_C', 35, 'theta_JC', 40, 'P_max', 3.45*0.003, 'A', 6373, 'Vs', 3.6);
failure_rate_ICM_20689 = calcFailureRate(ICM_20689)


%4 organic sealed, V_DD=3.6V, I=5mA, CMOS, nonhermetic DIPs with 16 pins,
% C1 UNKNOWN!!!!!(logical); DIP??????
BMI055 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.10, 'C2', 2e-4*(16)^1.23, ...
    'T_C', 35, 'theta_JC', 125, 'P_max', 3.6*0.005, 'A', 9270, 'Vs', 3.6);
failure_rate_BMI055 = calcFailureRate(BMI055)

%5 16 pins, Flatpack, V_DD=3.6V
% LGA (Non-hermetic??????); theta_JC UNKNOWN!!!!!!
% 14 bit data output??????;
IST8310 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.10, 'C2', 3e-5*(16)^1.82, ...
    'T_C', 35, 'theta_JC', 40, 'P_max', 3.6*(1200e-6), 'A', 6373, 'Vs', 3.6);
failure_rate_IST8310 = calcFailureRate(IST8310)

%6 V_DD=3.6V, Vs=4V, Imax=12.5*10e-6(is it too small????), DIP # 8;
% Hermetic DIPs with solder or weld seals (C2)?????????
% C1 UNKNOWN (sensor)??????
MS5611 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.10, 'C2', 2.8e-4*(8)^1.08, ...
    'T_C', 35, 'theta_JC', 30, 'P_max', 3.6*(12.5e-6), 'A', 6373, 'Vs', 4);
failure_rate_MS5611 = calcFailureRate(MS5611)

%7 DIP # 24; V_DD=3.6; hermetically sealed, reeled tapes (Hermetic DIPs
% with solder or weld seals)
% C1 UNKNOWN!!!!!!
Neo_M8N = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.15, 'C2', 2.8e-4*(24)^1.08, ...
    'T_C', 35, 'theta_JC', 25, 'P_max', 3.6*0.025, 'A', 6373, 'Vs', 3.6);
failure_rate_Neo_M8N = calcFailureRate(Neo_M8N)

% %8 same as No.5
% IST8310 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.03, 'C2', 3e-5*(24)^1.82, ...
%     'T_C', 35, 'theta_JC', 40, 'P_max', 3.45*0.003, 'A', 6373, 'Vs', 3.6);
% failure_rate_IST8310 = calcFailureRate(IST8310)



function lambda = calcFailureRate(device)
%% Calculations for Failure rate for STM32F765
pi_L = device.pi_L;      %learning factor
pi_Q = device.pi_Q;      %quality factor
pi_E = device.pi_E;       %application environment factor
C1 = device.C1;         %device complexity factor
C2 = device.C2;         %package complexity factor
T_C = device.T_C;       %case temperature
theta_JC = device.theta_JC;   %junction to case thermal resistance (C/watt) for PCB soldered device
P = device.P_max;          %worst-case power dissipation
A = device.A;                                   %function of technology and package type


T_j = T_C + theta_JC * P;       %worst-case junction temperature
pi_T = 0.1*exp(-A*((1/(T_j+273))-(1/298))); %temperature acceleration factor


Vs = device.Vs;       %supply voltage
x = 0.168*Vs*(T_j + 273) / 298;
pi_V = 0.11*exp(x);       %velocity stress factor

lambda = pi_L * pi_Q * (C1 * pi_T * pi_V + C2 * pi_E);
end