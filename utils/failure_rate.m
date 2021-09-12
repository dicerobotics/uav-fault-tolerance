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

STM32F765 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.12, 'C2', 0.20, ...
    'T_C', 35, 'theta_JC', 40, 'P_max', 1.5120, 'A', 9270, 'Vs', 3.6);
failure_rate_STM32F765 = calcFailureRate(STM32F765)

device2 = struct('pi_L', 10, 'pi_Q', 20, 'pi_E', 3.0, 'C1', 0.12, 'C2', 0.20, ...
    'T_C', 35, 'theta_JC', 40, 'P_max', 1.5120, 'A', 9270, 'Vs', 3.6);
failure_rate_device2 = calcFailureRate(device2)

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
