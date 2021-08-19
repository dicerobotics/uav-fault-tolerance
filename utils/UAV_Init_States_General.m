function [drone_initStates] = UAV_Init_States_General()

drone_initStates = [0, 0, -4, ...	% x, y, z
    0, 0, -3, ...           % dx, dy, dz
    0, 0, 0, ...            % phi, theta, psi
    0, 0, 0]'; 

end
