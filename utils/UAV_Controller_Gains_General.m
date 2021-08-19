function drone_gains = UAV_Controller_Gains_General()


drone_gains = containers.Map(...
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
end
