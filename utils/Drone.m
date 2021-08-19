classdef Drone < handle
    
%% MEMBERS    
    properties
        g
        t
        dt
        tf
        
        m
        l
        I
        
        x                                                                  %(X, Y, Z, dX, dY, dZ, phi, theta, psi, p, q, r)
        r                                                                  %(X, Y, Z)
        dr                                                                 %(dX, dY, dZ)
        euler                                                              %(phi, theta, psi)
        w                                                                  %(p, q, r)
        
        dx
        
        u
        pwm
        T
        M
    end
    
    properties
        phi_des
        phi_err
        phi_err_prev
        phi_err_sum
        
        theta_des
        theta_err
        theta_err_prev
        theta_err_sum
        
        psi_des
        psi_err
        psi_err_prev
        psi_err_sum
        
        x_des
        x_err
        x_err_prev
        x_err_sum

        y_des
        y_err
        y_err_prev
        y_err_sum

        z_des
        z_err
        z_err_prev
        z_err_sum
        
        xdd_cmd
        ydd_cmd

        zdot_des
        zdot_err
        zdot_err_prev
        zdot_err_sum
    end
    
    properties
        KP_phi
        KI_phi
        KD_phi
        
        KP_theta
        KI_theta
        KD_theta
        
        KP_psi
        KI_psi
        KD_psi
        
        KP_x
        KI_x
        KD_x

        KP_y
        KI_y
        KD_y

        KP_z
        KI_z
        KD_z

        KP_zdot
        KI_zdot
        KD_zdot
    end
    
    properties %motor
        kt %motor constant
        kq %motor torque constant
        motorThrustMin
        motorThrustMax
        matMotor
    end
    
%% METHODS
    methods
    %% CONSTRUCTOR
        function obj = Drone(params, initStates, initInputs, gains, simTime)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.01;
            obj.tf = simTime;
            
            obj.m = params('mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'),0,0 ; 0,params('Iyy'),0; 0,0,params('Izz')];
                        
            obj.x = initStates;
            obj.r = obj.x(1:3) + rand(3,1) * 0.1 * pi / 180;
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9) + rand(3,1) * 0.1 * pi / 180;
            obj.w = obj.x(10:12);
            
            obj.dx = zeros(12,1);
            
            obj.u = initInputs;
            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
            
            obj.motorThrustMin = params('motorThrustMin');
            obj.motorThrustMax = params('motorThrustMax');
            obj.kt = params('kt');
            obj.kq = params('kq');
            obj.matMotor = inv([obj.kt,        obj.kt,         obj.kt,  obj.kt; ...
                                     0,  obj.kt*obj.l,              0,  -obj.kt*obj.l; ...
                          obj.kt*obj.l,             0,  -obj.kt*obj.l,  0; ...
                                obj.kq,       -obj.kq,         obj.kq,  -obj.kq]);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.phi_err      = 0.0;
            obj.phi_err_prev = 0.0;
            obj.phi_err_sum  = 0.0;
            
            obj.theta_err      = 0.0;
            obj.theta_err_prev = 0.0;
            obj.theta_err_sum  = 0.0;
            
            obj.psi_err      = 0.0;
            obj.psi_err_prev = 0.0;
            obj.psi_err_sum  = 0.0;
            
            obj.x_err      = 0.0;
            obj.x_err_prev = 0.0;
            obj.x_err_sum  = 0.0;

            obj.y_err      = 0.0;
            obj.y_err_prev = 0.0;
            obj.y_err_sum  = 0.0;

            obj.z_err      = 0.0;
            obj.z_err_prev = 0.0;
            obj.z_err_sum  = 0.0;
            
            obj.xdd_cmd    = 0.0;
            obj.ydd_cmd    = 0.0;
            
            obj.zdot_err      = 0.0;
            obj.zdot_err_prev = 0.0;
            obj.zdot_err_sum  = 0.0;
			
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Find proper gains for the controller.
            obj.KP_phi=gains('P_phi');
            obj.KI_phi=gains('I_phi');
            obj.KD_phi=gains('D_phi');
            
            obj.KP_theta=gains('P_theta');
            obj.KI_theta=gains('I_theta');
            obj.KD_theta=gains('D_theta');
            
            obj.KP_psi=gains('P_psi');
            obj.KI_psi=gains('I_psi');
            obj.KD_psi=gains('D_psi');
            
            obj.KP_x=gains('P_x');
            obj.KI_x=gains('I_x');
            obj.KD_x=gains('D_x');

            obj.KP_y=gains('P_y');
            obj.KI_y=gains('I_y');
            obj.KD_y=gains('D_y');

            obj.KP_z=gains('P_z');
            obj.KI_z=gains('I_z');
            obj.KD_z=gains('D_z');

            obj.KP_zdot=gains('P_z');
            obj.KI_zdot=gains('I_z');
            obj.KD_zdot=gains('D_z');
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
    %% RETURNS DRONE STATE
        function state = GetState(obj)
            state = obj.x;
        end
        
    %% STATE SPACE (DIFFERENTIAL) EQUATIONS
        function obj = EvalEOM(obj)
            bRi = RPY2Rot(obj.euler);            
            R = bRi';                                                    
            
            % Translational Motions
            obj.dx(1:3) = obj.dr;
            obj.dx(4:6) = 1 / obj.m * ([0; 0; obj.m * obj.g] + R * obj.T * [0; 0; -1]);
            
            % Rotational Motions
            phi = obj.euler(1); theta = obj.euler(2);
            obj.dx(7:9) = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                           0 cos(phi)            -sin(phi);
                           0 sin(phi)*sec(theta) cos(phi)*sec(theta)] * obj.w;
                       
            obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I * obj.w));

        end

    %% PREDICT NEXT DRONE STATE
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            % Find(update) the next state of obj.X
            obj.EvalEOM();
            obj.x = obj.x + obj.dx.*obj.dt;
            
            obj.r     = obj.x(1:3);
            obj.dr    = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w     = obj.x(10:12);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Simulate the actual sensor output you can measure from the drone.
            % Examples down here are not accurate model 
%             obj.w(1) = obj.w(1) + randn() + 0.2;
%             obj.w(2) = obj.w(2) + randn() + 0.2;
%             obj.w(3) = obj.w(3) + randn() + 0.5;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
    %% CONTROLLER

    
        function obj = PositionCtrl(obj, refSig)
            obj.xdd_cmd = obj.KD_x*(-obj.x(4)) + obj.KP_x*(refSig(1)-obj.x(1));
            obj.ydd_cmd = obj.KD_y*(-obj.x(5)) + obj.KP_y*(refSig(2)-obj.x(2));
            obj.zdot_err = obj.KD_z*(0-obj.x(6)) + obj.KP_z*(refSig(3)-obj.x(3)); %%Correction Needed
            
            obj.u(1) = obj.m * obj.g;
            obj.u(1) = obj.m * obj.g - ...
                       (obj.KP_zdot * obj.zdot_err + ...
                        obj.KI_zdot * (obj.zdot_err_sum) + ...
                        obj.KD_zdot * (obj.zdot_err - obj.zdot_err_prev)/obj.dt);
                    
            obj.zdot_err_prev = obj.zdot_err;
            obj.zdot_err_sum  = obj.zdot_err_sum + obj.zdot_err;

            
            
            
            
        end
        
        function obj = AttitudeCtrl(obj, refSig)
            obj.x_des = refSig(1);
            obj.y_des = refSig(2);
            obj.z_des = refSig(3);
            obj.psi_des   = refSig(4);
            
            obj.phi_des   = (1/obj.g) * ( obj.ydd_cmd*cos(obj.x(9)) - obj.xdd_cmd*sin(obj.x(9)));
			obj.theta_des = -(1/obj.g) * (obj.xdd_cmd*cos(obj.x(9)) + obj.ydd_cmd*sin(obj.x(9)));
			
            
            
%             obj.zdot_des  = ;
			
            obj.phi_err   = obj.phi_des - obj.euler(1);
            obj.theta_err = obj.theta_des - obj.euler(2);
            obj.psi_err   = obj.psi_des - obj.euler(3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%% PID controllers %%%%%%%%%%%%%%%%%%%%%%%%%%%
 
            obj.u(2) = (obj.KP_phi * obj.phi_err + ...
                        obj.KI_phi * (obj.phi_err_sum) + ...
						obj.KD_phi * (0 - obj.w(1))); % With small angle Approx.
						%(Diff. is not stable in micro_processors such as DSP, ARM)(Gain might differ)
%                         obj.KD_phi * (obj.phi_err - obj.phi_err_prev)/obj.dt);
                      
            obj.u(3) = (obj.KP_theta * obj.theta_err + ...
                        obj.KI_theta * (obj.theta_err_sum) + ...
                        obj.KD_theta * (obj.theta_err - obj.theta_err_prev)/obj.dt);
                      
            obj.u(4) = (obj.KP_psi * obj.psi_err + ...
                        obj.KI_psi * (obj.psi_err_sum) + ...
                        obj.KD_psi * (obj.psi_err - obj.psi_err_prev)/obj.dt);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
            
            obj.phi_err_prev = obj.phi_err;
            obj.phi_err_sum  = obj.phi_err_sum + obj.phi_err;
            
            obj.theta_err_prev = obj.theta_err;
            obj.theta_err_sum  = obj.theta_err_sum + obj.theta_err;
            
            obj.psi_err_prev = obj.psi_err;
            obj.psi_err_sum  = obj.psi_err_sum + obj.psi_err;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Basic motor output assumed to make our problem easier.
            % Remove it when you want to put a realistic simulation
            % environment with the actual motor dynamics.
%             obj.zdot_err = obj.zdot_des - obj.dr(3);
%             
%             obj.u(1) = obj.m * obj.g;
%             obj.u(1) = obj.m * obj.g - ...
%                        (obj.KP_zdot * obj.zdot_err + ...
%                         obj.KI_zdot * (obj.zdot_err_sum) + ...
%                         obj.KD_zdot * (obj.zdot_err - obj.zdot_err_prev)/obj.dt);
%                     
%             obj.zdot_err_prev = obj.zdot_err;
%             obj.zdot_err_sum  = obj.zdot_err_sum + obj.zdot_err;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             obj.u(1) = 2 * obj.m * obj.g;
%             obj.u(2) = 0.0;
%             obj.u(3) = 0.0;
%             obj.u(4) = 0.0;

            obj.T = obj.u(1);
            obj.M = obj.u(2:4);
        end
        function obj = UpdatePWM(obj, itr)

            TM_Ctrl = obj.matMotor * [obj.T; obj.M];
            TM = Clip(TM_Ctrl, obj.motorThrustMin, obj.motorThrustMax);
            duty(1) = (TM(1) - obj.motorThrustMin) / (obj.motorThrustMax - obj.motorThrustMin) * 100;
%             Clip(obj.u(1), obj.motorThrustMin, obj.motorThrustMax);%
            if itr <= (duty(1)/10)
                obj.pwm(1) = true;
            else
                obj.pwm(1) = false;
            end
            
            duty(2) = (TM(2) - obj.motorThrustMin) / (obj.motorThrustMax - obj.motorThrustMin) * 100;
            if itr <= (duty(2)/10)
                obj.pwm(2) = true;
            else
                obj.pwm(2) = false;
            end
            
            duty(3) = (TM(3) - obj.motorThrustMin) / (obj.motorThrustMax - obj.motorThrustMin) * 100;
            if itr <= (duty(3)/10)
                obj.pwm(3) = true;
            else
                obj.pwm(3) = false;
            end
            
            duty(4) = (TM(4) - obj.motorThrustMin) / (obj.motorThrustMax - obj.motorThrustMin) * 100;
            if itr <= (duty(4)/10)
                obj.pwm(4) = true;
            else
                obj.pwm(4) = false;
            end
        end
    end
end

