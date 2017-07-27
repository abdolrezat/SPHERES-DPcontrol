classdef Simulator_CW < handle
    % Simulator_Class Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        N % number of stages
        Mass % Mass
        InertiaM % Moment of Inertia Matrix
        J1 % element 1 of Inertia Matrix
        J2 % ...
        J3 % ..
        
        T_final
        h
        N_stage
        defaultX0
        
        T_dist %Thruster placement distanceS
        %
        Thruster_max_F
        Thruster_max_M
        
        %controllers 
        current_controller %name of the controller mat file
        F_controller
        M_controller_J1
        M_controller_J2
        M_controller_J3
        
    end
    
    methods
        function this = Simulator_CW(simopts)
            if nargin == 0
                this.current_controller = 'controller_linspace2_20m_20deg_3F';
                this.T_final = 60;
                this.h = 0.005;
                
                dr0 = [10 12 0];
                dv0 = [0 0 0];
                q0 = flip(angle2quat(deg2rad(20),deg2rad(40),deg2rad(0)));
                
                w0 = [0 0 0];
                this.defaultX0 = [dr0 dv0 q0 w0]';
            else
                this.current_controller = simopts.current_controller;
                this.T_final = simopts.T_final;
                this.h = simopts.h;
                this.defaultX0 = simopts.defaultX0;
            end
            
                this.Mass = 4.16;
                inertia(1,1) =  0.02836 + 0.00016;
                inertia(2,1) =  0.026817 + 0.00150;
                inertia(3,1) =  0.023 + 0.00150;
                inertia(4,1) = -0.0000837;
                inertia(5,1) =  0.000014;
                inertia(6,1) = -0.00029;
                this.InertiaM = [inertia(1,1)  inertia(4,1)  inertia(5,1);...
                    inertia(4,1)  inertia(2,1)  inertia(6,1);...
                    inertia(5,1)  inertia(6,1)  inertia(3,1)];
                
                this.J1 = this.InertiaM(1);
                this.J2 = this.InertiaM(5);
                this.J3 = this.InertiaM(9);
               
            
            
            this.N_stage = this.T_final/this.h;
            
            if(~isinteger( this.N_stage))
                this.N_stage = ceil(this.N_stage);
                this.T_final = this.h*this.N_stage;
                warning('T_final is not a factor of h (dt), increasing T_final to %.2f\n',this.T_final)
            end
            
            %Thruster Forces
            this.Thruster_max_F = 0.13; % (N)
            this.T_dist = 9.65E-2; % (meters)
        end
        
        
        function [x_next,v_next,t_next,w_next] = next_stage_states_simplified(obj, X, V, T, W, f1,f2,f6,f7 ,J)
            %store length
            L_x = length(X);
            L_v = length(V);
            L_t = length(T);
            L_w = length(W);
            
            % reshape
            X = reshape(X,[L_x 1]  );
            V = reshape(V,[1 L_v]  );
            T = reshape(T,[1 1 L_t]  );
            W = reshape(W,[1 1 1 L_w]  );
            f1 = reshape(f1,[1 1 1 1 length(f1)]  );
            f2 = reshape(f2,[1 1 1 1 length(f2)]  );
            f6 = reshape(f6,[1 1 1 1 length(f6)]  );
            f7 = reshape(f7,[1 1 1 1 length(f7)]  );
            
            % ODE solve
            x_next = RK4_x(obj, X, V);
            v_next = RK4_v(obj, V, f1,f2,f6,f7);
            t_next = RK4_t(obj, T, W);
            w_next = RK4_w(obj, W, f1,f2,f6,f7 , J);
            

            %repmat each matrix to full size, as required for F inputs
            x_next = repmat(x_next,[1 1 L_t L_w length(f1)]);
            v_next = repmat(v_next,[L_x 1 L_t L_w 1]);
            t_next = repmat(t_next,[L_x L_v 1 1 length(f1)]);
            w_next = repmat(w_next,[L_x L_v L_t 1 1]);    
            end
        
        function X2 = RK4_x(obj, X1, V)
            % Runge-Kutta - 4th order
            % h = dt;
            k1 = xdynamics(obj, V);
            k2 = xdynamics(obj,(V + k1*obj.h/2));
            k3 = xdynamics(obj,(V + k2*obj.h/2));
            k4 = xdynamics(obj,(V + k3*obj.h));
            
            X2 = X1 + obj.h*(k1 + 2*k2 + 2*k3 + k4)/6;
            
        end
        
        function x_dot = xdynamics(~,v)
            x_dot = v;
        end
        
        function V2 = RK4_v(obj, V1, f1,f2,f6,f7) % does not need RK4, ki's are equal
            % Runge-Kutta - 4th order
            % h = dt;
            k1 = vdynamics(obj, V1 , f1,f2,f6,f7);
            k2 = vdynamics(obj,(V1 + k1*obj.h/2), f1,f2,f6,f7);
            k3 = vdynamics(obj,(V1 + k2*obj.h/2), f1,f2,f6,f7);
            k4 = vdynamics(obj,(V1 + k3*obj.h), f1,f2,f6,f7);
            
            V2 = V1 + obj.h*(k1 + 2*k2 + 2*k3 + k4)/6;
        end
        
        function v_dot = vdynamics(obj, ~, f1,f2,f6,f7)
            v_dot = (f1+f2+f6+f7)/obj.Mass;
        end
        
        function T2 = RK4_t(obj, T1, W1)
            %calculates next stage (k+1) states
            % X2 = X1 + dt*a_d(X,u) where a_d is the spacecraft dynamics
            %first order taylor expansion
            %X2 = X1 + dt*spacecraft_dynamics(spacecraft, X1, U);
            
            % Runge-Kutta - 4th order
            % h = dt;
            k1 = tdynamics(obj,W1);
            k2 = tdynamics(obj,(W1 + k1*obj.h/2));
            k3 = tdynamics(obj,(W1 + k2*obj.h/2));
            k4 = tdynamics(obj,(W1 + k3*obj.h));
            
            T2 = T1 + obj.h*(k1 + 2*k2 + 2*k3 + k4)/6;
        end
        
        
        function t_dot = tdynamics(~,w)
            t_dot = w;
        end
        
        function W2 = RK4_w(obj, w, f1,f2,f6,f7, J)
            %calculates next stage (k+1) states
            % X2 = X1 + dt*a_d(X,u) where a_d is the spacecraft dynamics
            %first order taylor expansion
            %X2 = X1 + dt*spacecraft_dynamics(spacecraft, X1, U);
            
            % Runge-Kutta - 4th order
            % h = dt;
            k1 = wdynamics(obj,w , f1,f2,f6,f7, J);
            k2 = wdynamics(obj,(w + k1*obj.h/2), f1,f2,f6,f7, J);
            k3 = wdynamics(obj,(w + k2*obj.h/2), f1,f2,f6,f7, J);
            k4 = wdynamics(obj,(w + k3*obj.h), f1,f2,f6,f7, J);
            
            W2 = w + obj.h*(k1 + 2*k2 + 2*k3 + k4)/6;
        end
        
        
        function w_dot = wdynamics(obj,~, f1,f2,f6,f7, J)
            w_dot = (f1*obj.T_dist +f2*(-obj.T_dist) + ...
                f6*obj.T_dist +f7*(-obj.T_dist) )/J;
        end
        
        function [f0,f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11] = get_thruster_on_off_optimal(obj,x,v,t,w,R0,V0,q)
            % gets the optimal on/off state of thrusters, method run() or
            % simplified_run() must be run before calling this function,
            % the inputs x,v are position and velocity of object B
            % to target A in RSW frame
            
            %transform vectors to body frame of reference
            rotM_RSW2ECI = RSW2ECI(obj, R0, V0);
            rotM_ECI2body = ECI2body(obj,q);
            
            x = rotM_ECI2body*(rotM_RSW2ECI* x');
            v = rotM_ECI2body*(rotM_RSW2ECI* v');
            
            %
            t_x = t(1); %rotation about x_axis
            t_y = t(2);
            t_z = t(3);
            
            w_x = w(1); %rotational speed about x_axis
            w_y = w(2);
            w_z = w(3);
            
            x1 = x(1);
            x2 = x(2);
            x3 = x(3);
            
            v1 = v(1);
            v2 = v(2);
            v3 = v(3);
            
            f0 = obj.Opt_F_Thr0(x1,v1, t_y, w_y );
            f1 = obj.Opt_F_Thr1(x1,v1, t_y, w_y );
            f6 = obj.Opt_F_Thr6(x1,v1, t_y, w_y );
            f7 = obj.Opt_F_Thr7(x1,v1, t_y, w_y );
            
            f2 = obj.Opt_F_Thr2(x2,v2, t_z, w_z );
            f3 = obj.Opt_F_Thr3(x2,v2, t_z, w_z );
            f8 = obj.Opt_F_Thr8(x2,v2, t_z, w_z );
            f9 = obj.Opt_F_Thr9(x2,v2, t_z, w_z );
            
            f4 = obj.Opt_F_Thr4(x3,v3, t_x, w_x );
            f5 = obj.Opt_F_Thr5(x3,v3, t_x, w_x );
            f10 = obj.Opt_F_Thr10(x3,v3, t_x, w_x );
            f11 = obj.Opt_F_Thr11(x3,v3, t_x, w_x );
            
        end
        
        
        function get_optimal_path(obj)
            mu = 398600;
            if nargin == 1
                %   Prescribed initial state vector of chaser B in the co-moving frame:
                X0 = obj.defaultX0;
                tf  = obj.T_final;
                N_total_sim = obj.N_stage;
            end
            
            %% set controllers
%             obj.set_controller('controller_linspace2_70m_70deg.mat');
            obj.set_controller(obj.current_controller)
            %%
            
            tspan = 0:obj.h:tf;
            X_ode45 = zeros(N_total_sim, 13);
            Force_Moment_log = zeros(N_total_sim, 6);
            X_ode45(1,:) = X0;
            % Calculate the target initial state vector
            [R0,V0] = get_target_R0V0();
            tic
            for k_stage=1:N_total_sim-1
                %determine F_Opt each Thruster
                X_stage = X_ode45(k_stage,:);
                q_stage = X_stage(7:10);
                
                % pre-computations
                [R,V] = update_RV_target(R0, V0, tspan(k_stage));
                norm_R = (R*R')^.5; %norm R
                RdotV = sum(R.*V); %dot product
                crossRV = [R(2).*V(3)-R(3).*V(2); % cross product of R and V
                    R(3).*V(1)-R(1).*V(3);
                    R(1).*V(2)-R(2).*V(1)];
                H  = (crossRV'*crossRV)^.5 ; %norm(crossRV);
                
                % moments (U_M) and directional forces (a_* |x,y,z|) with
                % respect to inertial frame
                [U_M, a_x, a_y, a_z] = Opt_Force_Moments(obj,R0,V0,X_stage,q_stage);
                
                %log
%                 F_Th_Opt(k_stage,:) = [f0,f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11];
                Force_Moment_log(k_stage,:) = [a_x, a_y, a_z, U_M'];
                %
                [~,X_temp] = ode45(@ode_eq,[tspan(k_stage), tspan(k_stage+1)], X_stage);
                X_ode45(k_stage+1,:) = X_temp(end,:);
            end
        toc
        T_ode45 = tspan(1:end-1)';
        % plot results
        plot_results(obj, T_ode45, Force_Moment_log, X_ode45 )
        
        %function declarations
            function x_dot = ode_eq(~,X1)
                x_dot = system_dynamics(X1);
                x_dot = x_dot';
                function X_dot = system_dynamics(X)
                    x1 = X(1);
                    x2 = X(2);
                    x3 = X(3);
                    v1 = X(4);
                    v2 = X(5);
                    v3 = X(6);
                    q1 = X(7);
                    q2 = X(8);
                    q3 = X(9);
                    q4 = X(10);
                    w1 = X(11);
                    w2 = X(12);
                    w3 = X(13);
                    w_vector = X(11:13);
                    %--- differential equations -------------------------
                    
                    % CW-equations
                    % position - x
                    X_dot(1) = v1;
                    X_dot(2) = v2;
                    X_dot(3) = v3;
                    
                    % position - v (a_x,y,z are in RSW frame of reference)
                    X_dot(4) =  (2*mu/norm_R^3 + H^2/norm_R^4)*x1 - 2*RdotV/norm_R^4*H*x2 + 2*H/norm_R^2*v2 ...
                        + a_x;
                    X_dot(5) = -(mu/norm_R^3 - H^2/norm_R^4)*x2 + 2*RdotV/norm_R^4*H*x1 - 2*H/norm_R^2*v1 ...
                        + a_y;
                    X_dot(6) = -mu/norm_R^3*x3 ...
                        + a_z;
                    
                    % attitude - q
                    X_dot(7) = 0.5*(w3.*q2 -w2.*q3 +w1.*q4);
                    X_dot(8) = 0.5*(-w3.*q1 +w1.*q3 +w2.*q4);
                    X_dot(9) = 0.5*(w2.*q1 -w1.*q2 +w3.*q4);
                    X_dot(10) = 0.5*(-w1.*q1 -w2.*q2 -w3.*q3);
                    
                    % attitude - w
                    w_dot = obj.InertiaM\(U_M - cross(w_vector, obj.InertiaM*w_vector));
                    X_dot(11) = w_dot(1);
                    X_dot(12) = w_dot(2);
                    X_dot(13) = w_dot(3);
                end
            end
        end
        
        
        function [U_M, a_x, a_y, a_z] = to_Moments_Forces(obj,f0,f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,R0,V0,q)
            % Moments
            U_M_y = (f0-f1+f6-f7)*obj.T_dist;
            U_M_z = (f2-f3+f8-f9)*obj.T_dist;
            U_M_x = (f4-f5+f10-f11)*obj.T_dist;
            U_M = [U_M_x; U_M_y; U_M_z];
            
            % Accelerations (expressed in body frame of reference)
            a_x_body = (f0+f1+f6+f7)/obj.Mass;
            a_y_body = (f2+f3+f8+f9)/obj.Mass;
            a_z_body = (f4+f5+f10+f11)/obj.Mass;
            % transform vectors
            rotM_RSW2ECI = RSW2ECI(obj, R0, V0);
            rotM_ECI2body = ECI2body(obj,q);
            
            accM = rotM_RSW2ECI\(rotM_ECI2body\[a_x_body a_y_body a_z_body]');
            a_x = accM(1);
            a_y = accM(2);
            a_z = accM(3);
        end
        
        function qrotMat = ECI2body(~, q)
            qrotMat = [1-2*(q(2)^2+q(3)^2), 2*(q(1)*q(2)+q(3)*q(4)), 2*(q(1)*q(3)-q(2)*q(4));...
                2*(q(2)*q(1)-q(3)*q(4)), 1-2*(q(1)^2+q(3)^2), 2*(q(2)*q(3) +q(1)*q(4));...
                2*(q(3)*q(1)+q(2)*q(4)), 2*(q(3)*q(2)-q(1)*q(4)), 1-2*(q(1)^2+q(2)^2)];
        end
        
        function rotMat = RSW2ECI(~, pos, vel)
            % rotMat = RSW2ECI(pos, vel);
            % Creates a rotation matrix which transforms RSW vectors to ECI vectors.
            % ECIvec = rotMat  *   RSW;
            % 3x1    = 3x3         3x1;
            % Inputs:
            %   pos:   ECI position vector
            %   vel:   ECI velocity vector
            % Outpus:
            %   rotMat: 3x3 rotation matrix from RSW to ECI
            
            R = pos/norm(pos);
            W = cross(pos,vel)/norm(cross(pos,vel));
            S = cross(W,R);
            
            rotMat = [R' S' W'];
        end
        
        function set_controller(obj, controller_name)
            path_ = strsplit(mfilename('fullpath'),'\\');
            path_ = strjoin(path_(1:end-1),'\');
            controller = load(strcat(path_,'\..\generate_controller\controller\',controller_name));
            obj.F_controller = griddedInterpolant(controller.F_gI,...
                single(controller.v_Fthruster(controller.F_U_Optimal_id)), 'linear','nearest');
            
            obj.M_controller_J1 = griddedInterpolant(controller.M_gI_J1,...
                single(controller.v_Mthruster(controller.M_U_Optimal_id_J1)), 'linear','nearest');
            obj.M_controller_J2 = griddedInterpolant(controller.M_gI_J2,...
                single(controller.v_Mthruster(controller.M_U_Optimal_id_J2)), 'linear','nearest');
            obj.M_controller_J3 = griddedInterpolant(controller.M_gI_J3,...
                single(controller.v_Mthruster(controller.M_U_Optimal_id_J3)), 'linear','nearest');
        end
        
        function [M, a_x, a_y, a_z] = Opt_Force_Moments(obj,R0,V0, Xin, q)
             % Forces (expressed in body frame of reference)
               a_x = obj.F_controller( Xin(1), Xin(4))/obj.Mass; % X(1:3) represent position and X(4:6) velocities
               a_y = obj.F_controller( Xin(2), Xin(5))/obj.Mass;
               a_z = obj.F_controller( Xin(3), Xin(6))/obj.Mass;
               
               %Moments
               M = zeros(3,1);
               M(1) = obj.M_controller_J1( 2*asin(Xin(7)) , Xin(11) ); %where X(11:13) represent rotational speeds
               M(2) = obj.M_controller_J2( 2*asin(Xin(8)) , Xin(12) ); %where X(11:13) represent rotational speeds
               M(3) = obj.M_controller_J3( 2*asin(Xin(9)) , Xin(13) ); %where X(11:13) represent rotational speeds
               
        end
    end
    
end
    


