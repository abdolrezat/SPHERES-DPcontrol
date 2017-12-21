classdef Simulator_CW < handle
    % Simulator_Class Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mode %simulator mode, 'fault' for one thruster inoperative situation
        faulty_thruster_index %index of faulty thruster #0 - #11
        N % number of stages
        Mass % Mass
        InertiaM % Moment of Inertia Matrix
        
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
        controller_params
        F_controller
        M_controller_J1
        M_controller_J2
        M_controller_J3
        controller_InterpmodeF %controller interpolation mode, 'nearest' for thruster on-off firings only
        controller_InterpmodeM
        thruster_allocation_mode % 'PWPF', 'Schmitt' (for schmitt trigger only) and 'none' for continuous force application
        active_set
        schmitt_trigger1
        schmitt_trigger2
        schmitt_trigger3
        schmitt_trigger4
        schmitt_trigger5
        schmitt_trigger6
        PWPF1
        PWPF2
        PWPF3
        PWPF4
        PWPF5
        PWPF6
        thrust_combs %used for one time calculation of combination of thruster firings
        history %structure to save all states and actions after run
        mu
        norm_R %target sat params
        RdotV
        H
        U_M %control actions
        a_x
        a_y
        a_z
    end
    
    methods
        function this = Simulator_CW(simopts, controller)
            if nargin == 0
                error('the simulator requires options structure as an input')
            else
                try
                    this.controller_params = controller; %uses options such as Ki
                catch 
                    warning('assuming DynamicProgramming controller')
                    this.controller_params.type = 'DP';
                end
                %set (DP) controller name
                if(~strcmp(this.controller_params.type,'PID'))
                    this.current_controller = simopts.current_controller;
                end
                
                this.T_final = simopts.T_final;
                this.h = simopts.h;
                this.defaultX0 = simopts.defaultX0;
                this.mode = simopts.mode;
                this.controller_InterpmodeF = simopts.controller_InterpmodeF;
                this.controller_InterpmodeM = simopts.controller_InterpmodeM;
                this.thruster_allocation_mode = simopts.thruster_allocation_mode;
                this.faulty_thruster_index = simopts.faulty_thruster_index;
                this.active_set.Weighting_Matrix = simopts.active_set.Weighting_Matrix;
            end
            
            if(strcmp(this.mode,'normal') == 1)
                this.faulty_thruster_index = [];
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
            
            
            if(rem(this.T_final,this.h) == 0)
                this.N_stage = this.T_final/this.h;
                
            else
                this.N_stage = this.T_final/this.h;
                this.N_stage = ceil(this.N_stage);
                this.T_final = this.h*this.N_stage;
                warning('T_final is not a factor of h (dt), increasing T_final to %.3f\n',this.T_final)
            end
            
            %Thruster Forces
            this.Thruster_max_F = simopts.Thruster_max_F; % (N)
            this.T_dist = simopts.Thruster_dist; % (meters)
            
            %6x schmitt trigger objects
            this.schmitt_trigger1 = Schmitt_trigger_c(...
                simopts.schmitt.Uout,simopts.schmitt.Uon,simopts.schmitt.Uoff);
            this.schmitt_trigger2 = Schmitt_trigger_c(...
                simopts.schmitt.Uout,simopts.schmitt.Uon,simopts.schmitt.Uoff);
            this.schmitt_trigger3 = Schmitt_trigger_c(...
                simopts.schmitt.Uout,simopts.schmitt.Uon,simopts.schmitt.Uoff);
            this.schmitt_trigger4 = Schmitt_trigger_c(...
                simopts.schmitt.Uout,simopts.schmitt.Uon,simopts.schmitt.Uoff);
            this.schmitt_trigger5 = Schmitt_trigger_c(...
                simopts.schmitt.Uout,simopts.schmitt.Uon,simopts.schmitt.Uoff);
            this.schmitt_trigger6 = Schmitt_trigger_c(...
                simopts.schmitt.Uout,simopts.schmitt.Uon,simopts.schmitt.Uoff);
            %6x PWPF objects
            this.PWPF1 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF.Km, simopts.PWPF.Tm, simopts.PWPF.h, ...
                simopts.schmitt.Uout, simopts.schmitt.Uon, simopts.schmitt.Uoff, simopts.PWPF.H_feed);
            this.PWPF2 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF.Km, simopts.PWPF.Tm, simopts.PWPF.h, ...
                simopts.schmitt.Uout, simopts.schmitt.Uon, simopts.schmitt.Uoff, simopts.PWPF.H_feed);
            this.PWPF3 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF.Km, simopts.PWPF.Tm, simopts.PWPF.h, ...
                simopts.schmitt.Uout, simopts.schmitt.Uon, simopts.schmitt.Uoff, simopts.PWPF.H_feed);
            this.PWPF4 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF.Km, simopts.PWPF.Tm, simopts.PWPF.h, ...
                simopts.schmitt.Uout, simopts.schmitt.Uon, simopts.schmitt.Uoff, simopts.PWPF.H_feed);
            this.PWPF5 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF.Km, simopts.PWPF.Tm, simopts.PWPF.h, ...
                simopts.schmitt.Uout, simopts.schmitt.Uon, simopts.schmitt.Uoff, simopts.PWPF.H_feed);
            this.PWPF6 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF.Km, simopts.PWPF.Tm, simopts.PWPF.h, ...
                simopts.schmitt.Uout, simopts.schmitt.Uon, simopts.schmitt.Uoff, simopts.PWPF.H_feed);
            % control allocation function to thrusters, from all combinations of Forces
            % and Moments that can be generated with the operating thrusters
            id_fault_channel_1 = this.faulty_thruster_index(...
                ismember(this.faulty_thruster_index, [0 1 6 7]));
            id_fault_channel_2 = this.faulty_thruster_index(...
                ismember(this.faulty_thruster_index, [2 3 8 9])) - 2;
            id_fault_channel_3 = this.faulty_thruster_index(...
                ismember(this.faulty_thruster_index, [4 5 10 11])) - 4;
            
            [this.thrust_combs.f0_comb,this.thrust_combs.f1_comb,...
                this.thrust_combs.f6_comb,this.thrust_combs.f7_comb] = ...
                all_feasible_thruster_u(id_fault_channel_1);
            [this.thrust_combs.f2_comb,this.thrust_combs.f3_comb,...
                this.thrust_combs.f8_comb,this.thrust_combs.f9_comb] = ...
                all_feasible_thruster_u(id_fault_channel_2);
            [this.thrust_combs.f4_comb,this.thrust_combs.f5_comb,...
                this.thrust_combs.f10_comb,this.thrust_combs.f11_comb] = ...
                all_feasible_thruster_u(id_fault_channel_3);
        end
        
        
        function f = get_thruster_on_off_optimal(obj,M_req, a_Body_req, X_stage)
            % gets the optimal on/off state of thrusters by assigning
            % thruster forces properly to make up accelerations in the Body
            % frame of reference
            F_Body_req = a_Body_req*obj.Mass;
            B = [F_Body_req;M_req/obj.T_dist];
            f = zeros(12,1);
            %Control allocation to Thruster Pairs
%             A = [1,1,0,0,0,0;...
%                 0,0,1,1,0,0;...
%                 0,0,0,0,1,1;...
%                 0,0,0,0,1,-1;...
%                 1,-1,0,0,0,0;...
%                 0,0,1,-1,0,0];
            invA = [0.5,  0,  0,  0,  0.5,  0; ...
                    0.5,  0,  0,  0,  -0.5,  0; ...
                    0,  0.5,  0,  0,  0,  0.5; ...
                    0,  0.5,  0,  0,  0,  -0.5; ...
                    0,  0,  0.5,  0.5,  0,  0; ...
                    0,  0,  0.5,  -0.5,  0,  0]; % A*f = [F;M/Tdist], invA = inv(A)
            f_pairs_req = invA*B;
            
            switch( lower(obj.thruster_allocation_mode) )
                case 'active set discrete'
                    [f0,f1,f6,f7] = asd_allocation_logic(obj, B(1), B(5), ...
                        obj.thrust_combs.f0_comb,obj.thrust_combs.f1_comb,...
                        obj.thrust_combs.f6_comb,obj.thrust_combs.f7_comb);
                    [f2,f3,f8,f9] = asd_allocation_logic(obj, B(2), B(6), ...
                        obj.thrust_combs.f2_comb,obj.thrust_combs.f3_comb,...
                        obj.thrust_combs.f8_comb,obj.thrust_combs.f9_comb);
                    [f4,f5,f10,f11] = asd_allocation_logic(obj, B(3), B(4), ...
                        obj.thrust_combs.f4_comb,obj.thrust_combs.f5_comb,...
                        obj.thrust_combs.f10_comb,obj.thrust_combs.f11_comb);
                    
                    f = [f0;f1;f2;f3;f4;f5;f6;f7;f8;f9;f10;f11];
                    
                case 'manual allocation'
                    [f0,f1,f6,f7] = manual_allocation_logic(B(1),B(5), X_stage(1), obj.Thruster_max_F);
                    [f2,f3,f8,f9] = manual_allocation_logic(B(2),B(6), X_stage(2), obj.Thruster_max_F);
                    [f4,f5,f10,f11] = manual_allocation_logic(B(3),B(4), X_stage(3), obj.Thruster_max_F);
                    f = [f0;f1;f2;f3;f4;f5;f6;f7;f8;f9;f10;f11];
                    
                case 'testfault'
                    
                    [f0,f1,f6,f7] = manual_allocation_logic_fault_one(B(1),B(5), X_stage(1), X_stage(12), obj.Thruster_max_F,0);
                    [f2,f3,f8,f9] = manual_allocation_logic(B(2),B(6), X_stage(2), obj.Thruster_max_F);
                    [f4,f5,f10,f11] = manual_allocation_logic(B(3),B(4), X_stage(3), obj.Thruster_max_F);
                    f = [f0;f1;f2;f3;f4;f5;f6;f7;f8;f9;f10;f11];
                    
                case 'pwpf'
                    % Single Thruster Allocation in each pair
                    f_pairs = PWPF_allpairs(obj,f_pairs_req);
                    for i=1:6
                        if(f_pairs(i) < 0)
                            f(i+6) = -f_pairs(i);
                        else
                            f(i) = f_pairs(i);
                        end
                    end
                    
                    if(strcmp(obj.mode,'fault') == 1) %control in fault mode, one thruster (f#0) off
                        f(obj.faulty_thruster_index +1) = 0;
                    end
                    
                case 'schmitt'
                    f_pairs = schmitt_allpairs(obj,f_pairs_req);
                    
                    for i=1:6
                        if(f_pairs(i) < 0)
                            f(i+6) = -f_pairs(i);
                        else
                            f(i) = f_pairs(i);
                        end
                    end
                    if(strcmp(obj.mode,'fault') == 1) %control in fault mode, one thruster (f#0) off
                        f(obj.faulty_thruster_index +1) = 0;
                    end
                    
                case 'none'
                    % debug code block
                    f_pairs = f_pairs_req; %cancels out schmitt trigger
                    
                    for i=1:6
                        if(f_pairs(i) < 0)
                            f(i+6) = -f_pairs(i);
                        else
                            f(i) = f_pairs(i);
                        end
                    end
                    if(strcmp(obj.mode,'fault') == 1) %control in fault mode, one thruster (f#0) off
                        f(obj.faulty_thruster_index +1) = 0;
                    end
                    %
                otherwise
                    error('unknown allocation method')
            end
            
        end
        
        function f_pairs = schmitt_allpairs(obj,f_pairs)
            f_pairs(1) = obj.schmitt_trigger1.signal_update(f_pairs(1));
            f_pairs(2) = obj.schmitt_trigger2.signal_update(f_pairs(2));
            f_pairs(3) = obj.schmitt_trigger3.signal_update(f_pairs(3));
            f_pairs(4) = obj.schmitt_trigger4.signal_update(f_pairs(4));
            f_pairs(5) = obj.schmitt_trigger5.signal_update(f_pairs(5));
            f_pairs(6) = obj.schmitt_trigger6.signal_update(f_pairs(6));
        end
        
        function f_pairs = PWPF_allpairs(obj,f_pairs)
            f_pairs(1) = obj.PWPF1.signal_update(f_pairs(1));
            f_pairs(2) = obj.PWPF2.signal_update(f_pairs(2));
            f_pairs(3) = obj.PWPF3.signal_update(f_pairs(3));
            f_pairs(4) = obj.PWPF4.signal_update(f_pairs(4));
            f_pairs(5) = obj.PWPF5.signal_update(f_pairs(5));
            f_pairs(6) = obj.PWPF6.signal_update(f_pairs(6));
        end
        
        function get_optimal_path(obj)
            obj.mu = 398600;
            if nargin == 1
                %   Prescribed initial state vector of chaser B in the co-moving frame:
                X0 = obj.defaultX0;
                tf  = obj.T_final;
                N_total_sim = obj.N_stage;
            end
            
            %% set controllers
            %             obj.set_controller('controller_linspace2_70m_70deg.mat');
            if(~strcmp(obj.controller_params.type,'PID'))
                obj.set_controller(obj.current_controller);
            else
                %else initialize (P I* D) Integral* errors
                obj.controller_params.integral_a1 = 0;
                obj.controller_params.integral_a2 = 0;
                obj.controller_params.integral_a3 = 0;
                obj.controller_params.integral_M1 = 0;
                obj.controller_params.integral_M2 = 0;
                obj.controller_params.integral_M3 = 0;
                
               obj.controller_params.prev_error_a1 = 0;
               obj.controller_params.prev_error_a2 = 0;
               obj.controller_params.prev_error_a3 = 0;
               obj.controller_params.prev_error_M1 = 0;
               obj.controller_params.prev_error_M2 = 0;
               obj.controller_params.prev_error_M3 = 0;
            end
            %%
            
            tspan = 0:obj.h:tf;
            X_ode45 = zeros(N_total_sim, 13);
            F_Th_Opt = zeros(N_total_sim, 12);
            Force_Moment_log = zeros(N_total_sim, 6);
            Force_Moment_log_req = zeros(N_total_sim, 6);
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
                obj.norm_R = (R*R')^.5; %norm R
                obj.RdotV = sum(R.*V); %dot product
                crossRV = [R(2).*V(3)-R(3).*V(2); % cross product of R and V
                    R(3).*V(1)-R(1).*V(3);
                    R(1).*V(2)-R(2).*V(1)];
                obj.H  = (crossRV'*crossRV)^.5 ; %norm(crossRV);
                
                % required (Optimal) moments (U_M) and directional accelerations (a_* |x,y,z|) with
                % respect to inertial frame
                [U_M_req, a_req] = Opt_Force_Moments(obj,X_stage);
                %rotation matrices
                rotM_RSW2ECI = RSW2ECI(obj, R0, V0);
                rotM_ECI2body = ECI2body(obj,q_stage);
                % required directional accelerations in Body frame of reference
                a_Body_req = rotM_ECI2body*rotM_RSW2ECI*  a_req;
                M_Body_req = U_M_req;
                % debugging block of code for a certain time
%                 if(tspan(k_stage) == 60.615) keyboard; end
                %
                f_thruster = get_thruster_on_off_optimal(obj, M_Body_req, a_Body_req, X_stage);
                % accelerations from thruster forces1
                [obj.U_M, obj.a_x, obj.a_y, obj.a_z] = to_Moments_Forces(obj,...
                    f_thruster, rotM_RSW2ECI, rotM_ECI2body);
                %
%                 %% test environment for controller,
%                 % enable the block below to check controller performance
%                 % without any thruster limit
%                 a_x = a_req(1);
%                 a_y = a_req(2);
%                 a_z = a_req(3);
%                 U_M = U_M_req;
%                 
                %log
                F_Th_Opt(k_stage,:) = f_thruster;
                Force_Moment_log(k_stage,:) = [obj.a_x, obj.a_y, obj.a_z, obj.U_M'];
                Force_Moment_log_req(k_stage,:) = [a_req;U_M_req];
                % use RK4 instead of ode45 for more speed and no less
                % accuracy
%                 X_temp = ode_1(obj, @ode_eq,[tspan(k_stage), tspan(k_stage+1)], X_stage);
                X_temp = ode_RK4(obj, X_stage);
%                 [~,X_temp] = ode23(@ode_eq,[tspan(k_stage), tspan(k_stage+1)], X_stage);
%                 [~,X_temp] = ode45(@ode_eq,[tspan(k_stage), tspan(k_stage+1)], X_stage);
                X_ode45(k_stage+1,:) = X_temp(end,:);
            end
            toc
            T_ode45 = tspan(1:end-1)';
            %save history
            obj.history = struct('T_ode45',T_ode45,'Force_Moment_log',Force_Moment_log,...
                'X_ode45',X_ode45,'F_Th_Opt',F_Th_Opt,'Force_Moment_log_req',Force_Moment_log_req);
            
        end
        
        function history = get_optimal_path_history(obj)
            %runs the simulator and outputs the history of states and
            %actions, useful for optimization purposes
           if isempty(obj.history)
               get_optimal_path(obj) % runs the simulator to get results, if not done previously
           end
           history = obj.history;
           
        end
        
        function plot_optimal_path(obj)
           if isempty(obj.history)
               get_optimal_path(obj) % runs the simulator to get results, if not done previously
           end
            % plot results
            plot_results(obj)
            % print response information (e.g. settling time & rise time) 
            x1_info = stepinfo(obj.history.X_ode45(:,1),obj.history.T_ode45,0)
        end
        
        function [U_M, a_x, a_y, a_z] = to_Moments_Forces(obj,f_thruster,rotM_RSW2ECI, rotM_ECI2body)
            f0 = f_thruster(1);
            f1 = f_thruster(2);
            f2 = f_thruster(3);
            f3 = f_thruster(4);
            f4 = f_thruster(5);
            f5 = f_thruster(6);
            f6 = f_thruster(7);
            f7 = f_thruster(8);
            f8 = f_thruster(9);
            f9 = f_thruster(10);
            f10 = f_thruster(11);
            f11 = f_thruster(12);
            
            % Moments
            U_M_y_body = (f0-f1-f6+f7)*obj.T_dist;
            U_M_z_body = (f2-f3-f8+f9)*obj.T_dist;
            U_M_x_body = (f4-f5-f10+f11)*obj.T_dist;
            U_M_body = [U_M_x_body; U_M_y_body; U_M_z_body];
            %             U_M = rotM_RSW2ECI\(rotM_ECI2body\U_M_body);
            U_M = U_M_body;
            
            % Accelerations (expressed in body frame of reference)
            a_x_body = (f0+f1-f6-f7)/obj.Mass;
            a_y_body = (f2+f3-f8-f9)/obj.Mass;
            a_z_body = (f4+f5-f10-f11)/obj.Mass;
            % transform vectors
            %             rotM_RSW2ECI = RSW2ECI(obj, R0, V0);
            %             rotM_ECI2body = ECI2body(obj,q);
            
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
                single(controller.v_Fthruster(controller.F_U_Optimal_id)), obj.controller_InterpmodeF,'nearest');
            
            obj.M_controller_J1 = griddedInterpolant(controller.M_gI_J1,...
                single(controller.v_Mthruster(controller.M_U_Optimal_id_J1)), obj.controller_InterpmodeM,'nearest');
            obj.M_controller_J2 = griddedInterpolant(controller.M_gI_J2,...
                single(controller.v_Mthruster(controller.M_U_Optimal_id_J2)), obj.controller_InterpmodeM,'nearest');
            obj.M_controller_J3 = griddedInterpolant(controller.M_gI_J3,...
                single(controller.v_Mthruster(controller.M_U_Optimal_id_J3)), obj.controller_InterpmodeM,'nearest');
        end
        
        function [M, a] = Opt_Force_Moments(obj, Xin)
            
            if(~strcmp(obj.controller_params.type,'PID'))
                % Dynamic Programming Controller
                % Forces (expressed in body frame of reference)
                a = (obj.F_controller( Xin(1:3), Xin(4:6))/obj.Mass)'; % X(1:3) represent position and X(4:6) velocities
                
                %Moments
                M = zeros(3,1);
                M(1) = obj.M_controller_J1( 2*asin(Xin(7)) , Xin(11) ); %where X(11:13) represent rotational speeds
                M(2) = obj.M_controller_J2( 2*asin(Xin(8)) , Xin(12) ); %where X(11:13) represent rotational speeds
                M(3) = obj.M_controller_J3( 2*asin(Xin(9)) , Xin(13) ); %where X(11:13) represent rotational speeds
                
            else
                % Integrals for PID controller
                obj.controller_params.integral_a1 = obj.controller_params.integral_a1 + Xin(1)*obj.h;
                obj.controller_params.integral_a2 = obj.controller_params.integral_a2 + Xin(2)*obj.h;
                obj.controller_params.integral_a3 = obj.controller_params.integral_a3 + Xin(3)*obj.h;
                obj.controller_params.integral_M1 = obj.controller_params.integral_M1 + 2*asin(Xin(7))*obj.h;
                obj.controller_params.integral_M2 = obj.controller_params.integral_M2 + 2*asin(Xin(8))*obj.h;
                obj.controller_params.integral_M3 = obj.controller_params.integral_M3 + 2*asin(Xin(9))*obj.h;
                
                a = zeros(3,1);
                a(1) = Xin(1)* -obj.controller_params.Kp_F + ...
                    (Xin(1) - obj.controller_params.prev_error_a1) * -obj.controller_params.Kd_F / obj.h + ...
                    obj.controller_params.integral_a1 * -obj.controller_params.Ki_F;
                
                a(2) = Xin(2)* -obj.controller_params.Kp_F + ...
                    (Xin(2) - obj.controller_params.prev_error_a2) * -obj.controller_params.Kd_F / obj.h + ...
                    obj.controller_params.integral_a2 * -obj.controller_params.Ki_F;
                
                a(3) = Xin(3)* -obj.controller_params.Kp_F + ...
                    (Xin(3) - obj.controller_params.prev_error_a3) * -obj.controller_params.Kd_F / obj.h + ...
                    obj.controller_params.integral_a3 * -obj.controller_params.Ki_F;
                
                a = a/obj.Mass; 
                
                M = zeros(3,1);
                M(1) = 2*asin(Xin(7)) * -obj.controller_params.Kp_M + ...
                    (2*asin(Xin(7)) - obj.controller_params.prev_error_M1) * -obj.controller_params.Kd_M / obj.h + ...
                    obj.controller_params.integral_M1 * -obj.controller_params.Ki_M;
                M(2) = 2*asin(Xin(8)) * -obj.controller_params.Kp_M + ...
                    (2*asin(Xin(8)) - obj.controller_params.prev_error_M2) * -obj.controller_params.Kd_M / obj.h + ...
                    obj.controller_params.integral_M2 * -obj.controller_params.Ki_M;
                M(3) = 2*asin(Xin(9)) * -obj.controller_params.Kp_M + ...
                    (2*asin(Xin(9)) - obj.controller_params.prev_error_M3) * -obj.controller_params.Kd_M / obj.h + ...
                    obj.controller_params.integral_M3 * -obj.controller_params.Ki_M;
               %save errors
               
               obj.controller_params.prev_error_a1 = Xin(1);
               obj.controller_params.prev_error_a2 = Xin(2);
               obj.controller_params.prev_error_a3 = Xin(3);
               obj.controller_params.prev_error_M1 = 2*asin(Xin(7));
               obj.controller_params.prev_error_M2 = 2*asin(Xin(8));
               obj.controller_params.prev_error_M3 = 2*asin(Xin(9));
                
            end
            
            
        end
        
        function [f0,f1,f6,f7] = asd_allocation_logic(obj, Freq, Mreq, ...
                f0_comb,f1_comb,f6_comb,f7_comb)
            
            L = length(f0_comb);
            B = [1,1,-1,-1;...
                1,-1,-1,1];
            all_evaluations = zeros(1,L);
            W = obj.active_set.Weighting_Matrix([1;4]);
            
            for ii=1:L
                %u = [f0_comb(ii);f1_comb(ii);f6_comb(ii);f7_comb(ii)] * obj.Thruster_max_F;
                %e = (B*u - [Freq; Mreq]);
                %all_evaluations(ii) = e' * obj.active_set.Weighting_Matrix * e;
                all_evaluations(1,ii) = sum((B*[f0_comb(ii);f1_comb(ii);f6_comb(ii);f7_comb(ii)] * obj.Thruster_max_F ...
                    - [Freq; Mreq]).^2 .* W);
            end
            
            [best_evaluation, best_evaluation_id] = min(all_evaluations, [], 2); %#ok<ASGLU>
            
%             if ( length(all_evaluations((all_evaluations - best_evaluation) == 0)) > 1)
%                 keyboard;
%             end
            
            f0 = f0_comb(best_evaluation_id)*obj.Thruster_max_F;
            f1 = f1_comb(best_evaluation_id)*obj.Thruster_max_F;
            f6 = f6_comb(best_evaluation_id)*obj.Thruster_max_F;
            f7 = f7_comb(best_evaluation_id)*obj.Thruster_max_F;
            
        end
        
        function X_dot = system_dynamics(obj, X)
            mu_ = obj.mu;
            norm_R_ = obj.norm_R;
            H_ = obj.H;
            RdotV_ = obj.RdotV;
            
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
            X_dot = zeros(13,1);
            X_dot(1) = v1;
            X_dot(2) = v2;
            X_dot(3) = v3;
            
            % position - v (a_x,y,z are in RSW frame of reference)
            X_dot(4) =  (2*mu_/norm_R_^3 + H_^2/norm_R_^4)*x1 - 2*RdotV_/norm_R_^4*H_*x2 + 2*H_/norm_R_^2*v2 ...
                + obj.a_x;
            X_dot(5) = -(mu_/norm_R_^3 - H_^2/norm_R_^4)*x2 + 2*RdotV_/norm_R_^4*H_*x1 - 2*H_/norm_R_^2*v1 ...
                + obj.a_y;
            X_dot(6) = -mu_/norm_R_^3*x3 ...
                + obj.a_z;
            
            % attitude - q
            X_dot(7) = 0.5*(w3.*q2 -w2.*q3 +w1.*q4);
            X_dot(8) = 0.5*(-w3.*q1 +w1.*q3 +w2.*q4);
            X_dot(9) = 0.5*(w2.*q1 -w1.*q2 +w3.*q4);
            X_dot(10) = 0.5*(-w1.*q1 -w2.*q2 -w3.*q3);
            
            % attitude - w
            w_dot = obj.InertiaM\(obj.U_M - cross(w_vector, obj.InertiaM*w_vector));
            X_dot(11) = w_dot(1);
            X_dot(12) = w_dot(2);
            X_dot(13) = w_dot(3);
        end
        
        function X2 = ode_RK4(obj, x)
            % Runge-Kutta - 4th order
            % h = dt;
            h_t = obj.h;
            x = x';
            k1 = system_dynamics(obj, x);
            k2 = system_dynamics(obj,(x + k1*h_t/2));
            k3 = system_dynamics(obj,(x + k2*h_t/2));
            k4 = system_dynamics(obj,(x + k3*h_t));
            
            X2 = x + h_t*(k1 + 2*k2 + 2*k3 + k4)/6;
            X2 = X2';
        end
        
          function X2 = ode_1(~, ode_fun, t_vector, x)
            % Runge-Kutta - 4th order
            % h = dt;
            t1 = t_vector(1);
            h_t = t_vector(2) - t1;
            x = x';
            k1 = ode_fun(t1, x);
            X2 = x + h_t*k1;
            X2 = X2';
          end
        
          
    end
    
end



