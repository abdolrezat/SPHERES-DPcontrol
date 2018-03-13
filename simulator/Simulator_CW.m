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
        F_controller_x
        F_controller_y
        F_controller_z
        M_controller_J1
        M_controller_J2
        M_controller_J3
        controller_InterpmodeF %controller interpolation mode, 'nearest' for thruster on-off firings only
        controller_InterpmodeM
        thruster_allocation_mode % 'PWPF', 'Schmitt' (for schmitt trigger only) and 'none' for continuous force application
        active_set
        quad_prog
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
        thruster_min_ontime
        thr_program
        thr_program_counter
        bool_channel_fault
    end
    
    methods
        function this = Simulator_CW(simopts, controller)
            if nargin == 0
                error('the simulator requires options structure as an input')
            else
                try
                    this.controller_params = controller; %uses options such as Ki
                catch 
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
%                 this.quad_prog.Weighting_Matrix_1_default = simopts.Quad_Prog.Weighting_Matrix_1;
%                 this.quad_prog.Weighting_Matrix_2_default = simopts.Quad_Prog.Weighting_Matrix_2;
%                 this.quad_prog.Weighting_Matrix_3_default = simopts.Quad_Prog.Weighting_Matrix_3;
                this.quad_prog.Weighting_Matrix_F_default = [1, 0; 0, 0.93];
                this.quad_prog.Weighting_Matrix_1 = this.quad_prog.Weighting_Matrix_F_default;
                this.quad_prog.Weighting_Matrix_2 = this.quad_prog.Weighting_Matrix_F_default;
                this.quad_prog.Weighting_Matrix_3 = this.quad_prog.Weighting_Matrix_F_default;
                this.quad_prog.Weighting_Matrix_1_scheduled =  this.quad_prog.Weighting_Matrix_F_default;
                this.quad_prog.Weighting_Matrix_2_scheduled =  this.quad_prog.Weighting_Matrix_F_default;
                this.quad_prog.Weighting_Matrix_3_scheduled =  this.quad_prog.Weighting_Matrix_F_default;
                
                this.quad_prog.angle_up_bound = 140;
                this.quad_prog.angle_low_bound = 90;
                this.quad_prog.threshold_positive = 0.2;
                this.quad_prog.threshold_negative = 0;
                
                % assign quadratic programming parameter interpolant based
                % on which thruster(s) are failed
                params_distances = [2 5 6 10];
                params_values = [1.07 1.03 0.93 0.93]; % this determines Weighting_matrix(4)
                params_distances_mirror = -params_distances(end:-1:1);
                params_values_mirror = params_values(end:-1:1); % this determines Weighting_matrix(4)
                %channel - x
                if ismember(this.faulty_thruster_index, [0 1 6 7])
                    if ismember(this.faulty_thruster_index, [0 1])
                        params_distances_x = params_distances;
                        params_values_x = params_values;
                        this.quad_prog.bool_channel_fault.x01 = true;
                    elseif ismember(this.faulty_thruster_index, [6 7])
                        params_distances_x = params_distances_mirror;
                        params_values_x = params_values_mirror;
                        this.quad_prog.bool_channel_fault.x01 = false;
                    end
                    this.quad_prog.Fw1 = griddedInterpolant(params_distances_x , params_values_x, 'nearest','nearest');
                end
                
                %channel - y
                if ismember(this.faulty_thruster_index, [2 3 8 9])
                    if ismember(this.faulty_thruster_index, [2 3])
                        params_distances_y = params_distances;
                        params_values_y = params_values;
                        this.quad_prog.bool_channel_fault.y23 = true;
                    elseif ismember(this.faulty_thruster_index, [8 9])
                        params_distances_y = params_distances_mirror;
                        params_values_y = params_values_mirror;
                        this.quad_prog.bool_channel_fault.y23 = false;
                    end
                    this.quad_prog.Fw2 = griddedInterpolant(params_distances_y , params_values_y, 'nearest','nearest');
                end
                %channel - z
                if ismember(this.faulty_thruster_index, [4 5 10 11])
                    if ismember(this.faulty_thruster_index, [4 5])
                        params_distances_z = params_distances;
                        params_values_z = params_values;
                        this.quad_prog.bool_channel_fault.z45 = true;
                    elseif ismember(this.faulty_thruster_index, [10 11])
                        params_distances_z = params_distances_mirror;
                        params_values_z = params_values_mirror;
                        this.quad_prog.bool_channel_fault.z45 = false;
                    end
                    this.quad_prog.Fw3 = griddedInterpolant(params_distances_z , params_values_z, 'nearest','nearest');
                end
                 
                
%                 % used to be
%                 this.active_set.Weighting_Matrix = simopts.active_set.Weighting_Matrix;
%                 this.active_set.Weighting_Matrix_default = simopts.active_set.Weighting_Matrix;
            end
            
            if(strcmp(this.mode,'normal') == 1)
                this.faulty_thruster_index = [];
            end
            
            this.Mass = 4.16;  %change to constant parameter
%             inertia(1,1) =  0.02836 + 0.00016;
%             inertia(2,1) =  0.026817 + 0.00150;
%             inertia(3,1) =  0.023 + 0.00150;
%             inertia(4,1) = -0.0000837;
%             inertia(5,1) =  0.000014;
%             inertia(6,1) = -0.00029;
            inertia(1,1) =  2.30E-2;  %kg m^2
            inertia(2,1) =  2.42E-2;
            inertia(3,1) =  2.14E-2;
            inertia(4,1) =  9.90E-05;
            inertia(5,1) = -2.95E-04;
            inertia(6,1) = -2.54E-05;
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
            this.thruster_min_ontime = 0.005;

            try
                simopts.PWPF1; %if doesn't exist
            catch
                simopts.PWPF1 = simopts.PWPF;
                simopts.PWPF2 = simopts.PWPF;
                simopts.PWPF3 = simopts.PWPF;
                simopts.PWPF4 = simopts.PWPF;
                simopts.PWPF5 = simopts.PWPF;
                simopts.PWPF6 = simopts.PWPF;
                simopts.schmitt1 = simopts.schmitt;
                simopts.schmitt2 = simopts.schmitt;
                simopts.schmitt3 = simopts.schmitt;
                simopts.schmitt4 = simopts.schmitt;
                simopts.schmitt5 = simopts.schmitt;
                simopts.schmitt6 = simopts.schmitt;
            end
            
            
            %6x schmitt trigger objects
            this.schmitt_trigger1 = Schmitt_trigger_c(...
                simopts.schmitt1.Uout,simopts.schmitt1.Uon,simopts.schmitt1.Uoff);
            this.schmitt_trigger2 = Schmitt_trigger_c(...
                simopts.schmitt2.Uout,simopts.schmitt2.Uon,simopts.schmitt2.Uoff);
            this.schmitt_trigger3 = Schmitt_trigger_c(...
                simopts.schmitt3.Uout,simopts.schmitt3.Uon,simopts.schmitt3.Uoff);
            this.schmitt_trigger4 = Schmitt_trigger_c(...
                simopts.schmitt4.Uout,simopts.schmitt4.Uon,simopts.schmitt4.Uoff);
            this.schmitt_trigger5 = Schmitt_trigger_c(...
                simopts.schmitt5.Uout,simopts.schmitt5.Uon,simopts.schmitt5.Uoff);
            this.schmitt_trigger6 = Schmitt_trigger_c(...
                simopts.schmitt6.Uout,simopts.schmitt6.Uon,simopts.schmitt6.Uoff);
           
            %6x PWPF objects
            this.PWPF1 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF1.Km, simopts.PWPF1.Tm, simopts.PWPF1.h, ...
                simopts.schmitt1.Uout, simopts.schmitt1.Uon, simopts.schmitt1.Uoff, simopts.PWPF1.H_feed);
            this.PWPF2 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF2.Km, simopts.PWPF2.Tm, simopts.PWPF2.h, ...
                simopts.schmitt2.Uout, simopts.schmitt2.Uon, simopts.schmitt2.Uoff, simopts.PWPF2.H_feed);
            this.PWPF3 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF3.Km, simopts.PWPF3.Tm, simopts.PWPF3.h, ...
                simopts.schmitt3.Uout, simopts.schmitt3.Uon, simopts.schmitt3.Uoff, simopts.PWPF3.H_feed);
            this.PWPF4 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF4.Km, simopts.PWPF4.Tm, simopts.PWPF4.h, ...
                simopts.schmitt4.Uout, simopts.schmitt4.Uon, simopts.schmitt4.Uoff, simopts.PWPF4.H_feed);
            this.PWPF5 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF5.Km, simopts.PWPF5.Tm, simopts.PWPF5.h, ...
                simopts.schmitt5.Uout, simopts.schmitt5.Uon, simopts.schmitt5.Uoff, simopts.PWPF5.H_feed);
            this.PWPF6 = PWPF_c(...  Km,Tm,h,Uout,Uon,Uoff)
                simopts.PWPF6.Km, simopts.PWPF6.Tm, simopts.PWPF6.h, ...
                simopts.schmitt6.Uout, simopts.schmitt6.Uon, simopts.schmitt6.Uoff, simopts.PWPF6.H_feed);
            
            % control allocation function to thrusters, from all combinations of Forces
            % and Moments that can be generated with the operating thrusters
            id_fault_channel_1 = this.faulty_thruster_index(...
                ismember(this.faulty_thruster_index, [0 1 6 7]));
            id_fault_channel_2 = this.faulty_thruster_index(...
                ismember(this.faulty_thruster_index, [2 3 8 9])) - 2;
            id_fault_channel_3 = this.faulty_thruster_index(...
                ismember(this.faulty_thruster_index, [4 5 10 11])) - 4;
            
            % variables to determine if a channel is faulty, these are used
            % in thruster allocation section where weighting matrices are
            % changed when angles exceed a threshold (adaptive)
            if(isempty(id_fault_channel_1))
                this.bool_channel_fault.x = false; %keep in mind, channel x provides moments in y direction
            else
                this.bool_channel_fault.x = true;
            end
            if(isempty(id_fault_channel_2))
                this.bool_channel_fault.y = false;% moments in z direction
            else
                this.bool_channel_fault.y = true;
            end
            if(isempty(id_fault_channel_3))
                this.bool_channel_fault.z = false;% moments in x direction
            else
                this.bool_channel_fault.z = true;
            end
            this.quad_prog.bool_angleoverflow.x = 0;
            this.quad_prog.bool_angleoverflow.y = 0;
            this.quad_prog.bool_angleoverflow.z = 0;
            
            
            if( strcmpi(this.thruster_allocation_mode, 'quadratic programming pulse modulation') ||...
                    strcmpi(this.thruster_allocation_mode, 'quadratic programming pulse modulation-adaptive'))
                v_ = (0:this.h:0.2) * 5;
                this.thrust_combs.f0167 = all_feasible_thruster_u_QP(id_fault_channel_1,v_).*this.Thruster_max_F/5;
                this.thrust_combs.f2389 = all_feasible_thruster_u_QP(id_fault_channel_2,v_).*this.Thruster_max_F/5;
                this.thrust_combs.f451011 = all_feasible_thruster_u_QP(id_fault_channel_3,v_).*this.Thruster_max_F/5;
            else
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
%             % see the search space (F,M combinations)
%             Fs = sum(this.thrust_combs.f0167 .* [1;1;-1;-1],1);
%             Ms = sum(this.thrust_combs.f0167 .* [1;-1;-1;1],1);
%             Fs2 = sum(this.thrust_combs.f2389 .* [1;1;-1;-1],1);
%             Ms2 = sum(this.thrust_combs.f2389 .* [1;-1;-1;1],1);
%             plot(Fs,Ms,'r.'),hold on
%             plot(Fs2,Ms2,'bx')
            
        end
        
        
        function f = get_thruster_on_off_optimal(obj,M_req, a_Body_req, X_stage, sim_time)
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
                case 'pseudoinverse pulse modulation'
%                     if(sim_time > 15.995) keyboard, end
                   status = mod(sim_time,1); 
                    if (status < 0.199)
                        if status < 1e-7 % == 0
                            % program up to 200ms of thruster firings to be
                            % executed
                            f_ = zeros(12,1);
                            for i=1:6
                                if(f_pairs_req(i) < 0)
                                    f_(i+6) = -f_pairs_req(i);
                                else
                                    f_(i) = f_pairs_req(i);
                                end
                            end
                    %saturate
                           f_(f_ > obj.Thruster_max_F/5) = obj.Thruster_max_F/5;
  
                            if(strcmp(obj.mode,'fault') == 1) %control in fault mode, one thruster (f#0) off
                                f_(obj.faulty_thruster_index +1) = 0;
                            end
                            %
                            N_ = 0.2/obj.h; % ;
                            obj.thr_program_counter = 0;
                            obj.thr_program = zeros(12,N_);
                            % get thrusters on-time by calculating impulse
                            t_on = f_/(obj.Thruster_max_F); 
                             %convert on-time to N thruster firings, to be
                             %programmed for the next 200ms 
                            thr_N_fires = round(t_on/obj.h);
                            for d = 1:12
                                obj.thr_program(d, 1:thr_N_fires(d)) = 1;
                            end
                        end
                        
                        obj.thr_program_counter = obj.thr_program_counter + 1;
                        f = obj.thr_program(:,obj.thr_program_counter)*obj.Thruster_max_F;
                    else
                       % SPHERES software spends this time updating its beacons
                       f = zeros(12,1);
                    end
                    
                case 'quadratic programming pulse modulation'
                    status = mod(sim_time,1);
                    if (status < 0.199)
                        if status < 1e-7 % == 0
                            % program up to 200ms of thruster firings to be
                            % executed
                            [f0,f1,f6,f7] = QuadProg_allocation(obj, B(1), B(5), ...
                                obj.thrust_combs.f0167, obj.quad_prog.Weighting_Matrix_1);
                            [f2,f3,f8,f9] = QuadProg_allocation(obj, B(2), B(6), ...
                                obj.thrust_combs.f2389, obj.quad_prog.Weighting_Matrix_2);
                            [f4,f5,f10,f11] = QuadProg_allocation(obj, B(3), B(4), ...
                                obj.thrust_combs.f451011, obj.quad_prog.Weighting_Matrix_3);
                            
                            f_t = [f0;f1;f2;f3;f4;f5;f6;f7;f8;f9;f10;f11];
                            %
                            N_ = 0.2/obj.h; % ;
                            obj.thr_program_counter = 0;
                            obj.thr_program = zeros(12,N_);
                            % get thrusters on-time by calculating impulse
                            t_on = f_t/(obj.Thruster_max_F);
                            %convert on-time to N thruster firings, to be
                            %programmed for the next 200ms
                            thr_N_fires = round(t_on/obj.h);
                            for d = 1:12
                                obj.thr_program(d, 1:thr_N_fires(d)) = 1;
                            end
                        end
                        
                        obj.thr_program_counter = obj.thr_program_counter + 1;
                        f = obj.thr_program(:,obj.thr_program_counter)*obj.Thruster_max_F;
                    else
                        % SPHERES software spends this time updating its beacons
                        f = zeros(12,1);
                    end
                case 'quadratic programming pulse modulation-adaptive'
                    status = mod(sim_time,1);
                    if (status < 0.199)
                        if status < 1e-7 % == 0
                            % program up to 200ms of thruster firings to be
                            % executed
                            
                            % adaptive: change quadratic programming Weights if theta
                            %exceeds a threshold. this operation is done
                            %only for channels that have one or more
                            %thrusters failed:
                            % channel - x (Fx & My)
                            theta2 = 2*asin(X_stage(8))*180/pi;
                            if(obj.bool_channel_fault.x)%if channel has failed thrusters:
                                if( abs(theta2) > obj.quad_prog.angle_up_bound)%Moment Control prioritized
                                    obj.quad_prog.bool_angleoverflow.x = 1;
                                else
                                    if(abs(theta2) < obj.quad_prog.angle_low_bound)%back to normal
                                        obj.quad_prog.bool_angleoverflow.x = 0;
                                    end
                                end
                                
                                if (obj.quad_prog.bool_channel_fault.x01)
                                    if(X_stage(1) < -obj.quad_prog.threshold_positive)
                                        obj.quad_prog.Weighting_Matrix_1_scheduled =  obj.quad_prog.Weighting_Matrix_F_default;
                                    elseif (X_stage(1) > 0 )
                                        obj.quad_prog.Weighting_Matrix_1_scheduled(4) =  obj.quad_prog.Fw1(X_stage(1));
                                    end
                                else
                                    if(X_stage(1) > obj.quad_prog.threshold_positive)
                                        obj.quad_prog.Weighting_Matrix_1_scheduled =  obj.quad_prog.Weighting_Matrix_F_default;
                                    elseif (X_stage(1) < 0 )
                                        obj.quad_prog.Weighting_Matrix_1_scheduled(4) =  obj.quad_prog.Fw1(X_stage(1));
                                    end
                                end
                                
                                if(obj.quad_prog.bool_angleoverflow.x)% (higher priority) moment control if angle
                                % overflows
%                                     obj.quad_prog.Weighting_Matrix_1(1) = 0;
                                    obj.quad_prog.Weighting_Matrix_1(4) = 100; %10 is ok
                                else
                                    obj.quad_prog.Weighting_Matrix_1 = obj.quad_prog.Weighting_Matrix_1_scheduled;
                                end
                            end
                            
                            % channel - y (Fy & Mz)
                            theta3 = 2*asin(X_stage(9))*180/pi;
                            if(obj.bool_channel_fault.y)%if channel has failed thrusters:
                                if( abs(theta3) > obj.quad_prog.angle_up_bound)%Moment Control prioritized
                                    obj.quad_prog.bool_angleoverflow.y = 1;
                                else
                                    if(abs(theta3) < obj.quad_prog.angle_low_bound)%back to normal
                                        obj.quad_prog.bool_angleoverflow.y = 0;
                                    end
                                end
                                
                                 if (obj.quad_prog.bool_channel_fault.y23)
                                        if(X_stage(2) < -obj.quad_prog.threshold_positive)
                                            obj.quad_prog.Weighting_Matrix_2_scheduled =  obj.quad_prog.Weighting_Matrix_F_default;
                                        elseif (X_stage(2) > 0 )
                                            obj.quad_prog.Weighting_Matrix_2_scheduled(4) =  obj.quad_prog.Fw2(X_stage(2));
                                        end
                                    else
                                        if(X_stage(2) > obj.quad_prog.threshold_positive)
                                            obj.quad_prog.Weighting_Matrix_2 =  obj.quad_prog.Weighting_Matrix_F_default;
                                        elseif (X_stage(2) < 0 )
                                            obj.quad_prog.Weighting_Matrix_2(4) =  obj.quad_prog.Fw2(X_stage(2));
                                        end
                                 end
                                    
                                if(obj.quad_prog.bool_angleoverflow.y)
                                    obj.quad_prog.Weighting_Matrix_2(4) = 100; %10 is ok
                                else
                                    obj.quad_prog.Weighting_Matrix_2 = obj.quad_prog.Weighting_Matrix_2_scheduled;
                                end
                            end
                           
                            
                            % channel - z (Fz & Mx)
                            theta1 = 2*asin(X_stage(7))*180/pi;
                            if(obj.bool_channel_fault.z)%if channel has failed thrusters:
                                if( abs(theta1) > obj.quad_prog.angle_up_bound)%Moment Control prioritized
                                    obj.quad_prog.bool_angleoverflow.z = 1;
                                else
                                    if(abs(theta1) < obj.quad_prog.angle_low_bound)%back to normal
                                        obj.quad_prog.bool_angleoverflow.z = 0;
                                    end
                                end
                                
                                if (obj.quad_prog.bool_channel_fault.z45)
                                    if(X_stage(3) < -obj.quad_prog.threshold_positive)
                                        obj.quad_prog.Weighting_Matrix_3_scheduled =  obj.quad_prog.Weighting_Matrix_F_default;
                                    elseif (X_stage(3) > 0 )
                                        obj.quad_prog.Weighting_Matrix_3_scheduled(4) =  obj.quad_prog.Fw3(X_stage(3));
                                    end
                                else
                                    if(X_stage(3) > obj.quad_prog.threshold_positive)
                                        obj.quad_prog.Weighting_Matrix_3_scheduled =  obj.quad_prog.Weighting_Matrix_F_default;
                                    elseif (X_stage(3) < 0 )
                                        obj.quad_prog.Weighting_Matrix_3_scheduled(4) =  obj.quad_prog.Fw3(X_stage(3));
                                    end
                                end
                                
                                if(obj.quad_prog.bool_angleoverflow.z)
                                    obj.quad_prog.Weighting_Matrix_3(4) = 100; %10 is ok
                                else
                                    obj.quad_prog.Weighting_Matrix_3 = obj.quad_prog.Weighting_Matrix_3_scheduled;
                                end
                            end
                           
                            % thruster firings allocation using the Quadratic Programming method
                            [f0,f1,f6,f7] = QuadProg_allocation(obj, B(1), B(5), ...
                                obj.thrust_combs.f0167, obj.quad_prog.Weighting_Matrix_1);
                            [f2,f3,f8,f9] = QuadProg_allocation(obj, B(2), B(6), ...
                                obj.thrust_combs.f2389, obj.quad_prog.Weighting_Matrix_2);
                            [f4,f5,f10,f11] = QuadProg_allocation(obj, B(3), B(4), ...
                                obj.thrust_combs.f451011, obj.quad_prog.Weighting_Matrix_3);
                            
                            f_t = [f0;f1;f2;f3;f4;f5;f6;f7;f8;f9;f10;f11];
                            %
                            N_ = 0.2/obj.h; % ;
                            obj.thr_program_counter = 0;
                            obj.thr_program = zeros(12,N_);
                            % get thrusters on-time by calculating impulse
                            t_on = f_t/(obj.Thruster_max_F);
                            %convert on-time to N thruster firings, to be
                            %programmed for the next 200ms
                            thr_N_fires = round(t_on/obj.h);
                            for d = 1:12
                                obj.thr_program(d, 1:thr_N_fires(d)) = 1;
                            end
                        end
                        
                        obj.thr_program_counter = obj.thr_program_counter + 1;
                        f = obj.thr_program(:,obj.thr_program_counter)*obj.Thruster_max_F;
                    else
                        % SPHERES software spends this time updating its beacons
                        f = zeros(12,1);
                    end
                    
                    
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
                    
                    if(strcmp(obj.mode,'fault') == 1) %control in fault mode, indicated thrusters become off
                        f(obj.faulty_thruster_index +1) = 0;
                    end
                    
                case 'hybrid_2p-x' 
                    %channel x uses 2 PWPF modulators, 
                    %channels y & z are controlled with Quadratic Programming
                    
                    % PWPF Modulators
                    f_pairs1 = obj.PWPF1.signal_update(f_pairs_req(1));
                    f_pairs2 = obj.PWPF2.signal_update(f_pairs_req(2));
                    if(f_pairs1 < 0)
                        f0 = 0;
                        f6 = -f_pairs1;
                    else
                        f0 = f_pairs1;
                        f6 = 0;
                    end
                    if(f_pairs2 < 0)
                        f1 = 0;
                        f7 = -f_pairs2;
                    else
                        f1 = f_pairs2;
                        f7 = 0;
                    end
                    
                    % Quadratic Programming
                    [f2,f3,f8,f9] = asd_allocation_logic(obj, B(2), B(6), ...
                        obj.thrust_combs.f2_comb,obj.thrust_combs.f3_comb,...
                        obj.thrust_combs.f8_comb,obj.thrust_combs.f9_comb);
                    [f4,f5,f10,f11] = asd_allocation_logic(obj, B(3), B(4), ...
                        obj.thrust_combs.f4_comb,obj.thrust_combs.f5_comb,...
                        obj.thrust_combs.f10_comb,obj.thrust_combs.f11_comb);
                    f = [f0;f1;f2;f3;f4;f5;f6;f7;f8;f9;f10;f11];

                    if(strcmp(obj.mode,'fault') == 1) %control in fault mode, indicated thrusters become off
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
                    
                    if(strcmp(obj.mode,'fault') == 1) %control in fault mode
                        f(obj.faulty_thruster_index +1) = 0;
                    end
                    
                case 'saturate-only'
                     f_pairs = f_pairs_req; 
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
                    %saturate
                    f(f > obj.Thruster_max_F) = obj.Thruster_max_F;
  
                    %
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
            Theta_2_history= zeros(N_total_sim, 1);
            T_ode45 = tspan(1:end-1)';
            X_ode45 = zeros(N_total_sim, 13);
            F_Th_Opt = zeros(N_total_sim, 12);
            Force_Moment_log = zeros(N_total_sim, 6);
            Force_Moment_log_req = zeros(N_total_sim, 6);
            X_ode45(1,:) = X0;
            % initial angle and rorational speed feed for controllers
            Theta_ = 2*asin(X_ode45(1,7:9)); 
            Theta_dot = X_ode45(1,11:13); 
            % Calculate the target initial state vector
            [R0,V0] = get_target_R0V0();
%             tic
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
                % respect to RSW frame
                [U_M_req, a_req] = Opt_Force_Moments(obj,X_stage,Theta_,Theta_dot);
                %rotation matrices
                rotM_RSW2ECI = RSW2ECI(obj, R0, V0);
                rotM_ECI2body = ECI2body(obj,q_stage);
                % required directional accelerations in Body frame of reference
                a_Body_req = rotM_ECI2body*rotM_RSW2ECI*  a_req;
                M_Body_req = U_M_req;
                % debugging block of code for a certain time
%                 if(tspan(k_stage) == 60.615) keyboard; end
                %
                f_thruster = get_thruster_on_off_optimal(obj, M_Body_req, a_Body_req, X_stage, tspan(k_stage));
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
                %                 X_temp = ode_1(obj, X_stage);
                %                 [~,X_temp] = ode23(@ode_eq,[tspan(k_stage), tspan(k_stage+1)], X_stage);
                %                 [~,X_temp] = ode45(@ode_eq,[tspan(k_stage), tspan(k_stage+1)], X_stage);
                %update state vector
                X_ode45(k_stage+1,:) = X_temp(end,:);
                %update rotational speed feedbacks for controllers (thetadots)
                temp_theta_new = 2*asin(X_ode45(k_stage+1,7:9));
%                 Theta_dot = (temp_theta_new - Theta_)/obj.h; %rad/s
                Theta_dot = X_ode45(k_stage+1,11:13); %rad/s
                Theta_ = temp_theta_new;
%% new test code block
%                 t2_from_trapz_w2 = cumtrapz(T_ode45,X_ode45(:,12));
%                 if (abs (t2_from_trapz_w2(k_stage)) > pi)
%                     %Theta_(2) = Theta_(2)/abs(Theta_(2))*(2*pi - Theta_(2));
%                     Theta_(2) = t2_from_trapz_w2(k_stage);
%                 end
%                 Theta_2_history(k_stage) = Theta_(2);
% %                     Theta_(2) = t2_from_trapz_w2(k_stage);
% %                 if(abs(t2_from_trapz_w2(k_stage)) >= pi)
% %                     Theta_(2) = 360 - Theta_(2);
%                 end
%                 if(t2_from_trapz_w2 < 150)
%                     Theta_(2) = 360 - Theta_(2);
%                 end
                %% 
            end
            %  toc
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
            try
                obj.F_controller_x = griddedInterpolant(controller.Fx_gI,...
                    single(controller.v_Fthruster_x(controller.Fx_U_Optimal_id)), obj.controller_InterpmodeF,'nearest');
                obj.F_controller_y = griddedInterpolant(controller.Fy_gI,...
                    single(controller.v_Fthruster_y(controller.Fy_U_Optimal_id)), obj.controller_InterpmodeF,'nearest');
                obj.F_controller_z = griddedInterpolant(controller.Fz_gI,...
                    single(controller.v_Fthruster_z(controller.Fz_U_Optimal_id)), obj.controller_InterpmodeF,'nearest');
                
            catch
                warning('one controller defined for all channels, do not use for fault case...\n')
            F_controller = griddedInterpolant(controller.F_gI,...
                single(controller.v_Fthruster(controller.F_U_Optimal_id)), obj.controller_InterpmodeF,'nearest');
            
            obj.F_controller_x = F_controller;
            obj.F_controller_y = F_controller;
            obj.F_controller_z = F_controller;
            end
            
            
            obj.M_controller_J1 = griddedInterpolant(controller.M_gI_J1,...
                single(controller.v_Mthruster_x(controller.M_U_Optimal_id_J1)), obj.controller_InterpmodeM,'nearest');
            obj.M_controller_J2 = griddedInterpolant(controller.M_gI_J2,...
                single(controller.v_Mthruster_y(controller.M_U_Optimal_id_J2)), obj.controller_InterpmodeM,'nearest');
            obj.M_controller_J3 = griddedInterpolant(controller.M_gI_J3,...
                single(controller.v_Mthruster_z(controller.M_U_Optimal_id_J3)), obj.controller_InterpmodeM,'nearest');
            
            obj.controller_params.controller = controller.controller;
        end
        
        function [M, a] = Opt_Force_Moments(obj, Xin, theta_, t_dot)
            if(~strcmp(obj.controller_params.type,'PID'))
                % Dynamic Programming Controller
                % Forces (expressed in body frame of reference)
                a = zeros(3,1);
                a(1) = (obj.F_controller_x( Xin(1), Xin(4))/obj.Mass); % X(1:3) represent position and X(4:6) velocities
                a(2) = (obj.F_controller_y( Xin(2), Xin(5))/obj.Mass); %
                a(3) = (obj.F_controller_z( Xin(3), Xin(6))/obj.Mass); %
                
                %Moments
                M = zeros(3,1);
%                 M(1) = obj.M_controller_J1( 2*asin(Xin(7)) , Xin(11) ); %where X(11:13) represent rotational speeds
%                 M(2) = obj.M_controller_J2( 2*asin(Xin(8)) , Xin(12) ); %where X(11:13) represent rotational speeds
%                 M(3) = obj.M_controller_J3( 2*asin(Xin(9)) , Xin(13) ); %where X(11:13) represent rotational speeds
                %% instead of using w1,w2,w3, we feed theta dot as rotational speed to the controllers
                M(1) = obj.M_controller_J1( theta_(1) , t_dot(1) ); %where X(11:13) represent rotational speeds
                M(2) = obj.M_controller_J2( theta_(2) , t_dot(2) ); %where X(11:13) represent rotational speeds
                M(3) = obj.M_controller_J3( theta_(3) , t_dot(3) ); %where X(11:13) represent rotational speeds                
                
                
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
        
        function [f0,f1,f6,f7] = QuadProg_allocation(~, Freq, Mreq, f_comb, Weighting_Matrix)
        eF = sum(f_comb .* [1;1;-1;-1],1) - Freq; %repmat is implied (5-6x faster) as of MATLAB R2016b
        eM = sum(f_comb .* [1;-1;-1;1],1) - Mreq;
        all_evals = (eF.*eF) * Weighting_Matrix(1) + ...
            (eM.*eM) * Weighting_Matrix(4);
        
        [best_evaluation, best_evaluation_id] = min(all_evals, [], 2); %#ok<ASGLU>
        f0 = f_comb(1,best_evaluation_id);
        f1 = f_comb(2,best_evaluation_id);
        f6 = f_comb(3,best_evaluation_id);
        f7 = f_comb(4,best_evaluation_id);
        end
        
        
        function [f0,f1,f6,f7] = asd_allocation_logic(obj, Freq, Mreq, ...
                f0_comb,f1_comb,f6_comb,f7_comb)
            
            L = length(f0_comb);
            B = [1,1,-1,-1;...
                1,-1,-1,1];
            all_evaluations = zeros(1,L);
%             W = obj.active_set.Weighting_Matrix([1;4]);
            
            for ii=1:L
                u = [f0_comb(ii);f1_comb(ii);f6_comb(ii);f7_comb(ii)] * obj.Thruster_max_F;
                e = (B*u - [Freq; Mreq]);
                all_evaluations(ii) = e' * obj.active_set.Weighting_Matrix * e;
%                 all_evaluations(1,ii) = sum((B*[f0_comb(ii);f1_comb(ii);f6_comb(ii);f7_comb(ii)] * obj.Thruster_max_F ...
%                     - [Freq; Mreq]).^2 .* W);
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
        
          function X2 = ode_1(obj, x)
            % Taylor
            % h = dt;
            h_t = obj.h;
            x = x';
            k1 = system_dynamics(obj, x);
            X2 = x + h_t*k1;
            X2 = X2';
          end
          
          function plot_xy_plane(obj)
              n_sampling = 30;
              a = 0.3;
              sep = 1.2*a; %seperation distance for sampling points
              col_g = 0.1;
              col_gh = 0.05; % bold heading
              head_LineWidth = 1.3;
             x = obj.history.X_ode45(:,1);
             y = obj.history.X_ode45(:,2);
             [~,theta2q2a,~] = quat2angle(obj.history.X_ode45(:,10:-1:7));
             theta2_c = 2*asin(obj.history.X_ode45(:,8));
%              theta2q2a = theta2q2a *180/pi;
             
             % sampling points:
             % method 1:
             i_sampling = round(linspace(1,length(x), n_sampling));
             % method 2: 
             i_sampling = 1;
             x0 = x(1);
             %p0 = x(1)^2 + y(1)^2;
             for counter = 1:length(x)
                 if(abs(x(counter) - x0) > sep)
                     %                  if(x(counter)^2 + y(counter)^2 - p0 > sep)
                     i_sampling(end+1) = counter;
                     x0 = x(counter);
                     %                      p0 = x(counter)^2 + y(counter)^2;
                     
                 end
             end
             
             [b,ixorigin] = min(x.^2 + y.^2);
             i_sampling(end) = ixorigin;
%              if(i_sampling(end) ~= counter)
%                  i_sampling(end + 1) = counter;
%              end
             
             x_samples = x(i_sampling);
             y_samples = y(i_sampling);
             t_samples = theta2q2a(i_sampling);
%              t_samples = theta2_c(i_sampling);
            
             
             
             Px_a = [a,a,-a,-a];
             Py_a = [a,-a,-a,a];
             figure
             %plot path
             plot(x,y,'-','Color', [col_g col_g col_g])
             hold on
             % drawing squares
             for ii= 1:length(i_sampling)
                  x_s = x_samples(ii);
                 y_s = y_samples(ii);
                 t2_s = t_samples(ii);
                  Px = x_s + cos(t2_s).*Px_a -sin(t2_s).*Py_a;
                 Py = y_s + sin(t2_s).*Px_a+cos(t2_s).*Py_a;
                 %center of mass
                 plot(x_s,y_s,'o','Color', [col_g col_g col_g])
                 %three grey body lines
                plot(Px([2,3,4,1]),Py([2,3,4,1]),'Color', [col_g col_g col_g])
               
                %heading line
                plot(Px([1,2]), Py([1,2]),'Color', [col_gh col_gh col_gh], 'LineWidth', head_LineWidth)
             end
             axis equal
             grid on
             xlabel('x')
             ylabel('y')
             q = gca;
             q.GridLineStyle = '--';
             q.GridAlpha = 0.3;
             
             %draw arrows
             x1 = x_samples(1);
             x2 = x_samples(2);
             
             y1 = y_samples(1);
             y2 = y_samples(2);
%              [figx figy] = dsxy2figxy([x1 y1],[x2 y2]) ;
%              arrowObj = annotation('arrow', [0.1 0.1], [0.5 0.5]);
%              set(arrowObj, 'Units', 'centimeters');
%              set(arrowObj, 'Position', [x1 y1 x2 y2]);
%              annotation(gcf,'arrow', figx,figy)

             drawArrow = @(x,y, varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1), varargin{:});    
             hq = drawArrow([x1,x2],...
                 [y1,y2], 'Color', [col_g col_g col_g], 'MaxHeadSize', 15);
%              U = hq.UData;
%              V = hq.VData;
%              X = hq.XData;
%              Y = hq.YData;
%              LineLength = 0.08;
%              headLength = 8;

%              for ii = 1:length(X)
%                  for ij = 1:length(X)
%                      
%                      headWidth = 5;
%                      ah = annotation('arrow',...
%                          'headStyle','cback1','HeadLength',headLength,'HeadWidth',headWidth);
%                      set(ah,'parent',gca);
%                      set(ah,'position',[X(ii,ij) Y(ii,ij) LineLength*U(ii,ij) LineLength*V(ii,ij)]);
%                      
%                  end
%              end
             
             % or use annotation arrows
%              
%              annotation(gcf,'arrow', xf,yf)
             %              headWidth = 5;
%              ah = annotation('arrow',...
%                  'headStyle','cback1','HeadLength',headLength,'HeadWidth',headWidth);
%              set(ah,'parent',gca);
%              set(ah,'position',[x_arrow y_arrow LineLength*U(ii,ij) LineLength*V(ii,ij)]);

        
             
%              for ii= 1:n_sampling
%                  x_s = x_samples(ii);
%                  y_s = y_samples(ii);
%                  t2_s = theta2q2a_s(ii);
%              polyin = polyshape(Px,Py);
%              poly2(ii) = rotate(polyin,t2_s,[x_s y_s]);
%              end
%              plot(poly2)
%              axis equal
             
          end
          
    end
    
end



