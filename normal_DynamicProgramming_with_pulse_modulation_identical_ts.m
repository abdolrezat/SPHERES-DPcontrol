function S = normal_DynamicProgramming_with_pulse_modulation_identical_ts(varargin)

%% generate controller
% if nargin == 0
    controller.name = 'controller_normal_DynamicProgramming_c'; %name of controller, will be saved under /controller directory
    Thruster_max_F = 0.12/5; % (N)
    Thruster_dist = (9.65E-2); % (meters)
    Thruster_max_M = Thruster_max_F*Thruster_dist;
    controller.lim_Fx = [-2*Thruster_max_F 2*Thruster_max_F];
    controller.lim_My = [-2*Thruster_max_M 2*Thruster_max_M];
    controller.lim_Fy = [-2*Thruster_max_F 2*Thruster_max_F];
    controller.lim_Mz = [-2*Thruster_max_M 2*Thruster_max_M];
    controller.lim_Fz = [-2*Thruster_max_F 2*Thruster_max_F];
    controller.lim_Mx = [-2*Thruster_max_M 2*Thruster_max_M];
    controller.n_mesh_Fx = 41;
    controller.n_mesh_My = 41;
    controller.n_mesh_Fy = 41;
    controller.n_mesh_Mz = 41;
    controller.n_mesh_Fz = 41;
    controller.n_mesh_Mx = 41;
    
    controller.Qx1 = 3e-6;
    controller.Qv1Qx1_ratio = 700; 
    controller.Qx2 = 3e-6;
    controller.Qv2Qx2_ratio = 700; 
    controller.Qx3 = 3e-6;
    controller.Qv3Qx3_ratio = 700; 
    
    controller.Qt = 0.1;
    controller.QwQt_ratio = 5;
% end
%     
    if(1)
        generate_Dynamic_Programming_controller(controller)
        test_surface(controller.name)
    end
    simulator_opts.current_controller = controller.name;
simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
  simulator_opts.thruster_allocation_mode = 'quadratic programming pulse modulation';

simulator_opts.Thruster_max_F = 0.12; % (N)
simulator_opts.h = 0.01; %simulation fixed time steps
  simulator_opts.T_final = 300; %simulation Tfinal

    dr0 = [-10 10 10]; %initial relative position offset
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(30),deg2rad(30),deg2rad(-30))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    S = simulation_normal_DP(simulator_opts);
    S.plot_optimal_path
    S.plot_xy_plane
end



function generate_Dynamic_Programming_controller(controller)%GENERATE_CONTROLLER is a sample of code for generating a controller
%   This generates 4 controllers (one for Transitional and three for Rotational motion)
%   using a vectorized Dynamic Programming algorithm, Using the m-files located
%   in the corresponding folder
%% generate controller
    %controller variables
    controller.Tf = 500; % Tfinal for Force controller run
    controller.Tm = 100; % Tfinal for Moment controller run
    %time variables
    controller.h = 1; % time step for controllers
    %Optimal Control constants
    controller.Qv1 = controller.Qv1Qx1_ratio * controller.Qx1;
    controller.Qv2 = controller.Qv2Qx2_ratio * controller.Qx2;
    controller.Qv3 = controller.Qv3Qx3_ratio * controller.Qx3;

    controller.Qw = controller.QwQt_ratio * controller.Qt;
    controller.R =  1.0;

    % mesh generation , limits
    controller.lim_x = [-40 40]; %in m
    controller.lim_v = [-1.5 1.5]; %in m/s
    controller.lim_t = deg2rad([-60 60]); %in degrees, converts to rad
    controller.lim_w = deg2rad([-50 50]); %in rad/s% mesh generation , Force and Moments

    % mesh generation , mesh resolutions
    controller.n_mesh_x = 502;
    controller.n_mesh_v = 500;
    controller.n_mesh_t = 502;
    controller.n_mesh_w = 500;

% generate the controller
generate_DP_ForceMoment_controller(controller)
end

function SC = simulation_normal_DP(simulator_opts)

Thruster_dist = (9.65E-2); % (meters)
%simulator variables
simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative

simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
simulator_opts.Thruster_dist = Thruster_dist;

% create and run the simulator
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path;
end




function plot12(S1,S2)

f1 = figure('Name','states - position',...
    'Position',[543.4000   49.0000  518.4000  326.4000],...
    'color', 'white');
T1 = S1.history.T_ode45;
T2 = S2.history.T_ode45;
% title('states - position (m)')
plot(T1, S1.history.X_ode45(:,1), '--')
hold on
plot(T2, S2.history.X_ode45(:,1))
legend('x1','x2')
        title('states - position (m)')

f2 = figure('Name','states - w',...
    'Position',[956.2000   47.4000  518.4000  326.4000],...
    'color', 'white');
plot(T1, S1.history.X_ode45(:,12)*180/pi)
title('states - rotational speeds (deg/sec)')

hold on
plot(T2, S2.history.X_ode45(:,12)*180/pi)
legend('w1','w2')

end
