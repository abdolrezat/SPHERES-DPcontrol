%TEST_RUN runs a sample of the code in this repository
%   First it generates a controller using a vectorized Dynamic Programming
%   algorithm and then it launches the simulator that uses this controller 
addpath(path,genpath('generate_controller'))
addpath(path,'simulator')

Thruster_max_F = 0.13; % (N)
Thruster_dist = (9.65E-2); % (meters)
%% generate controller
%controller variables
    controller.name = 'controller_attpositionf9'; %name of controller, will be saved under /controller directory
    controller.Tf = 4.5; % Tfinal for Force controller run
    controller.Tm = 2; % Tfinal for Moment controller run
    %time variables
    controller.h = 0.005; % time step for controllers
    %Optimal Control constants
    controller.Qx = 1.0;
    controller.Qv = 1.0*45;
    controller.Qt = 0.01;
    controller.QwQt_ratio = 1;
    controller.Qw = controller.QwQt_ratio * controller.Qt;
    controller.R =  1.0;
    % mesh generation , limits
    controller.lim_x = [-60 60]; %in m
    controller.lim_v = [-2.2 2.2]; %in m/s
    controller.lim_t = deg2rad([-180 180]); %in degrees, converts to rad
    controller.lim_w = deg2rad([-200 200]); %in rad/s
    % mesh generation , Force and Moments
    Thruster_max_M = Thruster_max_F*Thruster_dist;
    controller.lim_F = 2*[-Thruster_max_F Thruster_max_F];
    controller.lim_M = 2*[-Thruster_max_M Thruster_max_M];
    
    % mesh generation , mesh resolutions
    controller.n_mesh_x = 400;
    controller.n_mesh_v = 400;
    controller.n_mesh_t = 600;
    controller.n_mesh_w = 600;
    
    controller.n_mesh_F = 10;
    controller.n_mesh_M = 20;
% generate the controller
generate_DP_ForceMoment_controller(controller)
    %use this option to visualise the controller space convergance
% generate_DP_ForceMoment_controller(controller, '+visualization') 

% show the control surfaces
test_surface(controller.name);
    %use this function to plot surfaces with quivers
% test_surface_with_quivers(controller.name)
%% Simulate the results
%simulator variables
    simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one thruster inoperative
    simulator_opts.thruster_allocation_mode = 'active set discrete'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
    simulator_opts.faulty_thruster_index = [0 1 9]; %index of faulty thruster(s) #0-#11
    simulator_opts.current_controller = controller.name;
    simulator_opts.controller_InterpmodeF = 'linear'; %interpolation mehod of F controller output
    simulator_opts.controller_InterpmodeM = 'linear'; %interpolation mehod of M controller output
    simulator_opts.T_final = 200; %simulation Tfinal
    simulator_opts.h = 0.005; %simulation fixed time steps
    simulator_opts.Thruster_max_F = Thruster_max_F;
    simulator_opts.Thruster_dist = Thruster_dist;
    
    
    %initial state
    dr0 = [-30 0 0]; %initial relative position offset
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(0),deg2rad(0),deg2rad(0))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    
    %active set parameter
    simulator_opts.active_set.Weighting_Matrix = [1,0;...
                                                  0,0.56];
    %PWPF and schmitt trigger configuration parameters
    simulator_opts.PWPF.Km = 1.5;
    simulator_opts.PWPF.Tm = .2;
    simulator_opts.PWPF.h = 0.005; 
    simulator_opts.PWPF.H_feed = 0.6;
    
    simulator_opts.schmitt.Uout = Thruster_max_F;
    simulator_opts.schmitt.Uon = 0.8*simulator_opts.schmitt.Uout;
    simulator_opts.schmitt.Uoff = 0.8*simulator_opts.schmitt.Uon;
% create and run the simulator
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path()

