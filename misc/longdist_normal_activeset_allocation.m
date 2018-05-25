function longdist_normal_activeset_allocation()
%  this script runs a sample of the simulation code in this repository
%  the simulator options are defined in order to check control allocation
%  performances in thruster(s) normal or faulty situations
%  controller is generated from:
addpath(path,genpath('generate_controller'))
addpath(path,'simulator')

controllername = 'controller_attposition_9_longdistQ3';
%% regenerate controller
% uncomment if controller does not exist:
generate_controller_longdist_(controllername)

%% Simulation
    Thruster_max_F = 0.12; % (N)
    Thruster_dist = (9.65E-2); % (meters)
%simulator variables
    simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
    simulator_opts.thruster_allocation_mode = 'active set discrete'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
    simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
    simulator_opts.current_controller = controllername;
    simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
    simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
    simulator_opts.T_final = 120; %simulation Tfinal
    simulator_opts.h = 0.005; %simulation fixed time steps
    simulator_opts.Thruster_max_F = Thruster_max_F;
    simulator_opts.Thruster_dist = Thruster_dist;
    
    
    %initial state
    dr0 = [-30 -30 -30]; %initial relative position offset
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(0),deg2rad(0),deg2rad(0))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    
    %active set parameter
    simulator_opts.active_set.Weighting_Matrix = [1,0;...
                                                  0,2];
    %PWPF and schmitt trigger configuration parameters
    simulator_opts.PWPF.Km = 1.5;
    simulator_opts.PWPF.Tm = .2;
    simulator_opts.PWPF.h = 0.005; 
    simulator_opts.PWPF.H_feed = 0.6;
    
    simulator_opts.schmitt.Uout = Thruster_max_F;
    simulator_opts.schmitt.Uon = 0.3*simulator_opts.schmitt.Uout;
    simulator_opts.schmitt.Uoff = 0.8*simulator_opts.schmitt.Uon;
% create and run the simulator
SC = Simulator_CW(simulator_opts);
SC.plot_optimal_path()




function generate_controller_longdist_(Name)
%GENERATE_CONTROLLER is a sample of code for generating a controller
%   It generates 4 controllers (one for Transitional and three for Rotational motion)
%   using a vectorized Dynamic Programming algorithm, Using the m-files located
%   in the corresponding folder
addpath(path,genpath('generate_controller'))
%% generate controller
%controller variables
    controller.name = Name; %name of controller, will be saved under /controller directory
    controller.Tf = 120; % Tfinal for Force controller run
    controller.Tm = 3; % Tfinal for Moment controller run
    %time variables
    controller.h = 0.005; % time step for controllers
    %Optimal Control constants
    controller.Qx = 10;
    controller.QvQx_ratio = 45;
    controller.Qv = controller.QvQx_ratio * controller.Qx;
    controller.Qt = 0.01;
    controller.QwQt_ratio = 0.8;
    controller.Qw = controller.QwQt_ratio * controller.Qt;
    controller.R =  1.0;
    % mesh generation , limits
    controller.lim_x = [-60 60]; %in m
    controller.lim_v = [-2.2 2.2]; %in m/s
    controller.lim_t = deg2rad([-180 180]); %in degrees, converts to rad
    controller.lim_w = deg2rad([-200 200]); %in rad/s
    % mesh generation , Force and Moments
    Thruster_max_F = 0.12; % (N)
    Thruster_dist = (9.65E-2); % (meters)
    Thruster_max_M = Thruster_max_F*Thruster_dist;
    controller.lim_F = 2*[-Thruster_max_F Thruster_max_F];
    controller.lim_M = 2*[-Thruster_max_M Thruster_max_M];
    
    % mesh generation , mesh resolutions
    controller.n_mesh_x = 402;
    controller.n_mesh_v = 400;
    controller.n_mesh_t = 502;
    controller.n_mesh_w = 500;
    
    controller.n_mesh_F = 5;
    controller.n_mesh_M = 5;
% generate the controller
generate_DP_ForceMoment_controller(controller)
    %use this option to visualise the controller space convergance
% generate_DP_ForceMoment_controller(controller, '+visualization') 

% optional: show the control surfaces
test_surface(controller.name)
    % optional: use this function to plot surfaces with quivers that show where the
    %next stages end up under optimal commands
    % test_surface_with_quivers(controller.name)