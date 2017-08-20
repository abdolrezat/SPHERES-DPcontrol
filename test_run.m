%TEST_RUN runs a sample of the code in this repository
%   First it generates a controller using a vectorized Dynamic Programming
%   algorithm and then it launches the simulator that uses this controller 
addpath(path,'generate_controller')
addpath(path,'simulator')
%% generate controller
%controller variables
    controller.name = 'controller_attpositionf2';
    controller.Tf = 5;
    controller.Tm = 2;
    %time variables
    controller.h = 0.005;
    %Optimal Control constants
    controller.Qx = 2.0;
    controller.Qv = 2.0*40;
    controller.Qt = 0.05;
    controller.Qw = 0.01;
    controller.R =  1.0;
    % mesh generation
    controller.lim_x = [-50 50]; %in m
    controller.lim_v = [-2.2 2.2]; %in m/s
    controller.lim_t = deg2rad([-70 70]); %in degrees, converts to rad
    controller.lim_w = deg2rad([-50 50]); %in rad/s
    
    Thruster_max_F = 0.13; % (N)
    Thruster_dist = 9.65E-2; % (meters)
    Thruster_max_M = Thruster_max_F*Thruster_dist;
    controller.lim_F = 2*[-Thruster_max_F Thruster_max_F];
    controller.lim_M = 2*[-Thruster_max_M Thruster_max_M];
    
    controller.n_mesh_x = 200;
    controller.n_mesh_v = 200;
    controller.n_mesh_t = 200;
    controller.n_mesh_w = 200;
    
    controller.n_mesh_F = 25;
    controller.n_mesh_M = 25;
%
generate_DP_ForceMoment_controller(controller)
% generate_DP_ForceMoment_controller(controller, '+visualization')

% show the control surfaces
test_surface(controller.name)

%% Simulate the results

%simulator variables
    simulator_opts.mode = 'fault';
    simulator_opts.current_controller = controller.name;
    simulator_opts.controller_Interpmode = 'linear';
    simulator_opts.T_final = 150;
    simulator_opts.h = 0.005;
    %initial state
    dr0 = [-40 30 20];
    dv0 = [0 0 0];
    q0 = flip(angle2quat(deg2rad(30),deg2rad(-10),deg2rad(-15)));
    w0 = [0 0 0];
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    
    %schmitt trigger config
    simulator_opts.schmitt.Uout = Thruster_max_F;
    simulator_opts.schmitt.Uon = 0.6*simulator_opts.schmitt.Uout;
    simulator_opts.schmitt.Uoff = 0.2*simulator_opts.schmitt.Uon;
%
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path

