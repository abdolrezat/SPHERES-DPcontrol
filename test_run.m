%TEST_RUN runs a sample of the code in this repository
%   First it generates a controller using a vectorized Dynamic Programming
%   algorithm and then it launches the simulator that uses this controller 
addpath(path,'generate_controller')
addpath(path,'simulator')
%% generate controller
%controller variables
    controller.name = 'controller_faulttest';
    controller.Tf = 50;
    controller.Tm = 20;
    %time variables
    controller.h = 0.005;
    %Optimal Control constants
    controller.Qx = 0.5;
    controller.Qv = 0.5;
    controller.Qt = 0.1;
    controller.Qw = 0.1;
    controller.R =  1.0;
    % mesh generation
    controller.lim_x = [-50 50]; %in m
    controller.lim_v = [-2.2 2.2]; %in m/s
    controller.lim_t = deg2rad([-70 70]); %in degrees, converts to rad
    controller.lim_w = deg2rad([-50 50]); %in rad/s
    
    controller.n_mesh_x = 300;
    controller.n_mesh_v = 300;
    controller.n_mesh_t = 900;
    controller.n_mesh_w = 900;
%
generate_DP_ForceMoment_controller(controller)
% generate_DP_ForceMoment_controller(controller, '+visualization')

%% Simulate the results

%simulator variables
    simulator_opts.mode = 'fault';
    simulator_opts.current_controller = 'controller_linspace2_70m_70deg';
    simulator_opts.T_final = 200;
    simulator_opts.h = 0.01;
    %initial state
    dr0 = [3 2 5];
    dv0 = [0 0 0];
    q0 = flip(angle2quat(deg2rad(10),deg2rad(10),deg2rad(10)));
    w0 = [0 0 0];
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
%
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path


