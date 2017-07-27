%TEST_RUN runs a sample of the code in this repository
%   First it generates a controller using a vectorized Dynamic Programming
%   algorithm and then it launches the simulator that uses this controller 
addpath(path,'generate_controller')
addpath(path,'simulator')
%% generate controller
%controller variables
    controller.name = 'controller_50m_71deg';
    controller.Tf = 50;
    controller.Tm = 20;
    %time variables
    controller.h = 0.005;
    %Optimal Control constants
    controller.Qx = 1.0;
    controller.Qv = 1.0;
    controller.Qt = 1.0;
    controller.Qw = 1.0;
    controller.R =  1.0;
    % mesh generation
    controller.lim_x = [-50 50]; %in m
    controller.lim_v = [-2.2 2.2]; %in m/s
    controller.lim_t = deg2rad([-71 71]); %in degrees, converts to rad
    controller.lim_w = deg2rad([-50 50]); %in rad/s
    
    controller.n_mesh_x = 700;
    controller.n_mesh_v = 700;
    controller.n_mesh_t = 700;
    controller.n_mesh_w = 700;
%
generate_DP_ForceMoment_controller(controller)

%% Simulate the results

%simulator variables
    simulator_opts.current_controller = 'controller_50m_71deg';
    simulator_opts.T_final = 90;
    simulator_opts.h = 0.005;
    %initial state
    dr0 = [10 12 0];
    dv0 = [0 0 0];
    q0 = flip(angle2quat(deg2rad(20),deg2rad(40),deg2rad(0)));
    w0 = [0 0 0];
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
%
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path


