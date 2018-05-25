function generate_controller_closedist_normal()
%GENERATE_CONTROLLER is a sample of code for generating a controller
%   It generates 4 controllers (one for Transitional and three for Rotational motion)
%   using a vectorized Dynamic Programming algorithm, Using the m-files located
%   in the corresponding folder
addpath(path,genpath('generate_controller'))
%% generate controller
%controller variables
    controller.name = 'controller_attposition_9_closedist'; %name of controller, will be saved under /controller directory
    controller.Tf = 20; % Tfinal for Force controller run
    controller.Tm = 20; % Tfinal for Moment controller run
    %time variables
    controller.h = 0.005; % time step for controllers
    %Optimal Control constants
    controller.Qx = 0.3;
    controller.QvQx_ratio = 35;
    controller.Qv = controller.QvQx_ratio * controller.Qx;
    controller.Qt = 0.01;
    controller.QwQt_ratio = 0.8 ;
    controller.Qw = controller.QwQt_ratio * controller.Qt;
    controller.R =  1.0;
    % mesh generation , limits
    controller.lim_x = [-60 60]/20; %in m
    controller.lim_v = [-2.2 2.2]/20; %in m/s
    controller.lim_t = deg2rad([-180 180]/6); %in degrees, converts to rad
    controller.lim_w = deg2rad([-200 200]/6); %in rad/s
    % mesh generation , Force and Moments
    Thruster_max_F = 0.12; % (N)
    Thruster_dist = (9.65E-2); % (meters)
    Thruster_max_M = Thruster_max_F*Thruster_dist;
    controller.lim_F = 2*[-Thruster_max_F Thruster_max_F];
    controller.lim_M = 2*[-Thruster_max_M Thruster_max_M];
    
    % mesh generation , mesh resolutions
    controller.n_mesh_x = 202;
    controller.n_mesh_v = 200;
    controller.n_mesh_t = 302;
    controller.n_mesh_w = 300;
    
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