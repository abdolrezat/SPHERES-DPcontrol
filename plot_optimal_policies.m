function plot_optimal_policies()
%PLOT_OPTIMAL_POLICIES Summary of this function goes here
%   Detailed explanation goes here

%% generate controller
% if nargin == 0
    controller.name = 'controller_normal_DynamicProgramming_for_plot'; %name of controller, will be saved under /controller directory
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
    test_surface_gray(controller.name)
    
end



function generate_Dynamic_Programming_controller(controller)%GENERATE_CONTROLLER is a sample of code for generating a controller
%   This generates 4 controllers (one for Transitional and three for Rotational motion)
%   using a vectorized Dynamic Programming algorithm, Using the m-files located
%   in the corresponding folder
%% generate controller
    %controller variables
    controller.Tf = 100; % Tfinal for Force controller run
    controller.Tm = 100; % Tfinal for Moment controller run
    %time variables
    controller.h = 1; % time step for controllers
    %Optimal Control constants
    controller.Qv1 = controller.Qv1Qx1_ratio * controller.Qx1;
    controller.Qv2 = controller.Qv2Qx2_ratio * controller.Qx2;
    controller.Qv3 = controller.Qv3Qx3_ratio * controller.Qx3;

    controller.Qw = controller.QwQt_ratio * controller.Qt;
    controller.R =  1.0;

%     % mesh generation , limits
%     controller.lim_x = [-40 40]; %in m
%     controller.lim_v = [-1.5 1.5]; %in m/s
%     controller.lim_t = deg2rad([-60 60]); %in degrees, converts to rad
%     controller.lim_w = deg2rad([-50 50]); %in rad/s% mesh generation , Force and Moments
 % mesh generation , limits
    controller.lim_x = [-40 40]; %in m
    controller.lim_v = [-1.5 1.5]; %in m/s
    controller.lim_t = deg2rad([-500 500]); %in degrees, converts to rad
    controller.lim_w = [-1.4 1.4]; %in rad/s% mesh generation , Force and Moments

    % mesh generation , mesh resolutions
    controller.n_mesh_x = 602;
    controller.n_mesh_v = 600;
    controller.n_mesh_t = 1002;
    controller.n_mesh_w = 700;

% generate the controller
generate_DP_ForceMoment_controller(controller)
end
