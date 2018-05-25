function Dynamic_Programming_controller_parameter_tuning(varargin)
%DYNAMIC_PROGRAMMING_CONTROLLER_PARAMETER_TUNING 
%tune parameters of Dynamic Programming controllers as follows:
% create two controllers with different parameters ( it is suggested to
% change one variable at a time) and simulate both in failure conditions
% with saturate-only allocation

%% generate controller
% if nargin == 0
    controller.name = 'controller_DP_attposition_fault_thr2'; %name of controller, will be saved under /controller directory
    Thruster_max_F = 0.12/5; % (N)
    Thruster_dist = (9.65E-2); % (meters)
    Thruster_max_M = Thruster_max_F*Thruster_dist;
    controller.lim_Fx = [-2*Thruster_max_F 2*Thruster_max_F];
    controller.lim_My = [-2*Thruster_max_M 2*Thruster_max_M];
    controller.lim_Fy = [-2*Thruster_max_F Thruster_max_F];
    controller.lim_Mz = [-2*Thruster_max_M Thruster_max_M];
    controller.lim_Fz = [-2*Thruster_max_F 2*Thruster_max_F];
    controller.lim_Mx = [-2*Thruster_max_M 2*Thruster_max_M];
    controller.n_mesh_Fx = 41;
    controller.n_mesh_My = 41;
    controller.n_mesh_Fy = 31;
    controller.n_mesh_Mz = 31;
    controller.n_mesh_Fz = 41;
    controller.n_mesh_Mx = 41;
    
    controller.Qx1 = 1;
    controller.Qv1Qx1_ratio = 200; %% decrease (to 300 maybe)
    controller.Qx2 = 0.01;
    controller.Qv2Qx2_ratio = 500; %% decrease (to 300 maybe)
    controller.Qx3 = 1;
    controller.Qv3Qx3_ratio = 200; %% decrease (to 300 maybe)
    
    controller.Qt = 1;
    controller.QwQt_ratio = 5; %mayb*e a little decrease
% end
%     
    if(1)
        generate_Dynamic_Programming_controller(controller)
        test_surface(controller.name)
    end
%     generate_Dynamic_Programming_controller_2()
    %% put controller in test
    simulator_opts.current_controller = controller.name;
simulator_opts.faulty_thruster_index = [0]; %index of faulty thruster(s) #0-#11
  simulator_opts.thruster_allocation_mode = 'quadratic programming pulse modulation-adaptive'; %'spheres pulse modulation' {'active set discrete', 'PWPF', 'Schmitt', 'none'}
%   simulator_opts.Quad_Prog.Weighting_Matrix_1 = [1,0;...
%     0,0]; 
%   simulator_opts.Quad_Prog.Weighting_Matrix_2 = [1,0;...
%     0,0.93];
%   simulator_opts.Quad_Prog.Weighting_Matrix_3 = [1,0;...
%     0,0.93];
simulator_opts.Thruster_max_F = 0.12; % (N)
simulator_opts.h = 0.01; %simulation fixed time steps
  simulator_opts.T_final = 200; %simulation Tfinal

%     for r=[1,2,3,4,5,6,7,9,10,12,14,15,16,17]
%         for r=[0.6,1,4]
for r= -10
% for r=[0.6,1,2,3,5,7,10,12]
% for r = [20,22,25,28,30,32]
% for r = [0.6,1,4,10,20]
    dr0 = [-r 0 0]; %initial relative position offset
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(0),deg2rad(0),deg2rad(0))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
%  simulator_opts.defaultX0 = [-0.0734   -0.1612    0.0860   -0.0171   -0.0005   -0.0189   -0.2233   -0.2710    0.0843    0.9325    0.1187    0.0260   -0.0272]';
    S = simulation_fault_DP_noallocation(simulator_opts);
%     S2 =simulation_fault_DP_noallocation_2(simulator_opts);
%     close all
    S.plot_optimal_path
    S.plot_xy_plane
%     plot12(S2,S2)
    pause(.01)
    end
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

    % mesh generation , limits
    controller.lim_x = [-40 40]; %in m
    controller.lim_v = [-1.5 1.5]; %in m/s
    controller.lim_t = deg2rad([-200 200]); %in degrees, converts to rad
    controller.lim_w = deg2rad([-150 150]); %in rad/s% mesh generation , Force and Moments

    % mesh generation , mesh resolutions
    controller.n_mesh_x = 402;
    controller.n_mesh_v = 400;
    controller.n_mesh_t = 502;
    controller.n_mesh_w = 500;
    

% generate the controller
generate_DP_ForceMoment_controller(controller)
end

function SC = simulation_fault_DP_noallocation(simulator_opts)

Thruster_dist = (9.65E-2); % (meters)
%simulator variables
simulator_opts.mode = 'fault';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
% simulator_opts.current_controller = 'controller_DP_attposition';

simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
simulator_opts.Thruster_dist = Thruster_dist;

%PWPF for fault and schmitt trigger configuration parameters
PWPF1params = [26.0896    0.8597    0.2015    0.1493    0.8138]; %from optimization

simulator_opts.PWPF1.Km = PWPF1params(1);
simulator_opts.PWPF1.Tm = PWPF1params(2);
simulator_opts.PWPF1.h = simulator_opts.h;
simulator_opts.PWPF1.H_feed = PWPF1params(3);

simulator_opts.schmitt1.Uout = simulator_opts.Thruster_max_F;
simulator_opts.schmitt1.Uon = PWPF1params(4)*simulator_opts.schmitt1.Uout;
simulator_opts.schmitt1.Uoff = PWPF1params(5)*simulator_opts.schmitt1.Uon;
simulator_opts.PWPF2 = simulator_opts.PWPF1; simulator_opts.schmitt2 = simulator_opts.schmitt1;

%PWPF for normal conditions and schmitt trigger configuration parameters
PWPF3params = [45.1521    0.9972    0.2359    0.4493    0.9888]; %from optimization

simulator_opts.PWPF3.Km = PWPF3params(1);
simulator_opts.PWPF3.Tm = PWPF3params(2);
simulator_opts.PWPF3.h = 0;
simulator_opts.PWPF3.H_feed = PWPF3params(3);

simulator_opts.schmitt3.Uout = simulator_opts.Thruster_max_F;
simulator_opts.schmitt3.Uon = PWPF3params(4)*simulator_opts.schmitt3.Uout;
simulator_opts.schmitt3.Uoff = PWPF3params(5)*simulator_opts.schmitt3.Uon;
simulator_opts.PWPF4 = simulator_opts.PWPF3; simulator_opts.schmitt4 = simulator_opts.schmitt3;
simulator_opts.PWPF5 = simulator_opts.PWPF3; simulator_opts.schmitt5 = simulator_opts.schmitt3;
simulator_opts.PWPF6 = simulator_opts.PWPF3; simulator_opts.schmitt6 = simulator_opts.schmitt3;

% create and run the simulator
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path;
end



function generate_Dynamic_Programming_controller_2()%GENERATE_CONTROLLER is a sample of code for generating a controller
%   This generates 4 controllers (one for Transitional and three for Rotational motion)
%   using a vectorized Dynamic Programming algorithm, Using the m-files located
%   in the corresponding folder
%% generate controller
    %controller variables
    controller.name = 'controller_DP_attposition_test2'; %name of controller, will be saved under /controller directory
    controller.Tf = 60; % Tfinal for Force controller run
    controller.Tm = 10; % Tfinal for Moment controller run
    %time variables
    controller.h = 0.01; % time step for controllers
    %Optimal Control constants
    controller.Qx = 2;
    controller.QvQx_ratio = 35;
    controller.Qv = controller.QvQx_ratio * controller.Qx;
    controller.Qt = 0.01;
    controller.QwQt_ratio = 1.5;
    controller.Qw = controller.QwQt_ratio * controller.Qt;
    controller.R =  1.0;

    % mesh generation , limits
    controller.lim_x = [-40 40]; %in m
    controller.lim_v = [-1.5 1.5]; %in m/s
    controller.lim_t = deg2rad([-180 180]); %in degrees, converts to rad
    controller.lim_w = deg2rad([-220 220]); %in rad/s% mesh generation , Force and Moments
    Thruster_max_F = 0.12; % (N)
    Thruster_dist = (9.65E-2); % (meters)
    Thruster_max_M = Thruster_max_F*Thruster_dist;
    controller.lim_F = 2*[-Thruster_max_F Thruster_max_F];
    controller.lim_M = 2*[-Thruster_max_M Thruster_max_M];
    
    % mesh generation , mesh resolutions
    controller.n_mesh_x = 402;
    controller.n_mesh_v = 400;
    controller.n_mesh_t = 602;
    controller.n_mesh_w = 600;
    
    controller.n_mesh_F = 20;
    controller.n_mesh_M = 20;
% generate the controller
generate_DP_ForceMoment_controller(controller)
end

function SC = simulation_fault_DP_noallocation_2(simulator_opts)

Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
%simulator variables
simulator_opts.mode = 'fault';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
simulator_opts.thruster_allocation_mode = 'saturate-only'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
simulator_opts.current_controller = 'controller_DP_attposition';
% simulator_opts.current_controller = 'controller_DP_attposition_test2';

simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
simulator_opts.T_final = 200; %simulation Tfinal
simulator_opts.Thruster_max_F = Thruster_max_F;
simulator_opts.Thruster_dist = Thruster_dist;

%active set parameter
simulator_opts.active_set.Weighting_Matrix = [1,0;...
    0,1];
%PWPF for fault and schmitt trigger configuration parameters
PWPF1params = [26.0896    0.8597    0.2015    0.1493    0.8138]; %from optimization

simulator_opts.PWPF1.Km = PWPF1params(1);
simulator_opts.PWPF1.Tm = PWPF1params(2);
simulator_opts.PWPF1.h = simulator_opts.h;
simulator_opts.PWPF1.H_feed = PWPF1params(3);

simulator_opts.schmitt1.Uout = Thruster_max_F;
simulator_opts.schmitt1.Uon = PWPF1params(4)*simulator_opts.schmitt1.Uout;
simulator_opts.schmitt1.Uoff = PWPF1params(5)*simulator_opts.schmitt1.Uon;
simulator_opts.PWPF2 = simulator_opts.PWPF1; simulator_opts.schmitt2 = simulator_opts.schmitt1;

%PWPF for normal conditions and schmitt trigger configuration parameters
PWPF3params = [45.1521    0.9972    0.2359    0.4493    0.9888]; %from optimization

simulator_opts.PWPF3.Km = PWPF3params(1);
simulator_opts.PWPF3.Tm = PWPF3params(2);
simulator_opts.PWPF3.h = 0.005;
simulator_opts.PWPF3.H_feed = PWPF3params(3);

simulator_opts.schmitt3.Uout = Thruster_max_F;
simulator_opts.schmitt3.Uon = PWPF3params(4)*simulator_opts.schmitt3.Uout;
simulator_opts.schmitt3.Uoff = PWPF3params(5)*simulator_opts.schmitt3.Uon;
simulator_opts.PWPF4 = simulator_opts.PWPF3; simulator_opts.schmitt4 = simulator_opts.schmitt3;
simulator_opts.PWPF5 = simulator_opts.PWPF3; simulator_opts.schmitt5 = simulator_opts.schmitt3;
simulator_opts.PWPF6 = simulator_opts.PWPF3; simulator_opts.schmitt6 = simulator_opts.schmitt3;

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
