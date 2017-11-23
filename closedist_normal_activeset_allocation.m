%  this script runs a sample of the simulation code in this repository
%  the simulator options are defined in order to check control allocation
%  performances in thruster(s) normal or faulty situations
%  controller is generated from:
% generate_controller_closedist_normal()

addpath(path,genpath('generate_controller'))
addpath(path,'simulator')
%% Simulation
    Thruster_max_F = 0.12; % (N)
    Thruster_dist = (9.65E-2); % (meters)
%simulator variables
    simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
    simulator_opts.thruster_allocation_mode = 'active set discrete'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
    simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
    simulator_opts.current_controller = 'controller_attposition_9_closedist';
    simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
    simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
    simulator_opts.T_final = 100; %simulation Tfinal
    simulator_opts.h = 0.005; %simulation fixed time steps
    simulator_opts.Thruster_max_F = Thruster_max_F;
    simulator_opts.Thruster_dist = Thruster_dist;
    
    
    %initial state
    dr0 = [-0.6 -0.6 -0.6]; %initial relative position offset
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(10),deg2rad(10),deg2rad(10))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    
    %active set parameter
    simulator_opts.active_set.Weighting_Matrix = [1,0;...
                                                  0,1];
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
SC.get_optimal_path()

