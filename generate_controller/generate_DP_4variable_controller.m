function generate_DP_4variable_controller()
path(path,'functions')
%time variables
h = 0.005;
T_final = 50;

%Optimal Control constants
Qx = .6;
Qv = .6;
Qt = .5;
Qw = .5;
R =  .1;
%SPHERES config values, single channel
J = 0.02836 + 0.00016;
Mass = 4.16;
Thruster_max_F = 0.13; % (N)
Thruster_dist = 9.65E-2; % (meters)
F_Thr0 = [0 Thruster_max_F];
F_Thr1 = [0 Thruster_max_F];
F_Thr6 = -[0 Thruster_max_F];
F_Thr7 = -[0 Thruster_max_F];

% mesh generation
lim_x = [-10 10]; %in m
lim_v = [-.4 .4]; %in m/s
lim_t = deg2rad([-20 20]); %in degrees, converts to rad
lim_w = [-1 1]; %in rad/s
[s_x,s_v,s_t,s_w] = mesh_states_4var(lim_x, lim_v, lim_t, lim_w, 50, 50, 50, 50);

%% Run Calculations
file_name = 'test';
DP_4var_one_channel_U_Opt(s_x,s_v,s_t,s_w, ...
    F_Thr0, F_Thr1, F_Thr6, F_Thr7, Qx, Qv, Qt, Qw, R, h, T_final, Mass, J, Thruster_dist,file_name)