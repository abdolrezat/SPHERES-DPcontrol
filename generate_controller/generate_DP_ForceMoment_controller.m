function generate_DP_ForceMoment_controller(varargin)
%GENERATE_DP_X_V_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
path(path,'functions')
%time variables
h = 0.005;

%Optimal Control constants
Qx = 1.0;
Qv = 1.0;
Qt = 1.0;
Qw = 1.0;
R =  1;
%SPHERES config values
J1 = 0.02836 + 0.00016;
J2 =  0.026817 + 0.00150;
J3 =  0.023 + 0.00150;
Mass = 4.16;
Thruster_dist = 9.65E-2; % (meters)

Thruster_max_F = 0.13; % (N)
Thruster_max_M = Thruster_max_F*Thruster_dist;

% mesh generation
lim_x = [-20 20]; %in m
lim_v = [-2.0 2.0]; %in m/s
lim_t = deg2rad([-50 50]); %in degrees, converts to rad
lim_w = deg2rad([-50 50]); %in rad/s

%logarithmic spacing
s_x = mesh_state_log(lim_x, 1000, 0.06);
s_v = mesh_state_log(lim_v, 1000, 2.4);
s_t = mesh_state_log(lim_t, 1000, rad2deg(0.06));
s_w = mesh_state_log(lim_w, 1000, rad2deg(0.1));

%linear spacing
v_Fthruster = mesh_state(2*[-Thruster_max_F Thruster_max_F], 3);
v_Mthruster = mesh_state([-Thruster_max_M Thruster_max_M], 3);

path_ = strsplit(mfilename('fullpath'),'\\');
path_ = strjoin(path_(1:end-1),'\');

filename = strcat(path_,'\','controller\controller_linspace2_20m_20deg_3F');

%test environment
if(nargin > 0)
    if(varargin{1} == 'test')
        %logarithmic spacing
        s_x = mesh_state_log(lim_x, 100, 0.05);
        s_v = mesh_state_log(lim_v, 100, 2.2);
        s_t = mesh_state_log(lim_t, 100, rad2deg(0.04));
        s_w = mesh_state_log(lim_w, 100, 0.35);
        
        %linear spacing
        v_Fthruster = mesh_state(2*[-Thruster_max_F Thruster_max_F], 100);
        v_Mthruster = mesh_state(2*[-Thruster_max_M Thruster_max_M], 100);
        filename = [filename,'_test'];
    end
end
%end of test if

%% Run Calculations
%check folder exists
% if(exist('controller','dir') ~= 7) mkdir('controller'), end
% Run
T_final = 100;
[F_gI,F_U_Optimal_id] = DP_XV_one_channel_U_Opt(s_x,s_v, ...
    v_Fthruster,Qx,Qv,R, h, T_final, Mass);
save(filename,'F_gI', 'F_U_Optimal_id','v_Fthruster','v_Mthruster')
clear('F_gI', 'F_U_Optimal_id')

T_final = 25;
[M_gI_J1,M_U_Optimal_id_J1] = DP_TW_one_channel_U_Opt(s_t,s_w, ...
    v_Mthruster,Qt,Qw,R, h, T_final, J1);
save(filename, 'M_gI_J1' , 'M_U_Optimal_id_J1', '-append')
clear('M_gI_J1' , 'M_U_Optimal_id_J1')

[M_gI_J2,M_U_Optimal_id_J2] = DP_TW_one_channel_U_Opt(s_t,s_w, ...
    v_Mthruster,Qt,Qw,R, h, T_final, J2);
save(filename, 'M_gI_J2' , 'M_U_Optimal_id_J2', '-append')
clear('M_gI_J2' , 'M_U_Optimal_id_J2')

[M_gI_J3,M_U_Optimal_id_J3] = DP_TW_one_channel_U_Opt(s_t,s_w, ...
    v_Mthruster,Qt,Qw,R, h, T_final, J3);
save(filename, 'M_gI_J3' , 'M_U_Optimal_id_J3', '-append')
clear('M_gI_J3' , 'M_U_Optimal_id_J3')

end

