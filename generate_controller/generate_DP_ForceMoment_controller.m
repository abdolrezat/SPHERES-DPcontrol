function generate_DP_ForceMoment_controller(varargin)
%GENERATE_DP_FORCEMOMENT_CONTROLLER generates a controller using Dynamic
%Programming methods
%   Usage: generate_DP_ForceMoment_controller(controller) for customized
%   generate_DP_ForceMoment_controller(controller,'test') for test
% 
path_ = strsplit(mfilename('fullpath'),'\\');
path_ = strjoin(path_(1:end-1),'\');

addpath(path,strcat(path_,'\functions'))

if(nargin > 0)
    controller = varargin{1};
    if (nargin == 2)
        visualization = 1;
    else
        visualization = 0;
    end
    controller_name = controller.name;
    Tf = controller.Tf;
    Tm = controller.Tm;
    %time variables
    h = controller.h;
    %Optimal Control constants
    Qx = controller.Qx;
    Qv = controller.Qv;
    Qt = controller.Qt;
    Qw = controller.Qw;
    R =  controller.R;
    
    lim_x = controller.lim_x; %in m
    lim_v = controller.lim_v; %in m/s
    lim_t = controller.lim_t; %in degrees, converts to rad
    lim_w = controller.lim_w; %in rad/s
    
    n_mesh_x = controller.n_mesh_x;
    n_mesh_v = controller.n_mesh_v;
    n_mesh_t = controller.n_mesh_t;
    n_mesh_w = controller.n_mesh_w;
    
else
    controller_name = 'controller_linspace2_70m_70deg_3F_noname';
    Tf = 120;
    Tm = 20;
    %time variables
    h = 0.005;
    %Optimal Control constants
    Qx = 1.0;
    Qv = 1.0;
    Qt = 1.0;
    Qw = 1.0;
    R =  1;
    % mesh generation
    lim_x = [-20 20]; %in m
    lim_v = [-2.0 2.0]; %in m/s
    lim_t = deg2rad([-70 70]); %in degrees, converts to rad
    lim_w = deg2rad([-50 50]); %in rad/s
end
%SPHERES config values
J1 = 0.02836 + 0.00016;
J2 =  0.026817 + 0.00150;
J3 =  0.023 + 0.00150;
Mass = 4.16;
Thruster_dist = 9.65E-2; % (meters)

Thruster_max_F = 0.13; % (N)
Thruster_max_M = Thruster_max_F*Thruster_dist;

    %logarithmic spacing
    s_x = mesh_state_log(lim_x, n_mesh_x, 0.06);
    s_v = mesh_state_log(lim_v, n_mesh_v, 2.4);
    s_t = mesh_state_log(lim_t, n_mesh_t, rad2deg(0.06));
    s_w = mesh_state_log(lim_w, n_mesh_w, rad2deg(0.1));
    %linear spacing
    v_Fthruster = mesh_state(2*[-Thruster_max_F Thruster_max_F], 3);
    v_Mthruster = mesh_state(2*[-Thruster_max_M Thruster_max_M], 3);

filename = strcat(path_,'\','controller\',controller_name);


%% Run Calculations
%check folder exists
% if(exist('controller','dir') ~= 7) mkdir('controller'), end
% Run
T_final = Tf;
if(visualization == 0)
    [F_gI,F_U_Optimal_id] = DP_XV_one_channel_U_Opt(s_x,s_v, ...
        v_Fthruster,Qx,Qv,R, h, T_final, Mass);
else
    [F_gI,F_U_Optimal_id] = DP_XV_one_channel_U_Opt_visualization(s_x,s_v, ...
        v_Fthruster,Qx,Qv,R, h, T_final, Mass);
end
save(filename,'F_gI', 'F_U_Optimal_id','v_Fthruster','v_Mthruster')
clear('F_gI', 'F_U_Optimal_id')

T_final = Tm;
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

