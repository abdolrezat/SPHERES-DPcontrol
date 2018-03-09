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
    error('invalid or no input to the generate_controller function')
%     controller_name = 'controller_linspace2_70m_70deg_3F_noname';
%     Tf = 120;
%     Tm = 20;
%     %time variables
%     h = 0.005;
%     %Optimal Control constants
%     Qx = 1.0;
%     Qv = 1.0;
%     Qt = 1.0;
%     Qw = 1.0;
%     R =  1;
%     % mesh generation
%     lim_x = [-20 20]; %in m
%     lim_v = [-2.0 2.0]; %in m/s
%     lim_t = deg2rad([-70 70]); %in degrees, converts to rad
%     lim_w = deg2rad([-50 50]); %in rad/s
%     
%     n_mesh_x = 10;
%     n_mesh_v = 10;
%     n_mesh_t = 10;
%     n_mesh_w = 10;
%     n_mesh_F = 10;
%     n_mesh_M = 10;
end
%SPHERES config values
J1 = 2.3E-2;
J2 = 2.42E-2;
J3 = 2.14E-2;
Mass = 4.16;

    %logarithmic spacing
    s_x = mesh_state_log(lim_x, n_mesh_x, 5);
    s_v = mesh_state_log(lim_v, n_mesh_v, 5);
    s_t = mesh_state_log(lim_t, n_mesh_t, 5);
    s_w = mesh_state_log(lim_w, n_mesh_w, 5);
    %linear spacing
    v_Fthruster_x = mesh_state(controller.lim_Fx, controller.n_mesh_Fx);
    v_Fthruster_y = mesh_state(controller.lim_Fy, controller.n_mesh_Fy);
    v_Fthruster_z = mesh_state(controller.lim_Fz, controller.n_mesh_Fz);
    
    v_Mthruster_x = mesh_state(controller.lim_Mx, controller.n_mesh_Mx);
    v_Mthruster_y = mesh_state(controller.lim_My, controller.n_mesh_My);
    v_Mthruster_z = mesh_state(controller.lim_Mz, controller.n_mesh_Mz);

filename = strcat(path_,'\','controller\',controller_name);


%% Run Calculations
%check folder exists
% if(exist('controller','dir') ~= 7) mkdir('controller'), end
% Run
T_final = Tf;
if(visualization == 0)
    [Fx_gI,Fx_U_Optimal_id] = DP_XV_one_channel_U_Opt(s_x,s_v, ...
        v_Fthruster_x,controller.Qx1,controller.Qv1,R, h, T_final, Mass); %#ok<ASGLU>
else
    [Fx_gI,Fx_U_Optimal_id] = DP_XV_one_channel_U_Opt_visualization(s_x,s_v, ...
        v_Fthruster_x,controller.Qx1,controller.Qv1,R, h, T_final, Mass); %#ok<ASGLU>
end

% Fy_gI = Fx_gI; %#ok<NASGU>
% Fz_gI = Fx_gI; %#ok<NASGU>
% Fy_U_Optimal_id = Fx_U_Optimal_id; %#ok<NASGU>
% Fz_U_Optimal_id = Fx_U_Optimal_id; %#ok<NASGU>

save(filename,'Fx_gI', 'Fx_U_Optimal_id','v_Fthruster_x', 'controller')
clear('Fx_gI', 'Fx_U_Optimal_id')
% 
[Fy_gI,Fy_U_Optimal_id] = DP_XV_one_channel_U_Opt(s_x,s_v, ...
    v_Fthruster_y,controller.Qx2,controller.Qv2,R, h, T_final, Mass); %#ok<ASGLU>
save(filename,'Fy_gI', 'Fy_U_Optimal_id','v_Fthruster_y', '-append')
clear('Fy_gI', 'Fy_U_Optimal_id')

[Fz_gI,Fz_U_Optimal_id] = DP_XV_one_channel_U_Opt(s_x,s_v, ...
    v_Fthruster_z,controller.Qx3,controller.Qv3,R, h, T_final, Mass); %#ok<ASGLU>
save(filename,'Fz_gI', 'Fz_U_Optimal_id','v_Fthruster_z', '-append')
clear('Fz_gI', 'Fz_U_Optimal_id')

T_final = Tm;
[M_gI_J1,M_U_Optimal_id_J1] = DP_TW_one_channel_U_Opt(s_t,s_w, ... 
    v_Mthruster_x,Qt,Qw,R, h, T_final, J1); %#ok<ASGLU>
save(filename, 'M_gI_J1' , 'M_U_Optimal_id_J1','v_Mthruster_x', '-append')
clear('M_gI_J1' , 'M_U_Optimal_id_J1')

[M_gI_J2,M_U_Optimal_id_J2] = DP_TW_one_channel_U_Opt(s_t,s_w, ...
    v_Mthruster_y,Qt,Qw,R, h, T_final, J2); %#ok<ASGLU>
save(filename, 'M_gI_J2' , 'M_U_Optimal_id_J2','v_Mthruster_y', '-append')
clear('M_gI_J2' , 'M_U_Optimal_id_J2')

[M_gI_J3,M_U_Optimal_id_J3] = DP_TW_one_channel_U_Opt(s_t,s_w, ...
    v_Mthruster_z,Qt,Qw,R, h, T_final, J3); %#ok<ASGLU>
save(filename, 'M_gI_J3' , 'M_U_Optimal_id_J3','v_Mthruster_z', '-append')
clear('M_gI_J3' , 'M_U_Optimal_id_J3')

end

