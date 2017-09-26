function generate_PD_controller(controller)
%GENERATE_DP_FORCEMOMENT_CONTROLLER generates a controller using Dynamic
%Programming methods
%   Usage: generate_DP_ForceMoment_controller(controller) for customized
%   generate_DP_ForceMoment_controller(controller,'test') for test
% 
path_ = strsplit(mfilename('fullpath'),'\\');
path_ = strjoin(path_(1:end-1),'\');

addpath(path,strcat(path_,'\functions'))

    controller_name = controller.name;
    
    lim_x = controller.lim_x; %in m
    lim_v = controller.lim_v; %in m/s
    lim_t = controller.lim_t; %in degrees, converts to rad
    lim_w = controller.lim_w; %in rad/s
    
    n_mesh_x = controller.n_mesh_x;
    n_mesh_v = controller.n_mesh_v;
    n_mesh_t = controller.n_mesh_t;
    n_mesh_w = controller.n_mesh_w;
    
    Kp_F = controller.Kp_F;
    Kd_F = controller.Kd_F;
    Kp_M = controller.Kp_M;
    Kd_M = controller.Kd_M;

    %logarithmic spacing
    s_x = mesh_state_log(lim_x, n_mesh_x, 5);
    s_v = mesh_state_log(lim_v, n_mesh_v, 5);
    s_t = mesh_state_log(lim_t, n_mesh_t, 5);
    s_w = mesh_state_log(lim_w, n_mesh_w, 5);

filename = strcat(path_,'\','controller\',controller_name);


%% Calculations
%Force
    F_PD = s_x' * -Kp_F + s_v * -Kd_F;
    F_gI = {s_x,s_v}; %#ok<*NASGU>
    size_F = size(F_PD);
    [v_Fthruster, ~, ic] = unique(F_PD); %#ok<*ASGLU,*NASGU>
    F_U_Optimal_id = reshape(ic, size_F);
    % check: isequal(F_PD , v_Fthruster(F_U_Optimal_id))

%Moments
    M_PD = s_t' * -Kp_M + s_w * -Kd_M;
    M_gI_J1 = {s_t, s_w};
    size_M = size(M_PD);
    [v_Mthruster, ~, ic] = unique(M_PD);
    M_U_Optimal_id_J1 = reshape(ic, size_M);

    M_gI_J2 = M_gI_J1;
    M_gI_J3 = M_gI_J1;
    M_U_Optimal_id_J2 = M_U_Optimal_id_J1;
    M_U_Optimal_id_J3 = M_U_Optimal_id_J1;
    
    save(filename,'F_gI', 'F_U_Optimal_id','v_Fthruster','v_Mthruster')
    save(filename, 'M_gI_J1' , 'M_U_Optimal_id_J1', '-append')
    save(filename, 'M_gI_J2' , 'M_U_Optimal_id_J2', '-append')
    save(filename, 'M_gI_J3' , 'M_U_Optimal_id_J3', '-append')
end

