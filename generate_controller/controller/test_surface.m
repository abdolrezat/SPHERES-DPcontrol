function test_surface(filename)
%test the controller values to see what portion of their surface (optimal)
%is made up from max and min values of Forces or Moments
%this controller has the optimal values for Forces and moments after trying
%1000 unique and linearly spaces values between max and min (F,M)s, However
%analysis shows that more than 93% and 94% of these surfaces are made up of
%only max/ min values of F&M. therefore, one can greatly reduce the
%DP controller generation run time by only trying 3 values in F and M (in
%other words: vector_F = [-max_thrusters*2, 0, max_thrusters*2] would
%suffice to obtain a good estimate on these surfaces, and the calculation
%times are greatly reduced.
%
%example:
%filename = 'controller_linspace2_70m_70deg.mat'.
%
path_ = strsplit(mfilename('fullpath'),'\\');
path_ = strjoin(path_(1:end-1),'\');

controller = load(strcat(path_,'\',filename));

% refine boundaries
temp =  controller.F_gI{1};
controller.F_gI{1} = temp(2:end-1);

temp =  controller.F_gI{2};
controller.F_gI{2} = temp(2:end-1);

temp = controller.F_U_Optimal_id;
controller.F_U_Optimal_id = temp(2:end-1,2:end-1);

% refine boundaries (M)
temp =  controller.M_gI_J3{1};
controller.M_gI_J3{1} = temp(2:end-1);

temp =  controller.M_gI_J3{2};
controller.M_gI_J3{2} = temp(2:end-1);

temp = controller.M_U_Optimal_id_J3;
controller.M_U_Optimal_id_J3 = temp(2:end-1,2:end-1);

% %create surfaces
F_controller = griddedInterpolant(controller.F_gI,...
    single(controller.v_Fthruster(controller.F_U_Optimal_id)), 'linear','nearest');

M_controller_J1 = griddedInterpolant(controller.M_gI_J1,...
    single(controller.v_Mthruster(controller.M_U_Optimal_id_J1)), 'linear','nearest');
M_controller_J2 = griddedInterpolant(controller.M_gI_J2,...
    single(controller.v_Mthruster(controller.M_U_Optimal_id_J2)), 'linear','nearest');
M_controller_J3 = griddedInterpolant(controller.M_gI_J3,...
    single(controller.v_Mthruster(controller.M_U_Optimal_id_J3)), 'linear','nearest');

%plots
figure

XF1 = repmat(controller.F_gI{1}', [1 length(controller.F_gI{2})]);
XF2 = repmat(controller.F_gI{2}, [length(controller.F_gI{1}) 1]);

mesh(XF1,XF2,F_controller.Values)
title('F_optimal')
%calculations
percentage_F = (sum(F_controller.Values(:) > 0.25) +...
    sum(F_controller.Values(:) < -0.25)) / numel(F_controller.Values);
unq_F = unique(F_controller.Values(:));



%plots (M)
figure

XM1 = repmat(controller.M_gI_J3{1}', [1 length(controller.M_gI_J3{2})]);
XM2 = repmat(controller.M_gI_J3{2}, [length(controller.M_gI_J3{1}) 1]);

mesh(XM1,XM2,M_controller_J3.Values)
title('M_optimal')
%calculations
unq_M3 = unique(M_controller_J3.Values(:));
max_M3 = unq_M3(end-2);
percentage_M3 = (sum(M_controller_J3.Values(:) > max_M3) +...
    sum(M_controller_J3.Values(:) < -max_M3)) / numel(M_controller_J3.Values);



