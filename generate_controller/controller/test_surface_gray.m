function [axF,axM] = test_surface_gray(filename)
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
if nargin == 0
    filename = 'controller_DP_attposition';
end

    path_ = strsplit(mfilename('fullpath'),'\\');
    path_ = strjoin(path_(1:end-1),'\');

    controller = load(strcat(path_,'\',filename));

    
% refine boundaries
temp =  controller.Fx_gI{1};
controller.Fx_gI{1} = temp(2:end-1);

temp =  controller.Fx_gI{2};
controller.Fx_gI{2} = temp(2:end-1);

temp = controller.Fx_U_Optimal_id;
controller.Fx_U_Optimal_id = temp(2:end-1,2:end-1);

% refine boundaries (M)
temp =  controller.M_gI_J3{1};
controller.M_gI_J3{1} = temp(2:end-1);

temp =  controller.M_gI_J3{2};
controller.M_gI_J3{2} = temp(2:end-1);

temp = controller.M_U_Optimal_id_J3;
controller.M_U_Optimal_id_J3 = temp(2:end-1,2:end-1);

% %create surfaces
F_controller = griddedInterpolant(controller.Fx_gI,...
    single(controller.v_Fthruster_x(controller.Fx_U_Optimal_id)), 'linear','nearest');

% M_controller_J1 = griddedInterpolant(controller.M_gI_J1,...
%     single(controller.v_Mthruster(controller.M_U_Optimal_id_J1)), 'linear','nearest');
% M_controller_J2 = griddedInterpolant(controller.M_gI_J2,...
%     single(controller.v_Mthruster(controller.M_U_Optimal_id_J2)), 'linear','nearest');
M_controller_J3 = griddedInterpolant(controller.M_gI_J3,...
    single(controller.v_Mthruster_z(controller.M_U_Optimal_id_J3)), 'linear','nearest');


%plots
f1 = figure;
set(f1, 'color', 'white');

XF1 = repmat(controller.Fx_gI{1}', [1 length(controller.Fx_gI{2})]);
XF2 = repmat(controller.Fx_gI{2}, [length(controller.Fx_gI{1}) 1]);

axF = mesh(XF1,XF2,F_controller.Values);
axF.Parent.View= [0 90];
% title('Optimal Force from Position Controller')
xlabel('X [m]')
ylabel('V [m/s]')
zlabel('Force (N)')
colormap(custom_colormap_gray(length(unique(F_controller.Values))));
grid off
axis('tight')
%calculations
percentage_F = (sum(F_controller.Values(:) > 0.25) +...
    sum(F_controller.Values(:) < -0.25)) / numel(F_controller.Values);
unq_F = unique(F_controller.Values(:));
set(gca,'TickDir','out', 'Box', 'off','FontSize',13)
hold on
plot3([0],[0],+10000,'kx','MarkerSize',6)


%plots (M)
f2 = figure;
set(f2, 'color', 'white');

XM1 = rad2deg(repmat(controller.M_gI_J3{1}', [1 length(controller.M_gI_J3{2})]));
XM2 = repmat(controller.M_gI_J3{2}, [length(controller.M_gI_J3{1}) 1]);

axM = mesh(XM1,XM2,M_controller_J3.Values);
axM.Parent.View= [0 90];
% title('Optimal Moment from Attitude Controller')
xlabel('\theta_3 [deg]')
ylabel('\omega_3 [rad/s]')
zlabel('Moment (N.m)')
colormap(custom_colormap_gray(length(unique(M_controller_J3.Values))));
grid off
axis('tight')
%calculations
unq_M3 = unique(M_controller_J3.Values(:));
max_M3 = unq_M3(end-2);
percentage_M3 = (sum(M_controller_J3.Values(:) > max_M3) +...
    sum(M_controller_J3.Values(:) < -max_M3)) / numel(M_controller_J3.Values);
set(gca,'TickDir','out', 'Box', 'off','FontSize',13)
hold on
plot3([-360,0,360],[0 0 0],[0 0 0]+10000,'kx','MarkerSize',6)
% figure
% contourf(XM1,XM2,M_controller_J3.Values,'ShowText','on')
xticks([-360,-180,0,180,360])
