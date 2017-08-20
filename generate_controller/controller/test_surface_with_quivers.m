function test_surface_with_quivers(filename)
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
UF = F_controller.Values;close all

[x_next,v_next] = next_stage_states_simplified(XF1, XF2, UF, 0.005 ,4.16);
quiv_x = x_next - XF1;
quiv_v = v_next - XF2;


axF = mesh(XF1,XF2,UF);
hold on
quiver3(XF1,XF2,UF,quiv_x,quiv_v,quiv_v*0)

axF.Parent.View= [0 90];
title('Optimal Force from Position Controller')
xlabel('position (x)')
ylabel('velocity (v)')
zlabel('Force (N)')
colormap('jet')
%calculations
percentage_F = (sum(F_controller.Values(:) > 0.25) +...
    sum(F_controller.Values(:) < -0.25)) / numel(F_controller.Values);
unq_F = unique(F_controller.Values(:));



%plots (M)
figure

XM1 = rad2deg(repmat(controller.M_gI_J3{1}', [1 length(controller.M_gI_J3{2})]));
XM2 = repmat(controller.M_gI_J3{2}, [length(controller.M_gI_J3{1}) 1]);

axM = mesh(XM1,XM2,M_controller_J3.Values);
axM.Parent.View= [0 90];
title('Optimal Moment from Attitude Controller')
xlabel('angle (\theta)')
ylabel('rotational speed (\omega)')
zlabel('Moment (N.m)')
colormap('jet')
%calculations
unq_M3 = unique(M_controller_J3.Values(:));
max_M3 = unq_M3(end-2);
percentage_M3 = (sum(M_controller_J3.Values(:) > max_M3) +...
    sum(M_controller_J3.Values(:) < -max_M3)) / numel(M_controller_J3.Values);

figure
contourf(XM1,XM2,M_controller_J3.Values,'ShowText','on')
end

function [x_next,v_next] = next_stage_states_simplified(X, V, f ,h ,Mass)

% ODE solve
x_next =  RK4_x( X, V, h) ;
v_next =  RK4_v( V, f, h, Mass);
end

function X2 = RK4_x( X1, V, h)
% Runge-Kutta - 4th order
% h = dt;
k1 = xdynamics( V);
k2 = xdynamics((V + k1*h/2));
k3 = xdynamics((V + k2*h/2));
k4 = xdynamics((V + k3*h));

X2 = X1 + h*(k1 + 2*k2 + 2*k3 + k4)/6;

end

function x_dot = xdynamics(v)
x_dot = v;
end

function V2 = RK4_v( V1, f, h, Mass) % does not need RK4, ki's are equal
% Runge-Kutta - 4th order
% h = dt;
k1 = vdynamics( V1 , f, Mass);
k2 = vdynamics((V1 + k1*h/2), f, Mass);
k3 = vdynamics((V1 + k2*h/2), f, Mass);
k4 = vdynamics((V1 + k3*h), f, Mass);

V2 = V1 + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end

function v_dot = vdynamics( ~, f, Mass)
v_dot = (f)/Mass;
end
