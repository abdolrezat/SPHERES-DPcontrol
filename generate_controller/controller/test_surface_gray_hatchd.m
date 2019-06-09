function [axF,axM] = test_surface_gray_hatchd(filename)
%%%% newest plot function with hashes that cause no confusion in the paper
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

% XF1 = repmat(controller.Fx_gI{1}', [1 length(controller.Fx_gI{2})]);
% XF2 = repmat(controller.Fx_gI{2}, [length(controller.Fx_gI{1}) 1]);
% dF = F_controller.Values ;
max_s1 = max(controller.Fx_gI{1});
max_s2 = max(controller.Fx_gI{2});
n_plot = 1000;
s1 = linspace(-max_s1, max_s1, n_plot);
s2 = linspace(-max_s2, max_s2, n_plot);
XF1 = repmat(s1', [1 n_plot]);
XF2 = repmat(s2, [n_plot 1]);
dF = F_controller(XF1, XF2) ;

maxF = max(dF(:));
id1 = (dF < maxF & dF > -maxF);


XF1_deadzone = XF1;
XF1_deadzone(~id1) = NaN;

XF2_deadzone = XF2;
XF2_deadzone(~id1) = NaN;

dF_deadzone = dF;
dF_deadzone(~id1) = NaN;

axF = mesh(XF1_deadzone,XF2_deadzone,dF_deadzone);
axF.Parent.View= [0 90];

hold on
surf(XF1,XF2,XF1*0 -maxF,'EdgeColor','none','FaceColor', [0.2 0.2 0.2] );
% title('Optimal Force from Position Controller')
xlabel('X [m]')
ylabel('V [m/s]')
zlabel('Force (N)')
colormap(custom_colormap_gray_light(length(unique(dF))));
grid off
axis('tight')


%calculations
percentage_F = (sum(F_controller.Values(:) > 0.25) +...
    sum(F_controller.Values(:) < -0.25)) / numel(F_controller.Values);
unq_F = unique(F_controller.Values(:));
set(gca,'TickDir','out', 'Box', 'off','FontSize',13)



plot3([0],[0],+10000,'kx','MarkerSize',6)


n_hatch1 = 30;
hatch_col1 = repmat(0.25,[1 3]);
hatch_LW1 = 1;

n_hatch2 = 100;
hatch_col2 = repmat(0.35,[1 3]);
hatch_LW2 = 2.0;

[h1,h2] = hatch_3(s1, s2, n_hatch1);
id_min = (F_controller(h1,h2) > -maxF);
h1(id_min) = NaN;
h2(id_min) = NaN;
plot(h1,h2,'LineWidth',hatch_LW1,'Color',hatch_col1)

% 
% [h1m,h2m] = hatch_2(s1, s2,n_hatch1);
% id_max = (F_controller(h1m,h2m) > -maxF);
% h1m(id_max) = NaN;
% h2m(id_max) = NaN;
% plot(h1m,h2m,'LineWidth',hatch_LW1,'Color',hatch_col1)

[b1,b2] = hatch_2(s1,s2,n_hatch2);
id_max = (F_controller(b1,b2) < maxF);
b1(id_max) = NaN;
b2(id_max) = NaN;
plot(b1,b2,'--','LineWidth',hatch_LW2,'Color',hatch_col2)
% 
% [b1,b2] = hatch_2(s1,s2,n_hatch2);
% id_max = (F_controller(b1,b2) < maxF);
% b1(id_max) = NaN;
% b2(id_max) = NaN;
% plot(b1,b2,'LineWidth',hatch_LW2,'Color',hatch_col2)

c1 = colorbar('Ticks',[-maxF,0,maxF]*0.98,...
         'TickLabels',{'-F_{max}','0','+F_{max}'},'Direction','reverse');

     
%plots (M)
f2 = figure;
set(f2, 'color', 'white');

XM1 = rad2deg(repmat(controller.M_gI_J3{1}', [1 length(controller.M_gI_J3{2})]));
XM2 = repmat(controller.M_gI_J3{2}, [length(controller.M_gI_J3{1}) 1]);

max_s1 = rad2deg(max(controller.M_gI_J3{1}));
max_s2 = max(controller.M_gI_J3{2});
n_plot = 1000;
s1 = (linspace(-max_s1, max_s1, n_plot));
s2 = linspace(-max_s2, max_s2, n_plot);
XM1 = repmat(s1', [1 n_plot]);
XM2 = repmat(s2, [n_plot 1]);
dM = M_controller_J3(deg2rad(XM1), XM2) ;

maxM = max(dM(:));
id1 = (dM < maxM & dM > -maxM);


XM1_deadzone = XM1;
XM1_deadzone(~id1) = NaN;

XM2_deadzone = XM2;
XM2_deadzone(~id1) = NaN;

dM_deadzone = dM;
dM_deadzone(~id1) = NaN;


axM = mesh(XM1_deadzone,XM2_deadzone,dM_deadzone);
axM.Parent.View= [0 90];
% title('Optimal Moment from Attitude Controller')
xlabel('\theta_3 [deg]')
ylabel('\omega_3 [rad/s]')
zlabel('Moment (N.m)')
colormap(custom_colormap_gray_light(length(unique(dM))));
grid off
axis('tight')
%calculations
% unq_M3 = unique(M_controller_J3.Values(:));
% max_M3 = unq_M3(end-2);
% percentage_M3 = (sum(M_controller_J3.Values(:) > max_M3) +...
%     sum(M_controller_J3.Values(:) < -max_M3)) / numel(M_controller_J3.Values);
set(gca,'TickDir','out', 'Box', 'off','FontSize',13)
hold on
plot3([-360,0,360],[0 0 0],[0 0 0]+10000,'kx','MarkerSize',6)
% figure
% contourf(XM1,XM2,M_controller_J3.Values,'ShowText','on')
xticks([-360,-180,0,180,360])

%%%%

surf(XM1,XM2,XM1*0 -maxM,'EdgeColor','none','FaceColor', [0.2 0.2 0.2] );

[h1,h2] = hatch_3(s1, s2, n_hatch1);
id_min = (M_controller_J3(deg2rad(h1),h2) > -maxM);
h1(id_min) = NaN;
h2(id_min) = NaN;
plot(h1,h2,'LineWidth',hatch_LW1,'Color',hatch_col1)

% 
% [h1m,h2m] = hatch_2(s1, s2,n_hatch1);
% id_max = (F_controller(h1m,h2m) > -maxF);
% h1m(id_max) = NaN;
% h2m(id_max) = NaN;
% plot(h1m,h2m,'LineWidth',hatch_LW1,'Color',hatch_col1)

[b1,b2] = hatch_2(s1,s2,n_hatch2);
id_max = (M_controller_J3(deg2rad(b1),b2) < maxM);
b1(id_max) = NaN;
b2(id_max) = NaN;
plot(b1,b2,'--','LineWidth',hatch_LW2,'Color',hatch_col2)

c1 = colorbar('Ticks',[-maxM,0,maxM]*0.98,...
         'TickLabels',{'-M_{max}','0','+M_{max}'},'Direction','reverse');



function [X,Y] = hatch_3(s1, s2, p)
L_1 = 1:length(s1);
Y = [];
X = [];
max_di = length(s2) + length(s1);
for di = max_di:-p:1
    line_temp = (-L_1 + di);
    ids = line_temp > 0 & line_temp <= length(s2) ;
    line_pos = line_temp(ids);
    xline = L_1(ids);
    Y = [Y, NaN, s2(line_pos)];
    X = [X, NaN, s1(xline)];
end


function [X,Y] = hatch_2(s1, s2, p)
L_1 = 1:length(s1);
Y = [];
X = [];
max_di = length(s2) ;
for di = -max_di:p:max_di
    line_temp = (L_1 + di);
    ids = line_temp > 0 & line_temp <= length(s2) ;
    line_pos = line_temp(ids);
    xline = L_1(ids);
    Y = [Y, NaN, s2(line_pos)];
    X = [X, NaN, s1(xline)];
end

function [X,Y] = hatch_ver(s1, s2, p)
L_1 = 1:length(s1);
Y = [];
X = [];
for di = 1:p:length(s1)
    Y = [Y, NaN, s2];
    X = [X, NaN, repmat(s1(di),[1 length(s2)])];
end

function [X,Y] = hatch_hor(s1, s2, p)
L_1 = 1:length(s1);
Y = [];
X = [];
for di = 1:p:length(s2)
    Y = [Y, NaN, repmat(s2(di),[1 length(s1)])];
    X = [X, NaN, s1];
end

function colmap_custom = custom_colormap_gray_light(l_colmap)
%CUSTOM_COLORMAP generates a gray custom colormap
%    generates black/gray and white colormap for publication figures

dkgry = 220/255;
ltgry1 = 50/255;
ltgry2 = 50/255;
colmap_custom = repmat(interp1([1,floor(l_colmap/2),l_colmap],...
    [ltgry1,dkgry,ltgry2],1:l_colmap)',[1 3]);

