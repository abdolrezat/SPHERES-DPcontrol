function plot_compare_DPlyap_3initial_positions(obj, LQ, obj2,LQ2, obj3, LQ3)

set(0,'DefaultAxesFontName','Georgia')
T_ode45 = obj.history.T_ode45;
Force_Moment_log_Body = obj.history.Force_Moment_log_Body;
X_ode45 = obj.history.X_ode45;
F_Th_Opt = obj.history.F_Th_Opt;
Force_Moment_log_req = obj.history.Force_Moment_log_req;

%% default styles
gridAlpha = 0.3;
gridLineStyle = ':';

LineWidth_u = 0.1;
col_u = 0.1;

LineWidth_F = 0.1;
col_F = 0.08;

style_1 = '-';
style_2 = '-';
style_3 = '--';

LineWidth_1 = 3.5;
LineWidth_2 = 1.0;
LineWidth_3 = 2.0;

col_1 = 0.82;
col_2 = 0.4;
col_3 = 0.13;

style_1_v = '-';
LineWidth_1_v = 3.4;
col_1_v = 0.8;
style_2_v = '-';
LineWidth_2_v = 1.5;
col_2_v = 0.2;
style_3_v = '-';
LineWidth_3_v = 0.6;
col_3_v = 0.15;


style_1_angle = '-';
style_2_angle = '--';
style_3_angle = '-';

LineWidth_1_angle = 3.5;
LineWidth_2_angle = 1.6;
LineWidth_3_angle = 1.1;

col_1_angle = 0.8;
col_2_angle = 0.15;
col_3_angle = 0.2;

style_2_w = '-';
LineWidth_2_w = 0.6;
col_2_w = 0.15;
style_3_w = '-';
LineWidth_3_w = 1.5;
col_3_w = 0.2;

%% calculate fuel consumption
Thr_force = max(F_Th_Opt(:));
FC_history = cumsum(F_Th_Opt)/Thr_force;
FC_total = sum(FC_history(end,:));
fprintf('Total Thruster-On Time (Fuel Consumption) = %.3f seconds\n', FC_total*obj.h)

%% calculate Quadratic Cost function
try %#ok<*TRYNC>
    Qx = obj.controller_params.Qx;
    Qv = obj.controller_params.Qv;
    Qt = obj.controller_params.Qt;
    Qw = obj.controller_params.Qw;
    R = obj.controller_params.R;
    
    X = obj.history.X_ode45;
    U = obj.history.F_Th_Opt;
    costx = sum( sum( X(:,1:3).^2 ))*Qx;
    costv = sum( sum( X(:,4:6).^2 ))*Qv;
    costu = sum( U(:))*R;
    total_cost =  costx + costv + costu;
    
    fprintf('costX: %.2g | costV: %.2g | costU: %.2g, Total = %.3g\n',...
        costx,costv,costu,total_cost)
end
%% plot path over control surface
% theta as angles
% [theta1q2a,theta2q2a,theta3q2a] = quat2angle(X_ode45(:,10:-1:7));
% single rotation thetas as  controller 'sees' them:
theta1 = obj.history.Theta_history(:,1)*180/pi;
theta2 = obj.history.Theta_history(:,2)*180/pi;
theta3 = obj.history.Theta_history(:,3)*180/pi;

diff_t1 = [0; diff(theta1)]/obj.h;
diff_t2 = [0; diff(theta2)]/obj.h;
diff_t3 = [0; diff(theta3)]/obj.h;
diff_t1(abs(diff_t1) > 100) = NaN;
diff_t2(abs(diff_t2) > 100) = NaN;
diff_t3(abs(diff_t3) > 100) = NaN;

if(0) %skip
    
    x1 = X_ode45(:,1);
    v1 = X_ode45(:,4);
    F1 = Force_Moment_log(:,1)*obj.Mass;t3 = theta3*180/pi;
    w3 = X_ode45(:,13);
    M3 = Force_Moment_log(:,6);
    if(~strcmp(obj.controller_params.type,'PID'))
        [axF,axM] = test_surface(obj.current_controller);
        axes(axF.Parent)
        % colormap('winter')
        hold on, plot3(x1,v1,F1,'m:', 'LineWidth',2.0);
        
        axes(axM.Parent)
        % colormap('winter')
        hold on, plot3(t3,w3,M3,'m:', 'LineWidth',2.0);
    end
end

%% plot Thruster Firings
ylim_thr = [-0.01 .12];
pos_fig_thruster = [128.6,183.8,1127.6,542.8];
figure('Name','Thruster Firings','Position',pos_fig_thruster,...
    'color', 'white')
grid off

for i=1:12
    subplot(4,3,i)
    plot(T_ode45, F_Th_Opt(:,i),'Color', [col_u col_u col_u], 'LineWidth', LineWidth_u)
    %         title(strcat('u_{',num2str(i),'}'))
    % ylabel(,'FontAngle','italic','Interpreter','Latex')
    ylim(ylim_thr)
    yticks([0 0.12])
    yticklabels({'Off','On'})
    
    set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
    title(strcat('$\mathrm{\underline{u_{',num2str(i),'}}}$'), 'FontSize', 14, 'Interpreter', 'latex')
    
end

% lyapunov
pos_fig_thruster = [128.6,183.8,1127.6,542.8]+10;
figure('Name','Thruster Firings Lyapunov','Position',pos_fig_thruster,...
    'color', 'white')
grid off
thr_lyap = LQ.thr_hist.Data*0.12;
time_lyap = LQ.thr_hist.Time;
for i=1:12
    subplot(4,3,i)
    plot(time_lyap, thr_lyap(:,i),'Color', [col_u col_u col_u], 'LineWidth', LineWidth_u)
    %         title(strcat('u_{',num2str(i),'}'))
    % ylabel(,'FontAngle','italic','Interpreter','Latex')
    ylim(ylim_thr)
    yticks([0 0.12])
    yticklabels({'Off','On'})
    
    set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
    title(strcat('$\mathrm{\underline{u_{',num2str(i),'}}}$'), 'FontSize', 14, 'Interpreter', 'latex')
    
end

%
%% plot Forces and Moments
ylim_F = [-0.26 0.24];
ylim_M = [-0.26 0.24]*obj.T_dist;
pos_fig_FM = pos_fig_thruster;


% Forces
figure('Name','Forces','Position',pos_fig_FM,...
    'color', 'white')
F_label = {'x', 'y', 'z'};
for i=1:3
    subplot(3,1,i)
    plot(T_ode45, Force_Moment_log_Body(:,i)*obj.Mass, 'Color', [col_F col_F col_F], 'LineWidth', LineWidth_F)
    %         title('acceleration x-direction')
    grid off
    ylim(ylim_F)
    %         ylabel(strcat('F_{',num2str(i),'} [N]'))
    ylabel(strcat('${\mathrm{F}^\mathrm{B}_\mathrm{',F_label{i},'}}\:[N]$'), 'FontSize', 13, 'Interpreter', 'latex')
    yticks([-0.24 -0.12 0 0.12 0.24])
    yticklabels({'-0.24', '-0.12', '0', '0.12', '0.24'})
    
    set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
end

xlabel('Time [sec]')

% Lyap
F_M_lyap = LQ.F_M_out.Data ;
figure('Name','Forces - Lyapunov','Position',pos_fig_FM,...
    'color', 'white')
F_label = {'x', 'y', 'z'};
for i=1:3
    subplot(3,1,i)
    plot(time_lyap, F_M_lyap(:,i), 'Color', [col_F col_F col_F], 'LineWidth', LineWidth_F)
    %         title('acceleration x-direction')
    grid off
    ylim(ylim_F)
    %         ylabel(strcat('F_{',num2str(i),'} [N]'))
    ylabel(strcat('${\mathrm{F}^\mathrm{B}_\mathrm{',F_label{i},'}}\:[N]$'), 'FontSize', 13, 'Interpreter', 'latex')
    yticks([-0.24 -0.12 0 0.12 0.24])
    yticklabels({'-0.24', '-0.12', '0', '0.12', '0.24'})
    
    set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
end

xlabel('Time [sec]')



% Moments
figure('Name','Moments','Position',pos_fig_FM,...
    'color', 'white')

yticks_v = round([-0.24 -0.12 0 0.12 0.24]*obj.T_dist,2);
for i=1:3
    subplot(3,1,i)
    plot(T_ode45, Force_Moment_log_Body(:,i+3), 'Color', [col_F col_F col_F], 'LineWidth', LineWidth_F)
    %         title('acceleration x-direction')
    grid off
    ylim(ylim_M)
    ylabel(strcat('${\mathrm{M}^\mathrm{B}_\mathrm{',num2str(i),'}}\:[N s]$'), 'FontSize', 13, 'Interpreter', 'latex')
    yticks(yticks_v)
    yticklabels(num2cell(yticks_v))
    
    set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
end

xlabel('Time [sec]')

% Lyap
figure('Name','Moments - Lyap','Position',pos_fig_FM,...
    'color', 'white')

yticks_v = round([-0.24 -0.12 0 0.12 0.24]*obj.T_dist,2);
for i=1:3
    subplot(3,1,i)
    plot(time_lyap, F_M_lyap(:,i+3), 'Color', [col_F col_F col_F], 'LineWidth', LineWidth_F)
    %         title('acceleration x-direction')
    grid off
    ylim(ylim_M)
    ylabel(strcat('${\mathrm{M}^\mathrm{B}_\mathrm{',num2str(i),'}}\:[N s]$'), 'FontSize', 13, 'Interpreter', 'latex')
    yticks(yticks_v)
    yticklabels(num2cell(yticks_v))
    
    set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
end

xlabel('Time [sec]')


%% plot states - pos
states_lyap = LQ.states_out.Data;
pos_fig_x = [611.4000  100.2000  936.0000  676.4000];
labels1 = {'X_{rel} [m]','Y_{rel} [m]','Z_{rel} [m]',...
    'V_{X_{rel}} [m/s]','V_{Y_{rel}} [m/s]','V_{Z_{rel}} [m/s]'};
figure('Position',pos_fig_x, 'color', 'white')
plot_ids = [1 3 5 2 4 6];
for ii=1:6
    
    subplot(3,2,plot_ids(ii))
    %         title('states - position (m)')
    
    %% only DP lyap
    %         plot(T_ode45, X_ode45(:,ii),style_1,'Color', [col_1 col_1 col_1], 'LineWidth', LineWidth_1)
    %         hold on
    %         plot(time_lyap, states_lyap(:,ii),style_3,'Color', [col_3 col_3 col_3], 'LineWidth', LineWidth_3)
    %              legend('Dynamic Programming', 'Lyapunov')
    
    %% DP lyap ref
    %baseline (10 m)
    plot(time_lyap, LQ.refmodel_out.Data(:,ii) ,style_1,'Color', [col_1 col_1 col_1], 'LineWidth', LineWidth_1)
    hold on
    plot(time_lyap, states_lyap(:,ii),style_2,'Color', [col_2 col_2 col_2], 'LineWidth', LineWidth_2)
    plot(T_ode45, X_ode45(:,ii),style_3,'Color', [col_3 col_3 col_3], 'LineWidth', LineWidth_3)
    legend('Reference', 'Lyapunov','Dynamic Programming')
    
    %case 2 ( 5 m )
    plot(LQ2.states_out.Time, LQ2.refmodel_out.Data(:,ii) ,style_1,'Color', [col_1 col_1 col_1], 'LineWidth', LineWidth_1)
    hold on
    plot(LQ2.states_out.Time, LQ2.states_out.Data(:,ii),style_2,'Color', [col_2 col_2 col_2], 'LineWidth', LineWidth_2)
    plot(obj2.history.T_ode45, obj2.history.X_ode45(:,ii),style_3,'Color', [col_3 col_3 col_3], 'LineWidth', LineWidth_3)
    legend('Reference', 'Lyapunov','Dynamic Programming')
    
    %case 3 ( 15 m )
    plot(LQ3.states_out.Time, LQ3.refmodel_out.Data(:,ii) ,style_1,'Color', [col_1 col_1 col_1], 'LineWidth', LineWidth_1)
    hold on
    plot(LQ3.states_out.Time, LQ3.states_out.Data(:,ii),style_2,'Color', [col_2 col_2 col_2], 'LineWidth', LineWidth_2)
    plot(obj3.history.T_ode45, obj3.history.X_ode45(:,ii),style_3,'Color', [col_3 col_3 col_3], 'LineWidth', LineWidth_3)
    legend('Reference', 'Lyapunov','Dynamic Programming')
    
    grid on
    xlabel('Time [sec]')
    ylabel(strcat( labels1{ii}) )
    set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha)
    
end


%% plot - angles
eul = quat2eul(X_ode45(:,10:-1:7));
eul_lyap = quat2eul(flip(mrp2quat(LQ.states_out.Data(:,7:9)'))');
eul_ref =  quat2eul(flip(mrp2quat(LQ.refmodel_out.Data(:,7:9)'))');

eul_w_DP = [eul, X_ode45(:,11:13)]*180/pi;
eul_w_lyap = [eul_lyap, states_lyap(:,10:12)]*180/pi;
eul_w_ref = eul_ref *180/pi;
%% compare and plot states - angles and rotational speeds
pos_fig_w = [43.4000   86.6000  936.0000  676.4000];

labels1 = {'\phi [deg]','\theta [deg]','\psi [deg]',...
    '\omega_1 [deg/s]','\omega_2 [deg/s]','\omega_3 [deg/s]'};

figure('Position',pos_fig_w, 'color', 'white')
plot_ids = [1 3 5 2 4 6];

for ii=1:6
    
    subplot(3,2,plot_ids(ii))
    %         title('states - position (m)')
    
    %% only DP lyap
    % plot(T_ode45, eul_w_DP(:,ii),style_1,'Color', [col_1 col_1 col_1], 'LineWidth', LineWidth_1)
    %         hold on
    %         plot(time_lyap, eul_w_lyap(:,ii),style_3,'Color', [col_3 col_3 col_3], 'LineWidth', LineWidth_3)
    %
    %         legend('Dynamic Programming', 'Lyapunov')
    %
    if(ii < 4)
        %% DP lyap ref
        plot(time_lyap, eul_w_ref(:,ii) ,style_1,'Color', [col_1 col_1 col_1], 'LineWidth', LineWidth_1)
        hold on
        plot(time_lyap, eul_w_lyap(:,ii),style_2,'Color', [col_2 col_2 col_2], 'LineWidth', LineWidth_2)
        plot(T_ode45, eul_w_DP(:,ii),style_3,'Color', [col_3 col_3 col_3], 'LineWidth', LineWidth_3)
        legend('Reference', 'Lyapunov','Dynamic Programming')
    else
        %% only DP lyap
        plot(time_lyap, eul_w_lyap(:,ii),style_2,'Color', [col_2 col_2 col_2], 'LineWidth', LineWidth_2)
        hold on
        plot(T_ode45, eul_w_DP(:,ii),style_3,'Color', [col_3 col_3 col_3], 'LineWidth', LineWidth_3)

        legend('Lyapunov', 'Dynamic Programming')
        
    end
    grid on
    xlabel('Time [sec]')
    ylabel(strcat( labels1{ii}) )
    set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha)
    
end


end