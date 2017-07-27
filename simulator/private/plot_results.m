function plot_results(obj, T_ode45, Force_Moment_log, X_ode45 )
%% plot Thruster Firings
%         ylim_thr = [-.15 .15];
%         pos_fig_thruster = [7.4000   61.8000  538.4000  712.8000];
%         figure('Name','Thruster Firings','Position',pos_fig_thruster)
%         subplot(4,3,1)
%         plot(T_ode45, F_Th_Opt(:,1))
%         title('#0 (x)')
%         grid on
%         ylim(ylim_thr)
%         %% xlim([-0.05 Inf])
%         
%         subplot(4,3,2)
%         plot(T_ode45, F_Th_Opt(:,3))
%         title('#2 (y)')    
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,3)
%         plot(T_ode45, F_Th_Opt(:,5))
%         title('#4 (z)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,4)
%         plot(T_ode45, F_Th_Opt(:,2))
%         title('#1 (x)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,5)
%         plot(T_ode45, F_Th_Opt(:,4))
%         title('#3 (y)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,6)
%         plot(T_ode45, F_Th_Opt(:,6))
%         title('#5 (z)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,7)
%         plot(T_ode45, F_Th_Opt(:,7))
%         title('#6 (-x)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,8)
%         plot(T_ode45, F_Th_Opt(:,9))
%         title('#8 (-y)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,9)
%         plot(T_ode45, F_Th_Opt(:,11))
%         title('#10 (-z)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,10)
%         plot(T_ode45, F_Th_Opt(:,8))
%         title('#7 (-x)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,11)
%         plot(T_ode45, F_Th_Opt(:,10))
%         title('#9 (-y)')
%         grid on
%         ylim(ylim_thr)
%         
%         subplot(4,3,12)
%         plot(T_ode45, F_Th_Opt(:,12))
%         title('#11 (-z)')
%         grid on
%         ylim(ylim_thr)
%   
%% 
        %plot Moments
        ylim_F = [-0.3 0.3]/obj.Mass;
        ylim_M = [-0.3 0.3]*obj.T_dist;
        pos_fig_FM = [547.4000  449.8000  518.4000  326.4000];
        figure('Name','Forces and Moments','Position',pos_fig_FM)
        
        subplot(3,2,1)
        plot(T_ode45, Force_Moment_log(:,1))
        title('acceleration x-direction')
        grid on
        ylim(ylim_F)
        
        subplot(3,2,3)
        plot(T_ode45, Force_Moment_log(:,2))
        title('acceleration y-direction')
        grid on
        ylim(ylim_F)
        
        subplot(3,2,5)
        plot(T_ode45, Force_Moment_log(:,3))
        title('acceleration z-direction')
        grid on
        ylim(ylim_F)
        
        subplot(3,2,2)
        plot(T_ode45, Force_Moment_log(:,4))
        title('Moment x-direction')
        grid on
        ylim(ylim_M)
        
        subplot(3,2,4)
        plot(T_ode45, Force_Moment_log(:,5))
        title('Moment y-direction')
        grid on
        ylim(ylim_M)
        
        subplot(3,2,6)
        plot(T_ode45, Force_Moment_log(:,6))
        title('Moment z-direction')
        grid on
        ylim(ylim_M)
        
        % plot states - pos
        pos_fig_x = [543.4000   49.0000  518.4000  326.4000];
        figure('Name','states - position','Position',pos_fig_x)
        
        plot(T_ode45, X_ode45(:,1))
        hold on
        plot(T_ode45, X_ode45(:,2))
        plot(T_ode45, X_ode45(:,3))
        grid on
        legend('x1','x2','x3')
        
        % plot states - v
        pos_fig_v = [954.6000  446.6000  518.4000  326.4000];
        figure('Name','states - velocity','Position',pos_fig_v)
        
        plot(T_ode45, X_ode45(:,4))
        hold on
        plot(T_ode45, X_ode45(:,5))
        plot(T_ode45, X_ode45(:,6))
        grid on
        legend('v1','v2','v3')
        
        % plot states - q
        pos_fig_q = [973.0000  218.6000  518.4000  326.4000];
        figure('Name','states - quaternions','Position',pos_fig_q)
        
        plot(T_ode45, X_ode45(:,7))
        hold on
        plot(T_ode45, X_ode45(:,8))
        plot(T_ode45, X_ode45(:,9))
        plot(T_ode45, X_ode45(:,10))
        grid on
        legend('q1','q2','q3','q4')
        
        % plot states - w
        pos_fig_w = [956.2000   47.4000  518.4000  326.4000];
        figure('Name','states - rotational speeds','Position',pos_fig_w)
        
        plot(T_ode45, X_ode45(:,11)*180/pi)
        title('states - rotational speeds (deg/sec)')

        hold on
        plot(T_ode45, X_ode45(:,12)*180/pi)
        plot(T_ode45, X_ode45(:,13)*180/pi)
        grid on
        legend('w1','w2','w3')
end