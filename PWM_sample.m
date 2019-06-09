function PWM_sample()
%PWM_SAMPLE Summary of this function goes here
%   Detailed explanation goes here
u_on = 0.12;
u_eff = u_on/5;

t = 0:0.001:4.99;
t_id = floor(t);
u  = linspace(0,u_eff,length(unique(t_id)));
[~,max_u_id] = max(u);
u(max_u_id) = u_eff;

U_DP = t*0;
U_THR = U_DP;
t_on = u/u_eff*0.2 ;


for i = 1:length(t)
    
    U_DP(i) = u(t_id(i)+1);
    
    if(t(i) < t_on(t_id(i)+1) + t_id(i))
       U_THR(i) = u_on;
    end
    
end

figure('color','white')
subplot(2,1,1)
plot(t,U_DP,'k--')
area(t,U_DP,'FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.5)
gridAlpha = 0.3;
gridLineStyle = ':';
grid off
set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
ylim_thr = [-0.001,0.12/5];
ylim(ylim_thr)
yticks([0 0.12/5])
yticklabels({'U_{Off}','U_{max}'})
xticks([0:5])
ylabel('u_{command}')

hold on
subplot(2,1,2)
plot(t,U_THR,'k-')
ylim_thr = [-0.005,0.121];
ylim(ylim_thr)
yticks([0 0.12])
yticklabels({'U_{Off}','U_{On}'})
xticks([0:5])
set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
xlabel('Time [sec]')
xlim([0 5])
ylabel('Modulated u_{actual}')

figure('color','white')
% plot(t,U_DP,'k--')
area(t,U_DP,'LineStyle','--','FaceColor',[0.5,0.5,0.5],'FaceAlpha',0.3)
gridAlpha = 0.3;
gridLineStyle = ':';
grid off
set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
ylim_thr = [-0.002, 0.12];
ylim(ylim_thr)
yticks([0 0.12/5 0.12])
yticklabels({'U_{Off}','U_{max}','U_{on}'})
xticks([0:5])

hold on
% plot(t,U_THR,'k-')
area(t,U_THR,'FaceColor',[0.3,0.3,0.3],'FaceAlpha',0.5)
legend({'u_{control}','u_{actual}'})

set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 9)
xlabel('Time [sec]')
xlim([0 5])

end

