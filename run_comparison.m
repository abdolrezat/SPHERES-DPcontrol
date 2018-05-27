function run_comparison()
%RUN_COMPARISON Summary of this function goes here
%   Detailed explanation goes here
SC = normal_DynamicProgramming_with_pulse_modulation_identical_ts;
close all
sim('SPHERES_lyapnov_model_pulse_modulation')

plot_compare(SC, states_out,thr_hist)

end

function plot_compare(SC, states_out,thr_hist)
t_lyapunov = states_out.Time;
t_DP = SC.history.T_ode45;
style_lyapunov = 'b-';
style_DP = 'r--';
style_refmodel = 'k-';
%% plot states
x_lyapunov = states_out.Data(:,1);
y_lyapunov = states_out.Data(:,2);
z_lyapunov = states_out.Data(:,3);
x_refmodel = refmodel_out.Data(:,1);
y_refmodel = refmodel_out.Data(:,2);
z_refmodel = refmodel_out.Data(:,3);


x_DP = SC.history.X_ode45(:,1);
y_DP = SC.history.X_ode45(:,2);
z_DP = SC.history.X_ode45(:,3);


figure('Name','states','Position',[7.4000   61.8000  538.4000  712.8000],...
    'color', 'white')
subplot(3,2,1)
plot(t_DP, x_DP, style_DP)
hold on
grid on
plot(t_lyapunov, x_lyapunov, style_lyapunov)
plot(t_lyapunov, x_refmodel, style_refmodel)
xlabel('time (sec)')
ylabel('X_{rel} (m)')
axis fill

subplot(3,2,3)
plot(t_DP, y_DP, style_DP)
hold on
grid on
plot(t_lyapunov, y_lyapunov, style_lyapunov)
xlabel('time (sec)')
ylabel('Y_{rel} (m)')
axis fill

subplot(3,2,5)
plot(t_DP, z_DP, style_DP)
hold on
grid on
plot(t_lyapunov, z_lyapunov, style_lyapunov)
xlabel('time (sec)')
ylabel('Z_{rel} (m)')
axis fill



figure('Name','Thruster Firings','Position',[27.4000   61.8000  508.4000  712.8000],...
    'color', 'white')

end