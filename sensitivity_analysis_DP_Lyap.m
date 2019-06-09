function sensitivity_analysis_DP_Lyap

if ~exist('data_sensitivity_analysis.mat')
    generate_sensitivity_analysis_DP_Lyap()
end

plot_data_sensitivity
end

function plot_data_sensitivity
gridAlpha = 0.3;
gridLineStyle = ':';
%set(0,'DefaultAxesFontName','Georgia')
Data = load('data_sensitivity_analysis.mat');
r = Data.r;
styleDP = 'o--';
styleLyap = 's--';
lwDP = 1.2;
lwLyap = 0.8;
colDP = 0.15* ones(1,3);
colLyap = 0.4* ones(1,3);

% plot fuel vs R

 figure('Name','fuel',...
    'color', 'white')
plot(r, Data.Dynamic_Programming.impulse, styleDP, 'Color', colDP, 'LineWidth', lwDP);
hold on
grid on
plot(r, Data.Lyapunov.impulse, styleLyap, 'Color', colLyap, 'LineWidth', lwLyap)
ylabel('Impulse [N s]')
xlabel('Initial Position [m]')
        set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 11)
legend('Dynamic Programming','Lyapunov')
 
%% impulse ratio
figure('Name','fuel',...
    'color', 'white')
plot(r, Data.Lyapunov.impulse./Data.Dynamic_Programming.impulse, '--^', 'Color', colDP, 'LineWidth', lwDP);
grid on
ylabel('Fuel Consumption Ratio')
xlabel('Initial Position [m]')
        set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 11)

%%
 figure('Name','ts',...
    'color', 'white')
plot(r, Data.Dynamic_Programming.max_ts, styleDP, 'Color', colDP, 'LineWidth', lwDP);
hold on
grid on

plot(r, Data.Lyapunov.max_ts, styleLyap, 'Color', colLyap, 'LineWidth', lwLyap)
ylabel('Settling Time [sec]')
xlabel('Initial Position [m]')
        set(gca, 'TickDir','out', 'Box', 'off', 'GridLineStyle', gridLineStyle, 'GridAlpha', gridAlpha,'FontSize', 11)
legend('Dynamic Programming','Lyapunov')

end

function generate_sensitivity_analysis_DP_Lyap()
r = 15:-1:5;

% run DP
for ii = 1:length(r) %0.5:0.5:15
    dr0 = [r(ii), r(ii), r(ii)];
    S = simulation_normal_DP_for_sensitivity(dr0);
    max_ts(ii) = get_max_settling_time(S);
%     impulse(ii) = get_fuel_consumption(S);
    impulse(ii) = get_fuel_consumption_until_ts(S, max_ts(ii));

end
Dynamic_Programming.max_ts = max_ts;
Dynamic_Programming.impulse = impulse;

% run Lyap
load_system('SPHERES_lyapnov_model_pulse_modulation_for_Sensitivity'); %load up
set_param('SPHERES_lyapnov_model_pulse_modulation_for_Sensitivity', 'StopTime', num2str(S.history.T_ode45(end)))

for ii = 1:length(r) %0.5:0.5:15
    LQ = run_Lyapunov_with_initial_pos(r(ii));
    max_ts_l(ii) = get_max_settling_time(LQ);
%     impulse_l(ii) = get_fuel_consumption(LQ);
    impulse_l(ii) = get_fuel_consumption_until_ts(LQ, max_ts_l(ii));
end
Lyapunov.max_ts = max_ts_l;
Lyapunov.impulse = impulse_l;

save('data_sensitivity_analysis.mat','r','Lyapunov','Dynamic_Programming')



end


function SC = simulation_normal_DP_for_sensitivity(dr0)
controller.name = 'controller_normal_DynamicProgramming_c'; %name of controller, will be saved under /controller directory

simulator_opts.current_controller = controller.name;
simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
simulator_opts.thruster_allocation_mode = 'quadratic programming pulse modulation';

simulator_opts.Thruster_max_F = 0.12; % (N)
simulator_opts.h = 0.01; %simulation fixed time steps
simulator_opts.T_final = 300; %simulation Tfinal

dv0 = [0 0 0]; %initial relative velocity offset
q0 = flip(angle2quat(deg2rad(30),deg2rad(30),deg2rad(-30))); %initial angles offset (yaw,pitch,roll)
w0 = [0 0 0]; %initial rotational speed offset
simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';

Thruster_dist = (9.65E-2); % (meters)
%simulator variables
simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative

simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
simulator_opts.Thruster_dist = Thruster_dist;

% create and run the simulator
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path;
end


function max_settle = get_max_settling_time(obj)
X1 = stepinfo(obj.history.X_ode45(:,1),obj.history.T_ode45,0);
X2 = stepinfo(obj.history.X_ode45(:,2),obj.history.T_ode45,0);
X3 = stepinfo(obj.history.X_ode45(:,3),obj.history.T_ode45,0);

max_settle = max([X1.SettlingTime, X2.SettlingTime, X3.SettlingTime]);
end

function impulse = get_fuel_consumption(obj)

F_Th_Opt = obj.history.F_Th_Opt;
%% calculate fuel consumption
Thr_force = max(F_Th_Opt(:));
FC_history = cumsum(F_Th_Opt)/Thr_force;
FC_total = sum(FC_history(end,:));
impulse = FC_total*obj.h*obj.Thruster_max_F;
end


function impulse = get_fuel_consumption_until_ts(obj, max_ts)
% calculate fuel consumption up to settling time +10

F_Th_Opt = obj.history.F_Th_Opt(obj.history.T_ode45 < max_ts + 10,:);
%% calculate fuel consumption
Thr_force = max(F_Th_Opt(:));
FC_history = cumsum(F_Th_Opt)/Thr_force;
FC_total = sum(FC_history(end,:));
impulse = FC_total*obj.h*obj.Thruster_max_F;
end

function LQ = run_Lyapunov_with_initial_pos(r)
%%%%test block
xm0 = r;
ym0 = r;
zm0 = r;
RM.p_dot0 = [0,0,0];
RM.angles0 = deg2rad([30,30,-30]);
RM.w0 = deg2rad([ 0,0,0]);
RM.p0 = [xm0 ym0 zm0];
RM.q0 = flip(angle2quat(RM.angles0(1),RM.angles0(2),RM.angles0(3))); %MRPs
RM.s0 = RM.q0(1:3)/(1+RM.q0(4)); %verify MRP

init_states = [RM.p0, RM.p_dot0, RM.s0, RM.w0];

set_param('SPHERES_lyapnov_model_pulse_modulation_for_Sensitivity/Integrator', 'InitialCondition',strcat('[',num2str(init_states),']'))
set_param('SPHERES_lyapnov_model_pulse_modulation_for_Sensitivity/reference model/reference model - translational/Integrator1', 'InitialCondition',strcat('[',num2str(RM.p0),']'))
sim('SPHERES_lyapnov_model_pulse_modulation_for_Sensitivity');
LQ.h = 0.01;
LQ.Thruster_max_F = 0.12;
LQ.history.F_Th_Opt = thr_hist.Data;
LQ.history.T_ode45 = thr_hist.Time;
LQ.history.X_ode45 = states_out.Data;
end