function simulations_no_fault()
% Parameters of the representative maneuver
dr0 = [-10 -10 -10]; %initial relative position offset
dv0 = [0 0 0]; %initial relative velocity offset
q0 = flip(angle2quat(deg2rad(20),deg2rad(30),deg2rad(-40))); %initial angles offset (yaw,pitch,roll)
w0 = [0 0 0]; %initial rotational speed offset
init_state = [dr0 dv0 q0 w0]';


% Dynamic Programming control + PWPF Modulator control allocation
sim1 = simulation_DP_PWPF(init_state);
fuel_consumption(sim1)
% Dynamic Programming control + Quadratic Programming control allocation
sim2 = simulation_DP_QuadProg(init_state);
fuel_consumption(sim2)
% DP no allocation
sim5 = simulation_normal_DP_noallocation(init_state);
 
% PD control + PWPF modulator
sim3 = simulation_normal_PID_PWPF(init_state);
fuel_consumption(sim3)

% PD control + Quadratic Programming
sim4 = simulation_normal_PID_QuadProg(init_state);
fuel_consumption(sim4)

description = {'Dynamic Programming + PWPF', 'Dynamic Programming + Quadratic',...
    'PD control + PWPF', 'PD control + Quadratic'};

plot_simulations(sim1,sim2,sim3,sim4,description)

end


function SC = simulation_DP_PWPF(init_state)
simulator_opts.defaultX0 = init_state;
controller.type = 'DP';

Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
%simulator variables
simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
simulator_opts.thruster_allocation_mode = 'PWPF'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
simulator_opts.current_controller = 'controller_DP_attposition';
simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
simulator_opts.T_final = 200; %simulation Tfinal
simulator_opts.h = 0.005; %simulation fixed time steps
simulator_opts.Thruster_max_F = Thruster_max_F;
simulator_opts.Thruster_dist = Thruster_dist;

%active set parameter
simulator_opts.active_set.Weighting_Matrix = [1,0;...
    0,1];
%PWPF and schmitt trigger configuration parameters
PWPFparams = [7.6000    0.3953    0.2387    0.2609    0.998]; %from optimization

simulator_opts.PWPF.Km = PWPFparams(1);
simulator_opts.PWPF.Tm = PWPFparams(2);
simulator_opts.PWPF.h = 0.005;
simulator_opts.PWPF.H_feed = PWPFparams(3);

simulator_opts.schmitt.Uout = Thruster_max_F;
simulator_opts.schmitt.Uon = PWPFparams(4)*simulator_opts.schmitt.Uout;
simulator_opts.schmitt.Uoff = PWPFparams(5)*simulator_opts.schmitt.Uon;
% create and run the simulator
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path;
end

function SC = simulation_DP_QuadProg(init_state)
simulator_opts.defaultX0 = init_state;
controller.type = 'DP';

Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
%simulator variables
simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
simulator_opts.thruster_allocation_mode = 'active set discrete'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
simulator_opts.current_controller = 'controller_DP_attposition';
simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
simulator_opts.T_final = 200; %simulation Tfinal
simulator_opts.h = 0.005; %simulation fixed time steps
simulator_opts.Thruster_max_F = Thruster_max_F;
simulator_opts.Thruster_dist = Thruster_dist;

%active set parameter
simulator_opts.active_set.Weighting_Matrix = [1,0;...
    0,1];
%PWPF and schmitt trigger configuration parameters
simulator_opts.PWPF.Km = 1.5;
simulator_opts.PWPF.Tm = .2;
simulator_opts.PWPF.h = 0.005;
simulator_opts.PWPF.H_feed = 0.6;

simulator_opts.schmitt.Uout = Thruster_max_F;
simulator_opts.schmitt.Uon = 0.3*simulator_opts.schmitt.Uout;
simulator_opts.schmitt.Uoff = 0.8*simulator_opts.schmitt.Uon;
% create and run the simulator
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path;

end

function SC = simulation_normal_PID_PWPF(init_state)
simulator_opts.defaultX0 = init_state;

%   First it generates a PD controller and then it launches the simulator that uses this controller
Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
%% generate controller
%controller variables
KF = [10.55  114.40  0]; %optimized
controller.Kp_F = KF(1);
controller.Kd_F = KF(2);
controller.Ki_F = KF(3);

controller.Kp_M = 0.0078;
controller.Kd_M = 0.8047;
controller.Ki_M = 0;
controller.type = 'PID';

%% Simulate the results
%simulator variables
simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one thruster inoperative
simulator_opts.thruster_allocation_mode = 'PWPF'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
simulator_opts.controller_InterpmodeF = ''; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = ''; %interpolation mehod of M controller output
simulator_opts.T_final = 200; %simulation Tfinal
simulator_opts.h = 0.005; %simulation fixed time steps
simulator_opts.Thruster_max_F = Thruster_max_F;
simulator_opts.Thruster_dist = Thruster_dist;



%active set parameter
simulator_opts.active_set.Weighting_Matrix = [1,0;...
    0,1];
%PWPF and schmitt trigger configuration parameters
PWPFparams = [0.1000    0.9812    0.9985    0.9640    0.8761]; %from optimization

simulator_opts.PWPF.Km = PWPFparams(1);
simulator_opts.PWPF.Tm = PWPFparams(2);
simulator_opts.PWPF.h = 0.005;
simulator_opts.PWPF.H_feed = PWPFparams(3);

simulator_opts.schmitt.Uout = Thruster_max_F;
simulator_opts.schmitt.Uon = PWPFparams(4)*simulator_opts.schmitt.Uout;
simulator_opts.schmitt.Uoff = PWPFparams(5)*simulator_opts.schmitt.Uon;

% create and run the simulator
SC = Simulator_CW(simulator_opts,controller);
SC.get_optimal_path;

end

function SC = simulation_normal_PID_QuadProg(init_state)
simulator_opts.defaultX0 = init_state;

%   First it generates a PD controller and then it launches the simulator that uses this controller
Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
%% generate controller
%controller variables
KF = [10.55  114.40  0]; %optimized
controller.Kp_F = KF(1);
controller.Kd_F = KF(2);
controller.Ki_F = KF(3);

controller.Kp_M = 0.0078;
controller.Kd_M = 0.8047;
controller.Ki_M = 0;
controller.type = 'PID';

%% Simulate the results
%simulator variables
simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one thruster inoperative
simulator_opts.thruster_allocation_mode = 'active set discrete'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
simulator_opts.controller_InterpmodeF = ''; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = ''; %interpolation mehod of M controller output
simulator_opts.T_final = 200; %simulation Tfinal
simulator_opts.h = 0.005; %simulation fixed time steps
simulator_opts.Thruster_max_F = Thruster_max_F;
simulator_opts.Thruster_dist = Thruster_dist;



%active set parameter
simulator_opts.active_set.Weighting_Matrix = [1,0;...
    0,1];
%PWPF and schmitt trigger configuration parameters
simulator_opts.PWPF.Km = 1.5;
simulator_opts.PWPF.Tm = .2;
simulator_opts.PWPF.h = 0.005;
simulator_opts.PWPF.H_feed = 0.6;

simulator_opts.schmitt.Uout = Thruster_max_F;
simulator_opts.schmitt.Uon = 0.3*simulator_opts.schmitt.Uout;
simulator_opts.schmitt.Uoff = 0.8*simulator_opts.schmitt.Uon;

% create and run the simulator
SC = Simulator_CW(simulator_opts,controller);
SC.get_optimal_path;

end

function plot_simulations(sim1,sim2,sim3,sim4,description)

% x1 = temp_s.history.X_ode45(:,1);
% v1 = temp_s.history.X_ode45(:,4);
% F1 = temp_s.history.Force_Moment_log(:,1)*temp_s.Mass;
% [theta1,theta2,theta3] = quat2angle(temp_s.history.X_ode45(:,10:-1:7));
% t3 = theta3*180/pi;
% w3 = temp_s.history.X_ode45(:,13);
% M3 = temp_s.history.Force_Moment_log(:,6);
T_ode45 = sim1.history.T_ode45;

s1 = figure('Name','states - position',...
    'Position',[543.4000   49.0000  518.4000  326.4000],...
    'color', 'white');

% title('states - position (m)')
plot(T_ode45, sim1.history.X_ode45(:,1), '.')
hold on
plot(T_ode45, sim2.history.X_ode45(:,1))
plot(T_ode45, sim3.history.X_ode45(:,1))
plot(T_ode45, sim4.history.X_ode45(:,1))
grid on
legend(description)
xlabel('time (s)')
ylabel('relative position (m)')
end

function fuel_consumption(sim0)
% calculate fuel consumption
FC_history = cumsum(sim0.history.F_Th_Opt)/max(sim0.history.F_Th_Opt(:));
FC_total = sum(FC_history(end,:))*0.005;
fprintf('Total Thruster-On Time (Fuel Consumption) = %.3f seconds\n', FC_total)
end


function SC = simulation_normal_DP_noallocation(init_state)
simulator_opts.defaultX0 = init_state;

Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
%simulator variables
simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
simulator_opts.thruster_allocation_mode = 'saturate-only'; % {'active set discrete', 'PWPF', 'Schmitt', 'none'}
simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
simulator_opts.current_controller = 'controller_DP_attposition';
simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
simulator_opts.T_final = 200; %simulation Tfinal
simulator_opts.h = 0.005; %simulation fixed time steps
simulator_opts.Thruster_max_F = Thruster_max_F;
simulator_opts.Thruster_dist = Thruster_dist;

%active set parameter
simulator_opts.active_set.Weighting_Matrix = [1,0;...
    0,1];
%PWPF for fault and schmitt trigger configuration parameters
PWPF1params = [26.0896    0.8597    0.2015    0.1493    0.8138]; %from optimization

simulator_opts.PWPF1.Km = PWPF1params(1);
simulator_opts.PWPF1.Tm = PWPF1params(2);
simulator_opts.PWPF1.h = 0.005;
simulator_opts.PWPF1.H_feed = PWPF1params(3);

simulator_opts.schmitt1.Uout = Thruster_max_F;
simulator_opts.schmitt1.Uon = PWPF1params(4)*simulator_opts.schmitt1.Uout;
simulator_opts.schmitt1.Uoff = PWPF1params(5)*simulator_opts.schmitt1.Uon;
simulator_opts.PWPF2 = simulator_opts.PWPF1; simulator_opts.schmitt2 = simulator_opts.schmitt1;

%PWPF for normal conditions and schmitt trigger configuration parameters
PWPF3params = [45.1521    0.9972    0.2359    0.4493    0.9888]; %from optimization

simulator_opts.PWPF3.Km = PWPF3params(1);
simulator_opts.PWPF3.Tm = PWPF3params(2);
simulator_opts.PWPF3.h = 0.005;
simulator_opts.PWPF3.H_feed = PWPF3params(3);

simulator_opts.schmitt3.Uout = Thruster_max_F;
simulator_opts.schmitt3.Uon = PWPF3params(4)*simulator_opts.schmitt3.Uout;
simulator_opts.schmitt3.Uoff = PWPF3params(5)*simulator_opts.schmitt3.Uon;
simulator_opts.PWPF4 = simulator_opts.PWPF3; simulator_opts.schmitt4 = simulator_opts.schmitt3;
simulator_opts.PWPF5 = simulator_opts.PWPF3; simulator_opts.schmitt5 = simulator_opts.schmitt3;
simulator_opts.PWPF6 = simulator_opts.PWPF3; simulator_opts.schmitt6 = simulator_opts.schmitt3;

% create and run the simulator
SC = Simulator_CW(simulator_opts);
SC.get_optimal_path;
end