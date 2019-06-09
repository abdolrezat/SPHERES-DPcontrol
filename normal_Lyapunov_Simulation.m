function S = normal_Lyapunov_Simulation(thr_hist)

simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
  simulator_opts.thruster_allocation_mode = 'quadratic programming pulse modulation';

simulator_opts.Thruster_max_F = 0.12; % (N)
simulator_opts.h = 0.01; %simulation fixed time steps
  simulator_opts.T_final = 150; %simulation Tfinal

    dr0 = [-10 10 10]; %initial relative position offset
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(0),deg2rad(0),deg2rad(0))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    S = simulation_normal_Lyapunov(simulator_opts);
    S.get_optimal_path(thr_hist)
    S.plot_optimal_path
    S.plot_xy_plane
end

function SC = simulation_normal_Lyapunov(simulator_opts)

Thruster_dist = (9.65E-2); % (meters)
%simulator variables
simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one or more thrusters inoperative
    controller.name = 'controller_normal_DynamicProgramming_c'; %name of controller, will be saved under /controller directory
    simulator_opts.current_controller = controller.name;
simulator_opts.controller_InterpmodeF = 'nearest'; %interpolation mehod of F controller output
simulator_opts.controller_InterpmodeM = 'nearest'; %interpolation mehod of M controller output
simulator_opts.Thruster_dist = Thruster_dist;

% create and run the simulator
SC = Simulator_Lyapunov(simulator_opts);
end




function plot12(S1,S2)

f1 = figure('Name','states - position',...
    'Position',[543.4000   49.0000  518.4000  326.4000],...
    'color', 'white');
T1 = S1.history.T_ode45;
T2 = S2.history.T_ode45;
% title('states - position (m)')
plot(T1, S1.history.X_ode45(:,1), '--')
hold on
plot(T2, S2.history.X_ode45(:,1))
legend('x1','x2')
        title('states - position (m)')

f2 = figure('Name','states - w',...
    'Position',[956.2000   47.4000  518.4000  326.4000],...
    'color', 'white');
plot(T1, S1.history.X_ode45(:,12)*180/pi)
title('states - rotational speeds (deg/sec)')

hold on
plot(T2, S2.history.X_ode45(:,12)*180/pi)
legend('w1','w2')

end
