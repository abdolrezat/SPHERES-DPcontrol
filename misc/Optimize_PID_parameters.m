function [x,f]  = Optimize_PID_parameters( )
%OPTIMIZE_PID_PARAMETERS Optimize Kp,Ki,Kd 
%   optimizes parameters to get minimum of (1/2)*x'*Q*x + (1/2)*v'*Q*v 

K0 = [10.55  114.40  0]; % [kp kd ki]
% K0m = [ 0.0078 0.8047 0];
ub_scale = [150 500 2];

K0 = K0./ub_scale;
lb =[0 0 0];
ub = [1 1 .5];
%% patternsearch algorithm
options = optimoptions('patternsearch','Display','iter','PlotFcn',@psplotbestf,...
    'InitialMeshSize',0.2,'MeshTolerance', 1e-2, 'StepTolerance', 1e-2, 'MaxMeshSize', .5, ...
    'UseParallel', true, 'UseCompletePoll', true, 'AccelerateMesh', true); %

[x,f] = patternsearch(@objective_function,K0,[],[],[],[],lb,ub,[],options);

x = x.*ub_scale

%% global search with fmincon
% problem = createOptimProblem('fmincon','objective',...
%  @objective_function,'x0',K0,'lb',lb,'ub',ub,'options',options);
% gs = GlobalSearch('Display','iter');
%  [x,f] = run(gs,problem)


%% old
% Start with the default options
% options = optimoptions('fmincon');
% Modify options setting
% options = optimoptions(options,'Display', 'final');
% options = optimoptions(options,'PlotFcn', {  @optimplotx @optimplotfunccount @optimplotfval @optimplotstepsize });
% [x,fval,exitflag,output,lambda,grad,hessian] = ...
% fmincon(@objective_function,K0,[],[],[],[],lb,ub,[],options);


function total_cost = objective_function(K)
ub_scale = [150 500 2];
K = K.*ub_scale;
R0 = [-0.5 -0.5 -0.5];
cR2 = 10;
cR3 = 30;
cR4 = 60; 
cost1 = run_PID_control(R0, K) *cR4;
cost2 = run_PID_control(R0*cR2, K)*cR4/cR2;
cost3 = run_PID_control(R0*cR3, K)*cR4/cR3;
cost4 = run_PID_control(R0*cR4, K);

total_cost = cost1 + cost2 + cost3 + cost4;
fprintf('cost_path1: %.2g | cost_path2: %.2g | cost_path3: %.2g | cost_path4: %.2g, Total = %.2g\nKp = %.2f, Kd = %.2f, Ki = %.2f\n',...
    cost1,cost2,cost3,cost4,total_cost,K(1),K(2),K(3))
fprintf('----------------------------------------------------------------------\n')

function trip_cost = run_PID_control(POS_i, K)
% this script runs a sample of the code in this repository
%   First it generates a PD controller and then it launches the simulator that uses this controller 
% addpath(path,genpath('generate_controller'))
% addpath(path,'simulator')

Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
%% generate controller
%controller variables
    controller.Kp_F = K(1);
    controller.Kd_F = K(2);
    controller.Ki_F = K(3);
% KF = [9.5325  110.4300  0];
%     controller.Kp_F = KF(1);
%     controller.Kd_F = KF(2);
%     controller.Ki_F = KF(3);

%     controller.Kp_M = K(1);
%     controller.Kd_M = K(2);
%     controller.Ki_M = K(3);
    controller.Kp_M = 0.0078;
    controller.Kd_M = 0.8047;
    controller.Ki_M = 0;    
    controller.type = 'PID';

%% Simulate the results
%simulator variables
    simulator_opts.mode = 'normal';   % 'normal' for all thrusters operative or 'fault' for one thruster inoperative
    simulator_opts.thruster_allocation_mode = 'saturate-only'; % {'active set discrete', 'PWPF', 'Schmitt','saturate-only', 'none'}
    simulator_opts.faulty_thruster_index = []; %index of faulty thruster(s) #0-#11
    simulator_opts.controller_InterpmodeF = 'linear'; %interpolation mehod of F controller output
    simulator_opts.controller_InterpmodeM = 'linear'; %interpolation mehod of M controller output
    simulator_opts.T_final = 120; %simulation Tfinal
    simulator_opts.h = 0.005; %simulation fixed time steps
    simulator_opts.Thruster_max_F = Thruster_max_F;
    simulator_opts.Thruster_dist = Thruster_dist;
    
    
    %initial state1
    dr0 = POS_i; %initial relative position offset
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(0),deg2rad(0),deg2rad(0))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    
    %active set parameter
    simulator_opts.active_set.Weighting_Matrix = [0.95,0;...
                                                  0,0.05];
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
history = SC.get_optimal_path_history();
trip_cost = derive_cost(history, 'XVU');
% SC.plot_optimal_path()

function C = derive_cost(history, type)
Qx = 10;
Qv = 45*Qx;
Qt = 0.01;
Qw = 0.8*Qt;
R = 1; 
DIST = (9.65E-2);

X = history.X_ode45;
U = history.F_Th_Opt;

if(strcmp(type,'XVU'))
C = sum( sum( X(:,1:3).^2 ))*Qx + ...
    sum( sum( X(:,4:6).^2 ))*Qv + ...
    sum( U(:))*R;
elseif(strcmp(type,'TWU'))
[theta1,theta2,theta3] = quat2angle(X(:,10:-1:7));

C = sum( theta1.^2 + theta2.^2 + theta3.^2 )*Qt + ...
    sum( sum( X(:,11:13).^2 ))*Qw + ...
    sum( U(:))*R/DIST;
end