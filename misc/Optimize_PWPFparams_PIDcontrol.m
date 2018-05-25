function [P_optim,Fval_optim]  = Optimize_PWPFparams_PIDcontrol( )
% Optimize PWPF
%   optimizes parameters to get minimum of (1/2)*x'*Q*x + (1/2)*v'*Q*v 

N = 2;
lb =[0 0 0.2 0 0];
ub = [1 1 1 1 1];
Fval_optim = Inf;

for i = 1:N
M0(i,:) = lb + rand(size(lb)).*(ub - lb);
end

M0 = [M0;
    0.1000/10    0.9812    0.9985    0.9640    0.8761;
    18.6682/10    0.9910    0.2520    0.1853    0.9957;
    0.6880/10    0.1020    0.9725    0.6705    0.9735;
    1.5/10,0.2,0.6,0.3,0.8];

tic
for i = 1:size(M0,1) 
    
P0 = M0(i,:);
%% patternsearch algorithm
options = optimoptions('patternsearch','Display','iter','PlotFcn',@psplotbestf,...
    'InitialMeshSize',0.1,'MeshTolerance', 1e-2, 'StepTolerance', 1e-2, 'MaxMeshSize', 0.3, ...
    'UseParallel', true, 'UseCompletePoll', true, 'AccelerateMesh', true); %
[x,Fval,~,Output] = patternsearch(@objective_function,P0,[],[],[],[],lb,ub,[],options);
fprintf('The number of iterations was : %d\n', Output.iterations);
fprintf('The number of function evaluations was : %d\n', Output.funccount);
fprintf('The best function value found was : %g\n', Fval);
x

if(Fval < Fval_optim)
    Fval_optim = Fval
    P_optim = x
end

end
toc

P_optim(1) = P_optim(1)*10
Fval_optim

%% fminsearch
% options_fmins = optimset('TolX',1e-2,'TolFun',1,'FiniteDifferenceStepSize',1e-1);
% x = fminsearch(@objective_function,P0,options_fmins);


function total_cost = objective_function(P)
P(1) = P(1)*10;
R0 = [-.5 -0.5 -0.5];

cR2 = 10;
cR3 = 30;
cR4 = 60; 
cost1 = run_PID_control(R0, P) *cR4;
cost2 = run_PID_control(R0*cR2, P)*cR4/cR2;
cost3 = run_PID_control(R0*cR3, P)*cR4/cR3;
cost4 = run_PID_control(R0*cR4, P);
% 
total_cost = cost1 + cost2 + cost3 + cost4;
% fprintf('cost_path1: %.2g | cost_path2: %.2g | cost_path3: %.2g | cost_path4: %.2g, Total = %.2g\n',...
%     cost1,cost2,cost3,cost4,total_cost)
% disp(P)
% fprintf('----------------------------------------------------------------------\n')

function trip_cost = run_PID_control(POS_i, P)
% this script runs a sample of the code in this repository
%   First it generates a PD controller and then it launches the simulator that uses this controller 
% addpath(path,genpath('generate_controller'))
% addpath(path,'simulator')

Thruster_max_F = 0.12; % (N)
Thruster_dist = (9.65E-2); % (meters)
%% generate controller
%controller variables
KF = [10.55  114.40  0];
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
    simulator_opts.T_final = 100; %simulation Tfinal
    simulator_opts.h = 0.01; %simulation fixed time steps
    simulator_opts.Thruster_max_F = Thruster_max_F;
    simulator_opts.Thruster_dist = Thruster_dist;
    
    
    %initial state1
    dr0 = POS_i; %initial relative position offset
    dv0 = [0 0 0]; %initial relative velocity offset
    q0 = flip(angle2quat(deg2rad(0),deg2rad(0),deg2rad(0))); %initial angles offset (yaw,pitch,roll)
    w0 = [0 0 0]; %initial rotational speed offset
    simulator_opts.defaultX0 = [dr0 dv0 q0 w0]';
    
    %active set parameter
    simulator_opts.active_set.Weighting_Matrix = [10,0;...
                                                  0,0.005];
    %PWPF and schmitt trigger configuration parameters
    simulator_opts.PWPF.Km = P(1);
    simulator_opts.PWPF.Tm = P(2);
    simulator_opts.PWPF.h = simulator_opts.h; 
    simulator_opts.PWPF.H_feed = P(3);
    
    simulator_opts.schmitt.Uout = Thruster_max_F;
    simulator_opts.schmitt.Uon = P(4)*simulator_opts.schmitt.Uout;
    simulator_opts.schmitt.Uoff = P(5)*simulator_opts.schmitt.Uon;
    
% create and run the simulator
SC = Simulator_CW(simulator_opts,controller);
history = SC.get_optimal_path_history();
trip_cost = derive_cost(history, 'XVU') + derive_cost(history, 'TWU');
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